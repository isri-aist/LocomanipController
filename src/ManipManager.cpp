#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/NumberInput.h>
#include <mc_tasks/ImpedanceTask.h>

#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/MathUtils.h>
#include <LocomanipController/LocomanipController.h>
#include <LocomanipController/ManipManager.h>
#include <LocomanipController/ManipPhase.h>

using namespace LMC;

void ManipManager::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  mcRtcConfig("name", name);
  mcRtcConfig("objHorizon", objHorizon);
  mcRtcConfig("objPoseTopic", objPoseTopic);
  mcRtcConfig("objVelTopic", objVelTopic);
  mcRtcConfig("handTaskStiffness", handTaskStiffness);
  mcRtcConfig("preReachDuration", preReachDuration);
  mcRtcConfig("reachDuration", reachDuration);
  mcRtcConfig("reachHandDistThre", reachHandDistThre);

  if(mcRtcConfig.has("objToHandTranss"))
  {
    for(const auto & hand : Hands::Both)
    {
      mcRtcConfig("objToHandTranss")(std::to_string(hand), objToHandTranss.at(hand));
    }
  }
  if(mcRtcConfig.has("preReachTranss"))
  {
    for(const auto & hand : Hands::Both)
    {
      mcRtcConfig("preReachTranss")(std::to_string(hand), preReachTranss.at(hand));
    }
  }

  mcRtcConfig("impedanceGain", impGain);

  mcRtcConfig("graspCommands", graspCommands);
  mcRtcConfig("ungraspCommands", ungraspCommands);

  mcRtcConfig("objToFootMidTrans", objToFootMidTrans);
  mcRtcConfig("footstepDuration", footstepDuration);
  mcRtcConfig("doubleSupportRatio", doubleSupportRatio);
}

void ManipManager::VelModeData::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  mcRtcConfig("nonholonomicObjectMotion", nonholonomicObjectMotion);
}

void ManipManager::VelModeData::reset(bool enabled, const sva::PTransformd & currentObjPose)
{
  enabled_ = enabled;
  targetVel_.setZero();
  frontFootstep_ = nullptr;
  frontWaypointPose_ = currentObjPose;
  objDeltaTrans_.setZero();
}

ManipManager::ManipManager(LocomanipController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig)
: ctlPtr_(ctlPtr), objPoseFunc_(std::make_shared<BWC::CubicInterpolator<sva::PTransformd, sva::MotionVecd>>())
{
  config_.load(mcRtcConfig);

  if(mcRtcConfig.has("VelMode"))
  {
    velModeData_.config_.load(mcRtcConfig("VelMode"));
  }
}

void ManipManager::reset()
{
  // Setup ROS
  if(nh_)
  {
    mc_rtc::log::error("[ManipManager] ROS node handle is already instantiated.");
  }
  else
  {
    nh_ = std::make_shared<ros::NodeHandle>();
    // Use a dedicated queue so as not to call callbacks of other modules
    nh_->setCallbackQueue(&callbackQueue_);

    if(!config_.objPoseTopic.empty())
    {
      objPoseSub_ =
          nh_->subscribe<geometry_msgs::PoseStamped>(config_.objPoseTopic, 1, &ManipManager::objPoseCallback, this);
    }
    if(!config_.objVelTopic.empty())
    {
      objVelSub_ =
          nh_->subscribe<geometry_msgs::TwistStamped>(config_.objVelTopic, 1, &ManipManager::objVelCallback, this);
    }
  }

  objPoseOffsetFunc_.reset();
  objPoseOffset_ = sva::PTransformd::Identity();

  sva::PTransformd objPoseWithoutOffset = objPoseOffset_.inv() * ctl().obj().posW();
  objPoseFunc_->clearPoints();
  objPoseFunc_->appendPoint(std::make_pair(ctl().t(), objPoseWithoutOffset));
  objPoseFunc_->appendPoint(std::make_pair(ctl().t() + config_.objHorizon, objPoseWithoutOffset));
  objPoseFunc_->calcCoeff();
  lastWaypointPose_ = objPoseWithoutOffset;

  for(const auto & hand : Hands::Both)
  {
    manipPhases_.emplace(hand, std::make_shared<ManipPhase::Free>(hand, this));
  }

  requireImpGainUpdate_ = true;

  requireFootstepFollowingObj_ = false;

  velModeData_.reset(false, objPoseWithoutOffset);
}

void ManipManager::stop()
{
  objPoseSub_.shutdown();
  objVelSub_.shutdown();
  nh_.reset();

  removeFromGUI(*ctl().gui());
  removeFromLogger(ctl().logger());
}

void ManipManager::update()
{
  // Call ROS callback
  callbackQueue_.callAvailable(ros::WallDuration());

  if(velModeData_.enabled_)
  {
    updateForVelMode();
  }
  updateObjTraj();
  updateHandTraj();
  updateFootstep();
}

void ManipManager::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.addElement({ctl().name(), config_.name, "Status"},
                 mc_rtc::gui::Label("waypointQueueSize", [this]() { return std::to_string(waypointQueue_.size()); }));
  gui.addElement(
      {ctl().name(), config_.name, "Status"}, mc_rtc::gui::ElementsStacking::Horizontal,
      mc_rtc::gui::Label("LeftManipPhase", [this]() { return std::to_string(manipPhases_.at(Hand::Left)->label()); }),
      mc_rtc::gui::Label("RightManipPhase",
                         [this]() { return std::to_string(manipPhases_.at(Hand::Right)->label()); }));
  gui.addElement({ctl().name(), config_.name, "Status"}, mc_rtc::gui::ElementsStacking::Horizontal,
                 mc_rtc::gui::Label("LeftHandSurface", [this]() { return surfaceName(Hand::Left); }),
                 mc_rtc::gui::Label("RightHandSurface", [this]() { return surfaceName(Hand::Right); }));

  gui.addElement(
      {ctl().name(), config_.name, "Config"},
      mc_rtc::gui::NumberInput(
          "objHorizon", [this]() { return config_.objHorizon; }, [this](double v) { config_.objHorizon = v; }),
      mc_rtc::gui::NumberInput(
          "handTaskStiffness", [this]() { return config_.handTaskStiffness; },
          [this](double v) { config_.handTaskStiffness = v; }),
      mc_rtc::gui::NumberInput(
          "preReachDuration", [this]() { return config_.preReachDuration; },
          [this](double v) { config_.preReachDuration = v; }),
      mc_rtc::gui::NumberInput(
          "reachDuration", [this]() { return config_.reachDuration; }, [this](double v) { config_.reachDuration = v; }),
      mc_rtc::gui::NumberInput(
          "reachHandDistThre", [this]() { return config_.reachHandDistThre; },
          [this](double v) { config_.reachHandDistThre = v; }),
      mc_rtc::gui::NumberInput(
          "footstepDuration", [this]() { return config_.footstepDuration; },
          [this](double v) { config_.footstepDuration = v; }),
      mc_rtc::gui::NumberInput(
          "doubleSupportRatio", [this]() { return config_.doubleSupportRatio; },
          [this](double v) { config_.doubleSupportRatio = v; }));

  gui.addElement({ctl().name(), config_.name, "Config", "VelMode"},
                 mc_rtc::gui::Checkbox(
                     "nonholonomicObjectMotion", [this]() { return velModeData_.config_.nonholonomicObjectMotion; },
                     [this]() {
                       velModeData_.config_.nonholonomicObjectMotion = !velModeData_.config_.nonholonomicObjectMotion;
                     }));

  gui.addElement({ctl().name(), config_.name, "ImpedanceGain"},
                 mc_rtc::gui::ArrayInput(
                     "Mass", {"cx", "cy", "cz", "fx", "fy", "fz"},
                     [this]() -> const sva::ImpedanceVecd & { return config_.impGain.mass().vec(); },
                     [this](const Eigen::Vector6d & v) {
                       config_.impGain.mass().vec(v);
                       requireImpGainUpdate_ = true;
                     }),
                 mc_rtc::gui::ArrayInput(
                     "Damper", {"cx", "cy", "cz", "fx", "fy", "fz"},
                     [this]() -> const sva::ImpedanceVecd & { return config_.impGain.damper().vec(); },
                     [this](const Eigen::Vector6d & v) {
                       config_.impGain.damper().vec(v);
                       requireImpGainUpdate_ = true;
                     }),
                 mc_rtc::gui::ArrayInput(
                     "Spring", {"cx", "cy", "cz", "fx", "fy", "fz"},
                     [this]() -> const sva::ImpedanceVecd & { return config_.impGain.spring().vec(); },
                     [this](const Eigen::Vector6d & v) {
                       config_.impGain.spring().vec(v);
                       requireImpGainUpdate_ = true;
                     }),
                 mc_rtc::gui::ArrayInput(
                     "Wrench", {"cx", "cy", "cz", "fx", "fy", "fz"},
                     [this]() -> const sva::ImpedanceVecd & { return config_.impGain.wrench().vec(); },
                     [this](const Eigen::Vector6d & v) {
                       config_.impGain.wrench().vec(v);
                       requireImpGainUpdate_ = true;
                     }));
}

void ManipManager::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.removeCategory({ctl().name(), config_.name});
}

void ManipManager::addToLogger(mc_rtc::Logger & logger)
{
  logger.addLogEntry(config_.name + "_waypointQueueSize", this, [this]() { return waypointQueue_.size(); });

  logger.addLogEntry(config_.name + "_objPose_ref", this, [this]() { return ctl().obj().posW(); });
  logger.addLogEntry(config_.name + "_objPose_measured", this, [this]() { return ctl().realObj().posW(); });

  logger.addLogEntry(config_.name + "_objVel_ref", this, [this]() { return ctl().obj().velW(); });
  logger.addLogEntry(config_.name + "_objVel_measured", this, [this]() { return ctl().realObj().velW(); });

  MC_RTC_LOG_HELPER(config_.name + "_objPoseOffset", objPoseOffset_);

  for(const auto & hand : Hands::Both)
  {
    logger.addLogEntry(config_.name + "_manipPhase_" + std::to_string(hand), this,
                       [this, hand]() { return std::to_string(manipPhases_.at(hand)->label()); });
  }

  logger.addLogEntry(config_.name + "_velMode", this,
                     [this]() -> std::string { return velModeData_.enabled_ ? "ON" : "OFF"; });
  logger.addLogEntry(config_.name + "_targetVel", this, [this]() { return velModeData_.targetVel_; });
}

void ManipManager::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntries(this);
}

const std::string & ManipManager::surfaceName(const Hand & hand) const
{
  return ctl().handTasks_.at(hand)->surface();
}

bool ManipManager::appendWaypoint(const Waypoint & newWaypoint)
{
  // Check time of new waypoint
  if(newWaypoint.startTime < ctl().t())
  {
    mc_rtc::log::error("[ManipManager] Ignore a new waypoint with past time: {} < {}", newWaypoint.startTime,
                       ctl().t());
    return false;
  }
  if(!waypointQueue_.empty())
  {
    const Waypoint & lastWaypoint = waypointQueue_.back();
    if(newWaypoint.startTime < lastWaypoint.endTime)
    {
      mc_rtc::log::error("[ManipManager] Ignore a new waypoint earlier than the last waypoint: {} < {}",
                         newWaypoint.startTime, lastWaypoint.endTime);
      return false;
    }
  }

  // Push to the queue
  waypointQueue_.push_back(newWaypoint);

  return true;
}

void ManipManager::reachHandToObj()
{
  for(const auto & hand : Hands::Both)
  {
    if(manipPhases_.at(hand)->label() != ManipPhaseLabel::Free)
    {
      mc_rtc::log::error("[ManipManager] The hand must be in Free phase to reach, but the {} hand is in {} phase.",
                         std::to_string(hand), std::to_string(ctl().manipManager_->manipPhase(hand)->label()));
      continue;
    }
    sva::PTransformd reachHandPose = config_.objToHandTranss.at(hand) * ctl().obj().posW();
    double reachHandDist =
        (reachHandPose.translation() - ctl().handTasks_.at(hand)->surfacePose().translation()).norm();
    if(reachHandDist > config_.reachHandDistThre)
    {
      mc_rtc::log::error(
          "[ManipManager] The reaching position of the {} hand is too far from the current position: {} > {}.",
          std::to_string(hand), reachHandDist, config_.reachHandDistThre);
      continue;
    }
    manipPhases_.at(hand) = std::make_shared<ManipPhase::PreReach>(hand, this);
  }
}

void ManipManager::releaseHandFromObj()
{
  for(const auto & hand : Hands::Both)
  {
    if(manipPhases_.at(hand)->label() != ManipPhaseLabel::Hold)
    {
      mc_rtc::log::error("[ManipManager] The hand must be in Hold phase to release, but the {} hand is in {} phase.",
                         std::to_string(hand), std::to_string(ctl().manipManager_->manipPhase(hand)->label()));
      continue;
    }

    if(config_.ungraspCommands.empty())
    {
      manipPhases_.at(hand) = std::make_shared<ManipPhase::Release>(hand, this);
    }
    else
    {
      manipPhases_.at(hand) = std::make_shared<ManipPhase::Ungrasp>(hand, this);
    }
  }
}

bool ManipManager::setObjPoseOffset(const sva::PTransformd & newObjPoseOffset, double interpDuration)
{
  if(objPoseOffsetFunc_)
  {
    mc_rtc::log::error("[ManipManager] The object pose offset is being interpolated, so it cannot be set anew.");
    return false;
  }
  if(interpDuration < 0.0)
  {
    mc_rtc::log::error("[ManipManager] Ignore the object pose offset with negative interpolation duration: {}",
                       interpDuration);
    return false;
  }

  objPoseOffsetFunc_ = std::make_shared<BWC::CubicInterpolator<sva::PTransformd, sva::MotionVecd>>();
  objPoseOffsetFunc_->appendPoint(std::make_pair(ctl().t(), objPoseOffset_));
  objPoseOffsetFunc_->appendPoint(std::make_pair(ctl().t() + interpDuration, newObjPoseOffset));
  objPoseOffsetFunc_->calcCoeff();
  return true;
}

void ManipManager::requireFootstepFollowingObj()
{
  if(velModeData_.enabled_)
  {
    mc_rtc::log::error("[ManipManager] requireFootstepFollowingObj is not available in the velocity mode.");
    return;
  }
  requireFootstepFollowingObj_ = true;
}

bool ManipManager::startVelMode()
{
  if(velModeData_.enabled_)
  {
    mc_rtc::log::warning("[ManipManager] It is already in velocity mode, but startVelMode is called.");
    return false;
  }

  if(!(manipPhases_.at(Hand::Left)->label() == ManipPhaseLabel::Hold
       || manipPhases_.at(Hand::Right)->label() == ManipPhaseLabel::Hold))
  {
    mc_rtc::log::error(
        "[ManipManager] startVelMode is available only when the manipulation phase is Hold. Left: {}, Right: {}",
        std::to_string(manipPhases_.at(Hand::Left)->label()), std::to_string(manipPhases_.at(Hand::Right)->label()));
    return false;
  }

  if(!waypointQueue_.empty())
  {
    mc_rtc::log::error("[ManipManager] startVelMode is available only when the waypoint queue is empty: {}",
                       waypointQueue_.size());
    return false;
  }

  if(ctl().footManager_->velModeData().config_.enableOnlineFootstepUpdate)
  {
    mc_rtc::log::error("[ManipManager] enableOnlineFootstepUpdate must be false in the FootManager velocity mode.");
    return false;
  }

  if(!ctl().footManager_->startVelMode())
  {
    mc_rtc::log::error("[ManipManager] Failed to start velocity mode in Footmanager.");
    return false;
  }

  velModeData_.reset(true, calcRefObjPose(ctl().t()));

  requireFootstepFollowingObj_ = false;

  return true;
}

bool ManipManager::endVelMode()
{
  if(!velModeData_.enabled_)
  {
    mc_rtc::log::warning("[ManipManager] It is not in velocity mode, but endVelMode is called.");
    return false;
  }

  velModeData_.reset(false, calcRefObjPose(ctl().t()));

  ctl().footManager_->endVelMode();

  return true;
}

void ManipManager::setRelativeVel(const Eigen::Vector3d & targetVel)
{
  velModeData_.targetVel_ = targetVel;
  if(velModeData_.config_.nonholonomicObjectMotion)
  {
    velModeData_.targetVel_.y() = 0;
  }
}

void ManipManager::updateObjTraj()
{
  // Update waypointQueue_
  while(!waypointQueue_.empty() && waypointQueue_.front().endTime < ctl().t())
  {
    lastWaypointPose_ = waypointQueue_.front().pose;
    waypointQueue_.pop_front();
  }

  // Update objPoseFunc_
  {
    sva::PTransformd currentObjPose = lastWaypointPose_;

    objPoseFunc_->clearPoints();

    if(waypointQueue_.empty() || ctl().t() < waypointQueue_.front().startTime)
    {
      objPoseFunc_->appendPoint(std::make_pair(ctl().t(), currentObjPose));
    }

    for(const auto & waypoint : waypointQueue_)
    {
      objPoseFunc_->appendPoint(std::make_pair(waypoint.startTime, currentObjPose));

      currentObjPose = waypoint.pose;
      objPoseFunc_->appendPoint(std::make_pair(waypoint.endTime, currentObjPose));

      if(ctl().t() + config_.objHorizon <= waypoint.endTime && !requireFootstepFollowingObj_)
      {
        break;
      }
    }

    if(waypointQueue_.empty() || waypointQueue_.back().endTime < ctl().t() + config_.objHorizon)
    {
      objPoseFunc_->appendPoint(std::make_pair(ctl().t() + config_.objHorizon, currentObjPose));
    }

    objPoseFunc_->calcCoeff();
  }

  // Update objPoseOffset_
  {
    if(objPoseOffsetFunc_)
    {
      objPoseOffset_ = (*objPoseOffsetFunc_)(std::min(ctl().t(), objPoseOffsetFunc_->endTime()));
      if(objPoseOffsetFunc_->endTime() <= ctl().t())
      {
        objPoseOffsetFunc_.reset();
      }
    }
  }

  // Update control object pose
  sva::PTransformd objPoseWithoutOffset = calcRefObjPose(ctl().t());
  {
    ctl().obj().posW(objPoseOffset_ * objPoseWithoutOffset);
    ctl().obj().velW(calcRefObjVel(ctl().t()));
  }

  // Update object waypoints visualization
  {
    std::vector<sva::PTransformd> waypointPoseList = {objPoseWithoutOffset};
    for(const auto & waypoint : waypointQueue_)
    {
      waypointPoseList.push_back(waypoint.pose);
    }

    ctl().gui()->removeCategory({ctl().name(), config_.name, "WaypointsMarker"});
    ctl().gui()->addElement({ctl().name(), config_.name, "WaypointsMarker"},
                            mc_rtc::gui::Trajectory("Waypoints", {mc_rtc::gui::Color::Green, 0.04},
                                                    [waypointPoseList]() { return waypointPoseList; }));
  }
}

void ManipManager::updateHandTraj()
{
  // Update manipulation phase
  for(const auto & hand : Hands::Both)
  {
    manipPhases_.at(hand)->run();
    if(manipPhases_.at(hand)->complete())
    {
      auto nextManipPhase = manipPhases_.at(hand)->makeNextManipPhase();
      if(nextManipPhase)
      {
        manipPhases_.at(hand) = nextManipPhase;
      }
      else
      {
        mc_rtc::log::error_and_throw("[ManipManager] {} next manipulation phase is nullptr.", std::to_string(hand));
      }
    }
  }

  // Set impedance gains of hand tasks
  if(requireImpGainUpdate_)
  {
    requireImpGainUpdate_ = false;
    for(const auto & hand : Hands::Both)
    {
      ctl().handTasks_.at(hand)->gains() = config_.impGain;
    }
  }
}

void ManipManager::updateFootstep()
{
  if(!requireFootstepFollowingObj_)
  {
    return;
  }
  requireFootstepFollowingObj_ = false;

  if(waypointQueue_.empty())
  {
    mc_rtc::log::error("[ManipManager] Waypoint queue must not be empty in updateFootstep.");
    return;
  }
  if(!ctl().footManager_->footstepQueue().empty())
  {
    mc_rtc::log::error("[ManipManager] Footstep queue must be empty in updateFootstep.");
    return;
  }

  auto convertTo2d = [](const sva::PTransformd & pose) -> Eigen::Vector3d {
    return Eigen::Vector3d(pose.translation().x(), pose.translation().y(), mc_rbdyn::rpyFromMat(pose.rotation()).z());
  };
  auto convertTo3d = [](const Eigen::Vector3d & trans) -> sva::PTransformd {
    return sva::PTransformd(sva::RotZ(trans.z()), Eigen::Vector3d(trans.x(), trans.y(), 0));
  };

  BWC::Foot foot = BWC::Foot::Left;
  sva::PTransformd footMidpose = BWC::projGround(sva::interpolate(
      ctl().footManager_->targetFootPose(BWC::Foot::Left), ctl().footManager_->targetFootPose(BWC::Foot::Right), 0.5));
  double startTime = ctl().t() + 1.0;
  while(startTime < waypointQueue_.back().endTime)
  {
    double objPoseTime = startTime + config_.footstepDuration;
    if(objPoseFunc_->endTime() < objPoseTime)
    {
      objPoseTime = objPoseFunc_->endTime();
    }
    Eigen::Vector3d deltaTrans =
        convertTo2d(config_.objToFootMidTrans * calcRefObjPose(objPoseTime) * footMidpose.inv());
    footMidpose = convertTo3d(ctl().footManager_->clampDeltaTrans(deltaTrans, foot)) * footMidpose;
    const auto & footstep = makeFootstep(foot, footMidpose, startTime);
    ctl().footManager_->appendFootstep(footstep);

    foot = BWC::opposite(foot);
    startTime = footstep.transitEndTime;
  }
  const auto & footstep = makeFootstep(foot, footMidpose, startTime);
  ctl().footManager_->appendFootstep(footstep);
}

void ManipManager::updateForVelMode()
{
  auto convertTo2d = [](const sva::PTransformd & pose) -> Eigen::Vector3d {
    return Eigen::Vector3d(pose.translation().x(), pose.translation().y(), mc_rbdyn::rpyFromMat(pose.rotation()).z());
  };
  auto convertTo3d = [](const Eigen::Vector3d & trans) -> sva::PTransformd {
    return sva::PTransformd(sva::RotZ(trans.z()), Eigen::Vector3d(trans.x(), trans.y(), 0));
  };

  const auto & frontFootstep = ctl().footManager_->footstepQueue().front();

  // When the front footstep of queue switches to the next one, the corresponding waypoint is added to the queue
  if(velModeData_.frontFootstep_ != &frontFootstep)
  {
    velModeData_.frontFootstep_ = &frontFootstep;
    velModeData_.frontWaypointPose_ = convertTo3d(velModeData_.objDeltaTrans_) * velModeData_.frontWaypointPose_;
    appendWaypoint(Waypoint(std::max(frontFootstep.transitStartTime, ctl().t()), frontFootstep.transitEndTime,
                            velModeData_.frontWaypointPose_));
  }

  // Assuming that the front footstep of queue is fixed, find the objDeltaTrans where the next footstep is in
  // reachability (i.e., not changed by clampDeltaTrans)
  {
    sva::PTransformd frontFootMidpose =
        ctl().footManager_->config().midToFootTranss.at(frontFootstep.foot).inv() * frontFootstep.pose;
    auto calcFootstepDeltaTrans = [&](const Eigen::Vector3d & _objDeltaTrans) {
      sva::PTransformd newWaypointPose = convertTo3d(_objDeltaTrans) * velModeData_.frontWaypointPose_;
      sva::PTransformd nextFootMidpose = config_.objToFootMidTrans * newWaypointPose;
      return convertTo2d(nextFootMidpose * frontFootMidpose.inv());
    };
    constexpr size_t searchNum = 10;
    constexpr double reduceRatio = 0.8;
    constexpr double clampDeltaTransThre = 1e-6;
    Eigen::Vector3d footstepDeltaTrans;
    Eigen::Vector3d objDeltaTrans = ctl().footManager_->config().footstepDuration * velModeData_.targetVel_;
    for(size_t i = 1; i <= searchNum; i++)
    {
      if(i == searchNum)
      {
        objDeltaTrans.setZero();
        footstepDeltaTrans.setZero();
        break;
      }

      footstepDeltaTrans = calcFootstepDeltaTrans(objDeltaTrans);
      Eigen::Vector3d footstepDeltaTransClamped =
          ctl().footManager_->clampDeltaTrans(footstepDeltaTrans, BWC::opposite(frontFootstep.foot));
      if((footstepDeltaTrans - footstepDeltaTransClamped).norm() < clampDeltaTransThre)
      {
        break;
      }
      else
      {
        objDeltaTrans *= reduceRatio;
      }
    }
    velModeData_.objDeltaTrans_ = objDeltaTrans;
    ctl().footManager_->setRelativeVel(footstepDeltaTrans / ctl().footManager_->config().footstepDuration);
  }
}

BWC::Footstep ManipManager::makeFootstep(const BWC::Foot & foot,
                                         const sva::PTransformd & footMidpose,
                                         double startTime,
                                         const mc_rtc::Configuration & swingTrajConfig) const
{
  return BWC::Footstep(foot, ctl().footManager_->config().midToFootTranss.at(foot) * footMidpose, startTime,
                       startTime + 0.5 * config_.doubleSupportRatio * config_.footstepDuration,
                       startTime + (1.0 - 0.5 * config_.doubleSupportRatio) * config_.footstepDuration,
                       startTime + config_.footstepDuration, swingTrajConfig);
}

void ManipManager::objPoseCallback(const geometry_msgs::PoseStamped::ConstPtr & poseStMsg)
{
  // Update real object pose
  const auto & poseMsg = poseStMsg->pose;
  sva::PTransformd pose(
      Eigen::Quaterniond(poseMsg.orientation.w, poseMsg.orientation.x, poseMsg.orientation.y, poseMsg.orientation.z)
          .normalized()
          .toRotationMatrix()
          .transpose(),
      Eigen::Vector3d(poseMsg.position.x, poseMsg.position.y, poseMsg.position.z));
  ctl().realObj().posW(pose);
}

void ManipManager::objVelCallback(const geometry_msgs::TwistStamped::ConstPtr & twistStMsg)
{
  // Update real object velocity
  const auto & twistMsg = twistStMsg->twist;
  sva::MotionVecd vel(Eigen::Vector3d(twistMsg.angular.x, twistMsg.angular.y, twistMsg.angular.z),
                      Eigen::Vector3d(twistMsg.linear.x, twistMsg.linear.y, twistMsg.linear.z));
  ctl().realObj().velW(vel);
}
