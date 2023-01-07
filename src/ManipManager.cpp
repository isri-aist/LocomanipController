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
  mcRtcConfig("handTaskStiffnessInterpDuration", handTaskStiffnessInterpDuration);
  mcRtcConfig("preReachDuration", preReachDuration);
  mcRtcConfig("reachDuration", reachDuration);

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

ManipManager::ManipManager(LocomanipController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig)
: ctlPtr_(ctlPtr), objPoseFunc_(std::make_shared<BWC::CubicInterpolator<sva::PTransformd, sva::MotionVecd>>())
{
  config_.load(mcRtcConfig);
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

  objPoseFunc_->clearPoints();
  sva::PTransformd objPoseWithoutOffset = objPoseOffset_.inv() * ctl().obj().posW();
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

  velMode_ = false;
  targetVel_.setZero();
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

  if(velMode_)
  {
    updateObjForVelMode();
  }
  updateObjTraj();
  updateHandTraj();
  if(velMode_)
  {
    updateFootstepForVelMode();
  }
  else
  {
    updateFootstep();
  }
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
          "handTaskStiffnessInterpDuration", [this]() { return config_.handTaskStiffnessInterpDuration; },
          [this](double v) { config_.handTaskStiffnessInterpDuration = v; }),
      mc_rtc::gui::NumberInput(
          "preReachDuration", [this]() { return config_.preReachDuration; },
          [this](double v) { config_.preReachDuration = v; }),
      mc_rtc::gui::NumberInput(
          "reachDuration", [this]() { return config_.reachDuration; }, [this](double v) { config_.reachDuration = v; }),
      mc_rtc::gui::NumberInput(
          "footstepDuration", [this]() { return config_.footstepDuration; },
          [this](double v) { config_.footstepDuration = v; }),
      mc_rtc::gui::NumberInput(
          "doubleSupportRatio", [this]() { return config_.doubleSupportRatio; },
          [this](double v) { config_.doubleSupportRatio = v; }));

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

  for(const auto & hand : Hands::Both)
  {
    logger.addLogEntry(config_.name + "_manipPhase_" + std::to_string(hand), this,
                       [this, hand]() { return std::to_string(manipPhases_.at(hand)->label()); });
  }

  logger.addLogEntry(config_.name + "_velMode", this, [this]() -> std::string { return velMode_ ? "ON" : "OFF"; });
  logger.addLogEntry(config_.name + "_targetVel", this, [this]() { return targetVel_; });
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
      mc_rtc::log::error("[ManipManager] Cannot reach hand because {} manipulation phase is {}", std::to_string(hand),
                         std::to_string(ctl().manipManager_->manipPhase(hand)->label()));
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
      mc_rtc::log::error("[ManipManager] Cannot release hand because {} manipulation phase is {}", std::to_string(hand),
                         std::to_string(ctl().manipManager_->manipPhase(hand)->label()));
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

bool ManipManager::startVelMode()
{
  if(velMode_)
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

  if(!ctl().footManager_->startVelMode())
  {
    mc_rtc::log::error("[ManipManager] Failed to start velocity mode in Footmanager.");
    return false;
  }

  velMode_ = true;
  targetVel_.setZero();

  return true;
}

bool ManipManager::endVelMode()
{
  if(!velMode_)
  {
    mc_rtc::log::warning("[ManipManager] It is not in velocity mode, but endVelMode is called.");
    return false;
  }

  velMode_ = false;
  targetVel_.setZero();

  ctl().footManager_->endVelMode();

  return true;
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

  // Update control object pose
  if(!velMode_)
  {
    ctl().obj().posW(objPoseOffset_ * calcRefObjPose(ctl().t()));
    ctl().obj().velW(calcRefObjVel(ctl().t()));
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

  // Set stiffness of hand task
  if(handTaskStiffnessFunc_)
  {
    double stiffness = 0.0;
    if(ctl().t() < handTaskStiffnessFunc_->endTime())
    {
      stiffness = (*handTaskStiffnessFunc_)(ctl().t());
    }
    else
    {
      stiffness = (*handTaskStiffnessFunc_)(handTaskStiffnessFunc_->endTime());
      handTaskStiffnessFunc_.reset();
    }
    for(const auto & hand : Hands::Both)
    {
      ctl().handTasks_.at(hand)->stiffness(stiffness);
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

  BWC::Foot foot = BWC::Foot::Left;
  sva::PTransformd footMidpose = sva::PTransformd::Identity();
  double startTime = ctl().t() + 1.0;
  while(startTime < waypointQueue_.back().endTime)
  {
    double objPoseTime = startTime + config_.footstepDuration;
    if(objPoseFunc_->endTime() < objPoseTime)
    {
      objPoseTime = objPoseFunc_->endTime();
    }
    footMidpose = config_.objToFootMidTrans * calcRefObjPose(objPoseTime);
    const auto & footstep = makeFootstep(foot, footMidpose, startTime);
    ctl().footManager_->appendFootstep(footstep);

    foot = BWC::opposite(foot);
    startTime = footstep.transitEndTime;
  }
  const auto & footstep = makeFootstep(foot, footMidpose, startTime);
  ctl().footManager_->appendFootstep(footstep);
}

void ManipManager::updateObjForVelMode()
{
  sva::PTransformd footMidpose = BWC::projGround(sva::interpolate(
      ctl().footManager_->targetFootPose(BWC::Foot::Left), ctl().footManager_->targetFootPose(BWC::Foot::Right), 0.5));
  sva::MotionVecd footMidvel = BWC::projGround(
      0.5 * (ctl().footManager_->targetFootVel(BWC::Foot::Left) + ctl().footManager_->targetFootVel(BWC::Foot::Right)));

  sva::PTransformd objPoseWithoutOffset = config_.objToFootMidTrans.inv() * footMidpose;
  ctl().obj().posW(objPoseOffset_ * objPoseWithoutOffset);
  ctl().obj().velW(config_.objToFootMidTrans.invMul(footMidvel));
  lastWaypointPose_ = objPoseWithoutOffset;
}

void ManipManager::updateFootstepForVelMode()
{
  requireFootstepFollowingObj_ = false;

  auto convertTo2d = [](const sva::PTransformd & pose) -> Eigen::Vector3d {
    return Eigen::Vector3d(pose.translation().x(), pose.translation().y(), mc_rbdyn::rpyFromMat(pose.rotation()).z());
  };
  auto convertTo3d = [](const Eigen::Vector3d & trans) -> sva::PTransformd {
    return sva::PTransformd(sva::RotZ(trans.z()), Eigen::Vector3d(trans.x(), trans.y(), 0));
  };

  // objPoseWithoutOffset is set in lastWaypointPose_
  sva::PTransformd currentTargetFootMidpose = config_.objToFootMidTrans * lastWaypointPose_;
  sva::PTransformd nextTargetFootMidpose = config_.objToFootMidTrans * convertTo3d(targetVel_) * lastWaypointPose_;
  Eigen::Vector3d targetFootMidvel = convertTo2d(nextTargetFootMidpose * currentTargetFootMidpose.inv());

  ctl().footManager_->setRelativeVel(targetFootMidvel);
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
