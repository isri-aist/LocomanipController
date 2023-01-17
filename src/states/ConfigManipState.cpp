#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/MathUtils.h>
#include <LocomanipController/LocomanipController.h>
#include <LocomanipController/ManipManager.h>
#include <LocomanipController/ManipPhase.h>
#include <LocomanipController/states/ConfigManipState.h>

using namespace LMC;

void ConfigManipState::start(mc_control::fsm::Controller & _ctl)
{
  State::start(_ctl);

  phase_ = 0;

  output("OK");
}

bool ConfigManipState::run(mc_control::fsm::Controller &)
{
  if(phase_ == 0)
  {
    if(config_.has("configs") && config_("configs")("preUpdateObj", false))
    {
      ctl().manipManager_->appendWaypoint(
          Waypoint(ctl().t(), ctl().t() + 1.0, ctl().manipManager_->objPoseOffset().inv() * ctl().realObj().posW()));
      phase_ = 1;
    }
    else
    {
      phase_ = 2;
    }
  }
  else if(phase_ == 1)
  {
    if(ctl().manipManager_->waypointQueue().empty())
    {
      phase_ = 2;
    }
  }
  else if(phase_ == 2)
  {
    if(config_.has("configs") && config_("configs")("preWalk", false))
    {
      auto convertTo2d = [](const sva::PTransformd & pose) -> Eigen::Vector3d {
        return Eigen::Vector3d(pose.translation().x(), pose.translation().y(),
                               mc_rbdyn::rpyFromMat(pose.rotation()).z());
      };
      const sva::PTransformd & initialFootMidpose =
          BWC::projGround(sva::interpolate(ctl().footManager_->targetFootPose(BWC::Foot::Left),
                                           ctl().footManager_->targetFootPose(BWC::Foot::Right), 0.5));
      sva::PTransformd objToFootMidTrans =
          config_("configs")("objToFootMidTrans", ctl().manipManager_->config().objToFootMidTrans);
      ctl().footManager_->walkToRelativePose(
          convertTo2d(objToFootMidTrans * ctl().manipManager_->calcRefObjPose(ctl().t()) * initialFootMidpose.inv()));
      phase_ = 3;
    }
    else
    {
      phase_ = 4;
    }
  }
  else if(phase_ == 3)
  {
    if(ctl().footManager_->footstepQueue().empty())
    {
      phase_ = 4;
    }
  }
  else if(phase_ == 4)
  {
    bool isReached = (ctl().manipManager_->manipPhase(Hand::Left)->label() == ManipPhaseLabel::Hold
                      || ctl().manipManager_->manipPhase(Hand::Right)->label() == ManipPhaseLabel::Hold);
    if(config_.has("configs") && config_("configs")("reach", !isReached))
    {
      ctl().manipManager_->reachHandToObj();
      phase_ = 5;
    }
    else
    {
      phase_ = 6;
    }
  }
  else if(phase_ == 5)
  {
    if(ctl().manipManager_->manipPhase(Hand::Left)->label() == ManipPhaseLabel::Hold
       && ctl().manipManager_->manipPhase(Hand::Right)->label() == ManipPhaseLabel::Hold)
    {
      phase_ = 6;
    }
  }
  else if(phase_ == 6)
  {
    if(config_.has("configs") && config_("configs").has("preObjPoseOffset"))
    {
      ctl().manipManager_->setObjPoseOffset(config_("configs")("preObjPoseOffset"), 1.0);
      phase_ = 7;
    }
    else
    {
      phase_ = 8;
    }
  }
  else if(phase_ == 7)
  {
    if(!ctl().manipManager_->interpolatingObjPoseOffset())
    {
      phase_ = 8;
    }
  }
  else if(phase_ == 8)
  {
    if(config_.has("configs") && config_("configs").has("waypointList"))
    {
      double startTime = ctl().t();
      sva::PTransformd pose = ctl().manipManager_->calcRefObjPose(ctl().t());

      for(const auto & waypointConfig : config_("configs")("waypointList"))
      {
        if(waypointConfig.has("startTime"))
        {
          startTime = ctl().t() + static_cast<double>(waypointConfig("startTime"));
        }
        double endTime;
        if(waypointConfig.has("endTime"))
        {
          endTime = ctl().t() + static_cast<double>(waypointConfig("endTime"));
        }
        else if(waypointConfig.has("duration"))
        {
          endTime = startTime + static_cast<double>(waypointConfig("duration"));
        }
        if(waypointConfig.has("pose"))
        {
          pose = waypointConfig("pose");
        }
        else if(waypointConfig.has("relPose"))
        {
          pose = static_cast<sva::PTransformd>(waypointConfig("relPose")) * pose;
        }
        ctl().manipManager_->appendWaypoint(Waypoint(startTime, endTime, pose));

        startTime = endTime;
      }

      if(config_("configs")("footstep", true))
      {
        ctl().manipManager_->requireFootstepFollowingObj();
      }

      phase_ = 9;
    }
    else if(config_.has("configs") && config_("configs").has("velocityMode"))
    {
      ctl().manipManager_->startVelMode();
      ctl().manipManager_->setRelativeVel(config_("configs")("velocityMode")("velocity"));
      velModeEndTime_ = ctl().t() + static_cast<double>(config_("configs")("velocityMode")("duration"));

      phase_ = 9;
    }
    else
    {
      phase_ = 10;
    }
  }
  else if(phase_ == 9)
  {
    if(config_.has("configs") && config_("configs").has("velocityMode"))
    {
      if(ctl().t() > velModeEndTime_ - 1.0 && ctl().manipManager_->velModeEnabled())
      {
        ctl().manipManager_->setRelativeVel(Eigen::Vector3d::Zero());
      }
      if(ctl().t() > velModeEndTime_ && ctl().manipManager_->velModeEnabled())
      {
        ctl().manipManager_->endVelMode();
      }
    }

    if(ctl().manipManager_->waypointQueue().empty() && ctl().footManager_->footstepQueue().empty()
       && !ctl().manipManager_->velModeEnabled())
    {
      phase_ = 10;
    }
  }
  else if(phase_ == 10)
  {
    if(config_.has("configs") && config_("configs").has("postObjPoseOffset"))
    {
      ctl().manipManager_->setObjPoseOffset(config_("configs")("postObjPoseOffset"), 1.0);
      phase_ = 11;
    }
    else
    {
      phase_ = 12;
    }
  }
  else if(phase_ == 11)
  {
    if(!ctl().manipManager_->interpolatingObjPoseOffset())
    {
      phase_ = 12;
    }
  }
  else if(phase_ == 12)
  {
    if(config_.has("configs") && config_("configs")("release", true))
    {
      ctl().manipManager_->releaseHandFromObj();
      phase_ = 13;
    }
    else
    {
      phase_ = 14;
    }
  }
  else if(phase_ == 13)
  {
    if(ctl().manipManager_->manipPhase(Hand::Left)->label() == ManipPhaseLabel::Free
       && ctl().manipManager_->manipPhase(Hand::Right)->label() == ManipPhaseLabel::Free)
    {
      phase_ = 14;
    }
  }

  return phase_ == 14;
}

void ConfigManipState::teardown(mc_control::fsm::Controller &) {}

EXPORT_SINGLE_STATE("LMC::ConfigManip", ConfigManipState)
