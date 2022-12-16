#include <BaselineWalkingController/FootManager.h>
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
    if(config_.has("configs") && config_("configs")("reach", true))
    {
      ctl().manipManager_->reachHandToObj();
      phase_ = 1;
    }
    else
    {
      phase_ = 2;
    }
  }
  else if(phase_ == 1)
  {
    if(ctl().manipManager_->manipPhase(Hand::Left)->label() == ManipPhaseLabel::Hold
       && ctl().manipManager_->manipPhase(Hand::Right)->label() == ManipPhaseLabel::Hold)
    {
      phase_ = 2;
    }
  }
  else if(phase_ == 2)
  {
    // Set waypoint
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
        double endTime = startTime + static_cast<double>(waypointConfig("duration"));
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

      phase_ = 3;
    }
    else
    {
      phase_ = 4;
    }
  }
  else if(phase_ == 3)
  {
    if(ctl().manipManager_->waypointQueue().empty() && ctl().footManager_->footstepQueue().empty())
    {
      phase_ = 4;
    }
  }
  else if(phase_ == 4)
  {
    if(config_.has("configs") && config_("configs")("release", true))
    {
      ctl().manipManager_->releaseHandFromObj();
      phase_ = 5;
    }
    else
    {
      phase_ = 6;
    }
  }
  else if(phase_ == 5)
  {
    if(ctl().manipManager_->manipPhase(Hand::Left)->label() == ManipPhaseLabel::Free
       && ctl().manipManager_->manipPhase(Hand::Right)->label() == ManipPhaseLabel::Free)
    {
      phase_ = 6;
    }
  }

  return phase_ == 6;
}

void ConfigManipState::teardown(mc_control::fsm::Controller &) {}

EXPORT_SINGLE_STATE("LMC::ConfigManip", ConfigManipState)
