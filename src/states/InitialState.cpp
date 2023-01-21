#include <mc_rtc/gui/Button.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/OrientationTask.h>

#include <BaselineWalkingController/CentroidalManager.h>
#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/tasks/FirstOrderImpedanceTask.h>
#include <LocomanipController/LocomanipController.h>
#include <LocomanipController/ManipManager.h>
#include <LocomanipController/states/InitialState.h>

using namespace LMC;

void InitialState::start(mc_control::fsm::Controller & _ctl)
{
  State::start(_ctl);

  phase_ = 0;

  // Setup GUI
  ctl().gui()->addElement({ctl().name()}, mc_rtc::gui::Button("Start", [this]() { phase_ = 1; }));

  output("OK");
}

bool InitialState::run(mc_control::fsm::Controller &)
{
  if(phase_ == 0)
  {
    // Auto start
    if(config_.has("configs") && config_("configs").has("autoStartTime")
       && ctl().t() > static_cast<double>(config_("configs")("autoStartTime")))
    {
      phase_ = 1;
    }
  }
  if(phase_ == 1)
  {
    phase_ = 2;

    // Clean up GUI
    ctl().gui()->removeElement({ctl().name()}, "Start");

    // Reset and add tasks
    ctl().comTask_->reset();
    ctl().solver().addTask(ctl().comTask_);
    ctl().baseOriTask_->reset();
    ctl().solver().addTask(ctl().baseOriTask_);
    for(const auto & foot : Feet::Both)
    {
      ctl().footTasks_.at(foot)->reset();
      ctl().solver().addTask(ctl().footTasks_.at(foot));
    }

    // Setup task stiffness interpolation
    comTaskStiffness_ = ctl().comTask_->dimStiffness();
    baseOriTaskStiffness_ = ctl().baseOriTask_->dimStiffness();
    for(const auto & foot : Feet::Both)
    {
      footTasksStiffness_.emplace(foot, ctl().footTasks_.at(foot)->dimStiffness());
    }
    constexpr double stiffnessInterpDuration = 1.0; // [sec]
    stiffnessRatioFunc_ = std::make_shared<BWC::CubicInterpolator<double>>(
        std::map<double, double>{{ctl().t(), 0.0}, {ctl().t() + stiffnessInterpDuration, 1.0}});

    // Reset managers
    ctl().manipManager_->reset();
    ctl().footManager_->reset();
    ctl().centroidalManager_->reset();
    ctl().enableManagerUpdate_ = true;

    // Setup anchor frame
    ctl().centroidalManager_->setAnchorFrame();

    // Send commands to gripper
    if(config_.has("configs") && config_("configs").has("gripperCommands"))
    {
      for(const auto & gripperCommandConfig : config_("configs")("gripperCommands"))
      {
        ctl().robot().gripper(gripperCommandConfig("name")).configure(gripperCommandConfig);
      }
    }

    // Add GUI of managers
    ctl().manipManager_->addToGUI(*ctl().gui());
    ctl().footManager_->addToGUI(*ctl().gui());
    ctl().centroidalManager_->addToGUI(*ctl().gui());
  }
  else if(phase_ == 2)
  {
    phase_ = 3;

    // Add logger of managers
    // Considering the possibility that logger entries assume that variables are set in the manager's update method,
    // it is safe to call the update method once and then add the logger
    ctl().manipManager_->addToLogger(ctl().logger());
    ctl().footManager_->addToLogger(ctl().logger());
    ctl().centroidalManager_->addToLogger(ctl().logger());
  }

  // Interpolate task stiffness
  if(stiffnessRatioFunc_)
  {
    if(ctl().t() <= stiffnessRatioFunc_->endTime())
    {
      double stiffnessRatio = (*stiffnessRatioFunc_)(ctl().t());
      ctl().comTask_->stiffness(stiffnessRatio * comTaskStiffness_);
      ctl().baseOriTask_->stiffness(stiffnessRatio * baseOriTaskStiffness_);
      for(const auto & foot : Feet::Both)
      {
        ctl().footTasks_.at(foot)->stiffness(stiffnessRatio * footTasksStiffness_.at(foot));
      }
    }
    else
    {
      stiffnessRatioFunc_.reset();
    }
  }

  return complete();
}

void InitialState::teardown(mc_control::fsm::Controller &) {}

bool InitialState::complete() const
{
  if(phase_ != 3)
  {
    return false;
  }

  if(stiffnessRatioFunc_)
  {
    return false;
  }

  if(config_.has("configs") && config_("configs").has("gripperCommands"))
  {
    for(const auto & gripperCommandConfig : config_("configs")("gripperCommands"))
    {
      if(!ctl().robot().gripper(gripperCommandConfig("name")).complete())
      {
        return false;
      }
    }
  }

  return true;
}

EXPORT_SINGLE_STATE("LMC::Initial", InitialState)
