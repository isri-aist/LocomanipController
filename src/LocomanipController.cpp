#include <mc_tasks/ImpedanceTask.h>
#include <mc_tasks/MetaTaskLoader.h>

#include <LocomanipController/LocomanipController.h>
#include <LocomanipController/ManipManager.h>
#include <LocomanipController/centroidal/CentroidalManagerPreviewControlExtZmp.h>

using namespace LMC;

LocomanipController::LocomanipController(mc_rbdyn::RobotModulePtr rm,
                                         double dt,
                                         const mc_rtc::Configuration & _config,
                                         bool allowEmptyManager)
: BWC::BaselineWalkingController(rm, dt, _config, true)
{
  // Setup tasks
  if(config().has("HandTaskList"))
  {
    for(const auto & handTaskConfig : config()("HandTaskList"))
    {
      Hand hand = strToHand(handTaskConfig("hand"));
      handTasks_.emplace(hand,
                         mc_tasks::MetaTaskLoader::load<mc_tasks::force::ImpedanceTask>(solver(), handTaskConfig));
      handTasks_.at(hand)->name("HandTask_" + std::to_string(hand));
    }
  }
  else
  {
    mc_rtc::log::warning("[LocomanipController] HandTaskList configuration is missing.");
  }

  // Setup managers
  if(!centroidalManager_)
  {
    if(config().has("CentroidalManager"))
    {
      std::string centroidalManagerMethod = config()("CentroidalManager")("method", std::string(""));
      if(centroidalManagerMethod == "PreviewControlExtZmp")
      {
        centroidalManager_ =
            std::make_shared<CentroidalManagerPreviewControlExtZmp>(this, config()("CentroidalManager"));
      }
      else
      {
        if(!allowEmptyManager)
        {
          mc_rtc::log::error_and_throw("[LocomanipController] Invalid centroidalManagerMethod: {}.",
                                       centroidalManagerMethod);
        }
      }
    }
    else
    {
      mc_rtc::log::warning("[LocomanipController] CentroidalManager configuration is missing.");
    }
  }
  if(config().has("ManipManager"))
  {
    manipManager_ = std::make_shared<ManipManager>(this, config()("ManipManager"));
  }
  else
  {
    mc_rtc::log::warning("[LocomanipController] ManipManager configuration is missing.");
  }

  mc_rtc::log::success("[LocomanipController] Constructed.");
}

void LocomanipController::reset(const mc_control::ControllerResetData & resetData)
{
  BaselineWalkingController::reset(resetData);

  mc_rtc::log::success("[LocomanipController] Reset.");
}

bool LocomanipController::run()
{
  if(enableManagerUpdate_)
  {
    // Update managers
    manipManager_->update();
  }

  return BaselineWalkingController::run();
}

void LocomanipController::stop()
{
  // Clean up tasks
  for(const auto & hand : Hands::Both)
  {
    solver().removeTask(handTasks_.at(hand));
  }

  // Clean up managers
  manipManager_->stop();
  manipManager_.reset();

  BaselineWalkingController::stop();
}
