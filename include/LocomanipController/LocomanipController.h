#pragma once

#include <BaselineWalkingController/BaselineWalkingController.h>

#include <LocomanipController/HandTypes.h>

namespace mc_tasks
{
namespace force
{
struct ImpedanceTask;
} // namespace force
} // namespace mc_tasks

namespace LMC
{
class ManipManager;

/** \brief Humanoid loco-manipulation controller. */
struct LocomanipController : public BWC::BaselineWalkingController
{
public:
  /** \brief Constructor. */
  LocomanipController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  /** \brief Reset a controller.

      This method is called when starting the controller.
   */
  void reset(const mc_control::ControllerResetData & resetData) override;

  /** \brief Run a controller.

      This method is called every control period.
   */
  bool run() override;

  /** \brief Stop a controller.

      This method is called when stopping the controller.
   */
  void stop() override;

  /** \brief Accessor to the control object. */
  inline mc_rbdyn::Robot & obj()
  {
    return robot("obj");
  }

  /** \brief Const accessor to the control object. */
  inline const mc_rbdyn::Robot & obj() const
  {
    return robot("obj");
  }

  /** \brief Accessor to the real object. */
  inline mc_rbdyn::Robot & realObj()
  {
    return realRobot("obj");
  }

  /** \brief Const accessor to the real object. */
  inline const mc_rbdyn::Robot & realObj() const
  {
    return realRobot("obj");
  }

public:
  //! Hand tasks
  std::unordered_map<Hand, std::shared_ptr<mc_tasks::force::ImpedanceTask>> handTasks_;

  //! Manipulation manager
  std::shared_ptr<ManipManager> manipManager_;
};
} // namespace LMC
