#pragma once

#include <LocomanipController/State.h>

namespace LMC
{
/** \brief FSM state to initialize. */
struct InitialState : State
{
public:
  /** \brief Start. */
  void start(mc_control::fsm::Controller & ctl) override;

  /** \brief Run. */
  bool run(mc_control::fsm::Controller & ctl) override;

  /** \brief Teardown. */
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  /** \brief Check whether state is completed. */
  bool complete() const;

protected:
  //! Phase
  int phase_ = 0;

  //! Function to interpolate task stiffness
  std::shared_ptr<BWC::CubicInterpolator<double>> stiffnessRatioFunc_;

  //! Stiffness of CoM task
  Eigen::Vector3d comTaskStiffness_ = Eigen::Vector3d::Zero();

  //! Stiffness of base link orientation task
  Eigen::Vector3d baseOriTaskStiffness_ = Eigen::Vector3d::Zero();

  //! Stiffness of foot tasks
  std::unordered_map<BWC::Foot, Eigen::Vector6d> footTasksStiffness_;
};
} // namespace LMC
