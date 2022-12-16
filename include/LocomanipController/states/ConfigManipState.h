#pragma once

#include <LocomanipController/State.h>

namespace LMC
{
/** \brief FSM state to send manipulation commands from configuration. */
struct ConfigManipState : State
{
public:
  /** \brief Start. */
  void start(mc_control::fsm::Controller & ctl) override;

  /** \brief Run. */
  bool run(mc_control::fsm::Controller & ctl) override;

  /** \brief Teardown. */
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  //! Phase
  int phase_ = 0;
};
} // namespace LMC
