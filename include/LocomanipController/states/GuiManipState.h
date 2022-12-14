#pragma once

#include <LocomanipController/State.h>

namespace LMC
{
/** \brief FSM state to send manipulation command from GUI. */
struct GuiManipState : State
{
public:
  /** \brief Start. */
  void start(mc_control::fsm::Controller & ctl) override;

  /** \brief Run. */
  bool run(mc_control::fsm::Controller & ctl) override;

  /** \brief Teardown. */
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  //! Entry keys of GUI form
  //! @{
  const std::unordered_map<std::string, std::string> walkToObjConfigKeys_ = {
      {"x", "goal x relative to object [m]"},
      {"y", "goal y relative to object [m]"},
      {"yaw", "goal yaw relative to object [deg]"}};
  const std::unordered_map<std::string, std::string> moveObjConfigKeys_ = {{"x", "relative goal x [m]"},
                                                                           {"yaw", "relative goal yaw [deg]"},
                                                                           {"startTime", "start time from now [sec]"},
                                                                           {"endTime", "end time from now [sec]"},
                                                                           {"footstep", "whether to enable footstep"}};
  const std::unordered_map<std::string, std::string> updateObjConfigKeys_ = {
      {"target", "target object pose"},
      {"interpDuration", "interpolation duration [sec]"}};
  const std::unordered_map<std::string, std::string> poseOffsetConfigKeys_ = {
      {"xyz", "xyz position [m]"},
      {"rpy", "rpy orientation [deg]"},
      {"interpDuration", "interpolation duration [sec]"}};
  //! @}
};
} // namespace LMC
