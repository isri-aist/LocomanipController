#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/Form.h>

#include <LocomanipController/LocomanipController.h>
#include <LocomanipController/ManipManager.h>
#include <LocomanipController/ManipPhase.h>
#include <LocomanipController/states/GuiManipState.h>

using namespace LMC;

void GuiManipState::start(mc_control::fsm::Controller & _ctl)
{
  State::start(_ctl);

  // Setup GUI
  ctl().gui()->addElement({ctl().name(), "GuiManip"},
                          mc_rtc::gui::Button("Reach", [this]() { ctl().manipManager_->reachHandToObj(); }),
                          mc_rtc::gui::Button("Release", [this]() { ctl().manipManager_->releaseHandFromObj(); }));
  ctl().gui()->addElement(
      {ctl().name(), "GuiManip", "MoveObj"},
      mc_rtc::gui::Form(
          "MoveObj",
          [this](const mc_rtc::Configuration & config) {
            if(!(ctl().manipManager_->manipPhase(Hand::Left)->label() == ManipPhaseLabel::Hold
                 || ctl().manipManager_->manipPhase(Hand::Right)->label() == ManipPhaseLabel::Hold))
            {
              mc_rtc::log::error("[GuiManipState] The object waypoint command can be sent only when the manipulation "
                                 "phase is Hold. Left: {}, Right: {}",
                                 std::to_string(ctl().manipManager_->manipPhase(Hand::Left)->label()),
                                 std::to_string(ctl().manipManager_->manipPhase(Hand::Right)->label()));
              return;
            }
            if(ctl().manipManager_->waypointQueue().size() > 0)
            {
              mc_rtc::log::error(
                  "[GuiManipState] The object waypoint command can be sent only when the waypoint queue is empty: {}",
                  ctl().manipManager_->waypointQueue().size());
              return;
            }
            double startTime = ctl().t() + static_cast<double>(config(moveObjConfigKeys_.at("startTime")));
            double endTime = ctl().t() + static_cast<double>(config(moveObjConfigKeys_.at("endTime")));
            sva::PTransformd pose =
                sva::PTransformd(sva::RotZ(mc_rtc::constants::toRad(config(moveObjConfigKeys_.at("yaw")))),
                                 Eigen::Vector3d(config(moveObjConfigKeys_.at("x")), 0.0, 0.0))
                * ctl().manipManager_->calcRefObjPose(ctl().t());
            ctl().manipManager_->appendWaypoint(Waypoint(startTime, endTime, pose));
            if(config(moveObjConfigKeys_.at("footstep")))
            {
              ctl().manipManager_->requireFootstepFollowingObj();
            }
          },
          mc_rtc::gui::FormNumberInput(moveObjConfigKeys_.at("x"), true, 0.0),
          mc_rtc::gui::FormNumberInput(moveObjConfigKeys_.at("yaw"), true, 0.0),
          mc_rtc::gui::FormNumberInput(moveObjConfigKeys_.at("startTime"), true, 2.0),
          mc_rtc::gui::FormNumberInput(moveObjConfigKeys_.at("endTime"), true, 12.0),
          mc_rtc::gui::FormCheckbox(moveObjConfigKeys_.at("footstep"), true, true)));

  output("OK");
}

bool GuiManipState::run(mc_control::fsm::Controller &)
{
  return false;
}

void GuiManipState::teardown(mc_control::fsm::Controller &)
{
  // Clean up GUI
  ctl().gui()->removeCategory({ctl().name(), "GuiManip"});
}

EXPORT_SINGLE_STATE("LMC::GuiManip", GuiManipState)
