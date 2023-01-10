#include <mc_tasks/ImpedanceTask.h>

#include <LocomanipController/LocomanipController.h>
#include <LocomanipController/ManipManager.h>
#include <LocomanipController/ManipPhase.h>

using namespace LMC;
using namespace LMC::ManipPhase;

const LocomanipController & Base::ctl() const
{
  return manipManager_->ctl();
}

LocomanipController & Base::ctl()
{
  return manipManager_->ctl();
}

Free::Free(const Hand & hand, ManipManager * manipManager) : Base(ManipPhaseLabel::Free, hand, manipManager)
{
  ctl().solver().removeTask(ctl().handTasks_.at(hand_));
}

PreReach::PreReach(const Hand & hand, ManipManager * manipManager) : Base(ManipPhaseLabel::PreReach, hand, manipManager)
{
  // Add hand task
  ctl().handTasks_.at(hand_)->reset();
  ctl().solver().addTask(ctl().handTasks_.at(hand_));

  // Setup stiffness interpolation of hand task
  manipManager_->handTaskStiffnessFunc(std::make_shared<BWC::CubicInterpolator<double>>(
      std::map<double, double>{{ctl().t(), 0.0},
                               {ctl().t() + manipManager_->config().handTaskStiffnessInterpDuration,
                                manipManager_->config().handTaskStiffness}}));

  endTime_ = ctl().t() + manipManager_->config().preReachDuration;
}

void PreReach::run()
{
  // Set target pose of hand tasks
  ctl().handTasks_.at(hand_)->targetPose(manipManager_->config().preReachTranss.at(hand_)
                                         * manipManager_->config().objToHandTranss.at(hand_) * ctl().obj().posW());
}

bool PreReach::complete() const
{
  return endTime_ <= ctl().t();
}

std::shared_ptr<Base> PreReach::makeNextManipPhase() const
{
  return std::make_shared<Reach>(hand_, manipManager_);
}

Reach::Reach(const Hand & hand, ManipManager * manipManager) : Base(ManipPhaseLabel::Reach, hand, manipManager)
{
  // Setup reaching interpolation
  reachingRatioFunc_ = std::make_shared<BWC::CubicInterpolator<double>>(
      std::map<double, double>{{ctl().t(), 0.0}, {ctl().t() + manipManager_->config().reachDuration, 1.0}});
}

void Reach::run()
{
  // Set target pose of hand tasks
  if(reachingRatioFunc_)
  {
    double reachingRatio = 0.0;
    if(ctl().t() < reachingRatioFunc_->endTime())
    {
      reachingRatio = (*reachingRatioFunc_)(ctl().t());
    }
    else
    {
      reachingRatio = (*reachingRatioFunc_)(reachingRatioFunc_->endTime());
      reachingRatioFunc_.reset();
    }
    ctl().handTasks_.at(hand_)->targetPose(
        sva::interpolate(manipManager_->config().preReachTranss.at(hand_), sva::PTransformd::Identity(), reachingRatio)
        * manipManager_->config().objToHandTranss.at(hand_) * ctl().obj().posW());
  }
}

bool Reach::complete() const
{
  return !reachingRatioFunc_;
}

std::shared_ptr<Base> Reach::makeNextManipPhase() const
{
  if(manipManager_->config().graspCommands.empty())
  {
    return std::make_shared<Hold>(hand_, manipManager_);
  }
  else
  {
    return std::make_shared<Grasp>(hand_, manipManager_);
  }
}

Grasp::Grasp(const Hand & hand, ManipManager * manipManager) : Base(ManipPhaseLabel::Grasp, hand, manipManager)
{
  // Send gripper commands
  for(const auto & gripperCommandConfig : manipManager_->config().graspCommands)
  {
    ctl().robot().gripper(gripperCommandConfig("name")).configure(gripperCommandConfig);
  }
}

void Grasp::run()
{
  ctl().handTasks_.at(hand_)->targetPose(manipManager_->config().objToHandTranss.at(hand_) * ctl().obj().posW());
}

bool Grasp::complete() const
{
  for(const auto & gripperCommandConfig : manipManager_->config().graspCommands)
  {
    if(!ctl().robot().gripper(gripperCommandConfig("name")).complete())
    {
      return false;
    }
  }
  return true;
}

std::shared_ptr<Base> Grasp::makeNextManipPhase() const
{
  return std::make_shared<Hold>(hand_, manipManager_);
}

Hold::Hold(const Hand & hand, ManipManager * manipManager) : Base(ManipPhaseLabel::Hold, hand, manipManager) {}

void Hold::run()
{
  ctl().handTasks_.at(hand_)->targetPose(manipManager_->config().objToHandTranss.at(hand_) * ctl().obj().posW());
}

Ungrasp::Ungrasp(const Hand & hand, ManipManager * manipManager) : Base(ManipPhaseLabel::Ungrasp, hand, manipManager)
{
  // Send gripper commands
  for(const auto & gripperCommandConfig : manipManager_->config().ungraspCommands)
  {
    ctl().robot().gripper(gripperCommandConfig("name")).configure(gripperCommandConfig);
  }
}

void Ungrasp::run()
{
  ctl().handTasks_.at(hand_)->targetPose(manipManager_->config().objToHandTranss.at(hand_) * ctl().obj().posW());
}

bool Ungrasp::complete() const
{
  for(const auto & gripperCommandConfig : manipManager_->config().ungraspCommands)
  {
    if(!ctl().robot().gripper(gripperCommandConfig("name")).complete())
    {
      return false;
    }
  }
  return true;
}

std::shared_ptr<Base> Ungrasp::makeNextManipPhase() const
{
  return std::make_shared<Release>(hand_, manipManager_);
}

Release::Release(const Hand & hand, ManipManager * manipManager) : Base(ManipPhaseLabel::Release, hand, manipManager)
{
  // Setup reaching interpolation
  reachingRatioFunc_ = std::make_shared<BWC::CubicInterpolator<double>>(
      std::map<double, double>{{ctl().t(), 1.0}, {ctl().t() + manipManager_->config().reachDuration, 0.0}});
}

void Release::run()
{
  // Set target pose of hand tasks
  if(reachingRatioFunc_)
  {
    double reachingRatio = 0.0;
    if(ctl().t() < reachingRatioFunc_->endTime())
    {
      reachingRatio = (*reachingRatioFunc_)(ctl().t());
    }
    else
    {
      reachingRatio = (*reachingRatioFunc_)(reachingRatioFunc_->endTime());
      reachingRatioFunc_.reset();
    }
    ctl().handTasks_.at(hand_)->targetPose(
        sva::interpolate(manipManager_->config().preReachTranss.at(hand_), sva::PTransformd::Identity(), reachingRatio)
        * manipManager_->config().objToHandTranss.at(hand_) * ctl().obj().posW());
  }
}

bool Release::complete() const
{
  return !reachingRatioFunc_;
}

std::shared_ptr<Base> Release::makeNextManipPhase() const
{
  return std::make_shared<Free>(hand_, manipManager_);
}

std::string std::to_string(const ManipPhaseLabel & label)
{
  if(label == ManipPhaseLabel::Free)
  {
    return std::string("Free");
  }
  else if(label == ManipPhaseLabel::PreReach)
  {
    return std::string("PreReach");
  }
  else if(label == ManipPhaseLabel::Reach)
  {
    return std::string("Reach");
  }
  else if(label == ManipPhaseLabel::Grasp)
  {
    return std::string("Grasp");
  }
  else if(label == ManipPhaseLabel::Hold)
  {
    return std::string("Hold");
  }
  else if(label == ManipPhaseLabel::Ungrasp)
  {
    return std::string("Ungrasp");
  }
  else if(label == ManipPhaseLabel::Release)
  {
    return std::string("Release");
  }
  else
  {
    mc_rtc::log::error_and_throw("[to_string] Unsupported manipulation phase label: {}",
                                 std::to_string(static_cast<int>(label)));
  }
}
