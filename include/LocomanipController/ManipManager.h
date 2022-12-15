#pragma once

#include <deque>
#include <unordered_map>

#include <mc_rtc/constants.h>
#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/log/Logger.h>
#include <mc_tasks/ImpedanceGains.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <BaselineWalkingController/FootTypes.h>
#include <BaselineWalkingController/trajectory/CubicInterpolator.h>
#include <LocomanipController/HandTypes.h>

namespace LMC
{
class LocomanipController;

namespace ManipPhase
{
class Base;
};

/** \brief Waypoint of object trajectory. */
struct Waypoint
{
  /** \brief Constructor.
      \param _startTime start time [sec]
      \param _endTime end time [sec]
      \param _pose object pose
  */
  Waypoint(double _startTime, double _endTime, const sva::PTransformd & _pose)
  : startTime(_startTime), endTime(_endTime), pose(_pose){};

  //! Start time [sec]
  double startTime;

  //! End time [sec]
  double endTime;

  //! Object pose
  sva::PTransformd pose;
};

/** Manipulation manager.

    Manipulation manager sets object pose and hand poses.
*/
class ManipManager
{
  friend class ManipPhase::Base;

public:
  /** \brief Configuration. */
  struct Configuration
  {
    //! Name
    std::string name = "ManipManager";

    //! Horizon of object trajectory [sec]
    double objHorizon = 2.0;

    //! Object pose topic name (not subscribe if empty)
    std::string objPoseTopic;

    //! Object velocity topic name (not subscribe if empty)
    std::string objVelTopic;

    //! Stiffness of hand task
    double handTaskStiffness = 1000.0;

    //! Interpolation duration of stiffness of hand task [sec]
    double handTaskStiffnessInterpDuration = 4.0;

    //! Pre-reach duration [sec]
    double preReachDuration = 2.0;

    //! Reach duration [sec]
    double reachDuration = 1.0;

    //! Transformations from object to hand
    std::unordered_map<Hand, sva::PTransformd> objToHandTranss = {
        {Hand::Left, sva::PTransformd(sva::RotY(-1 * mc_rtc::constants::PI / 2), Eigen::Vector3d(0, 0.4, 0))},
        {Hand::Right, sva::PTransformd(sva::RotY(-1 * mc_rtc::constants::PI / 2), Eigen::Vector3d(0, -0.4, 0))}};

    //! Transformations for pre-reach
    std::unordered_map<Hand, sva::PTransformd> preReachTranss = {
        {Hand::Left, sva::PTransformd(Eigen::Vector3d(0, 0, 0.1))},
        {Hand::Right, sva::PTransformd(Eigen::Vector3d(0, 0, 0.1))}};

    //! Impedance gain for hand tasks
    mc_tasks::force::ImpedanceGains impGain = mc_tasks::force::ImpedanceGains::Default();

    //! Grasp gripper commands
    std::vector<mc_rtc::Configuration> graspCommands;

    //! Ungrasp gripper commands
    std::vector<mc_rtc::Configuration> ungraspCommands;

    //! Transformation from object to foot midpose
    sva::PTransformd objToFootMidTrans = sva::PTransformd(Eigen::Vector3d(0, 0, -0.6));

    //! Duration of one footstep. [sec]
    double footstepDuration = 1.6;

    //! Duration ratio of double support phase
    double doubleSupportRatio = 0.35;

    /** \brief Load mc_rtc configuration.
        \param mcRtcConfig mc_rtc configuration
    */
    void load(const mc_rtc::Configuration & mcRtcConfig);
  };

public:
  /** \brief Constructor.
      \param ctlPtr pointer to controller
      \param mcRtcConfig mc_rtc configuration
  */
  ManipManager(LocomanipController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig = {});

  /** \brief Reset.

      This method should be called once when controller is reset.
  */
  void reset();

  /** \brief Update.

      This method should be called once every control cycle.
  */
  virtual void update();

  /** \brief Stop.

      This method should be called once when stopping the controller.
  */
  void stop();

  /** \brief Const accessor to the configuration. */
  inline const Configuration & config() const noexcept
  {
    return config_;
  }

  /** \brief Add entries to the GUI. */
  void addToGUI(mc_rtc::gui::StateBuilder & gui);

  /** \brief Remove entries from the GUI. */
  void removeFromGUI(mc_rtc::gui::StateBuilder & gui);

  /** \brief Add entries to the logger. */
  void addToLogger(mc_rtc::Logger & logger);

  /** \brief Remove entries from the logger. */
  void removeFromLogger(mc_rtc::Logger & logger);

  /** \brief Get hand surface name. */
  const std::string & surfaceName(const Hand & hand) const;

  /** \brief Append a target waypoint to the queue.
      \param newWaypoint waypoint to append
      \return whether newWaypoint is appended
  */
  bool appendWaypoint(const Waypoint & newWaypoint);

  /** \brief Reach hand to object. */
  void reachHandToObj();

  /** \brief Release hand from object. */
  void releaseHandFromObj();

  /** \brief Calculate reference object pose.
      \param t time
  */
  inline sva::PTransformd calcRefObjPose(double t) const
  {
    return (*objPoseFunc_)(t);
  }

  /** \brief Calculate reference object velocity.
      \param t time
  */
  inline sva::MotionVecd calcRefObjVel(double t) const
  {
    return objPoseFunc_->derivative(t, 1);
  }

  /** \brief Access waypoint queue. */
  inline const std::deque<Waypoint> & waypointQueue() const noexcept
  {
    return waypointQueue_;
  }

  /** \brief Get manipulation phase. */
  inline const std::shared_ptr<ManipPhase::Base> & manipPhase(const Hand & hand) const
  {
    return manipPhases_.at(hand);
  }

  /** \brief Set function to interpolate stiffness of hand task. */
  inline void handTaskStiffnessFunc(const std::shared_ptr<BWC::CubicInterpolator<double>> & handTaskStiffnessFunc)
  {
    handTaskStiffnessFunc_ = handTaskStiffnessFunc;
  }

  /** \brief Require sending footstep command following an object. */
  inline void requireFootstepFollowingObj()
  {
    requireFootstepFollowingObj_ = true;
  }

protected:
  /** \brief Const accessor to the controller. */
  inline const LocomanipController & ctl() const
  {
    return *ctlPtr_;
  }

  /** \brief Accessor to the controller. */
  inline LocomanipController & ctl()
  {
    return *ctlPtr_;
  }

  /** \brief Update object trajectory. */
  virtual void updateObjTraj();

  /** \brief Update hand tasks. */
  virtual void updateHandTraj();

  /** \brief Update footstep. */
  virtual void updateFootstep();

  /** \brief Make a footstep.
      \param foot foot
      \param footMidpose middle pose of both feet
      \param startTime time to start the footstep
      \param mcRtcConfig mc_rtc configuration
  */
  BWC::Footstep makeFootstep(const BWC::Foot & foot,
                             const sva::PTransformd & footMidpose,
                             double startTime,
                             const mc_rtc::Configuration & mcRtcConfig = {}) const;

  /** \brief ROS callback of object pose topic. */
  void objPoseCallback(const geometry_msgs::PoseStamped::ConstPtr & poseStMsg);

  /** \brief ROS callback of object velocity topic. */
  void objVelCallback(const geometry_msgs::TwistStamped::ConstPtr & twistStMsg);

public:
  /** \brief Object pose function

      \note This is public only for asynchronous MPC; protected is generally sufficient.
   */
  std::shared_ptr<BWC::CubicInterpolator<sva::PTransformd, sva::MotionVecd>> objPoseFunc_;

protected:
  //! Configuration
  Configuration config_;

  //! Pointer to controller
  LocomanipController * ctlPtr_ = nullptr;

  //! Waypoint queue
  std::deque<Waypoint> waypointQueue_;

  //! Last waypoint pose
  sva::PTransformd lastWaypointPose_ = sva::PTransformd::Identity();

  //! Manipulation phases
  std::unordered_map<Hand, std::shared_ptr<ManipPhase::Base>> manipPhases_;

  //! Function to interpolate stiffness of hand task
  std::shared_ptr<BWC::CubicInterpolator<double>> handTaskStiffnessFunc_;

  //! Whether to require updating impedance gains
  bool requireImpGainUpdate_ = true;

  //! Whether to require sending footstep command following an object
  bool requireFootstepFollowingObj_ = false;

  //! ROS variables
  //! @{
  std::shared_ptr<ros::NodeHandle> nh_;
  ros::CallbackQueue callbackQueue_;
  ros::Subscriber objPoseSub_;
  ros::Subscriber objVelSub_;
  //! @}
};
} // namespace LMC
