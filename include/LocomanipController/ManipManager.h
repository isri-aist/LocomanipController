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

#include <TrajColl/CubicInterpolator.h>

#include <LocomanipController/FootTypes.h>
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

/** \brief Manipulation manager.

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

    //! Pre-reach duration [sec]
    double preReachDuration = 2.0;

    //! Reach duration [sec]
    double reachDuration = 1.0;

    //! Distance threshold of reaching hand [m]
    double reachHandDistThre = 0.5;

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

  /** \brief Velocity mode data.

      In the velocity mode, the object is moved at the specified velocity.
  */
  class VelModeData
  {
  public:
    /** \brief Configuration. */
    struct Configuration
    {
      //! Whether the object moves nonholonomic, like a wheel.
      bool nonholonomicObjectMotion = true;

      /** \brief Load mc_rtc configuration.
          \param mcRtcConfig mc_rtc configuration
      */
      void load(const mc_rtc::Configuration & mcRtcConfig);
    };

  public:
    /** \brief Constructor. */
    VelModeData() {}

    /** \brief Reset.
        \param enabled whether the velocity mode is enabled
        \param currentObjPose current object pose
     */
    void reset(bool enabled, const sva::PTransformd & currentObjPose);

  public:
    //! Configuration
    Configuration config_;

    //! Whether the velocity mode is enabled
    bool enabled_ = false;

    //! Relative target velocity of object in the velocity mode (x [m/s], y [m/s], theta [rad/s])
    Eigen::Vector3d targetVel_ = Eigen::Vector3d::Zero();

    //! Pointer to the front footstep in the footstep queue
    const Footstep * frontFootstep_ = nullptr;

    //! Pose of the front waypoint in the waypoint queue
    sva::PTransformd frontWaypointPose_ = sva::PTransformd::Identity();

    //! Object transformation in one footstep duration
    Eigen::Vector3d objDeltaTrans_ = Eigen::Vector3d::Zero();
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

  /** \brief Const accessor to the velocity mode data. */
  inline const VelModeData & velModeData() const noexcept
  {
    return velModeData_;
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

      A waypoint pose is an object pose that "does not" include an offset pose.
  */
  bool appendWaypoint(const Waypoint & newWaypoint);

  /** \brief Reach hand to object. */
  void reachHandToObj();

  /** \brief Release hand from object. */
  void releaseHandFromObj();

  /** \brief Calculate reference object pose.
      \param t time

      The returned reference object pose "does not" include an offset pose.
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

  /** \brief Access waypoint queue.

      A waypoint pose is an object pose that "does not" include an offset pose.
   */
  inline const std::deque<Waypoint> & waypointQueue() const noexcept
  {
    return waypointQueue_;
  }

  /** \brief Const accessor to the object pose offset. */
  inline const sva::PTransformd & objPoseOffset() const noexcept
  {
    return objPoseOffset_;
  }

  /** \brief Set object pose offset with interpolation.
      \param newObjPoseOffset object pose offset to set
      \param interpDuration interpolation duration [sec]
      \return whether newObjPoseOffset is successfully set
   */
  bool setObjPoseOffset(const sva::PTransformd & newObjPoseOffset, double interpDuration);

  /** \brief Whether the object pose offset is being interpolated. */
  inline bool interpolatingObjPoseOffset() const
  {
    return static_cast<bool>(objPoseOffsetFunc_);
  };

  /** \brief Get manipulation phase. */
  inline const std::shared_ptr<ManipPhase::Base> & manipPhase(const Hand & hand) const
  {
    return manipPhases_.at(hand);
  }

  /** \brief Require sending footstep command following an object. */
  void requireFootstepFollowingObj();

  /** \brief Start velocity mode.
      \return whether it is successfully started
   */
  bool startVelMode();

  /** \brief End velocity mode.
      \return whether it is successfully ended
   */
  bool endVelMode();

  /** \brief Set the relative target velocity
      \param targetVel relative target velocity of object in the velocity mode (x [m/s], y [m/s], theta [rad/s])
   */
  void setRelativeVel(const Eigen::Vector3d & targetVel);

  /** \brief Whether the velocity mode (i.e., moving the object at the relative target velocity) is enabled. */
  inline bool velModeEnabled() const
  {
    return velModeData_.enabled_;
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

  /** \brief Update object and footstep for velocity mode. */
  void updateForVelMode();

  /** \brief Make a footstep.
      \param foot foot
      \param footMidpose middle pose of both feet
      \param startTime time to start the footstep
      \param swingTrajConfig configuration for swing trajectory
  */
  Footstep makeFootstep(const Foot & foot,
                        const sva::PTransformd & footMidpose,
                        double startTime,
                        const mc_rtc::Configuration & swingTrajConfig = {}) const;

  /** \brief ROS callback of object pose topic. */
  void objPoseCallback(const geometry_msgs::PoseStamped::ConstPtr & poseStMsg);

  /** \brief ROS callback of object velocity topic. */
  void objVelCallback(const geometry_msgs::TwistStamped::ConstPtr & twistStMsg);

protected:
  //! Configuration
  Configuration config_;

  //! Velocity mode data
  VelModeData velModeData_;

  //! Pointer to controller
  LocomanipController * ctlPtr_ = nullptr;

  //! Waypoint queue
  std::deque<Waypoint> waypointQueue_;

  //! Last waypoint pose
  sva::PTransformd lastWaypointPose_ = sva::PTransformd::Identity();

  //! Object pose function
  std::shared_ptr<TrajColl::CubicInterpolator<sva::PTransformd, sva::MotionVecd>> objPoseFunc_;

  //! Object pose offset
  sva::PTransformd objPoseOffset_ = sva::PTransformd::Identity();

  //! Object pose offset function
  std::shared_ptr<TrajColl::CubicInterpolator<sva::PTransformd, sva::MotionVecd>> objPoseOffsetFunc_;

  //! Manipulation phases
  std::unordered_map<Hand, std::shared_ptr<ManipPhase::Base>> manipPhases_;

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
