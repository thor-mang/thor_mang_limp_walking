#ifndef LIMP_WALKING_CONTROLLER_H
#define LIMP_WALKING_CONTROLLER_H

// ROS
#include <ros/node_handle.h>

#include <Eigen/Eigen>

// ros_controls
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/imu_sensor_interface.h>

namespace walking_controllers {
	class LimpWalkingController : public controller_interface::ControllerBase {
  public:
    LimpWalkingController();

    // Non Real-Time Safe Functions
    bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

    // These two functions have to be implemented as we derive from ControllerBase
    std::string getHardwareInterfaceType() const;
    bool initRequest(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh,
                             std::set<std::string>& claimed_resources);

    // Real-Time Safe Functions
    void starting(const ros::Time& time);
    void stopping(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:
      hardware_interface::ImuSensorHandle imu_sensor_handle_;
  };
}

#endif
