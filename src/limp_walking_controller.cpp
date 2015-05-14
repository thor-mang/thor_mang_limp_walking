#include <thor_mang_limp_walking_controller/limp_walking_controller.h>

namespace walking_controllers {

  bool LimpWalkingController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {

  }

  void LimpWalkingController::starting(const ros::Time& time) {

  }

  void LimpWalkingController::stopping(const ros::Time& time) {

  }

  void LimpWalkingController::update(const ros::Time& time, const ros::Duration& period) {

  }

  std::string LimpWalkingController::getHardwareInterfaceType() const {
    return "hardware_interface::PositionJointInterface";
  }

  bool LimpWalkingController::initRequest(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh,
                           std::set<std::string>& claimed_resources) {
    // check if construction finished cleanly
    if (state_ != CONSTRUCTED){
        ROS_ERROR("[LimpWalking] Cannot initialize this controller because it failed to be constructed");
        return false;
    }

    // get a pointer to the hardware interface
    hardware_interface::PositionJointInterface* hw = robot_hw->get<hardware_interface::PositionJointInterface>();
    if (!hw) {
        ROS_ERROR_STREAM("[LimpWalking] This controller requires a hardware interface of type hardware_interface::PositionJointInterface.");
        return false;
    }
    //We have access to the full hw interface here and thus can grab multiple components of it

    // get pointer to IMU interface
    hardware_interface::ImuSensorInterface * imu_sensor_interface = robot_hw->get<hardware_interface::ImuSensorInterface>();
    if (!imu_sensor_interface){
        ROS_ERROR("[LimpWalking] This controller requires a hardware interface of type hardware_interface::ImuSensorInterface");
        return false;
    }

    try {
        imu_sensor_handle_ = imu_sensor_interface->getHandle("pelvis_imu");
    } catch (hardware_interface::HardwareInterfaceException e) {
        ROS_ERROR_STREAM("[LimpWalking] Couldn't get handle for imu sensor: 'pelvis_imu'. " << e.what());
        return false;
    }

    // init controller
    hw->clearClaims();
    if (!init(hw, root_nh, controller_nh)) {
        ROS_ERROR("[LimpWalking] Failed to initialize the controller");
        return false;
    }
    claimed_resources = hw->getClaims();
    hw->clearClaims();

    state_ = INITIALIZED;
    return true;
  }
}
