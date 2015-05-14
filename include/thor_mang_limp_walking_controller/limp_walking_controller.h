#ifndef LIMP_WALKING_CONTROLLER_H
#define LIMP_WALKING_CONTROLLER_H

// ROS
#include <ros/node_handle.h>

// ros_controls
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

namespace walking_controllers {
	class LimpWalkingController : public controller_interface::ControllerBase {
	}
}

#endif
