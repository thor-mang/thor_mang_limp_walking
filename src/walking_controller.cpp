// Pluginlib
#include <pluginlib/class_list_macros.h>

// Walking Controllers
#include <thor_mang_limp_walking_controller/limp_walking_controller.h>

PLUGINLIB_EXPORT_CLASS(walking_controllers::LimpWalkingController, controller_interface::ControllerBase)
