#include <math.h>
#include "TaskWaitForFace.h"

// Include custom_interfaces
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/face_array.hpp"

using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// Added: Face subscriber
TaskIndicator TaskWaitForFace::initialise()
{
    face_sub = node->create_subscription<custom_interfaces::msg::FaceArray>("detected_faces",1,
            std::bind(&TaskWaitForFace::faceCallback,this,std::placeholders::_1));
    triggered = false;
	return TaskStatus::TASK_INITIALISED;
}


TaskIndicator TaskWaitForFace::iterate()
{
    if (triggered) {
		return TaskStatus::TASK_COMPLETED;
    }
	return TaskStatus::TASK_RUNNING;
}

DYNAMIC_TASK(TaskFactoryWaitForFace)