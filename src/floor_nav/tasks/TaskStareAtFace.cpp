#include <math.h>
#include "TaskStareAtFace.h"

// Include custom_interfaces
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/face_array.hpp"

using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_GOTO

TaskIndicator TaskStareAtFace::initialise() 
{
    face_sub = node->create_subscription<custom_interfaces::msg::FaceArray>(
        "detected_faces", 1, std::bind(&TaskStareAtFace::faceCallback, this, std::placeholders::_1));
    const geometry_msgs::msg::Pose2D & tpose = env->getPose2D();
    initial_heading = tpose.theta;
    return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskStareAtFace::iterate() 
{
    uint32_t center_x = face_roi.x_offset + (face_roi.width / 2);
    uint32_t image_midpoint = cfg->image_width / 2; 
    int center_offset = center_x - image_midpoint;

    if (abs(center_offset) < cfg->center_pixel_threshold) {
        return TaskStatus::TASK_COMPLETED;
    }

    double rot = cfg->k_theta*center_offset;
    rot = -rot;
    if (rot > cfg->max_angular_velocity) rot = cfg->max_angular_velocity;
    if (rot <-cfg->max_angular_velocity) rot =-cfg->max_angular_velocity;
    env->publishVelocity(0.0, rot);
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskStareAtFace::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

void TaskStareAtFace::faceCallback(const custom_interfaces::msg::FaceArray::SharedPtr msg) {
    if (msg->faces.empty()) {
        return;
    }

    face_roi = msg->faces[0];
    return;
}

DYNAMIC_TASK(TaskFactoryStareAtFace);