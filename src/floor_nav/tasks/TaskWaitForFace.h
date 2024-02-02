#ifndef TASK_WAIT_FOR_FACE_H
#define TASK_WAIT_FOR_FACE_H

#include "task_manager_lib/TaskInstance.h"
#include "floor_nav/SimTasksEnv.h"

// Include custom_interfaces
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/face_array.hpp"

using namespace task_manager_lib;

namespace floor_nav {
    struct TaskWaitForFaceConfig : public TaskConfig {
        TaskWaitForFaceConfig() {

        }
    };

    class TaskWaitForFace : public TaskInstance<TaskWaitForFaceConfig,SimTasksEnv>
    {
        // Added: Face subscriber
        protected:
            rclcpp::Subscription<custom_interfaces::msg::FaceArray>::SharedPtr face_sub;
            bool triggered;

            void faceCallback(const custom_interfaces::msg::FaceArray msg) {
                RCLCPP_INFO(node->get_logger(),"Received face");
                triggered = true;
                RCLCPP_INFO(node->get_logger(),"Triggered");
            }

        public:
            TaskWaitForFace(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskWaitForFace() {};

            virtual TaskIndicator initialise();

            virtual TaskIndicator iterate();

    };
    class TaskFactoryWaitForFace : public TaskDefinition<TaskWaitForFaceConfig, SimTasksEnv, TaskWaitForFace>
    {
        public:
            TaskFactoryWaitForFace(TaskEnvironmentPtr env) : 
                Parent("WaitForFace","Do nothing until we reach a given destination",true,env) {}
            virtual ~TaskFactoryWaitForFace() {};
    };
}

#endif // TASK_WAIT_FOR_FACE_H