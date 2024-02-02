#include <math.h>
#include "TaskGoToPose.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_GOTOPOSE
#ifdef DEBUG_GOTOPOSE
#warning Debugging task GOTOPOSE
#endif


TaskIndicator TaskGoToPose::initialise() 
{
    RCLCPP_INFO(getNode()->get_logger(),"Going to %.2f %.2f %.2f",cfg->goal_x,cfg->goal_y, cfg->goal_theta);
    if (cfg->relative) {
        const geometry_msgs::msg::Pose2D & tpose = env->getPose2D();
        x_init = tpose.x;
        y_init = tpose.y;
        theta_init = tpose.theta;
    } else {
        x_init = 0.0;
        y_init = 0.0;
        theta_init = 0.0;
    }
    return TaskStatus::TASK_INITIALISED;
}


TaskIndicator TaskGoToPose::iterate()
{
    const geometry_msgs::msg::Pose2D & tpose = env->getPose2D();
    double rho = hypot(y_init + cfg->goal_y-tpose.y,x_init + cfg->goal_x-tpose.x);
    double alpha = remainder(atan2((y_init + cfg->goal_y-tpose.y),x_init + cfg->goal_x-tpose.x)-tpose.theta,2*M_PI);
    double beta = remainder(-tpose.theta - alpha + cfg->goal_theta,2*M_PI);
    double theta_error = remainder(theta_init + cfg->goal_theta - tpose.theta, 2 * M_PI);
    double vel = 0.0;
    double rot = 0.0;
    
    if (rho < cfg->dist_threshold && fabs(theta_error) < cfg->angle_threshold){
        // If Pose is reached, return TASK_COMPLETED
        return TaskStatus::TASK_COMPLETED;
    }

    if (cfg->holonomic) {
        vel = cfg->k_v * rho;
        // 
        rot = cfg->max_angular_velocity * (theta_error);
        if (vel > cfg->max_velocity) vel = cfg->max_velocity;
        if (vel <-cfg->max_velocity) vel = -cfg->max_velocity;
        // Use alpha and vel to compute desired linear_x and linear_y
        double linear_x = vel * cos(alpha);
        double linear_y = vel * sin(alpha);

        // publishVelocity(double linear_x, double linear_y, double angular)
        env->publishVelocity(linear_x, linear_y, rot);
        return TaskStatus::TASK_RUNNING;
    }

    if (cfg->smart) {
        // Smart control logic
        // If the robot is facing backwards, turn around
        if (fabs(alpha) > M_PI/2) {
            vel = 0.0;
            rot = ((alpha>0)?+1:-1)*cfg->max_angular_velocity;
        } else {
            // If the robot is facing forward, move forward
            vel = cfg->smart_k_rho * rho;
            rot = cfg->smart_k_alpha * alpha + cfg->smart_k_beta * beta;
        }
    } else {
        // Dumb control logic
        // Check if position is reached
        if (rho < cfg->dist_threshold) {
            // Once the position is reached, rotate to the desired orientation
            // Rotation control
            vel = 0.0;
            rot = ((theta_error > 0) ? 1 : -1) * cfg->max_angular_velocity;
            env->publishVelocity(0.0, rot);
        } else {
            double alpha = remainder(atan2((y_init + cfg->goal_y-tpose.y),x_init + cfg->goal_x-tpose.x)-tpose.theta,2*M_PI);
        #ifdef DEBUG_GOTOPOSE
            printf("c %.1f %.1f %.1f g %.1f %.1f rho %.3f alpha %.1f\n",
                    tpose.x, tpose.y, tpose.theta*180./M_PI,
                    cfg->goal_x,cfg->goal_y,rho,alpha*180./M_PI);
        #endif
            if (fabs(alpha) > M_PI/9) {
                vel = 0.0;
                rot = ((alpha>0)?+1:-1)*cfg->max_angular_velocity;
        #ifdef DEBUG_GOTOPOSE
                printf("Cmd v %.2f rho %.2f\n",0.,rot);
        #endif
            } else {
                vel = cfg->k_v * rho;
                rot = std::max(std::min(cfg->k_alpha*alpha,cfg->max_angular_velocity),-cfg->max_angular_velocity);
        #ifdef DEBUG_GOTOPOSE
                printf("Cmd v %.2f rho %.2f\n",vel,rot);
        #endif
            }
        }
    }
    if (vel > cfg->max_velocity) vel = cfg->max_velocity;
    if (vel <-cfg->max_velocity) vel = -cfg->max_velocity;
    if (rot > cfg->max_angular_velocity) rot = cfg->max_angular_velocity;
    if (rot <-cfg->max_angular_velocity) rot = -cfg->max_angular_velocity;
    env->publishVelocity(vel, rot);

	return TaskStatus::TASK_RUNNING;
}


TaskIndicator TaskGoToPose::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryGoToPose);
