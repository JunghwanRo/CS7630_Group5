
#include <vector>
#include <string>
#include <map>
#include <list>


#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <cs7630_msgs/msg/trajectory.hpp>
#include <cs7630_msgs/msg/trajectory_element.hpp>


using std::placeholders::_1;
using namespace std::chrono_literals;
class PathFollower : public rclcpp::Node {
    protected:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<cs7630_msgs::msg::Trajectory>::SharedPtr traj_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose2d_pub_;
        double look_ahead_;
        double Kx_,Ky_,Ktheta_;
        double max_rot_speed_;
        double max_velocity_;
        double max_y_error_;
        double max_error_;
        double delay_;
        double period_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
        geometry_msgs::msg::PoseStamped goal_;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


        std::string frame_id_, base_frame_;

        typedef std::map<double, cs7630_msgs::msg::TrajectoryElement> Trajectory;

        Trajectory traj_;

        void traj_cb(cs7630_msgs::msg::Trajectory::SharedPtr msg) {
            frame_id_ = msg->header.frame_id;
            delay_ = 0.0;
            traj_.clear();
            for (unsigned int i=0;i<msg->ts.size();i++) {
                traj_.insert(Trajectory::value_type(rclcpp::Time(msg->ts[i].header.stamp).seconds(), msg->ts[i]));
            }
            RCLCPP_INFO(this->get_logger(),"Trajectory received");
        }

        void target_cb(geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            goal_ = *msg;
        }



        geometry_msgs::msg::Pose2D computeError(const rclcpp::Time & now, const cs7630_msgs::msg::TrajectoryElement & te) {
            geometry_msgs::msg::TransformStamped transformStamped;
            try {
                std::string errStr;
                if (!tf_buffer->canTransform(base_frame_,frame_id_,now,rclcpp::Duration(1s),&errStr)) {
                    RCLCPP_ERROR(this->get_logger(),"Cannot transform target: %s",errStr.c_str());
                    return geometry_msgs::msg::Pose2D();
                }
                transformStamped = tf_buffer->lookupTransform(base_frame_,frame_id_, now);
            } catch (const tf2::TransformException & ex){
                RCLCPP_ERROR(this->get_logger(),"%s",ex.what());
                return geometry_msgs::msg::Pose2D();
            }
            geometry_msgs::msg::PoseStamped pose,error;
            pose.header.stamp = now;
            pose.header.frame_id = frame_id_;
            pose.pose  = te.pose;
            tf2::doTransform(pose,error,transformStamped);
            geometry_msgs::msg::Pose2D result;
            result.x = error.pose.position.x;
            result.y = error.pose.position.y;
            result.theta = tf2::getYaw(error.pose.orientation);
            RCLCPP_INFO(this->get_logger(),"Current error: %+6.2f %+6.2f %+6.2f",result.x,result.y,result.theta*180./M_PI);
            return result;
        }

    public:
        PathFollower(): rclcpp::Node("path_follower") {
            this->declare_parameter("~/base_frame",std::string("body"));
            this->declare_parameter("~/look_ahead",1.0);
            this->declare_parameter("~/Kx",1.0);
            this->declare_parameter("~/Ky",1.0);
            this->declare_parameter("~/Ktheta",1.0);
            this->declare_parameter("~/max_rot_speed",1.0);
            this->declare_parameter("~/max_velocity",1.0);
            this->declare_parameter("~/max_y_error",1.0);
            this->declare_parameter("~/max_error",0.5);
            this->declare_parameter("~/period",0.050);

            base_frame_ = this->get_parameter("~/base_frame").as_string();
            look_ahead_ = this->get_parameter("~/look_ahead").as_double();
            Kx_ = this->get_parameter("~/Kx").as_double();
            Ky_ = this->get_parameter("~/Ky").as_double();
            Ktheta_ = this->get_parameter("~/Ktheta").as_double();
            max_rot_speed_ = this->get_parameter("~/max_rot_speed").as_double();
            max_velocity_ = this->get_parameter("~/max_velocity").as_double();
            max_y_error_ = this->get_parameter("~/max_y_error").as_double();
            max_error_ = this->get_parameter("~/max_error").as_double();
            period_ = this->get_parameter("~/period").as_double();

            tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
            // Initialize the transform broadcaster
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


            traj_sub_ = this->create_subscription<cs7630_msgs::msg::Trajectory>("~/traj",1,
                    std::bind(&PathFollower::traj_cb,this,std::placeholders::_1));
            twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("~/twistCommand",1);
            pose2d_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("~/error",1);

            timer_ = this->create_wall_timer( std::chrono::duration<double>(period_),
                    std::bind(&PathFollower::timer_cb, this));

            // STEP 3
            target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("~/goal",1,
                std::bind(&PathFollower::target_cb,this,std::placeholders::_1));
            target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/goal",1);

        };

        inline double sat(double x, double max_x) {
            if (x < -max_x) return -max_x;
            if (x > +max_x) return +max_x;
            return x;
        }

        void timer_cb() {
            if (traj_.size() > 0) {
                bool final = false;
                rclcpp::Time now = this->get_clock()->now();
                // First find the reference point which corresponds best to
                // the current time. 
                // TODO: modify this part to react to tracking delays
                // introduced by obstacle avoidance or switch to manual.
                Trajectory::const_iterator it = traj_.lower_bound(now.seconds() + look_ahead_ - delay_);
                if (it == traj_.end()) {
                    // let's keep the final position
                    it --;
                    final = true;
                }
                // Now broadcast the reference pose as a TF for
                // visualization
                geometry_msgs::msg::TransformStamped t;
                // Read message content and assign it to
                // corresponding tf variables
                t.header.stamp = now;
                t.header.frame_id = frame_id_;
                t.child_frame_id = "carrot";

                t.transform.translation.x = it->second.pose.position.x;
                t.transform.translation.y = it->second.pose.position.y;
                t.transform.translation.z = 0.0;
                t.transform.rotation = it->second.pose.orientation;
                // Send the transformation
                tf_broadcaster_->sendTransform(t);



                // Compute the tracking error and 
                geometry_msgs::msg::Pose2D error = computeError(now,it->second);
                pose2d_pub_->publish(error);
                if (hypot(error.x,error.y)>max_error_) {
                    // add the time of while to the delay
                    delay_ += period_;
                    // there is a little bug, then the robot is blocked
                    // the target continue to go forward, but very slowly
                    // We didn't take into account the execution time
                    // of one iteration of a loop
                    RCLCPP_INFO(this->get_logger(),"New delay: %.2f", delay_);

#if 0
                    // STEP3
                    // We are stuck for too long time
                    if(delay_ > 5) {
                        target_pub_->publish(goal_); // Then remake atrajectory
                    }
#endif
                }

                geometry_msgs::msg::Twist twist;
                if (final && (error.x < 0.1)) {
                    // Finished
                    twist.linear.x = 0.0;
                    twist.angular.z = 0.0;
                } else {
                    twist.linear.x = it->second.twist.linear.x + Kx_ * error.x;
                    twist.linear.x = std::min(twist.linear.x,max_velocity_);
                    error.y = sat(error.y,max_y_error_);
                    twist.angular.z = it->second.twist.angular.z + Ktheta_ * error.theta 
                        + Ky_*it->second.twist.linear.x*exp(-error.theta*error.theta)*error.y;
                    twist.angular.z = sat(twist.angular.z,max_rot_speed_);

                    // printf("Twist: %.2f %.2f\n",twist.linear.x,twist.angular.z);
                }
                twist_pub_->publish(twist);
            } else {
                // Publish zero velocity
                geometry_msgs::msg::Twist twist;
                twist_pub_->publish(twist);
            }
        }
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollower>());
    rclcpp::shutdown();
}


