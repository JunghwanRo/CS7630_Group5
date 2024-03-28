

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define FREE 0xFF
#define UNKNOWN 0x80
#define OCCUPIED 0x00
#define SQR(X) ((X)*(X))
using std::placeholders::_1;
using namespace std::chrono_literals;

class ObstacleAvoidance : public rclcpp::Node {
    protected:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr current_vel_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_vel_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_vel_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;


        bool display_;
        double robot_radius_;
        double time_horizon_;
        double max_linear_velocity_;
        double max_angular_velocity_;
        double max_linear_accel_;
        double max_angular_accel_;
        double max_range_;
        double map_resolution_;
        double alpha_resolution_;
        double linear_velocity_resolution_;
        double angular_velocity_resolution_;
        double k_v_;
        double k_w_;
        std::string base_frame_;

        geometry_msgs::msg::Twist current_velocity_;
        pcl::PointCloud<pcl::PointXYZ> lastpc_;
        unsigned int grid_width_, n_d_, n_alpha_;
        cv::Mat_<uint8_t> og_, d_alpha_;
        cv::Mat_<float> dalpha_remap_x, dalpha_remap_y;

    public:
        static void xy_to_dalpha(double x, double y, double & d, double & alpha) {
            if (fabs(y) < 1e-3) {
                // straight line
                d = x;
                alpha = (y<0)?(-M_PI/2):(M_PI/2);
            } else {
                double h = hypot(x,y);
                double r = h*h/2*y;
                alpha = atan(r);
                if (y>=0) {
                    double gamma = atan2(x, r-y);
                    d = gamma * r;
                } else {
                    double gamma = atan2(x,y-r);
                    d = -gamma * r;
                }
            }
        }

        static void dalpha_to_xy(double d, double alpha, double & x, double & y) {
            if (fabs(fabs(alpha)-M_PI/2) < 1e-3) {
                // straight line
                x = d;
                y = 0;
            } else if (fabs(alpha) < 1e-3) {
                x = y = 0;
            } else {
                double r = tan(alpha);
                double gamma = d / r;
                x = r * sin(gamma);
                y = r * (1 - cos(gamma));
            }
        }
        
        static void test_one_conversion(double x, double y) {
            double d,alpha,x1,y1;
            xy_to_dalpha(x,y,d,alpha);
            dalpha_to_xy(d,alpha,x1,y1);
            if (hypot(x-x1,y-y1)>1e-2) {
                printf("Conversion: %.3f %.3f -> %.3f %.3f -> %.3f %.3f XXX\n",x,y,d,alpha,x1,y1);
            } else {
                printf("Conversion: %.3f %.3f -> %.3f %.3f -> %.3f %.3f\n",x,y,d,alpha,x1,y1);
            }

        }

        static void test_conversions() {
            test_one_conversion(0,0);
            test_one_conversion(1,0);
            test_one_conversion(1,1);
            test_one_conversion(0,1);
            test_one_conversion(-1,1);
            test_one_conversion(-1,0);
            test_one_conversion(-1,-1);
            test_one_conversion(0,-1);
            test_one_conversion(1,-1);
        }

    protected: // ROS Callbacks
        void command_velocity_cb(geometry_msgs::msg::Twist::SharedPtr msg) {
            geometry_msgs::msg::Twist filtered = findClosestAcceptableVelocity(*msg);
            // ROS_INFO("Speed limiter: desired %.2f controlled %.2f",msg->linear.x,filtered.linear.x);
            safe_vel_pub_->publish(filtered);
        }

        void current_velocity_cb(geometry_msgs::msg::Twist::SharedPtr msg) {
            current_velocity_ = *msg;
        }

        void scan_callback(sensor_msgs::msg::LaserScan::SharedPtr msg) {
            lastpc_.clear();
            for (size_t i=0;i<msg->ranges.size();i++) {
                double alpha = msg->angle_min + i*msg->angle_increment;
                pcl::PointXYZ P;
                P.x = msg->ranges[i] * cos(alpha);
                P.y = msg->ranges[i] * sin(alpha);
                P.z = 0;
                lastpc_.push_back(P);
            }

            og_ = FREE; // reset the map
            unsigned int n = lastpc_.size();
            for (unsigned int i=0;i<n;i++) {
                float x = lastpc_[i].x;
                float y = lastpc_[i].y;
                float d = hypot(x,y);
                if (d < 1e-2) {
                    // Bogus point, ignore
                    continue;
                }
                cv::Point P(x / map_resolution_ + grid_width_/2., y/map_resolution_ + grid_width_/2.);
                cv::circle(og_,P, ceil(robot_radius_/map_resolution_ + 1), cv::Scalar(OCCUPIED), -1); // filled circle
            }
            // Marked the robot own space as FREE, whatever the point cloud
            // is saying. This means that rotation on the spot are always
            // possible
            cv::Point myself(grid_width_/2,grid_width_/2);
            cv::circle(og_,myself, ceil(robot_radius_/map_resolution_ + 1), cv::Scalar(FREE), -1); // filled circle
            if (display_) {
                cv::imshow( "OccGrid", og_ );
            }
            //
            // First convert the obstacle into the ego-kinematic space
            // (d,alpha), where d is the distance on the arc of circle, and
            // alpha = atan2(v,w)
            cv::remap(og_,d_alpha_,dalpha_remap_x,dalpha_remap_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, UNKNOWN );
            // Now we need to go through all the d_alpha combination. If we hit
            // an obstacle at a given value of d, any further distance is also
            // going to hit it.
            for (unsigned int i=0;i<n_alpha_;i++) {
                for (int j=n_d_/2;j<(signed)n_d_;j++) {
                    if (d_alpha_(j,i) != FREE) {
                        for (;j<(signed)n_d_;j++) d_alpha_(j,i) = OCCUPIED;
                    }
                }
                for (int j=n_d_/2;j>=0;j--) {
                    if (d_alpha_(j,i) != FREE) {
                        for (;j>=0;j--) d_alpha_(j,i) = OCCUPIED;
                    }
                }
            }
            if (display_) {
                cv::imshow( "DAlpha", d_alpha_ );
            }

        }

        geometry_msgs::msg::Twist findClosestAcceptableVelocity(const geometry_msgs::msg::Twist & desired) {
            geometry_msgs::msg::Twist res = desired;
            // First the static limit: Vs
            double min_v = -max_linear_velocity_;
            double max_v = max_linear_velocity_;
            double min_w = -max_angular_velocity_;
            double max_w = max_angular_velocity_;
            // TODO: First update min_v/max_v and min_w/max_w to compute the intersection of Vs and Vd.
            min_v = std::max(min_v,min_v);
            max_v = std::min(max_v,max_v);
            min_w = std::max(min_w,min_w);
            max_w = std::min(max_w,max_w);

            // From that, we know which velocities we need to consider and we
            // creat a small matrix to help visualising Va (Vr)
            unsigned int n_v = ceil((max_v-min_v)/linear_velocity_resolution_+1); 
            unsigned int n_w = ceil((max_w-min_w)/angular_velocity_resolution_+1); 
            cv::Mat_<uint8_t> Va(n_v,n_w,FREE); // Vs inter Vd
            cv::Mat_<uint8_t> scores(n_v,n_w,(uint8_t)OCCUPIED); // Vs inter Vd

            // Now build Va (inside Vs inter Vd) by iterating over the local
            // map, and find the most appropriate speed
            // TODO: Fill Vas with the acceptable velocities (given the time
            // horizon)
            double best_score = 0;
            double best_v = 0, best_w = 0;
            for (unsigned int j=0;j<n_v;j++) {
                double v = min_v + j*linear_velocity_resolution_;
                for (unsigned int i=0;i<n_w;i++) {
                    double w = min_w + i*angular_velocity_resolution_;
                    Va(j,i) = UNKNOWN;
                    scores(j,i) = v+w;// Stupid value to avoid the "unused variable" warning
                }
            }
            RCLCPP_INFO(this->get_logger(),"Best score %f for (%f,%f)",
                    best_score,best_v,best_w);
            if (display_) {
                cv::resize(scores,scores,cv::Size(200,200));
                cv::imshow("Va",Va);
                cv::imshow("Scores",scores);
            }

            // TODO: Search the scores matrix for the best velocities and update
            // best_v, best_w
            res.linear.x = best_v;
            res.angular.z = best_w;

            return res;
        }

    public:
        ObstacleAvoidance() : rclcpp::Node("obstacle_avoidance") {
            alpha_resolution_ = 3. * M_PI/180.;

            this->declare_parameter("~/base_frame",std::string("body"));
            this->declare_parameter("~/display",true);
            this->declare_parameter("~/max_range",5.0);
            this->declare_parameter("~/max_linear_velocity",0.5);
            this->declare_parameter("~/max_angular_velocity",1.0);
            this->declare_parameter("~/max_linear_accel",0.5);
            this->declare_parameter("~/max_angular_accel",0.5);
            this->declare_parameter("~/map_resolution",0.05);
            this->declare_parameter("~/linear_velocity_resolution",0.05);
            this->declare_parameter("~/angular_velocity_resolution",0.05);
            this->declare_parameter("~/robot_radius",0.1);
            this->declare_parameter("~/time_horizon",0.1);
            this->declare_parameter("~/k_v",1.0);
            this->declare_parameter("~/k_w",1.0);

            base_frame_ = this->get_parameter("~/base_frame").as_string();
            display_ = this->get_parameter("~/display").as_bool();
            max_range_ = this->get_parameter("~/max_range").as_double();
            max_linear_velocity_ = this->get_parameter("~/max_linear_velocity").as_double();
            max_angular_velocity_ = this->get_parameter("~/max_angular_velocity").as_double();
            max_linear_accel_ = this->get_parameter("~/max_linear_accel").as_double();
            max_angular_accel_ = this->get_parameter("~/max_angular_accel").as_double();
            map_resolution_ = this->get_parameter("~/map_resolution").as_double();
            linear_velocity_resolution_ = this->get_parameter("~/linear_velocity_resolution").as_double();
            angular_velocity_resolution_ = this->get_parameter("~/angular_velocity_resolution").as_double();
            robot_radius_ = this->get_parameter("~/robot_radius").as_double();
            time_horizon_ = this->get_parameter("~/time_horizon").as_double();
            k_v_ = this->get_parameter("~/k_v").as_double();
            k_w_ = this->get_parameter("~/k_w").as_double();

            if (display_) {
                cv::namedWindow( "OccGrid", cv::WINDOW_AUTOSIZE );
                cv::namedWindow( "DAlpha", cv::WINDOW_AUTOSIZE );
                cv::namedWindow( "Va", cv::WINDOW_AUTOSIZE );
                cv::namedWindow( "Scores", cv::WINDOW_AUTOSIZE );
                timer_ = this->create_wall_timer( 50ms,
                        std::bind(&ObstacleAvoidance::timer_cb, this));
            }

	    rclcpp::QoS be = rclcpp::QoS(1).best_effort();
            tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
            scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("~/scans",be,
                    std::bind(&ObstacleAvoidance::scan_callback,this,std::placeholders::_1));
            current_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("~/current_velocity",1,
                    std::bind(&ObstacleAvoidance::current_velocity_cb,this,std::placeholders::_1));
            command_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("~/command_velocity",1,
                    std::bind(&ObstacleAvoidance::command_velocity_cb,this,std::placeholders::_1));
            safe_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("~/output_velocity",1);

            grid_width_ = 2*max_range_ / map_resolution_ + 1;
            og_ = cv::Mat_<uint8_t>(grid_width_,grid_width_,FREE);
            n_d_ = ceil(2*max_range_/map_resolution_); 
            n_alpha_ = ceil(M_PI / alpha_resolution_); 
            d_alpha_ = cv::Mat_<uint8_t>(n_d_,n_alpha_,FREE); // Vs inter Vd


            // Now prepare the remapping to convert obstacles between circular
            // coordinates and cartesian coorsinates
            unsigned int w = 2*max_range_ / map_resolution_ + 1;
            unsigned int nd = ceil(2*max_range_/map_resolution_); 
            unsigned int nalpha = ceil(M_PI / alpha_resolution_); 
            dalpha_remap_x = cv::Mat_<float>(nd,nalpha);
            dalpha_remap_y = cv::Mat_<float>(nd,nalpha);
            for (unsigned int j=0;j<nd;j++) {
                double d = (j - nd/2.) * map_resolution_;
                for (unsigned int i=0;i<nalpha;i++) {
                    double alpha = i * alpha_resolution_;
                    double x=0, y=0;
                    dalpha_to_xy(d,alpha,x,y);
                    dalpha_remap_x(j,i) = x / map_resolution_ + w/2.;
                    dalpha_remap_y(j,i) = y / map_resolution_ + w/2.;
                }
            }
        }

        void timer_cb() {
            cv::waitKey(5);
        }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleAvoidance>());
    rclcpp::shutdown();
}

