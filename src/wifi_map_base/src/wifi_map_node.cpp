#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <wpa_cli_msgs/msg/scan.hpp>


#define FREE 0xFF
#define UNKNOWN 0x80
#define OCCUPIED 0x00

class WifiMapNode : public rclcpp::Node {
    protected:
        struct WifiStrength {
            double x;
            double y;
            double value;
            WifiStrength() {}
            WifiStrength(double x, double y, double val) : x(x), y(y), value(val) {}
        };
        typedef std::vector<WifiStrength> WifiVector;
        typedef std::map<std::string,WifiVector> WifiMap;

        cv::Mat_<float> weights_;
        WifiMap map;

        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr og_sub_;
        rclcpp::Subscription<wpa_cli_msgs::msg::Scan>::SharedPtr scan_sub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr og_pub_;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;

        cv::Mat_<uint8_t> og_;
        cv::Point2i og_center_;
        nav_msgs::msg::MapMetaData info_;
        std_msgs::msg::Header og_header_;
        std::string frame_id_;
        std::string base_link_;
        std::string bssid_;
        bool ignore_header_;
        double measurement_radius_;

    protected:

        void wifi_callback(wpa_cli_msgs::msg::Scan::SharedPtr msg) {
            geometry_msgs::msg::TransformStamped transformStamped;
            if (frame_id_.empty()) {
                return;
            }
            rclcpp::Time now = this->get_clock()->now();
            std::string base = base_link_;
            if (!ignore_header_) {
                now = msg->header.stamp;
                base = msg->header.frame_id;
            }
            std::string errStr;
            // This converts target in the grid frame.
            if (!tf_buffer->canTransform(frame_id_, base, now,
                        rclcpp::Duration(std::chrono::duration<double>(1.0)),&errStr)) {
                RCLCPP_ERROR(this->get_logger(),"Cannot transform target: %s",errStr.c_str());
                return;
            }
            transformStamped = tf_buffer->lookupTransform(frame_id_, base, now);
            double x = transformStamped.transform.translation.x;
            double y = transformStamped.transform.translation.y;
            for (size_t i=0;i<msg->networks.size();i++) {
                map[msg->networks[i].bssid].push_back(WifiStrength(x,y,msg->networks[i].level));
#if 0
                if (msg->networks[i].bssid == bssid_) {
                    RCLCPP_INFO(this->get_logger(),"Received level %d for bssid %s",
                            msg->networks[i].level,bssid_.c_str());
                }
#endif
            }

            if (map.empty()) {
                return;
            }

            if (weights_.rows == 0) {
                RCLCPP_INFO(this->get_logger(),"Initialising voting weights");
                int winsize = round(2*measurement_radius_/info_.resolution);
                int sigma = 5;
                winsize += 1 - (winsize & 1); // Make it odd
                // Initialise the signal voting weights as a (winsize x winsize) matrix
                weights_ = cv::Mat_<float>(winsize,winsize,0.0f);
                for (int i=0;i<weights_.rows;i++) {
                    for (int j=0;j<weights_.cols;j++) {
                        // compute (x,y) as the vector from the window center
                        float y = i - winsize/2;
                        float x = j - winsize/2;
                        // TODO 1: Affect a weight to i,j as a function of x,y. Weights should be in [0,1]
                        /* Prepare a weight matrix to precompute w (x, y, 0, 0) for x , y ∈[−r , r ]2 . 
                        This needs to be done only once, and is indicated by “TODO 1” in the base file.*/

                        // check if it is in the measurement radius!
                        if (hypot(x,y) * info_.resolution < measurement_radius_) {

                        weights_(i,j) = exp(-0.5 * (pow(x, 2) + pow(y, 2)) / pow(sigma, 2));
                        }
                    }
                }
                cv::imwrite("weights.png",weights_*255);
            }


            WifiMap::const_iterator it;
            if (bssid_ == "first") {
                it = map.begin();
                bssid_ = it->first;
                RCLCPP_INFO(this->get_logger(),"WifiMap tracking BSSID '%s'",bssid_.c_str());
            } else {
                it = map.find(bssid_);
                if (it==map.end()) {
                    RCLCPP_WARN(this->get_logger(),"Selected bssid '%s' is not present so far",bssid_.c_str());
                    return;
                }
            }
            // TODO Initialise mat.
            // In the end, every pixel should contain sum(w_i s_i) / sum(w_i)

            cv::Mat_<float> numerator(info_.height, info_.width, 0.0);
            cv::Mat_<float> denominator(info_.height, info_.width, 0.0);
            cv::Mat_<uint8_t> value_v(info_.height, info_.width, uint8_t(0));

            RCLCPP_INFO(this->get_logger(),"Bssid %s has received %d measurements",it->first.c_str(),int(it->second.size()));
            for (size_t i=0;i<it->second.size();i++) {
                const WifiStrength & ws = it->second[i];
                float intensity = std::max<float>(0.,std::min<float>(100., 100 - float(ws.value)));
                // TODO 2: handle measurement i at position (ws.x,ws.y) with value intensity
                // You should use addWeightedWindow to add a smoothly weighted signal around (ws.x,ws.y).
                // Example usage:
                /* we need to compute individually the numerator and denominator. 
                Create two matrices (numerator, denominator) of the same size as the occupancy grid, 
                and for each measurement (xi , yi , si ) , add w×si and w to the numerator and denominator matrices. 
                This should be done as matrix operations for efficiency. 
                The addWeightedWindow function provides a way to do this efficiently while checking for boundary conditions. */
                cv::Point2i where(ws.x/info_.resolution + og_center_.x, ws.y/info_.resolution + og_center_.y); 

                cv::Mat_<float> weighted_intensity = weights_.mul(intensity);

                addWeightedWindow(numerator, weighted_intensity, where);
                addWeightedWindow(denominator, weights_, where);

            }

            // og_mat needs to be a clone of og_ for info_ to make sense
            cv::Mat_<uint8_t> og_mat = og_.clone();
            int og_min=255, og_max=0;
            for (unsigned int j=0;j<info_.height;j++) {
                for (unsigned int i=0;i<info_.width;i++) {
                    // TODO 3: Fill og_mat(j,i) with sum(w_i s_i)/sum(w_i) if not NaN,
                    // You can also account for the known occupancy grid og_(j,i),
                    // og_(j,i) can be OCCUPIED, FREE, UNKNOWN
                    // Example affectation for an unknown value
                    /* Once we have computed the numerator and denominator, one just need to compute the quotient to update the occupancy grid. Note that if the quotient at a given pixel is zero (or very small), the grid pixel should be labeled “unknown” (i.e. -1). You could also decide to just mark occupied any cell labeled occupied in the original occupancy grid: wifi signal strength only makes sense in the environment free space.*/
                    float quotient = 0.0;
                    if (denominator(j, i) != 0) {
                        quotient = numerator(j, i) / denominator(j, i);
                        // std::cout << "quotient: " << quotient << "\n";
                    }
                    if (quotient == 0 || isnan(quotient)) {
                        og_mat(j, i) = -1;
                    } else {
                        int value = static_cast<int>(quotient);
                        value = std::max(og_max, std::min(og_min, value)); 
                        if (og_(j, i) == OCCUPIED) {
                            og_mat(j, i) = og_min;
                        } else if (og_(j, i) == FREE) {
                            og_mat(j, i) = static_cast<uint8_t>(value);
                        } else {
                            og_mat(j, i) = (quotient < 1e-7) ? -1 : static_cast<uint8_t>(value);
                        }
                    }
                // print og_mat(j,i) to check the value
                // std::cout << "og_mat(j,i): " << static_cast<int>(og_mat(j,i)) << std::endl;
                }
            }

            nav_msgs::msg::OccupancyGrid og_out;
            mat_to_og(og_mat,og_out);
            // print the whole og_mat
            cv::imwrite("og_mat.png",og_mat);
            og_pub_->publish(og_out);
        }


        void og_callback(nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            info_ = msg->info;
            og_header_ = msg->header;
            frame_id_ = msg->header.frame_id;
            og_center_ = cv::Point2i(-info_.origin.position.x/info_.resolution,
                    -info_.origin.position.y/info_.resolution);


            og_to_mat(*msg, og_);

        }

        void addWeightedWindow(cv::Mat_<float> & dest,
                const cv::Mat_<float> & weights,
                const cv::Point2i & center) {
            cv::Point2i w_center(weights.cols/2,weights.rows/2);
            cv::Rect Rd(0,0,dest.cols,dest.rows);
            cv::Rect Rw(0,0,weights.cols,weights.rows);
            // Removing w_center to define the upper left corner of the rectangle, and adding center to move the
            // rectangle in the destination window
            Rw = Rw - w_center + center;
            // Computing the intersection between Rd and Rw in the destination matrix
            Rd = Rd & Rw;
            // Translating this intersection in the weight matrix
            Rw = Rd - center + w_center;
            // Now we can add two windows of exactly the same size, without risks of accessing pixels outside the matrix
            dest(Rd) += weights(Rw);
        }

        void og_to_mat(const nav_msgs::msg::OccupancyGrid & og, cv::Mat_<uint8_t> & mat) {
            // Create an image to store the value of the grid.
            mat = cv::Mat_<uint8_t>(og.info.height, og.info.width,UNKNOWN);

            // Convert the representation into something easy to display.
            for (unsigned int j=0;j<og.info.height;j++) {
                for (unsigned int i=0;i<og.info.width;i++) {
                    const int8_t & v = og.data[j*og.info.width + i];
                    switch (v) {
                        // Slam-toolbox convention: 0=FREE, 100=OCCUPIED, anything else unknown
                        // Cartographer: P(Occ) in [0:100]
                        case 0:
                            mat(j,i) = FREE;
                            break;
                        case 100:
                            mat(j,i) = OCCUPIED;
                            break;
                        case -1:
                        default:
                            // implicit
                            // og_(j,i) = UNKNOWN;
                            break;
                    }
                }
            }
        }

        void mat_to_og(cv::Mat_<uint8_t> & mat, nav_msgs::msg::OccupancyGrid & og) {
            // Create an image to store the value of the grid.
            og.info = info_;
            og.header = og_header_;
            assert(int(info_.width) == mat.cols);
            assert(int(info_.height) == mat.rows);
            og.data.resize(mat.cols*mat.rows);

            // Convert the representation into something easy to display.
            for (unsigned int j=0;j<og.info.height;j++) {
                for (unsigned int i=0;i<og.info.width;i++) {
                    og.data[j*og.info.width + i] = mat(j,i);
                }
            }
        }

    public:
        WifiMapNode() : rclcpp::Node("wifi_map_node") {
            this->declare_parameter("~/base_link",std::string("base_link"));
            this->declare_parameter("~/measurement_radius",2.0);
            this->declare_parameter("~/ignore_header",true);
            this->declare_parameter("~/bssid",std::string());
            base_link_ = this->get_parameter("~/base_link").as_string();
            measurement_radius_ = this->get_parameter("~/measurement_radius").as_double();
            ignore_header_ = this->get_parameter("~/ignore_header").as_bool();
            bssid_ = this->get_parameter("~/bssid").as_string();

            tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
            og_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("~/occ_grid",1,
                    std::bind(&WifiMapNode::og_callback,this,std::placeholders::_1));
            scan_sub_ = this->create_subscription<wpa_cli_msgs::msg::Scan>("~/scan",1,
                    std::bind(&WifiMapNode::wifi_callback,this,std::placeholders::_1));
            og_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("~/wifi_map",1);
            RCLCPP_INFO(this->get_logger(),"Wifi mapping node is ready. Tracking '%s'",bssid_.c_str());
        }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WifiMapNode>());
    rclcpp::shutdown();
}


