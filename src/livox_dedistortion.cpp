#include <rclcpp/rclcpp.hpp>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <mutex>
#include <thread>

#include "livox_undistortion/data_process.hpp"

bool b_exit = false;
bool b_reset = false;

void SigHandle(int sig) {
  b_exit = true;
  printf("catch sig %d", sig);
}

class LivoxDedistortionNode : public rclcpp::Node {
 public:
  LivoxDedistortionNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("livox_dedistortion_node", options) {

    p_imu = std::make_shared<ImuProcess>();

    topic_pcl = this->declare_parameter<std::string>("topic_pcl", "/livox/points");
    topic_imu = this->declare_parameter<std::string>("topic_imu", "/livox/imu");

    sub_pcl = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_pcl, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          this->pcl_cbk(msg);
        });
    sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
        topic_imu, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
          this->imu_cbk(msg);
        });

    pub_FirstPcl = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/livox_first_point", 100);
    pub_UndistortPcl = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/livox_undistort", 100);
    pub_OriginPcl = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/livox_origin", 100);

    th_prc = std::thread(&LivoxDedistortionNode::ProcessLoop, this);
  }

  ~LivoxDedistortionNode() {
    b_exit = true;
    if (th_prc.joinable()) th_prc.join();
  }

  void pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    const double timestamp = get_time_sec(msg->header.stamp);
    // RCLCPP_DEBUG(this->get_logger(), "get point cloud at time: %.6f", timestamp);
    std::lock_guard<std::mutex> lock(mtx_buffer);
    if (timestamp < last_timestamp_lidar) {
      RCLCPP_ERROR(this->get_logger(), "lidar loop back, clear buffer");
      lidar_buffer.clear();
    }
    last_timestamp_lidar = timestamp;
    lidar_buffer.push_back(msg);
    // std::cout << "received point size: " << float(msg->data.size())/float(msg->point_step) << "\n";
    sig_buffer.notify_one();
  }

  void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr &msg) {
    double timestamp = get_time_sec(msg->header.stamp);
    RCLCPP_DEBUG(this->get_logger(), "get imu at time: %.6f", timestamp);
    std::lock_guard<std::mutex> lock(mtx_buffer);
    if (timestamp < last_timestamp_imu) {
      RCLCPP_ERROR(this->get_logger(), "imu loop back, clear buffer");
      imu_buffer.clear();
      b_reset = true;
    }
    last_timestamp_imu = timestamp;
    imu_buffer.push_back(msg);
    sig_buffer.notify_one();
  }

  bool SyncMeasure(MeasureGroup &measgroup) {
    if (lidar_buffer.empty() || imu_buffer.empty()) {
      // Note: this will happen
      return false;
    }
    if (get_time_sec(imu_buffer.front()->header.stamp) >
        get_time_sec(lidar_buffer.back()->header.stamp)) {
      lidar_buffer.clear();
      printf("clear lidar buffer, only happen at the beginning");
      return false;
    }
    if (get_time_sec(imu_buffer.back()->header.stamp) <
        get_time_sec(lidar_buffer.front()->header.stamp)) {
      return false;
    }

    // Add lidar data, and pop from buffer
    measgroup.lidar = lidar_buffer.front();
    lidar_buffer.pop_front();
    double lidar_time = get_time_sec(measgroup.lidar->header.stamp);

    // Add imu data, and pop from buffer
    measgroup.imu.clear();
    int imu_cnt = 0;
    for (const auto &imu : imu_buffer) {
      double imu_time = get_time_sec(imu->header.stamp);
      if (imu_time <= lidar_time) {
        measgroup.imu.push_back(imu);
        imu_cnt++;
      }
    }
    for (int i = 0; i < imu_cnt; ++i) {
      imu_buffer.pop_front();
    }
    // RCLCPP_DEBUG(this->get_logger(), "add %d imu msg", imu_cnt);
    return true;
  }

  void ProcessLoop() {
    printf("Start ProcessLoop");
    rclcpp::Rate r(1000);
    while (rclcpp::ok() || !b_exit) {
      MeasureGroup meas;
      std::unique_lock<std::mutex> lock(mtx_buffer);
      sig_buffer.wait(lock, [this] {
        return (!lidar_buffer.empty() && !imu_buffer.empty()) || b_exit;
      });

      if (b_exit) {
        printf("b_exit=true, exit");
        break;
      }
      if (b_reset) {
        printf("reset when rosbag play back");
        p_imu->Reset();
        b_reset = false;
        continue;
      }
      bool prc_flg = SyncMeasure(meas);
      if (prc_flg) {
        std::vector<sensor_msgs::msg::PointCloud2> pc_to_publish;
        pc_to_publish = p_imu->Process(meas);
        if (!pc_to_publish.empty()) {
          pub_FirstPcl->publish(pc_to_publish[0]);
          pub_UndistortPcl->publish(pc_to_publish[1]);
          pub_OriginPcl->publish(pc_to_publish[2]);
          RCLCPP_INFO(this->get_logger(), "Publishing undistorded pointcloud");
        }
      }
      r.sleep();
    }
  }

 private:
  std::mutex mtx_buffer;
  std::condition_variable sig_buffer;
  // Buffers for measurements
  double last_timestamp_imu = -1;
  double last_timestamp_lidar = -1;
  std::deque<sensor_msgs::msg::Imu::ConstPtr> imu_buffer;
  std::deque<sensor_msgs::msg::PointCloud2::ConstPtr> lidar_buffer;
  // ROS Topic
  std::string topic_imu;
  std::string topic_pcl;
  // ROS Subscription
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl;
  // ROS Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_FirstPcl;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_UndistortPcl;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_OriginPcl;
  std::shared_ptr<ImuProcess> p_imu;
  // ProcessLoop thread
  std::thread th_prc;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  signal(SIGINT, SigHandle);

  auto node = std::make_shared<LivoxDedistortionNode>();
  rclcpp::spin(node);

  if (rclcpp::ok()) rclcpp::shutdown();

  return 0;
}
