#ifndef VL53L0X_SCAN_CONVERTER_H
#define VL53L0X_SCAN_CONVERTER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h> // Message being converted to
#include <sensor_msgs/Range.h>
#include <boost/asio.hpp>

namespace range_sensor { // Assuming your driver namespace (replace if different)

// Class definition for converting range data to a laser scan
class Vl53l0xScanConverter {
public:
  // Constructor declaration (arguments and return type)
  Vl53l0xScanConverter(ros::NodeHandle &nh, std::string port, int baud_rate, std::string frame_id);

  ~Vl53l0xScanConverter() = default;  
private:
  // Function declarations for private member functions (no implementation details)
  void rangeCallback(const sensor_msgs::Range::ConstPtr& range_msg);

  // Member variables (declarations without initial values)
  ros::NodeHandle nh_;
  ros::Publisher scan_pub_;
  ros::Subscriber range_sub_;
  // boost::asio::io_service io_;
  std::unique_ptr<range_sensor::Vl53l0x> range_finder_;
  std::shared_ptr<boost::asio::io_service> io_;

};

} // namespace range_sensor

#endif // VL53L0X_SCAN_CONVERTER_H
