#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>
#include <include/vl53l0x.h>
#include <boost/asio.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "vl53l0x_publisher");
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");

  std::string port;
  int baud_rate;
  std::string frame_id;
  int firmware_number;

  priv_nh.param("port", port, std::string("/dev/ttyUSB0"));
  priv_nh.param("baud_rate", baud_rate, 57600);
  priv_nh.param("frame_id", frame_id, std::string("ir_range"));
  priv_nh.param("firmware_version", firmware_number, 1);

  boost::asio::io_service io;

  try {
    range_sensor::Vl53l0x range(port, baud_rate, firmware_number, io);
    ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1000);

    // Assuming a FOV of 25 degrees
    const double FOV = 0.261799;  // Radians (25 degrees)

    while (ros::ok()) {
      sensor_msgs::LaserScan::Ptr scan_msg(new sensor_msgs::LaserScan);
      // Set header information for scan_msg
      scan_msg->header.frame_id = frame_id;
      scan_msg->header.stamp = ros::Time::now();

      scan_msg->angle_min = -FOV / 2.0;
      scan_msg->angle_max = FOV / 2.0;
      scan_msg->angle_increment = FOV / 10.0;  // Assuming 10 points within FOV
      // ... adjust range_min and range_max based on sensor specifications

      // Get range measurement
      sensor_msgs::Range::Ptr range_scan(new sensor_msgs::Range);
      range.poll(range_scan);

      // Assuming a single range measurement, extract it and calculate angle
      float range = range_scan->range;
      float angle = M_PI / 2.0f - FOV / 2.0f;  // Assuming centered scan

      // Populate ranges array with the received range value at the calculated angle
      scan_msg->ranges.resize(static_cast<int>(FOV / scan_msg->angle_increment) + 1);
      scan_msg->ranges[int(scan_msg->ranges.size() / 2)] = range;

      scan_pub.publish(scan_msg);
    }

    range.close();
    return 0;
  } catch (boost::system::system_error ex) {
    ROS_ERROR("Error instantiating vl53l0x object. Are you sure you have the correct port and baud rate? Error was %s", ex.what());
    return -1;
  }
}
