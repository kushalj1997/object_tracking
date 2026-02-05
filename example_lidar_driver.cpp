#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <lidar_driver_node/CaptureScan.srv> // Include the generated service header
#include <iostream>
#include <vector>

class LidarDriver
{
public:
    LidarDriver()
    {
        // Initialize the ROS node handle
        ros::NodeHandle nh;

        // Publisher to publish LaserScan data
        scan_pub_ = nh.advertise<sensor_msgs::LaserScan>("lidar_scan", 10);

        // Initialize the timer that triggers every 0.1 seconds (10 Hz)
        timer_ = nh.createTimer(ros::Duration(0.1), &LidarDriver::timerCallback, this);

        // Advertise the service
        capture_service_ = nh.advertiseService("capture_lidar_scan", &LidarDriver::captureScanCallback, this);

        // Initialize the simulated LIDAR scan data (just an example)
        num_readings_ = 360; // 360 readings for a full 360-degree scan
        min_angle_ = -M_PI / 2; // -90 degrees
        max_angle_ = M_PI / 2;  // 90 degrees
        range_min_ = 0.1;       // Minimum range in meters
        range_max_ = 10.0;      // Maximum range in meters
    }

    // Service callback function that captures a LIDAR scan when requested
    bool captureScanCallback(lidar_driver_node::CaptureScan::Request &req,
                             lidar_driver_node::CaptureScan::Response &res)
    {
        ROS_INFO("Service to capture LIDAR scan received.");

        // Create a LaserScan message to publish
        sensor_msgs::LaserScan scan_msg;

        // Set up some basic parameters of the scan
        scan_msg.header.stamp = ros::Time::now();
        scan_msg.header.frame_id = "laser_frame"; // Typically "base_laser_link" or similar
        scan_msg.angle_min = min_angle_;
        scan_msg.angle_max = max_angle_;
        scan_msg.angle_increment = (max_angle_ - min_angle_) / (num_readings_ - 1);
        scan_msg.range_min = range_min_;
        scan_msg.range_max = range_max_;

        // Fill the ranges with simulated data (example: range values)
        scan_msg.ranges.resize(num_readings_);
        for (int i = 0; i < num_readings_; ++i)
        {
            scan_msg.ranges[i] = range_min_ + (range_max_ - range_min_) * (i / double(num_readings_));
        }

        // Fill the intensities with dummy values (usually used for LIDAR intensity data)
        scan_msg.intensities.resize(num_readings_);
        for (int i = 0; i < num_readings_; ++i)
        {
            scan_msg.intensities[i] = 1000.0; // Example constant intensity
        }

        // Publish the scan message
        scan_pub_.publish(scan_msg);

        // Respond to the service request
        res.success = true;
        res.message = "LIDAR scan successfully captured and published.";
        return true;
    }

private:
    ros::Publisher scan_pub_;       // Publisher for the LaserScan data
    ros::Timer timer_;              // Timer to trigger the callback at regular intervals
    ros::ServiceServer capture_service_; // Service to trigger LIDAR scan capture

    // LIDAR configuration (simulated)
    int num_readings_;
    double min_angle_;
    double max_angle_;
    double range_min_;
    double range_max_;

    void timerCallback(const ros::TimerEvent&)
    {
        // This is a placeholder to simulate periodic LIDAR data updates.
        // You could choose to publish at a regular interval if needed.
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_driver_node");
    LidarDriver lidar_driver;

    // ROS loop, will handle callbacks such as the timer callback and service requests
    ros::spin();

    return 0;
}

// use rosrun lidar_driver_node lidar_driver_node to run the node
// use rosservice call /capture_lidar_scan to capture a lidar scan
