#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <vector>
#include <mutex>
#include <thread>

class DataCollector {
public:
    DataCollector() {
        save_thread = std::thread(&DataCollector::saveDataPeriodically, this);
    }

    ~DataCollector() {
        if (save_thread.joinable()) {
            save_thread.join();
        }
    }

    /**
     * @brief Callback function that is called when a new PoseStamped message is received.
     * 
     * This function is responsible for adding the received data to the data_vector.
     * It uses a mutex to ensure thread safety when accessing the data_vector.
     * 
     * @param data A pointer to the received PoseStamped message.
     */
    void callback(const geometry_msgs::PoseStamped::ConstPtr& data) {
        std::lock_guard<std::mutex> lock(data_mutex);
        data_vector.push_back(*data);

            // Log the received data for debugging
        ROS_INFO("Received data: x = %f, y = %f, z = %f", 
             data->pose.position.x, 
             data->pose.position.y, 
             data->pose.position.z);
    }

    void saveDataPeriodically() {
        ros::Rate rate(5.0); // Hz

        std::ofstream file("/home/mini/Drone_LTL_Planner/truePath.txt");

        //Check that file is open
        if (!file.is_open()) {
            ROS_ERROR("Could not open file");
            return;
        }

        while (ros::ok()) {
            std::lock_guard<std::mutex> lock(data_mutex);

            //Write to file
            for (const auto& pose_stamped : data_vector) {
                file << pose_stamped.header.stamp << ", " 
                     << pose_stamped.pose.position.x << ", " 
                     << pose_stamped.pose.position.y << ", " 
                     << pose_stamped.pose.position.z << ", "
                     << pose_stamped.pose.orientation.x << ", "
                     << pose_stamped.pose.orientation.y << ", "
                     << pose_stamped.pose.orientation.z << ", "
                     << pose_stamped.pose.orientation.w << std::endl;
            }

            rate.sleep();
        }

        file.close();
        ROS_INFO("Data collector node has stopped.");

    }

private:
    std::vector<geometry_msgs::PoseStamped> data_vector;
    std::mutex data_mutex;
    std::thread save_thread;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_logger");
    ros::NodeHandle nh;

    ROS_INFO("Data collector node has started.");

    DataCollector collector;

    // Subscribe to the mavros/local_position/pose topic
    ros::Subscriber sub = nh.subscribe("mavros/local_position/pose", 5000, &DataCollector::callback, &collector);

    ros::spin();
    return 0;
}
