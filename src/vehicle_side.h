#include <ros/ros.h>
#include <rosbag/bag.h>
#include <string>
#include <stdlib.h>
#include "aro_ft/aro_ft.h"
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include "dynamixel_controllers/SetSpeed.h"

class DynamixelClient {

    //All speeds are in rad/s
    //Spool radius is ~1.5cm ~0.6in ~0.5ft
    private:
        ros::NodeHandle &nh_;

        float reel_speed;       // rad/s
        float desired_depth;    // ft
        float sample_time;      // seconds

        ros::ServiceClient dynamixel_client;

    public:
        DynamixelClient(ros::NodeHandle &nh, float input_speed, float input_depth, float input_time):
                            nh_(nh),
                            reel_speed(input_speed),
                            desired_depth(input_depth),
                            sample_time(input_time),
                            dynamixel_client(nh_.serviceClient<dynamixel_controllers::SetSpeed>("/reel_controller/set_speed"))
        {}

        void actuateDynamixel() {
            dynamixel_controllers::SetSpeed temp_srv;
            temp_srv.request.speed = -reel_speed;   // Negative speed lets out line
            float duration = desired_depth / (reel_speed * 0.05);  // reel radius ~0.05ft

            ROS_INFO("Lowering Sonde.");
            dynamixel_client.call(temp_srv);
            ros::Duration(duration).sleep();        // Lower Sonde to desired depth

            temp_srv.request.speed = 0.0;
            dynamixel_client.call(temp_srv);        // Stop Sonde at desired Depth

            ROS_INFO("Sonde lowered, sampling at specified depth.");
            ros::Duration(sample_time).sleep();     // Let Sonde sample at desired depth

            ROS_INFO("Raising Sonde.");
            temp_srv.request.speed = reel_speed;   // Positive speed reels line in
            dynamixel_client.call(temp_srv);       // Raise Sonde
            ros::Duration(duration).sleep();

            temp_srv.request.speed = 0.0;
            dynamixel_client.call(temp_srv);
        }

};

class CollectData {
    private:
        int hb_count;
        bool collecting_data;
        bool look_for_heartbeat;
        bool sampling;
        float depth;                         // depth to sample at (ft)
        int sample_count;
        rosbag::Bag bag;
        ros::NodeHandle &nh_;
        ros::Subscriber begin_collection;    // receive start collection signal from user
        ros::Subscriber increment_heartbeat; // subscribe to teensy heartbeat
        ros::Timer check_count_timer;        // count teensy heartbeat
        ros::Publisher vehicle_signal;       // send teensy signal to begin sampling
        ros::Subscriber teensy_signal;    // receive teensy signal sampling has begun
        ros::Subscriber receive_data;        // receive aro-ft data from teensy

    public:
        CollectData(ros::NodeHandle &nh):
                    hb_count(0),
                    collecting_data(false),
                    look_for_heartbeat(false),
                    sampling(false),
                    depth(0.0),
                    sample_count(0),
                    bag("collected_data.bag", rosbag::bagmode::Write),
                    nh_(nh),
                    begin_collection(nh_.subscribe("begin_collection", 1, &CollectData::beginCollectionCb, this)),
                    increment_heartbeat(nh_.subscribe("teensy_hb", 10, &CollectData::receiveHeartbeatCb, this)),
                    check_count_timer(nh_.createTimer(ros::Duration(1), &CollectData::checkCountCb, this)),
                    vehicle_signal(nh_.advertise<std_msgs::Empty>("vehicle_signal", 1)),
                    teensy_signal(nh_.subscribe("teensy_signal", 1, &CollectData::teensySignalCb, this)),
                    receive_data(nh_.subscribe("collected_data", 1000, &CollectData::writeToBag, this))
/*
                    begin_sampling(nh_.advertise<std_msgs::Empty>("begin_sampling", 1)),
                    sampling_started(nh_.subscribe("sampling_started", 1, &CollectData::samplingStartedCb, this)),
                    transmit_start(nh_.advertise<std_msgs::Empty>("transmit_start", 1)),
                    receive_data(nh_.subscribe("aroft_data", 10000, &CollectData::writeToBag, this)),
                    transmit_end(nh_.subscribe("transmit_end", 1, &CollectData::endTransmitCb, this))
*/
        {
            check_count_timer.stop();
        }

        void beginCollectionCb(const std_msgs::Float32& msg) {
            //Subscriber that waits for Float32 msg to start transmission from user - value of Float is desired depth
            //if collecting is true, give a warning to the user that it's already collecting
            //if collecting is false, set heartbeat count to zero, start timer

            depth = msg.data;

            if (collecting_data) {
                ROS_WARN("Already collecting data");
            } else {
                ROS_INFO("Starting data collection process.");
                hb_count = 0;
                collecting_data = true;
                look_for_heartbeat = true;
                check_count_timer.start();
            }
        }

        void receiveHeartbeatCb(const std_msgs::Empty& msg) {
            //callback for incrementing count when heartbeat is received
            //if collecting is false do nothing, if it's true then increment count

            if (look_for_heartbeat && hb_count < 10) {
                hb_count++;
                ROS_INFO("heartbeat");
            }
        }

        void checkCountCb(const ros::TimerEvent &) {
            //if heartbeat count is > 3, publish empty msg to teensy to begin transmit then close timer

            if (hb_count > 3) {
                if (!sampling) {
                    ROS_INFO("Received heartbeat, starting probe sampling.");
                    vehicle_signal.publish(std_msgs::Empty());
                    sampling = true;
                    check_count_timer.stop();
                    look_for_heartbeat = false;
                } else {
                    ROS_INFO("Received heartbeat, starting transmission.");
                    vehicle_signal.publish(std_msgs::Empty());
                    sampling = false;
                    check_count_timer.stop();
                    look_for_heartbeat = false;
                }
            }
        }

        void teensySignalCb(const std_msgs::Empty& msg) {
            if (sampling) {
                ROS_INFO("Sampling started");
                DynamixelClient dynamixel(nh_, 5.0, depth, 5.0); //Node, reel speed (rad/s), depth to sample at (ft), time to sample at desired depth (sec)
                dynamixel.actuateDynamixel();

                //after Sonde has been raiseed again begin listening for heart beat
                ros::spinOnce();
                hb_count = 0;
                look_for_heartbeat = true;
                check_count_timer.start();
            } else {
                collecting_data = false;
                sample_count++;
                ROS_INFO("Done collecting");
            }
        }

        void writeToBag(const aro_ft::aro_ft& teensyData) {
            ROS_INFO("Receiving data");
            const char * bag_topic = "sample";
            char topic_array[8];
            strcpy(topic_array, bag_topic); 
            strcat(topic_array, "1");
            bag.write(topic_array, ros::Time::now(), teensyData);
        }
};
