#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/NavSatFix.h"

Pose gps_pose;


//fonction appelée par subscriber
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& fix){
    //recupere les lat, long, alt du gps
    gps_pose.Point.x = fix->poses.latitude;
    gps_pose.Point.y = fix->poses.longitude;
    gps_pose.Point.z = fix->poses.altitude;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "gps");
    ros::NodeHandle n;

    ros::Subscriber timer_sub = n.subscribe("rrbot/gps/fix", 1, gpsCallback);
    ros::Publisher gps_pose_pub = n.advertise<geometry_msgs::Pose>("gps_pose", 1);

    ros::Rate loop_rate(10);

    while(ros::ok()){

        //Publication du message
        gps_pose_pub.publish(gps_pose);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
