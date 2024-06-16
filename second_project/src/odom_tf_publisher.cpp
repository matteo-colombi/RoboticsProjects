#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"

class odom_tf_publisher {

	private:
        ros::NodeHandle n;
        ros::Subscriber odomSub;
        tf2_ros::TransformBroadcaster br;
        std::string input_odom, root_frame, child_frame;
   
    public:
        void odomCallBack(const nav_msgs::Odometry::ConstPtr& msg){
            geometry_msgs::TransformStamped stamped_transform;
            stamped_transform.header.stamp = ros::Time::now();
            stamped_transform.header.frame_id = root_frame;
            stamped_transform.child_frame_id = child_frame;
            geometry_msgs::Transform transform;
            geometry_msgs::Vector3 translation;
            translation.x = msg->pose.pose.position.x;
            translation.y = msg->pose.pose.position.y;
            translation.z = 0;
            transform.translation = translation;
            geometry_msgs::Quaternion rotation;
            rotation.x = msg->pose.pose.orientation.x;
            rotation.y = msg->pose.pose.orientation.y;
            rotation.z = msg->pose.pose.orientation.z;
            rotation.w = msg->pose.pose.orientation.w;
            transform.rotation = rotation;
            stamped_transform.transform = transform;

            br.sendTransform(stamped_transform);
        }

        odom_tf_publisher() {
            ros::NodeHandle n_private("~");
            n_private.getParam("input_odom", input_odom);
            n_private.getParam("root_frame", root_frame);
            n_private.getParam("child_frame", child_frame);
            odomSub = n.subscribe<nav_msgs::Odometry>(input_odom, 1000, &odom_tf_publisher::odomCallBack, this);
        }
};


int main(int argc, char *argv[]) {
    
	ros::init(argc, argv, "odom_tf_publisher");

	odom_tf_publisher node;

  	ros::spin();

	return 0;
}
