#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"

class odom_to_tf {

	private:
        ros::NodeHandle n;
        ros::Subscriber sub;
        tf::TransformBroadcaster br;
        std::string input_odom, child_frame, root_frame;
   
    public:
        void odomCallBack(const nav_msgs::Odometry::ConstPtr& msg){
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0) );
            tf::Quaternion q;
            q.setValue(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), root_frame, child_frame));
        }

        odom_to_tf(){
            ros::NodeHandle n_private("~");
            n_private.getParam("input_odom", input_odom);
            n_private.getParam("child_frame", child_frame);
            n_private.getParam("root_frame", root_frame);
            sub = n.subscribe<nav_msgs::Odometry>(input_odom, 1000, &odom_to_tf::odomCallBack, this);
        }
};


int main(int argc, char *argv[]) {
    
	ros::init(argc, argv, "odom_to_tf");

	odom_to_tf node;

  	ros::spin();

	return 0;
}
