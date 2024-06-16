#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

class pointcloud_remap {

	private:
        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Publisher pub;
        std::string input_topic, output_topic, sensor_frame;
   
    public:
        void callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
            sensor_msgs::PointCloud2 newmsg;
            newmsg = *msg;
            newmsg.header.frame_id = sensor_frame;
            newmsg.header.stamp = ros::Time::now();
            pub.publish(newmsg);
        }

        pointcloud_remap() {
            ros::NodeHandle n_private("~");
            n_private.getParam("sensor_frame", sensor_frame);
            n_private.getParam("input_topic", input_topic);
            n_private.getParam("output_topic", output_topic);
            sub = n.subscribe<sensor_msgs::PointCloud2>(input_topic, 1000, &pointcloud_remap::callback, this);
            pub = n.advertise<sensor_msgs::PointCloud2>(output_topic, 1000, this);
        }
};


int main(int argc, char *argv[]) {
    
	ros::init(argc, argv, "pointcloud_remap");

	pointcloud_remap node;

  	ros::spin();

	return 0;
}
