#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#include <first_project/lidarRemapConfig.h>
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <stdlib.h>
#include <cmath>  
#include <sstream>

class lidar_remap {

	private:
        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Publisher pub;
        std::string reference_odometry;
    public:
        void lidarCallBack(const sensor_msgs::PointCloud2::ConstPtr& msg){
            sensor_msgs::PointCloud2 newmsg;
            newmsg.data = msg->data;
            newmsg.height = msg->height;
            newmsg.width = msg->width;
            newmsg.fields = msg->fields;
            newmsg.is_dense = msg->is_dense;
            newmsg.is_bigendian = msg->is_bigendian;
            newmsg.point_step = msg->point_step;
            newmsg.row_step = msg->row_step;
            newmsg.header.seq = msg->header.seq;
            newmsg.header.stamp = ros::Time::now();
            newmsg.header.frame_id = reference_odometry;
            pub.publish(newmsg);
        }
        void reconfigureCallBack(first_project::lidarRemapConfig &config, uint32_t level){
            reference_odometry = config.reference_odometry;
            ROS_INFO("%s", reference_odometry.c_str());
        }

        lidar_remap(){
            reference_odometry = "wheel_odom";
            sub = n.subscribe<sensor_msgs::PointCloud2>("/os_cloud_node/points", 1000, &lidar_remap::lidarCallBack, this);
            pub = n.advertise<sensor_msgs::PointCloud2>("/pointcloud_remapped", 1000, this);

        }
};


int main(int argc, char *argv[])
{

	ros::init(argc, argv, "lidar_remap");

	lidar_remap node;

    dynamic_reconfigure::Server<first_project::lidarRemapConfig> server;
    dynamic_reconfigure::Server<first_project::lidarRemapConfig>::CallbackType f;
    f = boost::bind(&lidar_remap::reconfigureCallBack, &node, _1, _2);
    server.setCallback(f);

  	ros::spin();

	return 0;
}
