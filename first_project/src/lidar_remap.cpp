#include "ros/ros.h"
#include "dynamic_reconfigure/server.h"
#include "first_project/lidarRemapConfig.h"
#include "sensor_msgs/PointCloud2.h"

class lidar_remap {

	private:
        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Publisher pub;
        std::string reference_odometry;
   
    public:
        void lidarCallBack(const sensor_msgs::PointCloud2::ConstPtr& msg){
            sensor_msgs::PointCloud2 newmsg;
            newmsg = *msg;
            newmsg.header.frame_id = reference_odometry;
            newmsg.header.stamp = ros::Time::now();
            pub.publish(newmsg);
        }

        void reconfigureCallBack(first_project::lidarRemapConfig &config, uint32_t level){
            reference_odometry = config.reference_odometry;
            ROS_INFO("Set reference odometry to: %s", reference_odometry.c_str());
        }

        lidar_remap(){
            sub = n.subscribe<sensor_msgs::PointCloud2>("/os_cloud_node/points", 1000, &lidar_remap::lidarCallBack, this);
            pub = n.advertise<sensor_msgs::PointCloud2>("/pointcloud_remapped", 1000, this);
        }
};


int main(int argc, char *argv[]) {

	ros::init(argc, argv, "lidar_remap");

	lidar_remap node;

    dynamic_reconfigure::Server<first_project::lidarRemapConfig> server;
    dynamic_reconfigure::Server<first_project::lidarRemapConfig>::CallbackType f;
    f = boost::bind(&lidar_remap::reconfigureCallBack, &node, _1, _2);
    server.setCallback(f);

  	ros::spin();

	return 0;
}
