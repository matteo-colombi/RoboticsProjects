#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include <stdlib.h>
#include <cmath>  
#include <sstream>

#define A 6378137
#define B 6356752
#define E_SQUARE 0.006694478198

class gps_to_odom {

	private:
		ros::NodeHandle n;
		ros::Subscriber gpsSub;
		ros::Publisher odomPub;
		double lat_r, lon_r, alt_r;
		double sin_lat_r, cos_lat_r, sin_lon_r, cos_lon_r;
		double Xr, Yr, Zr;
		double xPrev, yPrev, zPrev;
		double heading;

		void gpsToECEF(double *X, double *Y, double *Z, double phi, double lambda, double h){
			double N = ((double)A)/sqrt(1 - E_SQUARE*(pow(sin(phi), 2)));
			*X = (N + h)*cos(phi)*cos(lambda);
			*Y = (N + h)*cos(phi)*sin(lambda);
			*Z = (N*(1-E_SQUARE)+h)*sin(phi);
		}

		void ECEFtoENU(double *x, double *y, double *z, double X, double Y, double Z) {
			*x = -sin_lon_r*(X-Xr) + cos_lon_r*(Y-Yr);
			*y = -sin_lat_r*cos_lon_r*(X-Xr) -sin_lat_r*sin_lon_r*(Y-Yr) + cos_lat_r*(Z-Zr);
			*z = cos_lat_r*cos_lon_r*(X-Xr) + cos_lat_r*sin_lon_r*(Y-Yr) + sin_lat_r*(Z-Zr);
		}

		void updateHeading(double dx, double dy) {
			if (dx != 0) {
				if (dx > 0) heading = atan(dy/dx);
				else heading = atan(dy/dx) + M_PI;
			} else if (dy > 0) {
				heading = M_PI_2;
			} else if (dy < 0) {
				heading = 3*M_PI_2;
			}
		}

		nav_msgs::Odometry coordinateConvert(sensor_msgs::NavSatFix gpsPos){
			nav_msgs::Odometry retval;
			
			double phi, lambda, h;
			phi = gpsPos.latitude*M_PI/180.0;
			lambda = gpsPos.longitude*M_PI/180.0;
			h = gpsPos.altitude;
			
			double X, Y, Z;
			gpsToECEF(&X, &Y, &Z, phi, lambda, h);
	
			double x, y, z;
			ECEFtoENU(&x, &y, &z, X, Y, Z);
	
			retval.pose.pose.position.x = x;
			retval.pose.pose.position.y = y;
			retval.pose.pose.position.z = z;

			retval.pose.pose.orientation.w = sin(heading/2);
			retval.pose.pose.orientation.z = cos(heading/2);
			retval.pose.pose.orientation.x = 0;
			retval.pose.pose.orientation.y = 0;

			double dx = x-xPrev;
			double dy = y-yPrev;

			updateHeading(dx, dy);

			yPrev = y;
			xPrev = x;

			return retval;
		}

	public:
		void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
			nav_msgs::Odometry converted = coordinateConvert(*msg);
			odomPub.publish(converted);
		}

		gps_to_odom(){
			gpsSub = n.subscribe("/fix", 1, &gps_to_odom::gpsCallback, this);
			odomPub = n.advertise<nav_msgs::Odometry>("/gps_odom", 1);

			n.getParam("lat_r", lat_r);
			n.getParam("lon_r", lon_r);
			n.getParam("alt_r", alt_r);
			lat_r = lat_r*M_PI/180.0;
			lon_r = lon_r*M_PI/180.0;
			gpsToECEF(&Xr, &Yr, &Zr, lat_r, lon_r, alt_r);

			sin_lat_r = sin(lat_r);
			cos_lat_r = cos(lat_r);
			sin_lon_r = sin(lon_r);
			cos_lon_r = cos(lon_r);
			heading = 0;
		}
};


int main(int argc, char *argv[])
{

	ros::init(argc, argv, "gps_to_odom");

	gps_to_odom node;

  	ros::spin();

	return 0;
}
