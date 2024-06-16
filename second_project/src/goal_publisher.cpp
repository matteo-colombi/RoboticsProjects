#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "iostream"
#include "fstream"
#include "sstream"
#include "vector"
#include "string"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
MoveBaseClient* actionClientPtr;

class CSVReader {
    private:
        // Function to split a line into tokens based on a delimiter
        std::vector<std::string> split(const std::string &line, char delimiter) {
            std::vector<std::string> tokens;
            std::stringstream ss(line);
            std::string token;
            while (std::getline(ss, token, delimiter)) {
                tokens.push_back(token);
            }
            return tokens;
        }

    public:
        CSVReader() {
        }

        std::vector<std::vector<std::string>> readAll(std::string filename) {
            std::ifstream file(filename);
            std::vector<std::vector<std::string>> contents;
            
            std::string line;
            while(std::getline(file, line)) {
                std::vector<std::string> row = split(line, ',');
                contents.push_back(row);
            }
            file.close();
            return contents;
        }
};

std::vector<geometry_msgs::Pose> parseCSV(std::string waypoints_file) {
    CSVReader reader;

    std::vector<geometry_msgs::Pose> goals;

    std::vector<std::vector<std::string>> contents = reader.readAll(waypoints_file);
    
    for (size_t i = 0; i < contents.size(); i++) {
        std::string x_string = contents[i][0];
        std::string y_string = contents[i][1];
        std::string theta_string = contents[i][2];

        double x = std::stod(x_string);
        double y = std::stod(y_string);
        double heading = std::stod(theta_string);

        geometry_msgs::Pose goal;
        goal.position.x = x;
        goal.position.y = y;
        goal.position.z = 0;
        goal.orientation.x = 0;
        goal.orientation.y = 0;
        goal.orientation.z = sin(heading/2);
        goal.orientation.w = cos(heading/2);
        
        goals.push_back(goal);
    }

    return goals;
}

void sendNewGoal(const geometry_msgs::Pose *pose) {
    move_base_msgs::MoveBaseGoal goal;
    move_base_msgs::MoveBaseActionGoal action_goal;

    action_goal.goal_id.stamp = ros::Time::now(); 
    action_goal.goal_id.id = "map";

    //Goal (geometry_msgs/PoseStamped)
    action_goal.goal.target_pose.header.frame_id = "map";
    action_goal.goal.target_pose.header.stamp = ros::Time::now();
    action_goal.goal.target_pose.pose.position.x = pose->position.x;
    action_goal.goal.target_pose.pose.position.y = pose->position.y;
    action_goal.goal.target_pose.pose.position.z = pose->position.z;
    action_goal.goal.target_pose.pose.orientation.x = pose->orientation.x;
    action_goal.goal.target_pose.pose.orientation.y = pose->orientation.y;
    action_goal.goal.target_pose.pose.orientation.z = pose->orientation.z;
    action_goal.goal.target_pose.pose.orientation.w = pose->orientation.w;

    goal = action_goal.goal;

    actionClientPtr->sendGoal(goal);
}

int main(int argc, char *argv[]) {
    
	ros::init(argc, argv, "goal_publisher");

    std::string waypoints_file;

    ros::NodeHandle n_private("~");
    n_private.getParam("waypoints_file", waypoints_file);

    std::vector<geometry_msgs::Pose> goals = parseCSV(waypoints_file);

    ROS_INFO("Done parsing CSV. There are %d goals.", (int)goals.size());

    MoveBaseClient client("move_base", true);
    client.waitForServer();

    actionClientPtr = &client;

    for(size_t i = 0; i < goals.size(); i++) {
        ROS_INFO("Sending goal position #%d.", (int)(i+1));
        sendNewGoal(&goals[i]);
        //Wait for action to finish (might take a while!)
        client.waitForResult();
    }

    ros::spin();
    
	return 0;
}
