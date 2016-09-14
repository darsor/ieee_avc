#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Poses {
    int px, py;
    int ox, oy, oz, ow;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "nav_goals");

    int numPoses = 3;
    struct Poses* poses = new Poses[3];
    //TODO: set poses here

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    for (int i=0; i<numPoses; ++i) {
        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = poses[i].px;
        goal.target_pose.pose.position.y = poses[i].py;
        goal.target_pose.pose.orientation.x = poses[i].ox;
        goal.target_pose.pose.orientation.y = poses[i].oy;
        goal.target_pose.pose.orientation.z = poses[i].oz;
        goal.target_pose.pose.orientation.w = poses[i].ow;

        ROS_INFO("Sending goal #%d", i);
        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Goal #%d reached", i);
        }
        else {
            ROS_INFO("Failed to reach goal #%d", i);
            break;
        }
    }

    return 0;
}
