#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Poses {
    float pos[2];
    float rot[2];
};

int main(int argc, char** argv){
    ros::init(argc, argv, "nav_goals");
    ros::param::set("/free_goal_vel", true);
    ros::param::set("/xy_goal_tolerance", 0.2);
    ros::param::set("/yaw_goal_tolerance", 0.5);

    int numPoses = 4;
    struct Poses* poses = new Poses[numPoses];
    poses[0].pos[0] = 23.090;
    poses[0].pos[1] = 22.641;
    poses[0].rot[0] = 0.998;
    poses[0].rot[1] = 0.068;

    poses[1].pos[0] = -2.735;
    poses[1].pos[1] = 24.452;
    poses[1].rot[0] = 0.994;
    poses[1].rot[1] = 0.111;

    poses[2].pos[0] = -21.556;
    poses[2].pos[1] = 10.139;
    poses[2].rot[0] = -0.797;
    poses[2].rot[1] = 0.604;

    poses[3].pos[0] = -8.291;
    poses[3].pos[1] = 0.322;
    poses[3].rot[0] = -0.188;
    poses[3].rot[1] = 0.982;

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

        goal.target_pose.pose.position.x = poses[i].pos[0];
        goal.target_pose.pose.position.y = poses[i].pos[1];
        goal.target_pose.pose.orientation.z = poses[i].rot[0];
        goal.target_pose.pose.orientation.w = poses[i].rot[1];

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
