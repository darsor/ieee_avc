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
    ros::param::set("/xy_goal_tolerance", 0.4);
    ros::param::set("/yaw_goal_tolerance", 1.0);


    int numPoses = 7;
    struct Poses* poses = new Poses[numPoses];

    //Position(12.840, -1.076, 0.000), Orientation(0.000, 0.000, -0.065, 0.998) = Angle: -0.131
    poses[0].pos[0] = 12.840;
    poses[0].pos[1] = -1.076;
    poses[0].rot[0] = -0.065;
    poses[0].rot[1] = 0.998;

    //Position(8.433, -14.722, 0.000), Orientation(0.000, 0.000, -0.732, 0.681) = Angle: -1.642
    poses[1].pos[0] = 8.433;
    poses[1].pos[1] = -14.722;
    poses[1].rot[0] = -0.732;
    poses[1].rot[1] = 0.681;

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[2].pos[0] = 8.202;
    poses[2].pos[1] = -22.073;
    poses[2].rot[0] = -0.753;
    poses[2].rot[1] = 0.658;

    //Position(-3.362, -29.020, 0.000), Orientation(0.000, 0.000, 0.999, 0.032) = Angle: 3.077
    poses[3].pos[0] = -3.362;
    poses[3].pos[1] = -29.020;
    poses[3].rot[0] = 0.999;
    poses[3].rot[1] = 0.032;

    //Position(-6.322, -19.478, 0.000), Orientation(0.000, 0.000, 0.656, 0.754) = Angle: 1.432
    poses[4].pos[0] = -6.322;
    poses[4].pos[1] = -19.478;
    poses[4].rot[0] = 0.656;
    poses[4].rot[1] = 0.754;

    //Position(-6.001, -10.733, 0.000), Orientation(0.000, 0.000, 0.619, 0.785) = Angle: 1.336
    poses[5].pos[0] = -6.001;
    poses[5].pos[1] = -10.733;
    poses[5].rot[0] = 0.619;
    poses[5].rot[1] = 0.785;

    //Position(3.628, 0.330, 0.000), Orientation(0.000, 0.000, -0.050, 0.999) = Angle: -0.101
    poses[6].pos[0] = 3.628;
    poses[6].pos[1] = 0.330;
    poses[6].rot[0] = -0.050;
    poses[6].rot[1] = 0.999;

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
	    --i;
	    continue;
        }
    }

    return 0;
}
