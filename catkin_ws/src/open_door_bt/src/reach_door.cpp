#include <iostream>
#include "math.h" 
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/service_client.h"
#include <std_srvs/Trigger.h>
#include <behavior_tree/behavior_tree.h>
using namespace std; 

class reach_door_action{
    private:
        ros::NodeHandle n;
        // ros::Publisher reach_door_pub;
        ros::ServiceClient door_srv;
    public:
        bt::Action action_reach_door;
        reach_door_action();
        void actionSet(int state);
        // void cmdreachdooring();
        void checkdoor();
};

reach_door_action :: reach_door_action() : action_reach_door("nav to door"){
    // reach_door_pub = n.advertise<std_msgs::String>("/state", 1000);
    door_srv = n.serviceClient<std_srvs::Trigger>("/check_door");
}

void reach_door_action :: actionSet(int state){
    switch(state){
        case 1:
            action_reach_door.set_success();
            break;
        case 0:
            action_reach_door.set_running();
            break;
        case -1:
            action_reach_door.set_failure();
            break;
    }
    action_reach_door.publish();
    return;
}

// void reach_door_action :: cmdreachdooring(){
//     reach_door_pub.publish("nav_door");
//     return;
// }

void reach_door_action :: checkdoor(){
    std_srvs::Trigger srv;
    if(door_srv.call(srv)){ return; }
    else{ ROS_INFO("Error"); }
    return;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "action_reach_door");
    reach_door_action in_ac;
    while(ros::ok()){
        if(in_ac.action_reach_door.is_active() && in_ac.action_reach_door.active_has_changed()){
            ROS_INFO("Action: open activiate");
            // in_ac.cmdreachdooring();
        }
        if(in_ac.action_reach_door.active_has_changed() && !(in_ac.action_reach_door.is_active())){
            ROS_INFO("Action: Done");
            in_ac.actionSet(1);
        }
        if(in_ac.action_reach_door.is_active()){
            in_ac.actionSet(0);
            in_ac.checkdoor();
        }
        ros::spinOnce();
    }
    return 0;
}