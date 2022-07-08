#include <iostream>
#include "math.h" 
#include "ros/ros.h"
#include "ros/service_client.h"
#include <std_srvs/Trigger.h>
#include <behavior_tree/behavior_tree.h>
using namespace std; 

class init_action{
    private:
        ros::NodeHandle n;
        ros::ServiceClient init_srv;
    public:
        bt::Action action_init;
        init_action();
        void actionSet(int state);
        void cmdiniting();
};

init_action :: init_action() : action_init("init"){
    init_srv = n.serviceClient<std_srvs::Trigger>("/go_init");
}

void init_action :: actionSet(int state){
    switch(state){
        case 1:
            action_init.set_success();
            break;
        case 0:
            action_init.set_running();
            break;
        case -1:
            action_init.set_failure();
            break;
    }
    action_init.publish();
    return;
}

void init_action :: cmdiniting(){
    std_srvs::Trigger srv;
    if(init_srv.call(srv)){ return; }
    else{ ROS_INFO("Error"); }
    return;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "action_init");
    init_action in_ac;
    while(ros::ok()){
        if(in_ac.action_init.is_active() && in_ac.action_init.active_has_changed()){
            ROS_INFO("Action: init activiate");
            in_ac.cmdiniting();
        }
        if(in_ac.action_init.active_has_changed() && !(in_ac.action_init.is_active())){
            ROS_INFO("Action: Done");
            in_ac.actionSet(1);
        }
        if(in_ac.action_init.is_active()){
            in_ac.actionSet(0);
        }
        ros::spinOnce();
    }
    return 0;
}