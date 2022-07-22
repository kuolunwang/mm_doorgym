#ifndef _BEHAVIOR_TREE_IMPLEMENTATION_H_
#define _BEHAVIOR_TREE_IMPLEMENTATION_H_

#include <behavior_tree_msgs/Status.h>
#include <behavior_tree_msgs/Active.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <stdint.h>
#include <ros/ros.h>
#include <unordered_map>

//===============================================================================================
// ------------------------------------- Base Node Class ----------------------------------------
//===============================================================================================

class Node {
private:
public:
  static double max_wait_time;
  
  std::string label;
  bool is_active;
  uint8_t status;
  std::vector<Node*> children;
  
  virtual void add_child(Node*);
  virtual bool tick(bool active, int traversal_count)=0;
};

//===============================================================================================
// ------------------------------------ Control Flow Nodes  -------------------------------------
//===============================================================================================

class ControlFlowNode : public Node {
private:
public:
};

class FallbackNode : public ControlFlowNode {
private:
public:
  FallbackNode();
  bool tick(bool active, int traversal_count) override;
};

class SequenceNode : public ControlFlowNode {
private:
public:
  SequenceNode();
  bool tick(bool active, int traversal_count) override;
};

class ParallelNode : public ControlFlowNode {
private:
public:
  int child_success_threshold;
  ParallelNode(int child_success_threshold);
  bool tick(bool active, int traversal_count) override;
};

//===============================================================================================
// -------------------------------------- Execution Nodes  --------------------------------------
//===============================================================================================

class ExecutionNode : public Node {
private:
public:
  ros::Time status_modification_time;
  int current_traversal_count;
  ros::Subscriber subscriber;
  ros::Publisher publisher;
  
  void init_ros();
  virtual std::string get_publisher_name()=0;
  virtual void init_publisher()=0;
  virtual std::string get_subscriber_name()=0;
  virtual void init_subscriber()=0;
  void set_status(uint8_t status);
  double get_status_age();
};

class ConditionNode : public ExecutionNode {
private:
  bool callback_status_updated;
  
public:
  ConditionNode(std::string label);
  bool tick(bool active, int traversal_count) override;
  std::string get_publisher_name() override;
  void init_publisher() override;
  std::string get_subscriber_name() override;
  void init_subscriber() override;
  void callback(std_msgs::Bool msg);
};

class ActionNode : public ExecutionNode {
private:
  int current_id;
  bool publisher_initialized;
  bool callback_status_updated;
  
public:
  bool is_newly_active;
  
  ActionNode(std::string label);
  bool tick(bool active, int traversal_count) override;
  std::string get_publisher_name() override;
  void init_publisher() override;
  std::string get_subscriber_name() override;
  void init_subscriber() override;
  void callback(behavior_tree_msgs::Status msg);
  void publish_active_msg(int active_id);
};


//===============================================================================================
// -------------------------------------- Decorator Nodes  --------------------------------------
//===============================================================================================

class DecoratorNode : public Node {
private:
public:
  void add_child(Node* node) override;
};


class NotNode : public DecoratorNode {
private:
public:
  NotNode();
  void add_child(Node* node) override;
  bool tick(bool active, int traversal_count) override;
};

//===============================================================================================
// --------------------------------------- Behavior Tree  ---------------------------------------
//===============================================================================================

class BehaviorTree {
public:
  std::string config_filename;
  std::vector<Node*> nodes;
  Node* root;
  int traversal_count;
  std::unordered_map<std::string, int> active_ids;
  ros::Publisher active_actions_pub;
  ros::Publisher graphviz_pub;
  ros::Publisher compressed_pub;
  bool first_tick;
  
  void parse_config();
  int count_tabs(std::string str);
  std::string strip_space(std::string str);
  std::string strip_brackets(std::string str);
  std::vector<int> get_arguments(std::string str);
  std::string get_graphviz();
  //public:
  BehaviorTree(std::string config_filename);
  bool tick();
};

#endif
