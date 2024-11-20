/*******************************************************************************
 *      Title     : pose_tracking_example.cpp
 *      Project   : moveit_servo
 *      Created   : 09/04/2020
 *      Author    : Adam Pettinger
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include <ros/ros.h>  

#include <std_msgs/Int8.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_servo/servo.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <thread>

#include <actionlib/server/simple_action_server.h>


#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//Custom
#include <moveit_pose_tracking/MoveToAction.h>



static const std::string LOGNAME = "pose_tracking_action";


// Pose tracking action server class

class PoseTrackingAction
{
public:

  PoseTrackingAction(ros::NodeHandle nh)
  {
    // Initialize handle
    pnh_ = nh;

    // Retrieve parameters from parameter server 
    pnh_.param<std::string>("planning_frame", planning_frame_, planning_frame_);
    pnh_.param<std::vector<double>>("lin_tol", lin_tol_, lin_tol_);
    pnh_.param<double>("rot_tol", rot_tol_, rot_tol_);
    pnh_.param<double>("timeout", timeout_, timeout_);

    lin_tol_vect_[0] = lin_tol_[0];
    lin_tol_vect_[1] = lin_tol_[1];
    lin_tol_vect_[2] = lin_tol_[2];

    // Initialize transform listener
    tfBuffer_ = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

    // Load the planning scene monitor
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    
    // update the planning scene monitor with the current state
    bool success = planning_scene_monitor_->requestPlanningSceneState("get_planning_scene");
    ROS_INFO_STREAM("Request planning scene " << (success ? "succeeded." : "failed."));

    if (!planning_scene_monitor_->getPlanningScene())
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Error in setting up the PlanningSceneMonitor.");
      exit(EXIT_FAILURE);
    }

    /* planning_scene_monitor_->startSceneMonitor(); */
    planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
    planning_scene_monitor_->startWorldGeometryMonitor(
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
        false /* skip octomap monitor */);
    planning_scene_monitor_->startStateMonitor();
    planning_scene_monitor_->monitorDiffs(true);

    /*     planning_scene_monitor_->requestPlanningSceneState();
        planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor_);

        // Get planning frame
        planning_frame = planning_scene->getPlanningFrame(); */

    // Create the pose tracker
    //moveit_servo::PoseTracking tracker(nh, planning_scene_monitor_);
   
    // Reset pose tracking action server
    bool autostart = false;
    pose_tracking_as_.reset(
        new actionlib::SimpleActionServer<moveit_pose_tracking::MoveToAction>(pnh_, "move_to",  boost::bind(&PoseTrackingAction::exectCB, this, _1) , autostart));
    pose_tracking_as_->registerPreemptCallback(boost::bind(&PoseTrackingAction::preemptCB, this));

    // Initialize goal pose publisher
    target_pose_pub_ = pnh_.advertise<geometry_msgs::PoseStamped>("target_pose", 1 /* queue */, true /* latch */);

    // Initialize pose tracking object
    pose_tracker_ = std::make_shared<moveit_servo::PoseTracking>(pnh_, planning_scene_monitor_);

    // Start pose tracking server
    pose_tracking_as_->start();
    ROS_INFO_STREAM("Started pose tracking action server: 'move to'");
  }

  void moveToPoseTracker(){
    moveit_servo::PoseTrackingStatusCode result = pose_tracker_->moveToPose(lin_tol_vect_, rot_tol_, timeout_ /* target pose timeout */);
    thread_flag_ = true;
    if(!(result == moveit_servo::PoseTrackingStatusCode::SUCCESS)){
      thread_result_ = false;
      return;
    }else{
      thread_result_ = true;
      return;
    }
  }

  void exectCB(const moveit_pose_tracking::MoveToGoalConstPtr &pose_tracking_goal_){

    ros::Rate loop_rate(10);

    ROS_INFO("Processing new pose tracking goal.");

    // Create target pose
    geometry_msgs::PoseStamped target_pose;
    geometry_msgs::PoseStamped target_pose_from_end_effector;

    // Construct transformation matrices 
    tf2::Transform toParentPose, toGlobalPose, toEndEffectorPose, toEndEffectorTransformPose;

    if ( pose_tracking_goal_->pose == target_pose){
      ROS_WARN("Please fill in action: 'move to' goal message with valid pose message");
      pose_tracking_result_.result = false;
      pose_tracking_result_.message = "Invalid 'move to' action pose goal message, fill in message";
      pose_tracking_as_->setAborted(pose_tracking_result_);
      return;
    }

    target_pose = pose_tracking_goal_->pose;

    tf2::fromMsg(target_pose.pose, toEndEffectorTransformPose);

    try{
      tf2::fromMsg(tfBuffer_->lookupTransform(planning_frame_, target_pose.header.frame_id,ros::Time::now(),ros::Duration(2.0)).transform, toGlobalPose);
      tf2::fromMsg(tfBuffer_->lookupTransform(pose_tracking_goal_->pose.header.frame_id, "tool0" ,ros::Time::now(),ros::Duration(2.0)).transform, toEndEffectorPose);
    }
    catch(tf2::TransformException ex){
      pose_tracking_result_.result = false;
      pose_tracking_result_.message = "Invalid goal frame, cannot find frame in tf tree";
      pose_tracking_as_->setAborted(pose_tracking_result_);
      return;
    }

    // Normalize quaternion and check if is valid
    tf2::Quaternion quat_orientation;
    tf2::fromMsg(target_pose.pose.orientation, quat_orientation);
    quat_orientation.normalize();
    if(std::isnan(quat_orientation.length()))
    {
      pose_tracking_result_.result = false;
      pose_tracking_result_.message = "Invalid quaternion in goal pose message, null value quaternion";
      pose_tracking_as_->setAborted(pose_tracking_result_);
      return;
    }
    target_pose.pose.orientation = tf2::toMsg(quat_orientation);

    // Build transform object
    tf2::fromMsg(target_pose.pose, toParentPose);

    // Compute target_pose by multipying by transformation matrices (from local pose -> parent pose -> global pose = toGlobalPose*toParentPose)   
    tf2::toMsg(toGlobalPose*toParentPose, target_pose.pose);

    target_pose.header.frame_id = planning_frame_;
    target_pose.header.stamp = ros::Time::now();
    
    tf2::toMsg(toEndEffectorPose, target_pose_from_end_effector.pose);
    double distance = std::sqrt(std::pow(std::abs(target_pose_from_end_effector.pose.position.x), 2) + std::pow(std::abs(target_pose_from_end_effector.pose.position.y), 2) + std::pow(std::abs(target_pose_from_end_effector.pose.position.z), 2));
    ROS_INFO_STREAM("DISTANCE FIRST: " << distance);
    if (distance < 1.25)
    {
      target_pose_pub_.publish(target_pose); 
    }else
    {
      pose_tracking_result_.result = false;
      pose_tracking_result_.message = "Invalid goal frame, distance bigger than allowed";
      pose_tracking_as_->setAborted(pose_tracking_result_);
      return;
    }

    // Run the pose tracking functionality in a thread
    boost::thread pose_tracker_thread_(&PoseTrackingAction::moveToPoseTracker,this);

    while(!thread_flag_  && !preempted_flag_){

      try{
      tf2::fromMsg(tfBuffer_->lookupTransform(planning_frame_, pose_tracking_goal_->pose.header.frame_id,ros::Time(0),ros::Duration(2.0)).transform, toGlobalPose);
      tf2::fromMsg(tfBuffer_->lookupTransform(pose_tracking_goal_->pose.header.frame_id, "tool0" ,ros::Time::now(),ros::Duration(2.0)).transform, toEndEffectorPose);
      }
      catch(tf2::TransformException ex){
        // Stop pose tracking thread
        pose_tracker_->stopMotion();
        pose_tracker_thread_.join();

        thread_flag_ = false;
        thread_result_ = false;
        preempted_flag_ = false;
        pose_tracking_result_.result = false;
        pose_tracking_result_.message = "Invalid goal frame, cannot find frame in tf tree";
        pose_tracking_as_->setAborted(pose_tracking_result_);
        return;
      }

      // Compute target_pose by multipying by transformation matrices (from local pose -> parent pose -> global pose = toGlobalPose*toParentPose)   
      tf2::toMsg(toGlobalPose*toParentPose, target_pose.pose);
      //tf2::toMsg(toEndEffectorTransformPose.inverse()*toEndEffectorPose, target_pose_from_end_effector.pose);
      tf2::toMsg(toEndEffectorPose, target_pose_from_end_effector.pose);
      //ROS_INFO_STREAM("REMAINING: " << target_pose_from_end_effector.pose);

      target_pose.header.frame_id = planning_frame_;
      target_pose.header.stamp = ros::Time::now();

      distance = std::sqrt(std::pow(std::abs(target_pose_from_end_effector.pose.position.x), 2) + std::pow(std::abs(target_pose_from_end_effector.pose.position.y), 2) + std::pow(std::abs(target_pose_from_end_effector.pose.position.z), 2));
      //ROS_INFO_STREAM("DISTANCE: " << distance);
      if (distance < 1.25)
      {
        target_pose_pub_.publish(target_pose); 
      }else
      {
        // Stop pose tracking thread
        pose_tracker_->stopMotion();
        pose_tracker_thread_.join();

        thread_flag_ = false;
        thread_result_ = false;
        preempted_flag_ = false;
        pose_tracking_result_.result = false;
        pose_tracking_result_.message = "Invalid goal frame, distance bigger than allowed";
        pose_tracking_as_->setAborted(pose_tracking_result_);
        return;
      }
      pose_tracking_feedback_.state = "moving";
      pose_tracking_feedback_.remaining = target_pose_from_end_effector.pose;
      pose_tracking_as_->publishFeedback(pose_tracking_feedback_);
      loop_rate.sleep();
    } 

    // Stop pose tracking thread
    pose_tracker_->stopMotion();
    pose_tracker_thread_.join();

    // Reset pose tracker thread flag
    thread_flag_ = false;
    preempted_flag_ = false;

    // Set action result
    if(pose_tracking_as_->isActive()){
      if(thread_result_ == true){
        pose_tracking_feedback_.state = "reached";
        pose_tracking_as_->publishFeedback(pose_tracking_feedback_);
        thread_result_ = false;
        pose_tracking_result_.result = true;
        pose_tracking_result_.message = "Goal reached, 'move to' action succeeded";
        pose_tracking_as_->setSucceeded(pose_tracking_result_);

      }else{
        pose_tracking_feedback_.state = "failed/aborted";
        pose_tracking_as_->publishFeedback(pose_tracking_feedback_);
        thread_result_ = false;
        pose_tracking_result_.result = false;
        pose_tracking_result_.message = "Cannot reach goal, 'move to' action aborted";
        pose_tracking_as_->setAborted(pose_tracking_result_);
      }
    }

    return;
  }

  void preemptCB(){
    ROS_INFO_STREAM("Pose tracking action preemted.");
    preempted_flag_ = true;
    return;
  }

protected:

  // Internal variables
  ros::NodeHandle nh_; // node handle
  ros::NodeHandle pnh_; // private node handle
  
  // MoveIt
  std::string planning_frame_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // MoveIt Servo
  moveit_servo::PoseTrackingPtr pose_tracker_;
  std::vector<double> lin_tol_;
  Eigen::Vector3d lin_tol_vect_;
  double rot_tol_;
  double timeout_;
  bool thread_flag_ = false;
  bool thread_result_ = false;
  bool preempted_flag_ = false;

  
  // Action Server
  std::shared_ptr<actionlib::SimpleActionServer<moveit_pose_tracking::MoveToAction>> pose_tracking_as_;
  //actionlib::SimpleActionServer<moveit_pose_tracking::MoveToAction>::GoalConstPtr pose_tracking_goal_;
  moveit_pose_tracking::MoveToResult pose_tracking_result_;
  moveit_pose_tracking::MoveToFeedback pose_tracking_feedback_;

  // Publishers
  ros::Publisher target_pose_pub_;

  // TF 
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;

};  

// Class for monitoring status of moveit_servo
class StatusMonitor
{
public:
  StatusMonitor(ros::NodeHandle& nh, const std::string& topic)
  {
    sub_ = nh.subscribe(topic, 1, &StatusMonitor::statusCB, this);
  }

private:

  void statusCB(const std_msgs::Int8ConstPtr& msg)
  {
    moveit_servo::StatusCode latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
    if (latest_status != status_)
    {
      status_ = latest_status;
      const auto& status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
      ROS_INFO_STREAM_NAMED(LOGNAME, "Servo status: " << status_str);
    }
  }
  moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
  ros::Subscriber sub_;
};


/**
 * Instantiate the pose tracking interface.
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, LOGNAME);
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(0);
  spinner.start();
  
  // Subscribe to servo status (and log it when it changes)
  StatusMonitor status_monitor(nh, "status");

  // Run pose tracker action
  PoseTrackingAction pose_tracker(nh);


  // Keep introspection alive
  ros::waitForShutdown();



  return EXIT_SUCCESS;
}
