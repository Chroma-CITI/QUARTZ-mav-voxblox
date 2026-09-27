#ifndef MAV_LOCAL_PLANNER_MAV_LOCAL_PLANNER_H_
#define MAV_LOCAL_PLANNER_MAV_LOCAL_PLANNER_H_

#include <ros/ros.h>
#include <string>
#include <thread>
#include <vector>

#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_path_smoothing/loco_smoother.h>
#include <mav_path_smoothing/polynomial_smoother.h>
#include <mav_path_smoothing/velocity_ramp_smoother.h>
#include <mav_planning_common/color_utils.h>
#include <mav_planning_common/path_utils.h>
#include <mav_planning_common/path_visualization.h>
#include <mav_planning_common/physical_constraints.h>
#include <mav_planning_common/semaphore.h>
#include <mav_planning_common/yaw_policy.h>
#include <mav_planning_msgs/PolynomialTrajectory4D.h>
#include <mav_visualization/helpers.h>
#include <minkindr_conversions/kindr_msg.h>
#include <voxblox_loco_planner/goal_point_selector.h>
#include <voxblox_loco_planner/voxblox_loco_planner.h>
#include "voxblox_ros/voxblox_server.h"

#include "mav_local_planner/mav_idle_checker.h"

#include "mav_local_planner/logger/logger.h"

namespace mav_planning {

class MavLocalPlanner {
 public:
  MavLocalPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  ~MavLocalPlanner();

  // Input data.
  void odometryCallback(const nav_msgs::Odometry& msg);
  void waypointCallback(const geometry_msgs::PoseStamped& msg);
  void waypointListCallback(const geometry_msgs::PoseArray& msg);

  // Stops path publishing and clears all recent trajectories.
  void clearTrajectory();
  // Same as above, but also sends the current pose of the helicopter to the
  // controller to clear the queue.
  void abort();

  // Service callbacks.
  bool startCallback(std_srvs::Empty::Request& request,
                     std_srvs::Empty::Response& response);
  bool pauseCallback(std_srvs::Empty::Request& request,
                     std_srvs::Empty::Response& response);
  bool stopCallback(std_srvs::Empty::Request& request,
                    std_srvs::Empty::Response& response);

  // Visualizations.
  void visualizeFullPath();
  void visualizeCurrentPath();

  // TODO -- TO IMPLEMENT:
  void polynomialTrajectoryCallback(
      const mav_planning_msgs::PolynomialTrajectory4D& msg) {}

 private:
  // Control for publishing.
  void startPublishingCommands();
  void commandPublishTimerCallback(const ros::TimerEvent& event);

  // Control for planning.
  void planningTimerCallback(const ros::TimerEvent& event);
  void planningStep();

  // Control for collision check
  void collisionCheckTimerCallback(const ros::TimerEvent& event);

  // Returns if the next waypoint is a valid waypoint.
  bool nextWaypoint();
  void finishWaypoints();

  void replacePath(const mav_msgs::EigenTrajectoryPointVector& path);

  // What to do if we fail to find a suitable path, depending on the
  // intermediate goal selection settings.
  bool dealWithFailure();

  // Functions to help out replanning.
  // Track a single waypoint, planning only in a short known horizon.
  void avoidCollisionsTowardWaypoint();
  // Get a path through a bunch of waypoints.
  bool planPathThroughWaypoints(
      const mav_msgs::EigenTrajectoryPointVector& waypoints,
      mav_msgs::EigenTrajectoryPointVector* path);

  // Map access.
  double getMapDistance(const Eigen::Vector3d& position) const;
  double getMapDistanceAndGradient(const Eigen::Vector3d& position,
                                   Eigen::Vector3d* gradient) const;

  // Double-check that everything is safe w.r.t. current map.
  bool isPathCollisionFree(
      const mav_msgs::EigenTrajectoryPointVector& path, int max_number_of_consecutive_blocked_points=1) const;
  bool isPathFeasible(const mav_msgs::EigenTrajectoryPointVector& path) const;

  // Other internal stuff.
  void sendCurrentPose();
  void SendRecoveryTrajectory();
  void sendRecoveryPose(const mav_msgs::EigenTrajectoryPoint& recovery_point);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // ROS inputs and outputs.
  ros::Subscriber waypoint_sub_;
  ros::Subscriber waypoint_list_sub_;
  // Current state of the MAV.
  ros::Subscriber odometry_sub_;

  ros::Publisher command_pub_;
  ros::Publisher path_marker_pub_;
  ros::Publisher full_trajectory_pub_;
  ros::Publisher id_idle_pub_;

  // Marker array to keep the full trajectory in memory
  visualization_msgs::MarkerArray full_trajectory_marker_;

  // Service calls for controlling the local planner.
  // Start will start publishing commands, pause will stop temporarily and you
  // can call start to un-pause, and stop will stop and clear the current
  // trajectory.
  ros::ServiceServer start_srv_;
  ros::ServiceServer pause_srv_;
  ros::ServiceServer stop_srv_;

  // Service client for getting the MAV interface to listen to our sent
  // commands.
  ros::ServiceClient position_hold_client_;

  // ROS async handling: callback queues and spinners for command publishing,
  // which is on a separate thread from the rest of the functions.
  // Same for planning.
  ros::CallbackQueue command_publishing_queue_;
  ros::AsyncSpinner command_publishing_spinner_;
  ros::CallbackQueue planning_queue_;
  ros::AsyncSpinner planning_spinner_;
  ros::CallbackQueue collision_check_queue_;
  ros::AsyncSpinner collision_check_spinner_;

  // Publisher for new messages to the controller.
  ros::Timer command_publishing_timer_;
  ros::Timer planning_timer_;
  ros::Timer collision_check_timer_;

  // Settings -- general
  bool verbose_;
  bool safe_navigation_;

  // Settings -- frames
  std::string namespace_;
  std::string global_frame_id_;
  std::string local_frame_id_;

  // Settings -- constraints.
  PhysicalConstraints constraints_;

  // Settings -- controller interface.
  int mpc_prediction_horizon_;  // Units: timesteps.
  // TODO(helenol): do I need these two to be separate? I guess so...
  double command_publishing_dt_;
  double replan_dt_;
  double replan_lookahead_sec_;
  double collision_check_dt_;

  // Settings -- general planning.
  bool avoid_collisions_;
  bool autostart_;  // Whether to auto-start publishing any new path or wait
  // for start service call.
  bool plan_to_start_;  // Whether to start planning at the current odometry.
  std::string smoother_name_;

  // Settings -- collision checker
  int number_of_points_to_collision_check_;
  int number_of_consecutive_blocked_points_for_collision_;

  // State -- robot state.
  mav_msgs::EigenOdometry odometry_;

  // State -- waypoints.
  mav_msgs::EigenTrajectoryPointVector waypoints_;
  int64_t current_waypoint_;

  // State -- current tracked path.
  mav_msgs::EigenTrajectoryPointVector path_queue_;
  size_t path_index_;
  // Super important: mutex for locking the path queues.
  std::recursive_mutex path_mutex_;
  std::recursive_mutex map_mutex_;
  RosSemaphore should_replan_;

  // State -- planning.
  int max_failures_;
  int num_failures_;

  // Map!
  voxblox::VoxbloxServer esdf_server_;

  // Planners -- yaw policy
  YawPolicy yaw_policy_;

  // Planners -- local planners.
  VoxbloxLocoPlanner loco_planner_;

  // Planners -- path smoothers.
  VelocityRampSmoother ramp_smoother_;
  PolynomialSmoother poly_smoother_;
  LocoSmoother loco_smoother_;

  // Intermediate goal selection, optionally in case of path-planning failures:
  GoalPointSelector goal_selector_;
  bool temporary_goal_;

  // Idle checker
  IdleChecker idle_checker_;

  // Logger
  bool should_save_logs_ = false;
  Logger logger_;
  ros::Time last_logged_odom_;
  mav_msgs::EigenTrajectoryPointVector current_logged_trajectory_;
  int number_of_trajectories = 0;
  double total_trajectory_length_;
};

}  // namespace mav_planning

#endif  // MAV_LOCAL_PLANNER_MAV_LOCAL_PLANNER_H_
