// Copyright 2020 eve autonomy inc. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License

#ifndef AUTOWARE_STATE_MACHINE__AUTOWARE_STATE_MACHINE_HPP_
#define AUTOWARE_STATE_MACHINE__AUTOWARE_STATE_MACHINE_HPP_

#include <shared_mutex>
#include <string>
#include <map>
#include <cmath>
#include <limits>
#include <utility>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "autoware_state_machine_msgs/msg/vehicle_button.hpp"
#include "tier4_system_msgs/msg/autoware_state.hpp"
#include "tier4_api_msgs/msg/awapi_autoware_status.hpp"
#include "tier4_api_msgs/msg/awapi_vehicle_status.hpp"
#include "tier4_api_utils/tier4_api_utils.hpp"
#include "tier4_external_api_msgs/srv/engage.hpp"
#include "tier4_external_api_msgs/srv/set_operator.hpp"
#include "tier4_planning_msgs/msg/stop_reason_array.hpp"
#include "tier4_vehicle_msgs/msg/turn_signal.hpp"
#include "tier4_vehicle_msgs/msg/control_mode.hpp"
#include "autoware_state_machine_msgs/msg/state_machine.hpp"
#include "autoware_state_machine_msgs/msg/state_sound_done.hpp"
#include "autoware_state_machine_msgs/msg/state_lock.hpp"
#include "go_interface_msgs/msg/vehicle_status.hpp"
#include "go_interface_msgs/msg/change_lock_flg.hpp"

namespace autoware_state_machine
{

enum class ChangeStateReturnItem
{
  ERROR = 0,
  NONE,
  TRANSITION,
};

class AutowareStateMachine : public rclcpp::Node
{
public:
  explicit AutowareStateMachine(const rclcpp::NodeOptions & options);
  ~AutowareStateMachine();

private:
  rclcpp::Node * node_;

  std::shared_mutex mtx_;

  // Callback group
  rclcpp::CallbackGroup::SharedPtr callback_group_service_;
  rclcpp::CallbackGroup::SharedPtr callback_group_subscription_;

  // Subscriber
  rclcpp::Subscription<tier4_api_msgs::msg::AwapiAutowareStatus>::SharedPtr
    sub_awapi_autoware_state_;
  rclcpp::Subscription<tier4_api_msgs::msg::AwapiVehicleStatus>::SharedPtr
    sub_awapi_vehicle_state_;
  rclcpp::Subscription<autoware_state_machine_msgs::msg::VehicleButton>::SharedPtr
    sub_calls_delivery_reservation_button_;
  rclcpp::Subscription<autoware_state_machine_msgs::msg::StateSoundDone>::SharedPtr
    sub_engage_sound_done_;
  rclcpp::Subscription<go_interface_msgs::msg::VehicleStatus>::SharedPtr
    sub_calls_vehicle_state_;
  void onAwapiAutowareState(
    const tier4_api_msgs::msg::AwapiAutowareStatus::ConstSharedPtr msg_ptr);
  void onAwapiVehicleState(
    const tier4_api_msgs::msg::AwapiVehicleStatus::ConstSharedPtr msg_ptr);
  void onCallsDeliveryReservationButton(
    const autoware_state_machine_msgs::msg::VehicleButton::ConstSharedPtr msg_ptr);
  void onStateSoundDone(
    const autoware_state_machine_msgs::msg::StateSoundDone::ConstSharedPtr msg_ptr);
  void onCallsVehicleState(
    const go_interface_msgs::msg::VehicleStatus::ConstSharedPtr msg_ptr);

  // Publisher
  rclcpp::Publisher<autoware_state_machine_msgs::msg::StateMachine>::SharedPtr pub_state_;
  rclcpp::Publisher<autoware_state_machine_msgs::msg::StateLock>::SharedPtr pub_delivery_reservation_state_;
  rclcpp::Publisher<go_interface_msgs::msg::ChangeLockFlg>::SharedPtr
    pub_calls_req_change_lock_;

  // Service
  rclcpp::Service<tier4_external_api_msgs::srv::Engage>::SharedPtr srv_engage_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_set_request_start_api_;
  void execEngageProcess(
    const tier4_external_api_msgs::srv::Engage::Request::SharedPtr request,
    const tier4_external_api_msgs::srv::Engage::Response::SharedPtr response);
  void setRequestStartAPI(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    const std_srvs::srv::Trigger::Response::SharedPtr response);

  // Client
  rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedPtr cli_engage_;
  rclcpp::Client<tier4_external_api_msgs::srv::SetOperator>::SharedPtr cli_set_operator_;

  // Timer
  void onTimer();
  rclcpp::TimerBase::SharedPtr timer_;

  // internal state
  uint16_t current_service_layer_state_;
  uint8_t current_control_layer_state_;

  uint16_t current_delivery_reservation_state_;

  builtin_interfaces::msg::Time pre_autoware_state_recv_time_;
  builtin_interfaces::msg::Time pre_vehicle_state_recv_time_;

  std::string cur_autoware_state_;
  std::string stop_reason_;
  int32_t cur_control_mode_;
  bool is_engage_requesting_;
  bool is_engage_accepted_;
  double velocity_;
  double engage_threshold_velocity_;
  double stop_threshold_velocity_;
  int32_t turn_signal_;
  bool flag_init_state_machine_;
  bool flag_arrived_state_machine_;
  double dist_to_stop_pose_max_th_;
  double dist_to_stop_pose_min_th_;
  double cur_dist_to_stop_pose_;

  double vehicle_state_overtime_;
  double autoware_state_overtime_;

  builtin_interfaces::msg::Time engage_wait_time_;
  double engage_wait_overtime_;

  builtin_interfaces::msg::Time delivery_reservation_verification_time_;
  double delivery_reservation_verification_overtime_;

  bool emergency_recover_mode_;

  bool flag_calls_vehicle_voice_;
  bool flag_calls_active_schedule_exists_;

  bool use_overridable_vehicle_;

  bool cur_emergency_holding_;

  typedef struct tuple
  {
    int32_t sec;
    uint32_t nsec;
    double done_overtime;
    bool done_flag;
  } SoundDoneTuple_t;
  std::map<uint16_t, SoundDoneTuple_t> sound_done_param_;

  // judge
  void ChangeState(void);
  ChangeStateReturnItem changeState4NodeAlive(void);
  ChangeStateReturnItem changeState4DuringWakeup(void);
  ChangeStateReturnItem changeState4ManualControl(void);
  ChangeStateReturnItem changeState4DuringReceiveRoute(void);
  ChangeStateReturnItem changeState4WaintingEngageInstruction(void);
  ChangeStateReturnItem changeState4InformEngage(void);
  ChangeStateReturnItem changeState4InstructEngage(void);
  ChangeStateReturnItem changeState4RunAndStop(void);
  ChangeStateReturnItem changeState4Restart(void);
  ChangeStateReturnItem changeState4DuringObstacleAvoidance(void);
  ChangeStateReturnItem changeState4Arrived(void);
  ChangeStateReturnItem changeState4Emergency(void);
  ChangeStateReturnItem changeControlLayerState(void);

  void setEngageProcess(bool request, bool accept);
  std::pair<bool, bool> getEngageProcess(void);
  bool waitingForEngageAccept(void);
};

}  // namespace autoware_state_machine
#endif  // AUTOWARE_STATE_MACHINE__AUTOWARE_STATE_MACHINE_HPP_
