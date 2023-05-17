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

#include <limits>
#include <memory>
#include <utility>
#include "autoware_state_machine/autoware_state_machine.hpp"

namespace autoware_state_machine
{

#define DEBUG_THROTTLE_TIME  5000  // ms

void AutowareStateMachine::onAwapiAutowareState(
  const tier4_api_msgs::msg::AwapiAutowareStatus::ConstSharedPtr msg_ptr)
{
  const auto current_time = this->now();
  constexpr double double_epsilon = std::numeric_limits<double>::epsilon();

  pre_autoware_state_recv_time_ = msg_ptr->header.stamp;
  cur_autoware_state_ = msg_ptr->autoware_state;
  cur_control_mode_ = msg_ptr->control_mode;
  cur_emergency_holding_ = msg_ptr->hazard_status.status.emergency_holding;

  /* Multiple "StopReasons" data are stored in the array. The storage order changes each time depending on the situation.
     Finally, narrow down to one "StopReason" data, which is the highest priority.
     The judgment process is as follows
       - Check "SurrondObstacleCheck" with the highest priority.
         In the "SurrondObstacleCheck", the distance information "dist_to_stop_pose" is not evaluated because the object is already approaching.
       - For other "StopReasons", the one with the smallest distance information "dist_to_stop_pose" is given priority.
     * Since it is a Float expression, the epsilon value is used because the matching comparison of 0 values is not valid. */
  stop_reason_ = "";
  cur_dist_to_stop_pose_ = dist_to_stop_pose_max_th_;
  if (msg_ptr->stop_reason.stop_reasons.size() != 0) {
    for (const auto & tmp_stop_reason : msg_ptr->stop_reason.stop_reasons) {
      if (tmp_stop_reason.reason ==
        tier4_planning_msgs::msg::StopReason::SURROUND_OBSTACLE_CHECK)
      {
        stop_reason_ = tier4_planning_msgs::msg::StopReason::SURROUND_OBSTACLE_CHECK;
        cur_dist_to_stop_pose_ = 0.0;
        break;
      }

      if (tmp_stop_reason.stop_factors.size() != 0) {
        for (const auto & tmp_stop_factor : tmp_stop_reason.stop_factors) {
          if (std::abs(tmp_stop_factor.dist_to_stop_pose) <= double_epsilon) {
            cur_dist_to_stop_pose_ = 0.0;
            stop_reason_ = tmp_stop_reason.reason;
          } else if (tmp_stop_factor.dist_to_stop_pose <= cur_dist_to_stop_pose_) {
            cur_dist_to_stop_pose_ = tmp_stop_factor.dist_to_stop_pose;
            stop_reason_ = tmp_stop_reason.reason;
          }
        }  // for(const auto & tmp_stop_factor : tmp_stop_reason.stop_factors)
      }  // if(tmp_stop_reason.stop_factors.size() != 0)
    }  // for(const auto & tmp_stop_reason : msg_ptr->stop_reason.stop_reasons)
  }  // if(msg_ptr->stop_reason.stop_reasons.size() != 0)

  ChangeState();
}

void AutowareStateMachine::onAwapiVehicleState(
  const tier4_api_msgs::msg::AwapiVehicleStatus::ConstSharedPtr msg_ptr)
{
  pre_vehicle_state_recv_time_ = msg_ptr->header.stamp;

  /* "velocity" is current velocity of self. "target_velocity" is the value self
      want to reach after acceleration / deceleration in the future */
  velocity_ = msg_ptr->velocity;
  turn_signal_ = msg_ptr->turn_signal;

  ChangeState();
}

void AutowareStateMachine::execEngageProcess(
  const tier4_external_api_msgs::srv::Engage::Request::SharedPtr request,
  const tier4_external_api_msgs::srv::Engage::Response::SharedPtr response)
{
  RCLCPP_DEBUG_THROTTLE(
    this->get_logger(),
    *this->get_clock(), DEBUG_THROTTLE_TIME,
    "[autoware_state_machine] Engage Request Is %d ",
    request->engage);

  if (current_control_layer_state_ == autoware_state_machine_msgs::msg::StateMachine::MANUAL) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), DEBUG_THROTTLE_TIME,
      "[autoware_state_machine] Engage Request Is Not Ready.");
    response->status = tier4_api_utils::response_error("It is not ready to engage.");
    return;
  }

  const auto is_engage_ready_state =
    (current_service_layer_state_ ==
    autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_ENGAGE_INSTRUCTION) ||
    (current_service_layer_state_ ==
    autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_CALL_PERMISSION);

  if (!is_engage_ready_state) {
    // Do not make error notification
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), DEBUG_THROTTLE_TIME,
      "[autoware_state_machine] Preceding Engage Request %d ",
      current_service_layer_state_);
    response->status = tier4_api_utils::response_error("It is not ready to engage.");
    return;
  }

  if (current_delivery_reservation_state_ ==
    autoware_state_machine_msgs::msg::StateLock::STATE_VERIFICATION)
  {
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(),
      *this->get_clock(), DEBUG_THROTTLE_TIME,
      "[autoware_state_machine] Under Verification of Lock Button ");
    response->status = tier4_api_utils::response_error("It is not ready to engage.");
    return;
  }

  // Provisional support
  // Set operator to AUTONOMOUS and set "/vehicle/engage" to True internally.
  auto operator_req =
    std::make_shared<tier4_external_api_msgs::srv::SetOperator::Request>();
  operator_req->mode.mode = tier4_external_api_msgs::msg::Operator::AUTONOMOUS;
  auto operator_future = cli_set_operator_->async_send_request(operator_req);
  if (!tier4_api_utils::is_success(operator_future.get()->status)) {
    response->status = tier4_api_utils::response_error("Operator set failed.");
    return;
  }

  bool is_success = waitingForEngageAccept();
  if (!is_success) {
    response->status = tier4_api_utils::response_error("It is not ready to engage.");
    return;
  }

  auto engage_future = cli_engage_->async_send_request(request);
  response->status = engage_future.get()->status;
}

void AutowareStateMachine::setRequestStartAPI(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  const std_srvs::srv::Trigger::Response::SharedPtr response
)
{
  /* The voice guidance at the time of engage is done in execEngageProcess function,
      so there is no need to do it there.
    Treats the state as ready to engage and returns true. */
  if (current_service_layer_state_ ==
    autoware_state_machine_msgs::msg::StateMachine::STATE_INSTRUCT_ENGAGE)
  {
    response->success = true;
    return;
  }

  if (current_control_layer_state_ == autoware_state_machine_msgs::msg::StateMachine::MANUAL) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), DEBUG_THROTTLE_TIME,
      "[autoware_state_machine] Start API Is Not Ready ");
    response->success = false;
    return;
  }

  const auto is_running_state =
    (current_service_layer_state_ >=
    autoware_state_machine_msgs::msg::StateMachine::STATE_RUNNING) &&
    (current_service_layer_state_ <
    autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL);

  if (!is_running_state) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), DEBUG_THROTTLE_TIME,
      "[autoware_state_machine] Start API Is Not Ready ");
    response->success = false;
    return;
  }

  const auto is_exception_state =
    (current_service_layer_state_ ==
    autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_ENGAGE) ||
    (current_service_layer_state_ ==
    autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_RESTART);

  // Remove exceptions that fall within the scope of RunningState.
  if (is_exception_state) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), DEBUG_THROTTLE_TIME,
      "[autoware_state_machine] Start API Is Not Ready ");
    response->success = false;
    return;
  }

  bool is_success = waitingForEngageAccept();
  response->success = is_success;
}

void AutowareStateMachine::onCallsDeliveryReservationButton(
  const autoware_state_machine_msgs::msg::VehicleButton::ConstSharedPtr msg_ptr)
{
  RCLCPP_DEBUG_THROTTLE(
    this->get_logger(),
    *this->get_clock(), DEBUG_THROTTLE_TIME,
    "[autoware_state_machine] CallsDeliveryReservationButton %d ",
    msg_ptr->data);

  if (flag_calls_active_schedule_exists_) {
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(),
      *this->get_clock(), DEBUG_THROTTLE_TIME,
      "[autoware_state_machine] Active schedule exists ");
    return;
  }

  // Do not accept processing during verification
  if (current_delivery_reservation_state_ ==
    autoware_state_machine_msgs::msg::StateLock::STATE_VERIFICATION)
  {
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(),
      *this->get_clock(), DEBUG_THROTTLE_TIME,
      "[autoware_state_machine] Under Verification ");
    return;
  }

  go_interface_msgs::msg::ChangeLockFlg change_lock;
  change_lock.stamp = this->now();
  /* If the lock status is OFF,
      set it to verification status and ask the web server if it can be turned ON. */
  if (current_delivery_reservation_state_ == autoware_state_machine_msgs::msg::StateLock::STATE_OFF) {
    delivery_reservation_verification_time_ = this->now();
    current_delivery_reservation_state_ = autoware_state_machine_msgs::msg::StateLock::STATE_VERIFICATION;
    change_lock.flg = true;

    // Set the LED to blinking.
    autoware_state_machine_msgs::msg::StateLock state_lock;
    state_lock.stamp = this->now();
    state_lock.state = current_delivery_reservation_state_;
    pub_delivery_reservation_state_->publish(state_lock);
  } else {
    // When the lock status is ON, ask the web server if it can be turned OFF.
    change_lock.flg = false;
  }
  pub_calls_req_change_lock_->publish(change_lock);
}

void AutowareStateMachine::onStateSoundDone(
  const autoware_state_machine_msgs::msg::StateSoundDone::ConstSharedPtr msg_ptr)
{
  if (msg_ptr->done == false) {
    // Do not make error notification
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), DEBUG_THROTTLE_TIME,
      "[autoware_state_machine] Sound Done False");
    return;
  }
  if (current_service_layer_state_ != msg_ptr->state) {
    // Do not make error notification
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), DEBUG_THROTTLE_TIME,
      "[autoware_state_machine] Sound Done Bad Timing "
      "service_layer_state: %u, control_layer_state %u",
      current_service_layer_state_,
      current_control_layer_state_);
    if (sound_done_param_.count(msg_ptr->state) != 0) {
      /* false safe */
      sound_done_param_[msg_ptr->state].done_flag = false;
    }
    return;
  }

  if (sound_done_param_.count(msg_ptr->state) != 0) {
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(),
      *this->get_clock(), DEBUG_THROTTLE_TIME,
      "[autoware_state_machine] Sound Done %d, %d ", msg_ptr->state,
      msg_ptr->done);
    sound_done_param_[msg_ptr->state].done_flag = true;
    ChangeState();
  }
}

void AutowareStateMachine::onCallsVehicleState(
  const go_interface_msgs::msg::VehicleStatus::ConstSharedPtr msg_ptr)
{
  bool change_lock_state = false;

  // lock state off or lock state verification
  if (current_delivery_reservation_state_ == autoware_state_machine_msgs::msg::StateLock::STATE_OFF ||
    (current_delivery_reservation_state_ == autoware_state_machine_msgs::msg::StateLock::STATE_VERIFICATION))
  {
    if (msg_ptr->lock_flg == true) {
      current_delivery_reservation_state_ = autoware_state_machine_msgs::msg::StateLock::STATE_ON;
      change_lock_state = true;
    }
  } else {  // lock state on
    if (msg_ptr->lock_flg == false) {
      current_delivery_reservation_state_ = autoware_state_machine_msgs::msg::StateLock::STATE_OFF;
      change_lock_state = true;
    }
  }

  if (change_lock_state) {
    autoware_state_machine_msgs::msg::StateLock pub;
    pub.stamp = this->now();
    pub.state = current_delivery_reservation_state_;
    pub_delivery_reservation_state_->publish(pub);
  }

  /* The voice flag only has meaning for the transition from False to True.
    When the voice flag is True,
      the transition from "STATE_WAITING_ENGAGE_INSTRUCTION" to "STATE_WAITING_CALL_PERMISSION" is possible.
    The voice flag is set to false once when the transition is made to "STATE_WAITING_ENGAGE_INSTRUCTION". */
  if (flag_calls_vehicle_voice_ != msg_ptr->voice_flg) {
    flag_calls_vehicle_voice_ = msg_ptr->voice_flg;
  }

  if (flag_calls_active_schedule_exists_ != msg_ptr->active_schedule_exists) {
    flag_calls_active_schedule_exists_ = msg_ptr->active_schedule_exists;
  }

  ChangeState();
}

void AutowareStateMachine::onTimer()
{
  const auto current_time = this->now();

  if ( (current_time - pre_vehicle_state_recv_time_).seconds() > vehicle_state_overtime_) {
    /* "/awapi/vehicle/get/status" topic that should have been sent regularly does not come */
    // Make error notification
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(),
      *this->get_clock(), DEBUG_THROTTLE_TIME,
      "[autoware_state_machine] Vehicle State Missing %f ",
      (current_time - pre_vehicle_state_recv_time_).seconds());
  }

  if ( (current_time - pre_autoware_state_recv_time_).seconds() > autoware_state_overtime_) {
    /* "/awapi/autoware/get/status" topic that should have been sent regularly does not come */
    // Make error notification
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(),
      *this->get_clock(), DEBUG_THROTTLE_TIME,
      "[autoware_state_machine] Autoware State Missing %f ",
      (current_time - pre_autoware_state_recv_time_).seconds());
  }

  if (current_delivery_reservation_state_ ==
    autoware_state_machine_msgs::msg::StateLock::STATE_VERIFICATION)
  {
    if ( (current_time - delivery_reservation_verification_time_).seconds() >
      delivery_reservation_verification_overtime_)
    {
      /* If the verification process continues for a long time,
          return the status to OFF. */
      current_delivery_reservation_state_ = autoware_state_machine_msgs::msg::StateLock::STATE_OFF;
      autoware_state_machine_msgs::msg::StateLock pub;
      pub.stamp = this->now();
      pub.state = current_delivery_reservation_state_;
      pub_delivery_reservation_state_->publish(pub);
    }
  }

  if (sound_done_param_.count(current_service_layer_state_) != 0) {
    /* FalseSafe : Waiting for the completion of audio playback for a long time */
    if (sound_done_param_[current_service_layer_state_].done_flag != true) {
      rclcpp::Time current_service_layer_state_start_time(
        sound_done_param_[current_service_layer_state_].sec,
        sound_done_param_[current_service_layer_state_].nsec,
        RCL_ROS_TIME);
      if ( (current_time - current_service_layer_state_start_time).seconds() >
        sound_done_param_[current_service_layer_state_].done_overtime)
      {
        autoware_state_machine_msgs::msg::StateMachine pub;

        // Do not make error notification
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(), DEBUG_THROTTLE_TIME,
          "[autoware_state_machine] Sound Donee Missing %d %f ",
          current_service_layer_state_,
          (current_time - current_service_layer_state_start_time).seconds());
/*
        Publish audio playback again
        sound_done_param_[current_service_layer_state_].sec = current_time.sec;
        sound_done_param_[current_service_layer_state_].nsec = current_time.nsec;
        pub.header.stamp = current_time;
        pub.state = current_service_layer_state_;
        pub_state_->publish(pub);
*/
      }
    }
  }

  /* Since there is a possibility that there is no regular Topic reception,
      call it with this timer */
  if ( (current_service_layer_state_ ==
    autoware_state_machine_msgs::msg::StateMachine::STATE_UNDEFINED) ||
    (current_service_layer_state_ ==
    autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE) ||
    (current_service_layer_state_ ==
    autoware_state_machine_msgs::msg::StateMachine::STATE_EMERGENCY_STOP) )
  {
    ChangeState();
  }

  /* Currently, there is no way to directly check the AUTO / MANUAL status on the vehicle side.
     Even if the vehicle switch is turned off for automatic driving, the engage request is accepted.
     However, since the "engage service" in the MANUAL state is ignored, the vehicle does not actually depart.
     In the future, the state will be returned after a timeout so that the engage request after the vehicle switch is turned on for automatic driving can be accepted. */
  if (current_service_layer_state_ ==
    autoware_state_machine_msgs::msg::StateMachine::STATE_INSTRUCT_ENGAGE)
  {
    if ( (current_time - engage_wait_time_).seconds() > engage_wait_overtime_) {
      if (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::WAITING_FOR_ENGAGE) {
        cur_autoware_state_ = tier4_system_msgs::msg::AutowareState::PLANNING;
        ChangeState();
      }
    }
  }

  const auto is_now_engage_playing =
    (current_service_layer_state_ ==
    autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_ENGAGE) ||
    (current_service_layer_state_ ==
    autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_RESTART);

  /* If engage is requested and is not yet playing, then execute ChangeState function. */
  auto [is_request, _] = getEngageProcess();
  if (is_request && !is_now_engage_playing) {
    ChangeState();
  }
}


void AutowareStateMachine::ChangeState(void)
{
  ChangeStateReturnItem service_layer_ret;
  ChangeStateReturnItem control_layer_ret;
  builtin_interfaces::msg::Time wait_done_time = this->now();

  switch (current_service_layer_state_) {
    /* -------------------------------------------------------------- */
    case autoware_state_machine_msgs::msg::StateMachine::STATE_UNDEFINED:
      /* At the first Autoware startup after the power is turned on. Force a transition. */
      // Normal  Pattern
      flag_init_state_machine_ = false;
      sound_done_param_[
        autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE].done_flag =
        false;
      sound_done_param_[
        autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE].sec =
        wait_done_time.sec;
      sound_done_param_[
        autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE].nsec =
        wait_done_time.nanosec;
      current_service_layer_state_ =
        autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE;
      service_layer_ret = ChangeStateReturnItem::TRANSITION;
      break;

    /* -------------------------------------------------------------- */
    case autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE:
      service_layer_ret = changeState4NodeAlive();
      break;

    /* -------------------------------------------------------------- */
    case autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_WAKEUP:
      service_layer_ret = changeState4DuringWakeup();
      break;

    /* -------------------------------------------------------------- */
    case autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_CLOSE:
      // Normal  Pattern
      flag_init_state_machine_ = false;
      service_layer_ret = ChangeStateReturnItem::NONE;
      break;

    case autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_RECEIVE_ROUTE:
      service_layer_ret = changeState4DuringReceiveRoute();
      break;

    /* -------------------------------------------------------------- */
    case autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_ENGAGE_INSTRUCTION:
    case autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_CALL_PERMISSION:
      service_layer_ret = changeState4WaintingEngageInstruction();
      break;

    /* -------------------------------------------------------------- */
    case autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_ENGAGE:
      service_layer_ret = changeState4InformEngage();
      break;

    /* -------------------------------------------------------------- */
    case autoware_state_machine_msgs::msg::StateMachine::STATE_INSTRUCT_ENGAGE:
      service_layer_ret = changeState4InstructEngage();
      break;

    /* -------------------------------------------------------------- */
    case autoware_state_machine_msgs::msg::StateMachine::STATE_RUNNING:
    case autoware_state_machine_msgs::msg::StateMachine::STATE_RUNNING_TOWARD_STOP_LINE:
    case autoware_state_machine_msgs::msg::StateMachine::STATE_RUNNING_TOWARD_OBSTACLE:
    case autoware_state_machine_msgs::msg::StateMachine::STATE_TURNING_LEFT:
    case autoware_state_machine_msgs::msg::StateMachine::STATE_TURNING_RIGHT:
    case autoware_state_machine_msgs::msg::StateMachine::STATE_STOP_DUETO_TRAFFIC_CONDITION:
    case autoware_state_machine_msgs::msg::StateMachine::STATE_STOP_DUETO_APPROACHING_OBSTACLE:
    case autoware_state_machine_msgs::msg::StateMachine::STATE_STOP_DUETO_SURROUNDING_PROXIMITY:
    case autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_RESTART:
      service_layer_ret = changeState4RunAndStop();
      break;

    /* -------------------------------------------------------------- */
    case autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_OBSTACLE_AVOIDANCE:
      service_layer_ret = changeState4DuringObstacleAvoidance();
      break;

    /* -------------------------------------------------------------- */
    case autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL:
      service_layer_ret = changeState4Arrived();
      break;

    /* -------------------------------------------------------------- */
    case autoware_state_machine_msgs::msg::StateMachine::STATE_EMERGENCY_STOP:
      service_layer_ret = changeState4Emergency();
      break;

    /* -------------------------------------------------------------- */
    default:
      service_layer_ret = ChangeStateReturnItem::ERROR;
      break;
  }

  control_layer_ret = changeControlLayerState();

  if (service_layer_ret == ChangeStateReturnItem::TRANSITION ||
    control_layer_ret == ChangeStateReturnItem::TRANSITION)
  {
    autoware_state_machine_msgs::msg::StateMachine pub;

    RCLCPP_INFO(
      this->get_logger(),
      "[autoware_state_machine] Change service_layer_state %d, control_layer_state %d",
      current_service_layer_state_,
      current_control_layer_state_);
    pub.stamp = this->now();
    pub.service_layer_state = current_service_layer_state_;
    pub.control_layer_state = current_control_layer_state_;
    pub_state_->publish(pub);
  } else if (service_layer_ret == ChangeStateReturnItem::ERROR) {
    // Make error notification
    RCLCPP_ERROR_STREAM(
      this->get_logger(),
      "[autoware_state_machine] Detect Error : " <<
        current_service_layer_state_ << " : " << cur_autoware_state_);
  }
}

ChangeStateReturnItem AutowareStateMachine::changeState4NodeAlive(void)
{
  // No transition  Pattern
  if (flag_init_state_machine_ == false) {
    if (sound_done_param_[
        autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE].done_flag ==
      false)
    {
      return ChangeStateReturnItem::NONE;
    }
    /* Audio playback completed */
    // Normal  Pattern
    sound_done_param_[
      autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE].done_flag =
      false;
    flag_init_state_machine_ = true;
  }

  // Normal  Pattern
  if (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::INITIALIZING_VEHICLE) {
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_WAKEUP;
    return ChangeStateReturnItem::TRANSITION;
  }

  // Tolerance Pattern
  if ( (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::WAITING_FOR_ROUTE) ||
    (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::PLANNING) )
  {
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_RECEIVE_ROUTE;
    return ChangeStateReturnItem::TRANSITION;
  }
  if (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::WAITING_FOR_ENGAGE) {
    flag_calls_vehicle_voice_ = false;
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_ENGAGE_INSTRUCTION;

    return ChangeStateReturnItem::TRANSITION;
  }
  if (cur_emergency_holding_ == true) {
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_EMERGENCY_STOP;
    return ChangeStateReturnItem::TRANSITION;
  }

  // Error Pattern
  return ChangeStateReturnItem::ERROR;
}

ChangeStateReturnItem AutowareStateMachine::changeState4DuringWakeup(void)
{
  // No transition  Pattern
  if (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::INITIALIZING_VEHICLE) {
    return ChangeStateReturnItem::NONE;
  }

  // Force  Pattern
  if (cur_emergency_holding_ == true) {
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_EMERGENCY_STOP;
    return ChangeStateReturnItem::TRANSITION;
  }

  // Normal  Pattern
  if ( (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::WAITING_FOR_ROUTE) ||
    (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::PLANNING) )
  {
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_RECEIVE_ROUTE;
    return ChangeStateReturnItem::TRANSITION;
  }

  // Tolerance Pattern
  if (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::WAITING_FOR_ENGAGE) {
    flag_calls_vehicle_voice_ = false;
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_ENGAGE_INSTRUCTION;

    return ChangeStateReturnItem::TRANSITION;
  }

  // Error Pattern
  return ChangeStateReturnItem::ERROR;
}

ChangeStateReturnItem AutowareStateMachine::changeState4DuringReceiveRoute(void)
{
  // Force  Pattern
  if (cur_emergency_holding_ == true) {
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_EMERGENCY_STOP;
    return ChangeStateReturnItem::TRANSITION;
  }

  // No transition  Pattern
  if ( (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::WAITING_FOR_ROUTE) ||
    (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::PLANNING) )
  {
    return ChangeStateReturnItem::NONE;
  }

  // Normal  Pattern
  if (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::WAITING_FOR_ENGAGE) {
    flag_calls_vehicle_voice_ = false;
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_ENGAGE_INSTRUCTION;
    return ChangeStateReturnItem::TRANSITION;
  }

  // Tolerance Pattern
  if (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::INITIALIZING_VEHICLE) {
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_WAKEUP;
    return ChangeStateReturnItem::TRANSITION;
  }

  // Error Pattern
  return ChangeStateReturnItem::ERROR;
}

ChangeStateReturnItem AutowareStateMachine::changeState4WaintingEngageInstruction(void)
{
  // Force  Pattern
  if (cur_emergency_holding_ == true) {
    setEngageProcess(false, false);
    flag_calls_vehicle_voice_ = false;
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_EMERGENCY_STOP;
    return ChangeStateReturnItem::TRANSITION;
  }

  if ( (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::WAITING_FOR_ENGAGE) ||
    (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::DRIVING) )
  {
    builtin_interfaces::msg::Time wait_done_time = this->now();

    if (current_service_layer_state_ ==
      autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_ENGAGE_INSTRUCTION)
    {
      if (flag_calls_vehicle_voice_ &&
        (current_delivery_reservation_state_ == autoware_state_machine_msgs::msg::StateLock::STATE_ON))
      {
        /* Once move to "STATE_WAITING_CALL_PERMISSION",
            can't go back to "STATE_WAITING_ENGAGE_INSTRUCTION". */
        current_service_layer_state_ =
          autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_CALL_PERMISSION;
        return ChangeStateReturnItem::TRANSITION;
      }
    }

    // No transition  Pattern
    auto [is_request, _] = getEngageProcess();
    if ((is_request == false) ||
      (current_delivery_reservation_state_ ==
      autoware_state_machine_msgs::msg::StateLock::STATE_VERIFICATION))
    {
      return ChangeStateReturnItem::NONE;
    }

    // Normal  Pattern
    flag_calls_vehicle_voice_ = false;
    sound_done_param_[
      autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_ENGAGE].done_flag =
      false;
    sound_done_param_[
      autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_ENGAGE].sec =
      wait_done_time.sec;
    sound_done_param_[
      autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_ENGAGE].nsec =
      wait_done_time.nanosec;
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_ENGAGE;
    return ChangeStateReturnItem::TRANSITION;
  }

  // Tolerance Pattern
  setEngageProcess(false, false);
  if ( (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::WAITING_FOR_ROUTE) ||
    (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::PLANNING) )
  {
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_RECEIVE_ROUTE;
    return ChangeStateReturnItem::TRANSITION;
  }

  if (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::ARRIVAL_GOAL) {
    builtin_interfaces::msg::Time wait_done_time = this->now();

    flag_arrived_state_machine_ = true;
    sound_done_param_[
      autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL].done_flag =
      false;
    sound_done_param_[
      autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL].sec =
      wait_done_time.sec;
    sound_done_param_[
      autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL].nsec =
      wait_done_time.nanosec;
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL;
    return ChangeStateReturnItem::TRANSITION;
  }

  if (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::INITIALIZING_VEHICLE) {
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_WAKEUP;
    return ChangeStateReturnItem::TRANSITION;
  }

  // Error Pattern
  return ChangeStateReturnItem::ERROR;
}

ChangeStateReturnItem AutowareStateMachine::changeState4InformEngage(void)
{
  // Force  Pattern
  if (cur_emergency_holding_ == true) {
    setEngageProcess(false, false);
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_EMERGENCY_STOP;
    return ChangeStateReturnItem::TRANSITION;
  }

  if ( (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::WAITING_FOR_ENGAGE) ||
    (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::DRIVING) )
  {
    // No transition  Pattern
    if (sound_done_param_[
        autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_ENGAGE].done_flag ==
      false)
    {
      return ChangeStateReturnItem::NONE;
    }
    /* Audio playback completed */
    // Normal  Pattern
    sound_done_param_[
      autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_ENGAGE].done_flag =
      false;

    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_INSTRUCT_ENGAGE;
    {
      setEngageProcess(false, true);
      engage_wait_time_ = this->now();
    }
    return ChangeStateReturnItem::TRANSITION;
  }

  // Tolerance Pattern
  setEngageProcess(false, false);
  if ( (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::WAITING_FOR_ROUTE) ||
    (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::PLANNING) )
  {
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_RECEIVE_ROUTE;
    return ChangeStateReturnItem::TRANSITION;
  }
  if (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::ARRIVAL_GOAL) {
    builtin_interfaces::msg::Time wait_done_time = this->now();

    flag_arrived_state_machine_ = true;
    sound_done_param_[
      autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL].done_flag =
      false;
    sound_done_param_[
      autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL].sec =
      wait_done_time.sec;
    sound_done_param_[
      autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL].nsec =
      wait_done_time.nanosec;
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL;
    return ChangeStateReturnItem::TRANSITION;
  }
  if (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::INITIALIZING_VEHICLE) {
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_WAKEUP;
    return ChangeStateReturnItem::TRANSITION;
  }

  // Error Pattern
  return ChangeStateReturnItem::ERROR;
}

ChangeStateReturnItem AutowareStateMachine::changeState4InstructEngage(void)
{
  // Force  Pattern
  if (cur_emergency_holding_ == true) {
    setEngageProcess(false, false);
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_EMERGENCY_STOP;
    return ChangeStateReturnItem::TRANSITION;
  }

  // No transition  Pattern
  if (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::WAITING_FOR_ENGAGE) {
    return ChangeStateReturnItem::NONE;
  }

  // Normal  Pattern
  if (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::ARRIVAL_GOAL) {
    setEngageProcess(false, false);
    builtin_interfaces::msg::Time wait_done_time = this->now();

    flag_arrived_state_machine_ = true;
    sound_done_param_[
      autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL].done_flag =
      false;
    sound_done_param_[
      autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL].sec =
      wait_done_time.sec;
    sound_done_param_[
      autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL].nsec =
      wait_done_time.nanosec;
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL;
    return ChangeStateReturnItem::TRANSITION;
  }

  // Tolerance Pattern
  if (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::INITIALIZING_VEHICLE) {
    setEngageProcess(false, false);
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_WAKEUP;
    return ChangeStateReturnItem::TRANSITION;
  }
  if ( (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::WAITING_FOR_ROUTE) ||
    (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::PLANNING) )
  {
    setEngageProcess(false, false);
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_RECEIVE_ROUTE;
    return ChangeStateReturnItem::TRANSITION;
  }

  if (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::DRIVING) {
    /* Since it is the first engage from approving the engage request,
       the judgment of whether or not the state has fallen into the pause state in changeState4RunAndStop () is skipped. */
    current_service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_RUNNING;
    return ChangeStateReturnItem::TRANSITION;
  }

  // Error Pattern
  return ChangeStateReturnItem::ERROR;
}

ChangeStateReturnItem AutowareStateMachine::changeState4RunAndStop(void)
{
  builtin_interfaces::msg::Time wait_done_time = this->now();

  // Force  Pattern
  if (cur_emergency_holding_ == true) {
    setEngageProcess(false, false);
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_EMERGENCY_STOP;
    return ChangeStateReturnItem::TRANSITION;
  }

  // Normal  Pattern
  if (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::DRIVING) {
    /* If the voice guidance due to restart after pausing is playing, confirm the completion. */
    /* Do not transition to another state so as not to overwhelm the voice guidance of restart. */
    if (current_service_layer_state_ ==
      autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_RESTART)
    {
      if (sound_done_param_[
          autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_RESTART].done_flag ==
        false)
      {
        return ChangeStateReturnItem::NONE;
      }
      /* Audio playback completed */
      /* Since the voice guidance for restarting is completed, the judgment of whether or not
          the state has fallen into the pause state after this is skipped. */
      setEngageProcess(false, true);
      sound_done_param_[
        autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_RESTART].done_flag =
        false;
      current_service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_RUNNING;
      return ChangeStateReturnItem::TRANSITION;
    }

    /* If restart is requested, voice guidance will be played. */
    auto [is_request, is_accept] = getEngageProcess();
    if (is_request && !is_accept) {
      sound_done_param_[
        autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_RESTART].done_flag =
        false;
      sound_done_param_[
        autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_RESTART].sec =
        wait_done_time.sec;
      sound_done_param_[
        autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_RESTART].nsec =
        wait_done_time.nanosec;
      current_service_layer_state_ =
        autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_RESTART;
      return ChangeStateReturnItem::TRANSITION;
    }

    if (stop_reason_ == "") {
      const auto isStopState =
        (current_service_layer_state_ ==
        autoware_state_machine_msgs::msg::StateMachine::STATE_STOP_DUETO_APPROACHING_OBSTACLE) ||
        (current_service_layer_state_ ==
        autoware_state_machine_msgs::msg::StateMachine::STATE_STOP_DUETO_SURROUNDING_PROXIMITY) ||
        (current_service_layer_state_ ==
        autoware_state_machine_msgs::msg::StateMachine::STATE_STOP_DUETO_TRAFFIC_CONDITION);

      /* Suspension without reason is treated as STATE_STOP_DUETO_TRAFFIC_CONDITION.
         If there is no request to restart vehicle and vehicle start running,
          it will exceptionally transition to STATE_RUNNING, STATE_TURNING_LEFT, or STATE_TURNING_RIGHT. */
      if (isStopState && (velocity_ < engage_threshold_velocity_)) {
        if (current_service_layer_state_ ==
          autoware_state_machine_msgs::msg::StateMachine::STATE_STOP_DUETO_TRAFFIC_CONDITION)
        {
          return ChangeStateReturnItem::NONE;
        } else {
          current_service_layer_state_ =
            autoware_state_machine_msgs::msg::StateMachine::STATE_STOP_DUETO_TRAFFIC_CONDITION;
          return ChangeStateReturnItem::TRANSITION;
        }
      }
      if (turn_signal_ == tier4_vehicle_msgs::msg::TurnSignal::LEFT) {
        if (autoware_state_machine_msgs::msg::StateMachine::STATE_TURNING_LEFT !=
          current_service_layer_state_)
        {
          current_service_layer_state_ =
            autoware_state_machine_msgs::msg::StateMachine::STATE_TURNING_LEFT;
          return ChangeStateReturnItem::TRANSITION;
        }
        return ChangeStateReturnItem::NONE;
      }
      if (turn_signal_ == tier4_vehicle_msgs::msg::TurnSignal::RIGHT) {
        if (autoware_state_machine_msgs::msg::StateMachine::STATE_TURNING_RIGHT !=
          current_service_layer_state_)
        {
          current_service_layer_state_ =
            autoware_state_machine_msgs::msg::StateMachine::STATE_TURNING_RIGHT;
          return ChangeStateReturnItem::TRANSITION;
        }
        return ChangeStateReturnItem::NONE;
      }

      if (autoware_state_machine_msgs::msg::StateMachine::STATE_RUNNING !=
        current_service_layer_state_)
      {
        current_service_layer_state_ =
          autoware_state_machine_msgs::msg::StateMachine::STATE_RUNNING;
        return ChangeStateReturnItem::TRANSITION;
      }
      return ChangeStateReturnItem::NONE;
    }

    if (stop_reason_ == tier4_planning_msgs::msg::StopReason::SURROUND_OBSTACLE_CHECK) {
      /* Transition to the state where voice guidance is played for SourroundProximity.
          Do not check the distance because it is already close. */
      if (autoware_state_machine_msgs::msg::StateMachine::STATE_STOP_DUETO_SURROUNDING_PROXIMITY !=
        current_service_layer_state_)
      {
        RCLCPP_DEBUG_STREAM_THROTTLE(
          this->get_logger(),
          *this->get_clock(), DEBUG_THROTTLE_TIME,
          "[autoware_state_machine] StopReason : " << stop_reason_ << " : " << velocity_);
        current_service_layer_state_ =
          autoware_state_machine_msgs::msg::StateMachine::STATE_STOP_DUETO_SURROUNDING_PROXIMITY;
        return ChangeStateReturnItem::TRANSITION;
      }
      return ChangeStateReturnItem::NONE;
    } else if ( (stop_reason_ == tier4_planning_msgs::msg::StopReason::OBSTACLE_STOP) ||
      (stop_reason_ == tier4_planning_msgs::msg::StopReason::DETECTION_AREA) ||
      (stop_reason_ == tier4_planning_msgs::msg::StopReason::CROSSWALK) )
    {
      /* Transition to a state in which voice guidance is played for obstacle detection. */
      /* Make sure the own vehicle is close enough to the target. */
      if (cur_dist_to_stop_pose_ <= dist_to_stop_pose_min_th_) {
        if (autoware_state_machine_msgs::msg::StateMachine::STATE_STOP_DUETO_APPROACHING_OBSTACLE !=
          current_service_layer_state_)
        {
          RCLCPP_DEBUG_STREAM_THROTTLE(
            this->get_logger(),
            *this->get_clock(), DEBUG_THROTTLE_TIME,
            "[autoware_state_machine] StopReason : " << stop_reason_ << " : " << velocity_);
          current_service_layer_state_ =
            autoware_state_machine_msgs::msg::StateMachine::STATE_STOP_DUETO_APPROACHING_OBSTACLE;
          return ChangeStateReturnItem::TRANSITION;
        }
        return ChangeStateReturnItem::NONE;
      } else {
        if (autoware_state_machine_msgs::msg::StateMachine::STATE_RUNNING_TOWARD_OBSTACLE !=
          current_service_layer_state_)
        {
          RCLCPP_DEBUG_STREAM_THROTTLE(
            this->get_logger(),
            *this->get_clock(), DEBUG_THROTTLE_TIME,
            "[autoware_state_machine] StopReason : " << stop_reason_ << " : " << velocity_);
          current_service_layer_state_ =
            autoware_state_machine_msgs::msg::StateMachine::STATE_RUNNING_TOWARD_OBSTACLE;
          return ChangeStateReturnItem::TRANSITION;
        }
        return ChangeStateReturnItem::NONE;
      }

      // Error Pattern
      return ChangeStateReturnItem::ERROR;
    } else {
      // other conditions go into stop mode
      /* Since it is a StopReason that is not an obstacle,
          the transition to a state where voice guidance is not played is made. */
      /* Make sure the own vehicle is close enough to the target. */
      if (cur_dist_to_stop_pose_ <= dist_to_stop_pose_min_th_) {
        if (autoware_state_machine_msgs::msg::StateMachine::STATE_STOP_DUETO_TRAFFIC_CONDITION !=
          current_service_layer_state_)
        {
          RCLCPP_DEBUG_STREAM_THROTTLE(
            this->get_logger(),
            *this->get_clock(), DEBUG_THROTTLE_TIME,
            "[autoware_state_machine] StopReason : " << stop_reason_ << " : " << velocity_);
          current_service_layer_state_ =
            autoware_state_machine_msgs::msg::StateMachine::STATE_STOP_DUETO_TRAFFIC_CONDITION;
          return ChangeStateReturnItem::TRANSITION;
        }
        return ChangeStateReturnItem::NONE;
      } else {
        if (autoware_state_machine_msgs::msg::StateMachine::STATE_RUNNING_TOWARD_STOP_LINE !=
          current_service_layer_state_)
        {
          RCLCPP_DEBUG_STREAM_THROTTLE(
            this->get_logger(),
            *this->get_clock(), DEBUG_THROTTLE_TIME,
            "[autoware_state_machine] StopReason : " << stop_reason_ << " : " << velocity_);
          current_service_layer_state_ =
            autoware_state_machine_msgs::msg::StateMachine::STATE_RUNNING_TOWARD_STOP_LINE;
          return ChangeStateReturnItem::TRANSITION;
        }
        return ChangeStateReturnItem::NONE;
      }

      // Error Pattern
      return ChangeStateReturnItem::ERROR;
    }

    // Error Pattern
    return ChangeStateReturnItem::ERROR;
  }

  setEngageProcess(false, false);
  if (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::ARRIVAL_GOAL) {
    flag_arrived_state_machine_ = true;
    /* Play voice guidance for arrival at the goal */
    sound_done_param_[
      autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL].done_flag =
      false;
    sound_done_param_[
      autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL].sec =
      wait_done_time.sec;
    sound_done_param_[
      autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL].nsec =
      wait_done_time.nanosec;
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL;
    return ChangeStateReturnItem::TRANSITION;
  }

  // Tolerance Pattern
  if (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::INITIALIZING_VEHICLE) {
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_WAKEUP;
    return ChangeStateReturnItem::TRANSITION;
  }
  if ( (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::WAITING_FOR_ROUTE) ||
    (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::PLANNING) )
  {
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_RECEIVE_ROUTE;
    return ChangeStateReturnItem::TRANSITION;
  }
  if (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::WAITING_FOR_ENGAGE) {
    flag_calls_vehicle_voice_ = false;
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_ENGAGE_INSTRUCTION;

    return ChangeStateReturnItem::TRANSITION;
  }

  // Error Pattern
  return ChangeStateReturnItem::ERROR;
}

ChangeStateReturnItem AutowareStateMachine::changeState4DuringObstacleAvoidance(void)
{
  /* Since the Autoware side is not supported,
      the state transition of obstacle avoidance is provisionally implemented. */

  // Force  Pattern
  if (cur_emergency_holding_ == true) {
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_EMERGENCY_STOP;
    return ChangeStateReturnItem::TRANSITION;
  }

  // Tolerance Pattern
  if (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::INITIALIZING_VEHICLE) {
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_WAKEUP;
    return ChangeStateReturnItem::TRANSITION;
  }
  if ( (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::WAITING_FOR_ROUTE) ||
    (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::PLANNING) )
  {
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_RECEIVE_ROUTE;
    return ChangeStateReturnItem::TRANSITION;
  }

  // Error Pattern
  return ChangeStateReturnItem::ERROR;
}

ChangeStateReturnItem AutowareStateMachine::changeState4Arrived(void)
{
  // Force  Pattern
  if (cur_emergency_holding_ == true) {
    flag_arrived_state_machine_ = false;
    sound_done_param_[
      autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL].done_flag =
      false;
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_EMERGENCY_STOP;
    return ChangeStateReturnItem::TRANSITION;
  }

  /* Do not transition to another state until the arrival voice guidance is completed */
  if (flag_arrived_state_machine_ == true) {
    if (sound_done_param_[
        autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL].done_flag ==
      false)
    {
      return ChangeStateReturnItem::NONE;
    }
    /* Audio playback completed */
    flag_arrived_state_machine_ = false;
    sound_done_param_[
      autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL].done_flag =
      false;
  }

  // No transition  Pattern
  if (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::ARRIVAL_GOAL) {
    return ChangeStateReturnItem::NONE;
  }

  // Normal  Pattern
  if ( (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::WAITING_FOR_ROUTE) ||
    (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::PLANNING) )
  {
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_RECEIVE_ROUTE;
    return ChangeStateReturnItem::TRANSITION;
  }

  // Tolerance Pattern
  if (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::INITIALIZING_VEHICLE) {
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_WAKEUP;
    return ChangeStateReturnItem::TRANSITION;
  }
  if (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::WAITING_FOR_ENGAGE) {
    flag_calls_vehicle_voice_ = false;
    current_service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_ENGAGE_INSTRUCTION;

    return ChangeStateReturnItem::TRANSITION;
  }

  // Error Pattern
  return ChangeStateReturnItem::ERROR;
}

ChangeStateReturnItem AutowareStateMachine::changeState4Emergency(void)
{
  // No transition  Pattern
  if (cur_emergency_holding_ == true) {
    return ChangeStateReturnItem::NONE;
  }

  // Normal  Pattern
  if (emergency_recover_mode_ == true) {
    /* There is no recovery pattern from EM in normal specifications.
       For debug, consider the case of manually recovering from EM after turning off the vehicle side switch to automatic driving. */
    if (current_control_layer_state_ == autoware_state_machine_msgs::msg::StateMachine::MANUAL) {
      if (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::INITIALIZING_VEHICLE) {
        current_service_layer_state_ =
          autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_WAKEUP;
        return ChangeStateReturnItem::TRANSITION;
      }

      if ((cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::WAITING_FOR_ROUTE) ||
        (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::PLANNING))
      {
        current_service_layer_state_ =
          autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_RECEIVE_ROUTE;
        return ChangeStateReturnItem::TRANSITION;
      }

      if ((cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::WAITING_FOR_ENGAGE) ||
        (cur_autoware_state_ == tier4_system_msgs::msg::AutowareState::DRIVING))
      {
        flag_calls_vehicle_voice_ = false;
        current_service_layer_state_ =
          autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_ENGAGE_INSTRUCTION;
        return ChangeStateReturnItem::TRANSITION;
      }
    }
  }

  return ChangeStateReturnItem::NONE;
}

ChangeStateReturnItem AutowareStateMachine::changeControlLayerState(void)
{
  uint16_t control_layer_state;
  auto is_before_driving =
    (current_service_layer_state_ >=
    autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_WAKEUP) &&
    (current_service_layer_state_ < autoware_state_machine_msgs::msg::StateMachine::STATE_RUNNING);

  if (use_overridable_vehicle_ && is_before_driving) {
    if (velocity_ >= stop_threshold_velocity_) {
      control_layer_state = autoware_state_machine_msgs::msg::StateMachine::MANUAL;
    } else {
      control_layer_state = autoware_state_machine_msgs::msg::StateMachine::AUTO;
    }
  } else {
    if (cur_control_mode_ == tier4_vehicle_msgs::msg::ControlMode::MANUAL) {
      control_layer_state = autoware_state_machine_msgs::msg::StateMachine::MANUAL;
    } else {
      control_layer_state = autoware_state_machine_msgs::msg::StateMachine::AUTO;
    }
  }

  if (current_control_layer_state_ != control_layer_state) {
    current_control_layer_state_ = control_layer_state;
    return ChangeStateReturnItem::TRANSITION;
  }

  return ChangeStateReturnItem::NONE;
}

void AutowareStateMachine::setEngageProcess(bool request, bool accept)
{
  /* In multi-threaded system, there is a possibility of simultaneous accesses from different threads,
      so exclusion control is performed.
     Set request to "true" when we want the audio to play when vehicle departs and restarts.
     Set accept to "true" when the audio playback is complete.
     If you want to suspend waiting for playback to complete for some reason,
      set both values to "false". */
  std::lock_guard<std::shared_mutex> lock(mtx_);
  is_engage_requesting_ = request;
  is_engage_accepted_ = accept;
}

std::pair<bool, bool> AutowareStateMachine::getEngageProcess()
{
  /* In multi-threaded system, there is a possibility of simultaneous accesses from different threads,
      so exclusion control is performed.
     Check both values to determine if the playback is played, interrupted, or completed. */
  std::pair<bool, bool> value;
  {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    value = std::make_pair(is_engage_requesting_, is_engage_accepted_);
  }
  return value;
}

bool AutowareStateMachine::waitingForEngageAccept()
{
  setEngageProcess(true, false);

  rclcpp::Rate rate(10);
  while (rclcpp::ok()) {
    auto [is_request, is_accept] = getEngageProcess();
    if (is_accept) {
      break;
    }
    if (!is_request) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(), DEBUG_THROTTLE_TIME,
        "[autoware_state_machine] Engage Request Interruption ");
      return false;
    }
    rate.sleep();
  }
  setEngageProcess(false, false);
  return true;
}

AutowareStateMachine::AutowareStateMachine(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("autoware_state_machine", options)
{
  double update_rate;

  // Init Parameter
  flag_init_state_machine_ = false;
  flag_arrived_state_machine_ = false;
  current_service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_UNDEFINED;
  current_control_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::MANUAL;
  current_delivery_reservation_state_ = autoware_state_machine_msgs::msg::StateLock::STATE_OFF;
  cur_autoware_state_ = "";
  stop_reason_ = "";
  is_engage_requesting_ = false;
  is_engage_accepted_ = false;
  velocity_ = 0.0;
  turn_signal_ = tier4_vehicle_msgs::msg::TurnSignal::HAZARD;
  pre_autoware_state_recv_time_ = this->now();
  pre_vehicle_state_recv_time_ = this->now();
  engage_wait_time_ = this->now();
  delivery_reservation_verification_time_ = this->now();
  cur_dist_to_stop_pose_ = 0.0;
  flag_calls_vehicle_voice_ = false;
  flag_calls_active_schedule_exists_ = true;
  cur_emergency_holding_ = false;

  builtin_interfaces::msg::Time wait_done_time = this->now();
  sound_done_param_[
    autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_ENGAGE] =
  {wait_done_time.sec, wait_done_time.nanosec, 10.0, false};
  sound_done_param_[
    autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_RESTART] =
  {wait_done_time.sec, wait_done_time.nanosec, 10.0, false};
  sound_done_param_[
    autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE] =
  {wait_done_time.sec, wait_done_time.nanosec, 10.0, false};
  sound_done_param_[
    autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL] =
  {wait_done_time.sec, wait_done_time.nanosec, 10.0, false};

  // Adjustment Parameter
  update_rate = 1.0;
  engage_threshold_velocity_ = 0.0278 * 3;  // 0.3[km/h]=0.0278 * 3[m/s]
  stop_threshold_velocity_ = 0.0278 * 3;  // 0.3[km/h]=0.0278 * 3[m/s]
  vehicle_state_overtime_ = 0.3;       // /awapi/autoware/get/status is received at 20ms intervals
  autoware_state_overtime_ = 0.3;      // /awapi/vehicle/get/status is received at 20ms intervals
  engage_wait_overtime_ = 1.0;
  delivery_reservation_verification_overtime_ = 15.0;
  dist_to_stop_pose_max_th_ = 10.0;

  double stop_dist_to_prohibit_engage = this->declare_parameter<double>(
    "stop_dist_to_prohibit_engage", 0.30);
  // Add a value of 0.05 to `stop_dist_to_prohibit_engage`.
  dist_to_stop_pose_min_th_ = stop_dist_to_prohibit_engage + 0.05;

  use_overridable_vehicle_ = this->declare_parameter<bool>(
    "use_overridable_vehicle", true);

  RCLCPP_DEBUG(
    this->get_logger(),
    "[autoware_state_machine] init");

  // Callback group
  // This type of callback group only allows one callback to be executed at a time
  callback_group_service_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_subscription_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto subscribe_option = rclcpp::SubscriptionOptions();
  subscribe_option.callback_group = callback_group_subscription_;

  // Subscriber
  /* Publisher is set to 'volatile' durability QoS.
      So subscriber needs to be set to 'volatile' for ensuring compatibility. */
  sub_awapi_autoware_state_ =
    this->create_subscription<tier4_api_msgs::msg::AwapiAutowareStatus>(
    "/awapi/autoware/get/status", rclcpp::QoS{1},
    std::bind(&AutowareStateMachine::onAwapiAutowareState, this, std::placeholders::_1),
    subscribe_option);
  /* Publisher is set to 'volatile' durability QoS.
      So subscriber needs to be set to 'volatile' for ensuring compatibility. */
  sub_awapi_vehicle_state_ = this->create_subscription<tier4_api_msgs::msg::AwapiVehicleStatus>(
    "/awapi/vehicle/get/status", rclcpp::QoS{1},
    std::bind(&AutowareStateMachine::onAwapiVehicleState, this, std::placeholders::_1),
    subscribe_option);
  sub_calls_delivery_reservation_button_ =
    this->create_subscription<autoware_state_machine_msgs::msg::VehicleButton>(
    "input/delivery_reservation_button", rclcpp::QoS{1}.transient_local(),
    std::bind(&AutowareStateMachine::onCallsDeliveryReservationButton, this, std::placeholders::_1),
    subscribe_option);
  sub_engage_sound_done_ =
    this->create_subscription<autoware_state_machine_msgs::msg::StateSoundDone>(
    "/autoware_state_machine/state_sound_done", rclcpp::QoS{1}.transient_local(),
    std::bind(&AutowareStateMachine::onStateSoundDone, this, std::placeholders::_1),
    subscribe_option);
  /* Publisher is set to 'volatile' durability QoS.
      So subscriber needs to be set to 'volatile' for ensuring compatibility. */
  sub_calls_vehicle_state_ = this->create_subscription<go_interface_msgs::msg::VehicleStatus>(
    "api_vehicle_status", rclcpp::QoS{1},
    std::bind(&AutowareStateMachine::onCallsVehicleState, this, std::placeholders::_1),
    subscribe_option);
  // Publisher
  pub_state_ = this->create_publisher<autoware_state_machine_msgs::msg::StateMachine>(
    "/autoware_state_machine/state", rclcpp::QoS{3}.transient_local());
  pub_delivery_reservation_state_ = this->create_publisher<autoware_state_machine_msgs::msg::StateLock>(
    "/autoware_state_machine/lock_state", rclcpp::QoS{3}.transient_local());
  pub_calls_req_change_lock_ = this->create_publisher<go_interface_msgs::msg::ChangeLockFlg>(
    "req_change_lock_flg", rclcpp::QoS{1});

  // Service
  srv_engage_ = this->create_service<tier4_external_api_msgs::srv::Engage>(
    "/api/external/set/engage",
    std::bind(
      &AutowareStateMachine::execEngageProcess, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, callback_group_service_);
  srv_set_request_start_api_ = this->create_service<std_srvs::srv::Trigger>(
    "/api/autoware/set/start_request",
    std::bind(
      &AutowareStateMachine::setRequestStartAPI, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, callback_group_service_);
  // Client
  cli_engage_ = this->create_client<tier4_external_api_msgs::srv::Engage>(
    "/api/autoware/set/engage",
    rmw_qos_profile_services_default);
  cli_set_operator_ = this->create_client<tier4_external_api_msgs::srv::SetOperator>(
    "/api/autoware/set/operator",
    rmw_qos_profile_services_default);

  emergency_recover_mode_ = true;

  // Timer
  double timer_period;
  std::chrono::milliseconds timer_period_msec;

  timer_period = this->declare_parameter<double>("print_period", update_rate);
  timer_period_msec = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(timer_period));

  auto timer_callback = std::bind(&AutowareStateMachine::onTimer, this);
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), timer_period_msec, std::move(timer_callback),
    this->get_node_base_interface()->get_context()
  );
  this->get_node_timers_interface()->add_timer(timer_, callback_group_subscription_);
}

AutowareStateMachine::~AutowareStateMachine()
{
}

}  // namespace autoware_state_machine

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(autoware_state_machine::AutowareStateMachine)
