/**
 * Copyright (c) 2024-2025 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/
#include <chrono>
#include "rclcpp/logger.hpp"

#include "adi_tmc_coe_core/tmc_coe_motor.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
#include <bitset> // For debug

TmcCoeMotor::TmcCoeMotor(
  rclcpp::Node::SharedPtr p_node, TmcCoeInterpreter * p_tmc_coe_interpreter,
  uint8_t slave_number, uint8_t motor_number, std::string device_name)
: p_node_(p_node),
  p_tmc_coe_interpreter_(p_tmc_coe_interpreter),
  slave_number_(slave_number),
  motor_number_(motor_number),
  device_name_(device_name),
  logger_prefix_(p_node_->get_logger().get_name()),
  logger_(rclcpp::get_logger(logger_prefix_ + ".TmcCoeMotor.slave" + std::to_string(slave_number_) +
    ".motor" + std::to_string(motor_number_)))
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");

  this->initMotorParams();
}

TmcCoeMotor::~TmcCoeMotor()
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
}

void TmcCoeMotor::initMotorParams()
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");

  rcl_interfaces::msg::ParameterDescriptor param_desc;
  rcl_interfaces::msg::IntegerRange param_int_range;

  param_desc.name = "slv" + std::to_string(slave_number_) + ".motor" + \
    std::to_string(motor_number_) + ".en_pub_tmc_coe_info";
  param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
  param_desc.description = "Enable/Disable publishing of TMC information";
  param_desc.additional_constraints.clear();
  param_desc.read_only = true;
  p_node_->declare_parameter(param_desc.name, false, param_desc);
  param_en_pub_tmc_info_ = p_node_->get_parameter(param_desc.name).as_bool();

  param_desc.name = "slv" + std::to_string(slave_number_) + ".motor" + \
    std::to_string(motor_number_) + ".pub_rate_tmc_coe_info";
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  param_desc.description = "Publish rate (hertz) of TMC information";
  param_desc.additional_constraints.clear();
  param_desc.read_only = true;
  param_int_range.from_value = 10;
  param_int_range.to_value = 100;
  param_int_range.step = 1;
  param_desc.integer_range.push_back(param_int_range);
  p_node_->declare_parameter(param_desc.name, 10, param_desc);
  param_pub_rate_tmc_coe_info_ = p_node_->get_parameter(param_desc.name).as_int();

  param_desc.name = "slv" + std::to_string(slave_number_) + ".motor" + \
    std::to_string(motor_number_) + ".pub_actual_vel";
  param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
  param_desc.description =
    "Enable/Disable actual velocity that the user can optionally publish\
     every publish rate as long as en_pub_tmc_info is true";
  param_desc.additional_constraints.clear();
  param_desc.read_only = true;
  p_node_->declare_parameter(param_desc.name, false, param_desc);
  param_pub_actual_vel_ = p_node_->get_parameter(param_desc.name).as_bool();

  param_desc.name = "slv" + std::to_string(slave_number_) + ".motor" + \
    std::to_string(motor_number_) + ".pub_actual_pos";
  param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
  param_desc.description =
    "Enable/Disable actual position that the user can optionally publish\
    every publish rate as long as en_pub_tmc_info is true";
  param_desc.additional_constraints.clear();
  param_desc.read_only = true;
  p_node_->declare_parameter(param_desc.name, false, param_desc);
  param_pub_actual_pos_ = p_node_->get_parameter(param_desc.name).as_bool();

  param_desc.name = "slv" + std::to_string(slave_number_) + ".motor" + \
    std::to_string(motor_number_) + ".pub_actual_trq";
  param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
  param_desc.description =
    "Enable/Disable actual torque that the user can optionally publish\
    every publish rate as long as en_pub_tmc_info is true";
  param_desc.additional_constraints.clear();
  param_desc.read_only = true;
  p_node_->declare_parameter(param_desc.name, false, param_desc);
  param_pub_actual_trq_ = p_node_->get_parameter(param_desc.name).as_bool();

  param_desc.name = "slv" + std::to_string(slave_number_) + ".motor" + \
    std::to_string(motor_number_) + ".tmc_coe_info_topic";
  param_desc.type = rclcpp::ParameterType::PARAMETER_STRING;
  param_desc.description =
    "tmc_coe_info topics that will contain chosen TMC info that will \
    be published";
  param_desc.additional_constraints.clear();
  param_desc.read_only = true;
  std::string tmc_coe_info_topic_default = "tmc_coe_info_" + std::to_string(slave_number_) + "_" + \
    std::to_string(motor_number_);
  p_node_->declare_parameter(param_desc.name, tmc_coe_info_topic_default, param_desc);
  param_tmc_info_topic_ = p_node_->get_parameter(param_desc.name).as_string();

  param_desc.name = "slv" + std::to_string(slave_number_) + ".motor" + \
    std::to_string(motor_number_) + ".tmc_cmd_vel_topic";
  param_desc.type = rclcpp::ParameterType::PARAMETER_STRING;
  param_desc.description =
    "Twist topics that will be the source of target velocity to be set on\
    the TMC";
  param_desc.additional_constraints.clear();
  param_desc.read_only = true;
  std::string tmc_cmd_vel_topic_default = "cmd_vel_" + std::to_string(slave_number_) + "_" + \
    std::to_string(motor_number_);
  p_node_->declare_parameter(param_desc.name, tmc_cmd_vel_topic_default, param_desc);
  param_tmc_cmd_vel_topic_ = p_node_->get_parameter(param_desc.name).as_string();

  param_desc.name = "slv" + std::to_string(slave_number_) + ".motor" + \
    std::to_string(motor_number_) + ".tmc_cmd_abspos_topic";
  param_desc.type = rclcpp::ParameterType::PARAMETER_STRING;
  param_desc.description =
    "Int32 topics that will be the source of target position to be set on\
    the TMC";
  param_desc.additional_constraints.clear();
  param_desc.read_only = true;
  std::string tmc_cmd_abspos_topic_default = "cmd_abspos_" + std::to_string(slave_number_) + "_" + \
    std::to_string(motor_number_);
  p_node_->declare_parameter(param_desc.name, tmc_cmd_abspos_topic_default, param_desc);
  param_tmc_cmd_abspos_topic_ = p_node_->get_parameter(param_desc.name).as_string();

  param_desc.name = "slv" + std::to_string(slave_number_) + ".motor" + \
    std::to_string(motor_number_) + ".tmc_cmd_relpos_topic";
  param_desc.type = rclcpp::ParameterType::PARAMETER_STRING;
  param_desc.description =
    "Int32 topics that will be the source of target position to be set on\
    the TMC";
  param_desc.additional_constraints.clear();
  param_desc.read_only = true;
  std::string tmc_cmd_relpos_topic_default = "cmd_relpos_" + std::to_string(slave_number_) + "_" + \
    std::to_string(motor_number_);
  p_node_->declare_parameter(param_desc.name, tmc_cmd_relpos_topic_default, param_desc);
  param_tmc_cmd_relpos_topic_ = p_node_->get_parameter(param_desc.name).as_string();

  param_desc.name = "slv" + std::to_string(slave_number_) + ".motor" + \
    std::to_string(motor_number_) + ".tmc_cmd_trq_topic";
  param_desc.type = rclcpp::ParameterType::PARAMETER_STRING;
  param_desc.description =
    "Int32 topics that will be the source of target torque to be set on\
    the TMC";
  param_desc.additional_constraints.clear();
  param_desc.read_only = true;
  std::string tmc_cmd_trq_topic_default = "cmd_trq_" + std::to_string(slave_number_) + "_" + \
    std::to_string(motor_number_);
  p_node_->declare_parameter(param_desc.name, tmc_cmd_trq_topic_default, param_desc);
  param_tmc_cmd_trq_topic_ = p_node_->get_parameter(param_desc.name).as_string();

  param_desc.name = "slv" + std::to_string(slave_number_) + ".motor" + \
    std::to_string(motor_number_) + ".wheel_diameter";
  param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
  param_desc.description =
    "Wheel diameter (in meters) that is attached on the motor shaft\
    directly. This is to convert linear values to rpm";
  param_desc.additional_constraints = "If wheel diameter is 0.0, cmd_vel is equal to rpm";
  param_desc.read_only = true;
  p_node_->declare_parameter(param_desc.name, 0.0, param_desc);
  param_wheel_diameter_ = p_node_->get_parameter(param_desc.name).as_double();

  param_desc.name = "slv" + std::to_string(slave_number_) + ".motor" + \
    std::to_string(motor_number_) + ".additional_ratio_vel";
  param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
  param_desc.description =
    "Additional Ratio for velocity for general purposes (adhoc mode,\
    added pulley or gear trains). Default value 1 means disabled";
  param_desc.additional_constraints.clear();
  param_desc.read_only = true;
  p_node_->declare_parameter(param_desc.name, 0.0, param_desc);
  param_add_ratio_vel_ = p_node_->get_parameter(param_desc.name).as_double();

  param_desc.name = "slv" + std::to_string(slave_number_) + ".motor" + \
    std::to_string(motor_number_) + ".additional_ratio_pos";
  param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
  param_desc.description =
    "Additional Ratio for position for general purposes (adhoc mode,\
    added pulley or gear trains). Default value 1 means disabled";
  param_desc.additional_constraints.clear();
  param_desc.read_only = true;
  p_node_->declare_parameter(param_desc.name, 1.0, param_desc);
  param_add_ratio_pos_ = p_node_->get_parameter(param_desc.name).as_double();

  param_desc.name = "slv" + std::to_string(slave_number_) + ".motor" + \
    std::to_string(motor_number_) + ".additional_ratio_trq";
  param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
  param_desc.description =
    "Additional Ratio for torque for general purposes (adhoc mode). \
    Default value 1 means disabled";
  param_desc.additional_constraints.clear();
  param_desc.read_only = true;
  p_node_->declare_parameter(param_desc.name, 1.0, param_desc);
  param_add_ratio_trq_ = p_node_->get_parameter(param_desc.name).as_double();
}

void TmcCoeMotor::init()
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");

  this->initPublisher();
  this->initSubscriber();
  RCLCPP_INFO_STREAM(logger_, "  Velocity unit: rpm");
  RCLCPP_INFO_STREAM(logger_, "  Position unit: pulses");
  RCLCPP_INFO_STREAM(logger_, "  Torque unit: mA");

  RCLCPP_INFO_STREAM(logger_, "Motor" << static_cast<int>(motor_number_) << " Initialized!\n");
  return;
}

void TmcCoeMotor::initPublisher()
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  int period_ms = 0;
  std::string node_namespace = p_node_->get_namespace();
  tmc_coe_info_frame_id_ = node_namespace + "/tmcm" + device_name_ + "_mtr" + \
    std::to_string(motor_number_) + "_frame";

  RCLCPP_DEBUG_STREAM(logger_, "Frame id is " << tmc_coe_info_frame_id_);

  pub_callback_group_ = p_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  if (param_en_pub_tmc_info_)
  {
    RCLCPP_INFO_STREAM(logger_, "Publish of tmc_coe_info is enabled.");
    tmc_coe_info_pub_ = p_node_->create_publisher<adi_tmc_coe_interfaces::msg::TmcCoeInfo>(
      param_tmc_info_topic_, 10);

    period_ms = (1000 / param_pub_rate_tmc_coe_info_);
    RCLCPP_DEBUG_STREAM(logger_, "period ms = " << period_ms);

    tmc_coe_info_pub_timer_ = p_node_->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&TmcCoeMotor::publishTmcCoeInfo, this), pub_callback_group_);
  }
  return;
}

void TmcCoeMotor::initSubscriber()
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  rclcpp::SubscriptionOptions options;
  sub_callback_group_ = p_node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  options.callback_group = sub_callback_group_;

  cmd_vel_sub_ = p_node_->create_subscription<geometry_msgs::msg::Twist>(
    param_tmc_cmd_vel_topic_, 10, std::bind(&TmcCoeMotor::cmdVelCallback, this, _1),
    options);

  cmd_abspos_sub_ = p_node_->create_subscription<std_msgs::msg::Int32>(
    param_tmc_cmd_abspos_topic_, 10, std::bind(&TmcCoeMotor::cmdAbsPosCallback, this, _1),
    options);

  cmd_relpos_sub_ = p_node_->create_subscription<std_msgs::msg::Int32>(
    param_tmc_cmd_relpos_topic_, 10, std::bind(&TmcCoeMotor::cmdRelPosCallback, this, _1),
    options);

  cmd_trq_sub_ = p_node_->create_subscription<std_msgs::msg::Int32>(
    param_tmc_cmd_trq_topic_, 10, std::bind(&TmcCoeMotor::cmdTrqCallback, this, _1),
    options);

  return;
}

void TmcCoeMotor::publishTmcCoeInfo()
{
  RCLCPP_DEBUG_STREAM(logger_, "[" << __func__ << "] called");
  auto message = adi_tmc_coe_interfaces::msg::TmcCoeInfo();
  int8_t mode_of_operation = 0;
  std::string mode_of_operation_str = "";
  message.header.stamp = p_node_->now();
  message.header.frame_id = tmc_coe_info_frame_id_;
  message.interface_name = p_node_->get_parameter("interface_name").as_string();
  message.slave_number = slave_number_;
  message.motor_number = motor_number_;

  mode_of_operation = p_tmc_coe_interpreter_->input_pdo_[slave_number_]->modes_of_operation_display;
  switch (static_cast<mode_of_operation_t>(mode_of_operation))
  {
    case MODE_OF_OPERATION_NONE:
      mode_of_operation_str = "None";
      break;
    case PROFILE_POSITION:
      mode_of_operation_str = "Profile Position";
      break;
    case PROFILE_VELOCITY:
      mode_of_operation_str = "Profile Velocity";
      break;
    case HOMING_MODE:
      mode_of_operation_str = "Homing Mode";
      break;
    case CYCLIC_SYNC_POS:
      mode_of_operation_str = "Cyclic Synchronous Position Mode";
      break;
    case CYCLIC_SYNC_VEL:
      mode_of_operation_str = "Cyclic Synchronous Velocity Mode";
      break;
    case CYCLIC_SYNC_TRQ:
      mode_of_operation_str = "Cyclic Synchronous Torque Mode";
      break;
    default:
      mode_of_operation_str = "None";
      break;
  }
  message.mode_of_operation = mode_of_operation_str;
  message.status_word = p_tmc_coe_interpreter_->input_pdo_[slave_number_]->status_word;

  if (param_pub_actual_vel_)
  {
    message.velocity = p_tmc_coe_interpreter_->input_pdo_[slave_number_]->velocity_actual_value *
      param_add_ratio_vel_;
  }

  if (param_pub_actual_pos_)
  {
    message.position = p_tmc_coe_interpreter_->input_pdo_[slave_number_]->position_actual_value *
      param_add_ratio_pos_;
  }

  if (param_pub_actual_trq_)
  {
    message.torque = p_tmc_coe_interpreter_->input_pdo_[slave_number_]->torque_actual_value *
      param_add_ratio_trq_;
  }
  tmc_coe_info_pub_->publish(message);
  return;
}

void TmcCoeMotor::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  int32_t val = 0;
  int prev_cycle_count = 0;
  int retries = 0;
  int SDO_PDO_retries = p_node_->get_parameter("SDO_PDO_retries").as_int();

  val = static_cast<int32_t>(msg->linear.x / param_add_ratio_vel_);
  RCLCPP_DEBUG_STREAM(logger_, "val= " << val);

  p_tmc_coe_interpreter_->startCycleCounter();

  while (SDO_PDO_retries >= retries)
  {
    if (p_tmc_coe_interpreter_->isCycleFinished())
    {
      // Make sure mode of operation is PROFILE_VELOCITY
      if (PROFILE_VELOCITY !=
        p_tmc_coe_interpreter_->input_pdo_[slave_number_]->modes_of_operation_display)
      {
        p_tmc_coe_interpreter_->output_pdo_[slave_number_]->modes_of_operation = PROFILE_VELOCITY;
      }

      p_tmc_coe_interpreter_->output_pdo_[slave_number_]->target_velocity = val;

      while ((p_tmc_coe_interpreter_->getCycleCounter() - prev_cycle_count) < 1)
      {
        // Wait until 1 cycle has elapsed
      }

      if (val == p_tmc_coe_interpreter_->output_pdo_[slave_number_]->target_velocity)
      {
        RCLCPP_DEBUG_STREAM(logger_, "Target velocity set successfully");
        break;
      }
      else
      {
        prev_cycle_count = p_tmc_coe_interpreter_->getCycleCounter();
        retries++;
      }
    }
  }

  p_tmc_coe_interpreter_->stopCycleCounter();

  if (val != p_tmc_coe_interpreter_->output_pdo_[slave_number_]->target_velocity)
  {
    RCLCPP_WARN_STREAM(logger_, "Failed to set velocity");
  }
  return;
}

void TmcCoeMotor::cmdAbsPosCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  int32_t val = 0;
  int SDO_PDO_retries = p_node_->get_parameter("SDO_PDO_retries").as_int();
  val = static_cast<int32_t>(msg->data / param_add_ratio_pos_);
  RCLCPP_DEBUG_STREAM(logger_, "val= " << val);

  p_tmc_coe_interpreter_->startCycleCounter();

  while (SDO_PDO_retries >= p_tmc_coe_interpreter_->getCycleCounter())
  {
    if (p_tmc_coe_interpreter_->isCycleFinished())
    {
      // Make sure mode of operation is PROFILE_POSITION
      if (PROFILE_POSITION !=
        p_tmc_coe_interpreter_->input_pdo_[slave_number_]->modes_of_operation_display)
      {
        p_tmc_coe_interpreter_->output_pdo_[slave_number_]->modes_of_operation = PROFILE_POSITION;
      }

      p_tmc_coe_interpreter_->output_pdo_[slave_number_]->target_position = val;

      uint32_t control_word = ENABLE_OPERATION | (CW_NEW_SET_POINT_START <<
        CW_OMS_NEW_SET_POINT_BIT) |
        (CW_NEW_POSITION_ABSOLUTE << CW_OMS_ABSOLUTE_RELATIVE_BIT);
      RCLCPP_DEBUG_STREAM(logger_, "Set controlword to 0b" << std::bitset<16>(control_word));
      p_tmc_coe_interpreter_->output_pdo_[slave_number_]->control_word = control_word;
    }

    if (p_tmc_coe_interpreter_->isStatusWordState(slave_number_, SET_POINT_ACK_IN_PROCESS))
    {
      break;
    }
  }

  p_tmc_coe_interpreter_->stopCycleCounter();

  p_tmc_coe_interpreter_->startCycleCounter();
  while (SDO_PDO_retries >= p_tmc_coe_interpreter_->getCycleCounter())
  {
    if (p_tmc_coe_interpreter_->isCycleFinished())
    {
      // Set Controlword to ENABLE_OPERATION
      uint32_t control_word = ENABLE_OPERATION | (CW_NEW_SET_POINT_STOP <<
        CW_OMS_NEW_SET_POINT_BIT);
      RCLCPP_DEBUG_STREAM(logger_, "Set controlword to 0b" << std::bitset<16>(control_word));
      p_tmc_coe_interpreter_->output_pdo_[slave_number_]->control_word = control_word;
    }
    if (!p_tmc_coe_interpreter_->isStatusWordState(slave_number_, SET_POINT_ACK_IN_PROCESS))
    {
      break;
    }
  }
  p_tmc_coe_interpreter_->stopCycleCounter();

  if (val == p_tmc_coe_interpreter_->output_pdo_[slave_number_]->target_position)
  {
    RCLCPP_DEBUG_STREAM(logger_, "Target position set successfully");
  }
  else
  {
    RCLCPP_WARN_STREAM(logger_, "Failed to set Absolute Position");
  }
  return;
}


void TmcCoeMotor::cmdRelPosCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  int32_t val = 0;
  int SDO_PDO_retries = p_node_->get_parameter("SDO_PDO_retries").as_int();
  val = static_cast<int32_t>(msg->data / param_add_ratio_pos_);
  RCLCPP_DEBUG_STREAM(logger_, "val= " << val);

  p_tmc_coe_interpreter_->startCycleCounter();

  while (SDO_PDO_retries >= p_tmc_coe_interpreter_->getCycleCounter())
  {
    if (p_tmc_coe_interpreter_->isCycleFinished())
    {
      // Make sure mode of operation is PROFILE_POSITION
      if (PROFILE_POSITION !=
        p_tmc_coe_interpreter_->input_pdo_[slave_number_]->modes_of_operation_display)
      {
        p_tmc_coe_interpreter_->output_pdo_[slave_number_]->modes_of_operation = PROFILE_POSITION;
      }

      p_tmc_coe_interpreter_->output_pdo_[slave_number_]->target_position = val;

      uint32_t control_word = ENABLE_OPERATION | (CW_NEW_SET_POINT_START <<
        CW_OMS_NEW_SET_POINT_BIT) |
        (CW_NEW_POSITION_RELATIVE << CW_OMS_ABSOLUTE_RELATIVE_BIT);
      RCLCPP_DEBUG_STREAM(logger_, "Set controlword to 0b" << std::bitset<16>(control_word));
      p_tmc_coe_interpreter_->output_pdo_[slave_number_]->control_word = control_word;
    }

    if (p_tmc_coe_interpreter_->isStatusWordState(slave_number_, SET_POINT_ACK_IN_PROCESS))
    {
      break;
    }
  }

  p_tmc_coe_interpreter_->stopCycleCounter();

  p_tmc_coe_interpreter_->startCycleCounter();
  while (SDO_PDO_retries >= p_tmc_coe_interpreter_->getCycleCounter())
  {
    if (p_tmc_coe_interpreter_->isCycleFinished())
    {
      // Set Controlword to ENABLE_OPERATION
      uint32_t control_word = ENABLE_OPERATION | (CW_NEW_SET_POINT_STOP <<
        CW_OMS_NEW_SET_POINT_BIT);
      RCLCPP_DEBUG_STREAM(logger_, "Set controlword to 0b" << std::bitset<16>(control_word));
      p_tmc_coe_interpreter_->output_pdo_[slave_number_]->control_word = control_word;
    }
    if (!p_tmc_coe_interpreter_->isStatusWordState(slave_number_, SET_POINT_ACK_IN_PROCESS))
    {
      break;
    }
  }
  p_tmc_coe_interpreter_->stopCycleCounter();

  if (val == p_tmc_coe_interpreter_->output_pdo_[slave_number_]->target_position)
  {
    RCLCPP_DEBUG_STREAM(logger_, "Target position set successfully");
  }
  else
  {
    RCLCPP_WARN_STREAM(logger_, "Failed to set Relative Position");
  }
  return;
}


void TmcCoeMotor::cmdTrqCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");

  RCLCPP_DEBUG_STREAM(logger_, "[DEBUG] msg->data is " << msg->data);
  RCLCPP_WARN_STREAM(logger_, "Controlling via Torque is not yet supported");
  return;
}
