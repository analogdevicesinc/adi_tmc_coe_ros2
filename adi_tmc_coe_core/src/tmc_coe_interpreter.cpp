/**
 * Copyright (c) 2024-2025 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include "rclcpp/logger.hpp"
#include "adi_tmc_coe_core/tmc_coe_interpreter.hpp"

#include <bitset> // For debug
using namespace std::chrono_literals;

// SOEM headers
#include "ethercat.h"

TmcCoeInterpreter::TmcCoeInterpreter(
  uint8_t SDO_PDO_retries, double interface_timeout, std::string
  logger_prefix)
: logger_prefix_(logger_prefix),
  logger_(rclcpp::get_logger((logger_prefix_ + ".TmcCoeInterpreter").c_str()))
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  SDO_PDO_retries_ = SDO_PDO_retries;
  interface_timeout_ = interface_timeout;
  b_interface_unresponsive_ = false;
  b_exit_threads_ = false;
  b_cycle_finished_ = false;
  b_start_cycle_count_ = false;
}

TmcCoeInterpreter::~TmcCoeInterpreter()
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  input_pdo_.clear();
  output_pdo_.clear();
}

uint8_t TmcCoeInterpreter::initInterface(std::string ifname)
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  RCLCPP_DEBUG_STREAM(logger_, "Interface Name: " << ifname.c_str());

  if (ec_init(ifname.c_str()))
  {
    RCLCPP_INFO_STREAM(logger_, "Init on " << ifname << " succeeded");

    if (ec_config_init(FALSE) > 0)
    {
      RCLCPP_INFO_STREAM(logger_, "Config Init on " << ifname << " succeeded");
    }
    else
    {
      RCLCPP_ERROR_STREAM(logger_, "No slaves found! Exiting");
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(logger_, "No socket connection on " << ifname);
  }

  // Resize the following accordingly
  all_obj_name_.resize(ec_slavecount + 1);
  all_index_.resize(ec_slavecount + 1);
  all_sub_index_.resize(ec_slavecount + 1);
  all_datatype_.resize(ec_slavecount + 1);
  return ec_slavecount;
}

bool TmcCoeInterpreter::initDictionary(
  uint8_t slave_number, std::vector<std::string> all_obj_name,
  std::vector<std::string> all_index, std::vector<std::string> all_sub_index,
  std::vector<std::string> all_datatype)
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  bool b_result = false;

  all_obj_name_[slave_number] = all_obj_name;
  all_index_[slave_number] = all_index;
  all_sub_index_[slave_number] = all_sub_index;
  all_datatype_[slave_number] = all_datatype;

  if ( (all_obj_name_[slave_number].size() == all_index_[slave_number].size()) &&
    (all_obj_name_[slave_number].size() == all_sub_index_[slave_number].size()) &&
    (all_obj_name_[slave_number].size() == all_datatype_[slave_number].size()) )
  {
    b_result = true;
  }

  return b_result;
}

bool TmcCoeInterpreter::safeOperationalState(uint8_t slave_number)
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  RCLCPP_INFO_STREAM(logger_, "Slave " << std::to_string(slave_number) << " change to SAFE_OP");
  bool b_result = false;
  int total_slave = ec_slavecount;

  input_pdo_.resize(total_slave + 1, nullptr);
  output_pdo_.resize(total_slave + 1, nullptr);

  ec_config_map(&IOmap);
  ec_configdc();

  if (total_slave >= slave_number)
  {
    output_pdo_[slave_number] = (output_pdo_t *)ec_slave[slave_number].outputs;
    input_pdo_[slave_number] = (input_pdo_t *)ec_slave[slave_number].inputs;
    if (SAFE_OP == this->changeNMTState(slave_number, SAFE_OP))
    {
      RCLCPP_INFO_STREAM(
        logger_, "Slave " << static_cast<int>(slave_number) <<
          " mapped. Set slave to SAFE_OP");
      b_result = true;
    }
    else
    {
      RCLCPP_ERROR_STREAM(
        logger_, "Slave " << static_cast<int>(slave_number) <<
          "unable to reach SAFE_OP");
      b_result = false;
    }
  }

  return b_result;
}

bool TmcCoeInterpreter::operationalState(uint8_t slave_number, bool b_init)
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  RCLCPP_INFO_STREAM(logger_, "Slave " << std::to_string(slave_number) << " change to OPERATIONAL");
  bool b_result = false;
  int sequence_retries = 0;
  int total_slave = ec_slavecount;

  if (total_slave >= slave_number)
  {
    if (b_init)
    {
      if (OPERATIONAL == this->changeNMTState(slave_number, OPERATIONAL))
      {
        RCLCPP_INFO_STREAM(
          logger_, "Slave " << static_cast<int>(slave_number) <<
            " mapped. Set slave to OPERATIONAL");
        b_result = true;
      }
      else
      {
        RCLCPP_ERROR_STREAM(
          logger_, "Slave " << static_cast<int>(slave_number) <<
            "unable to reach OPERATIONAL");
        b_result = false;
      }
    }
    while (sequence_retries < SDO_PDO_retries_)
    {
      if (READY_TO_SWITCH_ON == this->changeCiA402State(slave_number, READY_TO_SWITCH_ON))
      {
        b_result = true;
      }
      if (b_result && (SWITCHED_ON == this->changeCiA402State(slave_number, SWITCHED_ON)))
      {
        b_result = true;
      }
      if (b_result && (OPERATION_ENABLED == this->changeCiA402State(
          slave_number,
          OPERATION_ENABLED)))
      {
        b_result = true;
      }

      if (b_result)
      {
        break;
      }
      else
      {
        sequence_retries++;
        RCLCPP_WARN_STREAM(logger_, "[DEBUG] Sequence retries= " << sequence_retries);
      }
    }
  }

  return b_result;
}

std::string TmcCoeInterpreter::getSlaveName(uint8_t slave_number)
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  std::string slave_name = "";

  slave_name = ec_slave[slave_number].name;
  RCLCPP_INFO_STREAM(
    logger_, "Device name of slave " << std::to_string(slave_number) << " is " <<
      slave_name);
  // Only retain "XXXX" in "TMCM-XXXX" Slave Name
  slave_name = slave_name.substr(5, 4);

  return slave_name;
}

nmt_state_t TmcCoeInterpreter::changeNMTState(uint8_t slave_number, nmt_state_t state)
{
  RCLCPP_DEBUG_STREAM(logger_, "[" << __func__ << "] called");
  RCLCPP_DEBUG_STREAM(
    logger_, "Slave= " << std::to_string(slave_number) << "State= " <<
      state);
  nmt_state_t current_state = NONE;
  bool b_valid_state = false;
  bool b_result = false;
  uint16_t ec_slave_state = 0;
  std::string state_str = "";
  switch (state)
  {
    case INIT:
      ec_slave_state = EC_STATE_INIT;
      state_str = "INIT";
      b_valid_state = true;
      break;
    case PRE_OP:
      ec_slave_state = EC_STATE_PRE_OP;
      state_str = "PRE_OP";
      b_valid_state = true;
      break;
    case SAFE_OP:
      ec_slave_state = EC_STATE_SAFE_OP;
      state_str = "SAFE_OP";
      b_valid_state = true;
      break;
    case OPERATIONAL:
      ec_slave_state = EC_STATE_OPERATIONAL;
      state_str = "OPERATIONAL";
      b_valid_state = true;
      break;
    case SAFE_OP_ERROR_ACK:
      ec_slave_state = EC_STATE_SAFE_OP + EC_STATE_ACK;
      state_str = "ERROR ACKNOWLEDGED; Changed to SAFE_OP";
      b_valid_state = true;
      break;
    default:
      RCLCPP_ERROR_STREAM(logger_, "Invalid State request");
      break;
  }

  if (b_valid_state)
  {
    ec_slave[slave_number].state = ec_slave_state;
    ec_writestate(slave_number);

    /* After error acknowledgment, the slave transitions to the requested state. 
     * The ERROR bit (0x10) is cleared, and AL_Status reflects only the new state. 
     * Ref: https://manualmachine.com/beckhoff/ethercatregisterssectionii/1503469-user-manual/#33
     * In this implementation, we request SAFE_OP after error acknowledgement 
     * to ensure a safe recovery state.*/
    if (SAFE_OP_ERROR_ACK != ec_slave_state)
    {
      current_state = static_cast<nmt_state_t>(ec_statecheck(
        slave_number, ec_slave_state,
        EC_TIMEOUTSTATE));
    }
    else
    {
      current_state = static_cast<nmt_state_t>(ec_statecheck(
        slave_number, EC_STATE_SAFE_OP,
        EC_TIMEOUTSTATE));
    }

    if ((SAFE_OP_ERROR_ACK == ec_slave_state) && (SAFE_OP == current_state))
    {
      b_result = true;
    }
    else if (current_state == ec_slave_state)
    {
      b_result = true;
    }

    RCLCPP_DEBUG_STREAM_EXPRESSION(
      logger_, b_result, "State Change for Slave " << static_cast<int>(slave_number) <<
        " is successful");
    RCLCPP_DEBUG_STREAM_EXPRESSION(
      logger_, b_result, "Slave " << static_cast<int>(slave_number) << " state is " <<
        state_str);

    RCLCPP_ERROR_STREAM_EXPRESSION(
      logger_, !b_result, "State Change for Slave " << static_cast<int>(slave_number) <<
        " is unsuccessful");
  }

  ec_readstate();
  current_state = static_cast<nmt_state_t>(ec_slave[slave_number].state);
  return current_state;
}

fsa_state_t TmcCoeInterpreter::changeCiA402State(uint8_t slave_number, fsa_state_t state)
{
  RCLCPP_DEBUG_STREAM(logger_, "[" << __func__ << "] called");

  fsa_state_t current_state = NOT_READY_TO_SWITCH_ON;
  rclcpp::Clock clock;
  rclcpp::Time start_time = clock.now();
  rclcpp::Time end_time = start_time;
  int n_retries = 0;
  int CW_return = CONTROL_WORD_FAIL;
  while (n_retries <= SDO_PDO_retries_)
  {
    if (isStatusWordState(slave_number, FAULT))
    {
      CW_return = this->setControlWord(slave_number, SWITCH_ON_DISABLE, FAULT_RESET);
    }

    if (!isStatusWordState(slave_number, FAULT))
    {
      if (SWITCH_ON_DISABLE == state)
      {
        if (isStatusWordState(slave_number, READY_TO_SWITCH_ON) ||
          isStatusWordState(slave_number, SWITCHED_ON) ||
          isStatusWordState(slave_number, OPERATION_ENABLED) ||
          isStatusWordState(slave_number, QUICK_STOP_ACTIVE))
        {
          CW_return = this->setControlWord(slave_number, state, DISABLE_VOLTAGE);
        }
        else
        {
          RCLCPP_WARN_STREAM(logger_, "Invalid state request.");
        }
      }
      else if (READY_TO_SWITCH_ON == state)
      {
        if (isStatusWordState(slave_number, SWITCH_ON_DISABLE) ||
          isStatusWordState(slave_number, SWITCHED_ON) ||
          isStatusWordState(slave_number, OPERATION_ENABLED))
        {
          CW_return = this->setControlWord(slave_number, state, SHUTDOWN);
        }
        else
        {
          RCLCPP_WARN_STREAM(logger_, "Invalid state request.");
        }
      }
      else if (SWITCHED_ON == state)
      {
        if (isStatusWordState(slave_number, READY_TO_SWITCH_ON))
        {
          CW_return = this->setControlWord(slave_number, state, SWITCH_ON);
        }
        else if (isStatusWordState(slave_number, OPERATION_ENABLED))
        {
          CW_return = this->setControlWord(slave_number, state, DISABLE_OPERATION);
        }
        else
        {
          RCLCPP_WARN_STREAM(logger_, "Invalid state request.");
        }
      }
      else if (OPERATION_ENABLED == state)
      {
        if (isStatusWordState(slave_number, SWITCHED_ON) ||
          isStatusWordState(slave_number, QUICK_STOP_ACTIVE))
        {
          CW_return = this->setControlWord(slave_number, state, ENABLE_OPERATION);
        }
        else
        {
          RCLCPP_WARN_STREAM(logger_, "Invalid state request.");
        }
      }
      else if (QUICK_STOP_ACTIVE == state)
      {
        if (isStatusWordState(slave_number, OPERATION_ENABLED))
        {
          CW_return = this->setControlWord(slave_number, state, QUICK_STOP);
        }
        else
        {
          RCLCPP_WARN_STREAM(logger_, "Invalid state request.");
        }
      }
    }

    end_time = clock.now();
    rclcpp::Duration duration = end_time - start_time;
    if (interface_timeout_ < duration.seconds())
    {
      CW_return = CONTROL_WORD_TIMEOUT;
    }
    if (CONTROL_WORD_FAIL == CW_return)
    {
      n_retries++;
    }
    else
    {
      break;
    }
  }

  // Return current state regardless if successful or not
  current_state = checkStatusWordState(slave_number);

  RCLCPP_WARN_STREAM_EXPRESSION(
    logger_, ((CONTROL_WORD_FAIL == CW_return)),
    "Failed to change CiA402 state; Tried " << n_retries << " times");
  return current_state;
}

bool TmcCoeInterpreter::isStatusWordState(uint8_t slave_number, fsa_state_t state)
{
  RCLCPP_DEBUG_STREAM(logger_, "[" << __func__ << "] called");
  bool b_result = false;
  uint16_t statusword = 0;
  uint16_t statusword_masked = 0;

  statusword = input_pdo_[slave_number]->status_word;
  RCLCPP_DEBUG_STREAM(logger_, "statusword is 0b" << std::bitset<16>(statusword));

  statusword_masked = statusword & status_state_mask_[state];
  RCLCPP_DEBUG_STREAM(logger_, "statusword_masked is 0b" << std::bitset<16>(statusword_masked));

  if (statusword_masked == state_coding_val_[state])
  {
    b_result = true;
    RCLCPP_DEBUG_STREAM_EXPRESSION(
      logger_, state == NOT_READY_TO_SWITCH_ON,
      "Statusword is NOT_READY_TO_SWITCH_ON");
    RCLCPP_DEBUG_STREAM_EXPRESSION(
      logger_, state == SWITCH_ON_DISABLE,
      "Statusword is SWITCH_ON_DISABLE");
    RCLCPP_DEBUG_STREAM_EXPRESSION(
      logger_, state == READY_TO_SWITCH_ON,
      "Statusword is READY_TO_SWITCH_ON");
    RCLCPP_DEBUG_STREAM_EXPRESSION(logger_, state == SWITCHED_ON, "Statusword is SWITCHED_ON");
    RCLCPP_DEBUG_STREAM_EXPRESSION(
      logger_, state == OPERATION_ENABLED,
      "Statusword is OPERATION_ENABLED");
    RCLCPP_DEBUG_STREAM_EXPRESSION(
      logger_, state == QUICK_STOP_ACTIVE,
      "Statusword is QUICK_STOP_ACTIVE");
    RCLCPP_DEBUG_STREAM_EXPRESSION(
      logger_, state == FAULT_REACTION_ACTIVE,
      "Statusword is FAULT_REACTION_ACTIVE");
    RCLCPP_DEBUG_STREAM_EXPRESSION(logger_, state == FAULT, "Statusword is FAULT");
    RCLCPP_DEBUG_STREAM_EXPRESSION(
      logger_, state == SET_POINT_ACK_IN_PROCESS,
      "Statusword is SET_POINT_ACK_IN_PROCESS");
  }

  return b_result;
}

fsa_state_t TmcCoeInterpreter::checkStatusWordState(uint8_t slave_number)
{
  RCLCPP_DEBUG_STREAM(logger_, "[" << __func__ << "] called");
  fsa_state_t current_state = UNKNOWN;
  bool b_valid_state = false;
  if (!b_valid_state && isStatusWordState(slave_number, NOT_READY_TO_SWITCH_ON))
  {
    current_state = NOT_READY_TO_SWITCH_ON;
    b_valid_state = true;
  }

  if (!b_valid_state && isStatusWordState(slave_number, SWITCH_ON_DISABLE))
  {
    current_state = SWITCH_ON_DISABLE;
    b_valid_state = true;
  }

  if (!b_valid_state && isStatusWordState(slave_number, READY_TO_SWITCH_ON))
  {
    current_state = READY_TO_SWITCH_ON;
    b_valid_state = true;
  }

  if (!b_valid_state && isStatusWordState(slave_number, SWITCHED_ON))
  {
    current_state = SWITCHED_ON;
    b_valid_state = true;
  }

  if (!b_valid_state && isStatusWordState(slave_number, OPERATION_ENABLED))
  {
    current_state = OPERATION_ENABLED;
    b_valid_state = true;
  }

  if (!b_valid_state && isStatusWordState(slave_number, QUICK_STOP_ACTIVE))
  {
    current_state = QUICK_STOP_ACTIVE;
    b_valid_state = true;
  }

  if (!b_valid_state && isStatusWordState(slave_number, FAULT_REACTION_ACTIVE))
  {
    current_state = FAULT_REACTION_ACTIVE;
    b_valid_state = true;
  }

  if (!b_valid_state && isStatusWordState(slave_number, FAULT))
  {
    current_state = FAULT;
    b_valid_state = true;
  }

  if (!b_valid_state && isStatusWordState(slave_number, SET_POINT_ACK_IN_PROCESS))
  {
    current_state = SET_POINT_ACK_IN_PROCESS;
    b_valid_state = true;
  }
  return current_state;
}

int8_t TmcCoeInterpreter::setControlWord(uint8_t slave_number, fsa_state_t response_SW,
  control_word_val_t requested_CW)
{
  RCLCPP_DEBUG_STREAM(logger_, "[" << __func__ << "] called");

  int result = CONTROL_WORD_FAIL;
  rclcpp::Clock clock;
  rclcpp::Time start_time = clock.now();
  rclcpp::Time end_time = start_time;

  RCLCPP_DEBUG_STREAM_EXPRESSION(
    logger_, requested_CW == SHUTDOWN,
    "Setting Controlword to SHUTDOWN");
  RCLCPP_DEBUG_STREAM_EXPRESSION(
    logger_, requested_CW == SWITCH_ON,
    "Setting Controlword to SWITCH_ON");
  RCLCPP_DEBUG_STREAM_EXPRESSION(
    logger_, requested_CW == SWITCH_ON_ENABLE_OPERATION,
    "Setting Controlword to SWITCH_ON_ENABLE_OPERATION or ENABLE_OPERATION");
  RCLCPP_DEBUG_STREAM_EXPRESSION(
    logger_, requested_CW == DISABLE_VOLTAGE, "Setting Controlword to " <<
      "DISABLE_VOLTAGE");
  RCLCPP_DEBUG_STREAM_EXPRESSION(
    logger_, requested_CW == QUICK_STOP, "Setting Controlword to " <<
      "QUICK_STOP");
  RCLCPP_DEBUG_STREAM_EXPRESSION(
    logger_, requested_CW == DISABLE_OPERATION, "Setting Controlword to " <<
      "DISABLE_OPERATION");
  RCLCPP_DEBUG_STREAM_EXPRESSION(
    logger_, requested_CW == ENABLE_OPERATION, "Setting Controlword to " <<
      "ENABLE_OPERATION");
  RCLCPP_DEBUG_STREAM_EXPRESSION(
    logger_, requested_CW == FAULT_RESET, "Setting Controlword to " <<
      "FAULT_RESET");

  this->startCycleCounter();

  while ((result >= CONTROL_WORD_FAIL) &&
    (SDO_PDO_retries_ >= this->getCycleCounter()) &&
    ( !(this->isStatusWordState(slave_number, response_SW))))
  {
    if (this->isCycleFinished())
    {
      RCLCPP_DEBUG_STREAM(logger_, "Set Controlword");
      output_pdo_[slave_number]->control_word = requested_CW;
    }

    end_time = clock.now();
    rclcpp::Duration duration = end_time - start_time;
    if (interface_timeout_ < duration.seconds())
    {
      RCLCPP_ERROR_STREAM(logger_, "Timeout: Slave did not respond");
      result = CONTROL_WORD_TIMEOUT;
      break;
    }
  }
  this->stopCycleCounter();

  if (this->isStatusWordState(slave_number, response_SW))
  {
    result = CONTROL_WORD_SUCCESS;
    RCLCPP_DEBUG_STREAM(logger_, "Controlword successfully set");
  }

  RCLCPP_WARN_STREAM_EXPRESSION(
    logger_, (result <= CONTROL_WORD_FAIL), "Failed to " << __func__ <<
      "; Tried to set controlword " << std::to_string(SDO_PDO_retries_) << " times already.");
  return result;
}

void TmcCoeInterpreter::createProcessDataThread()
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");

  if (!processdata_thread_.joinable())
  {
    RCLCPP_INFO_STREAM(logger_, "Creating Process Data thread...");
    processdata_thread_ = std::thread(&TmcCoeInterpreter::processData, this);
  }
}

void TmcCoeInterpreter::createErrorCheckThread()
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");

  if (!error_check_thread_.joinable())
  {
    RCLCPP_INFO_STREAM(logger_, "Creating Error Check thread...");
    error_check_thread_ = std::thread(&TmcCoeInterpreter::errorCheck, this);
  }
}

void TmcCoeInterpreter::processData()
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");

  while (!b_exit_threads_)
  {
    b_cycle_finished_ = false;
    ec_send_processdata();
    work_count_ = ec_receive_processdata(EC_TIMEOUTRET);

    b_cycle_finished_ = true;

    if (b_start_cycle_count_)
    {
      cycle_counter_++;
    }
    else
    {
      cycle_counter_ = 0;
    }

    std::this_thread::sleep_for(std::chrono::microseconds(PROCESS_DATA_DELAY));
  }
  return;
}

void TmcCoeInterpreter::errorCheck()
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  bool b_error_cleared = true;
  uint8_t slave_number = 1;
  int expected_work_count = ec_slavecount * TOTAL_WORK_COUNTER_PER_SLAVE;
  rclcpp::Clock clock;
  rclcpp::Time start_time = clock.now();
  rclcpp::Time end_time = start_time;
  rclcpp::Duration duration = end_time - start_time;

  while (!b_exit_threads_ && !b_interface_unresponsive_)
  {
    start_time = clock.now();
    end_time = start_time;
    slave_number = 1;

    while ((work_count_ < expected_work_count) && (slave_number <= ec_slavecount))
    {
      /* NOTE: PRE OP state is excluded in error correction as it is assumed that the user is accountable in changing the
        state of the slave. This package allows user to stay in PRE OP state to make any modifications on the configuration
        of the slaves (Please only do this if you have enough knowledge on the slave and ofcourse CoE protocol as it may
        destroy the firmware of the slave) */
      if (ec_slave[slave_number].state == PRE_OP)
      {
        b_error_cleared = true;
      }
      else
      {
        b_error_cleared = false;
      }
      duration = end_time - start_time;
      while (!b_error_cleared && (interface_timeout_ > (duration.seconds())))
      {
        ec_readstate();
        RCLCPP_WARN_STREAM(
          logger_, "Slave " << static_cast<int>(slave_number) << " state is " <<
            ec_slave[slave_number].state);
        if ((ec_slave[slave_number].state == OPERATIONAL) && isStatusWordState(
            slave_number,
            OPERATION_ENABLED))
        {
          start_time = clock.now(); // Reset time count
          b_error_cleared = true;
        }
        else if ((ec_slave[slave_number].state == OPERATIONAL) && !isStatusWordState(
            slave_number,
            OPERATION_ENABLED))
        {
          RCLCPP_WARN_STREAM(logger_, "OPERATIONAL state but not OPERATION_ENABLED");
          if (operationalState(slave_number, false))
          {
            b_error_cleared = true;
          }
          else
          {
            changeNMTState(slave_number, PRE_OP);
          }
        }
        else if (ec_slave[slave_number].state == EC_STATE_NONE)
        {
          RCLCPP_ERROR_STREAM(
            logger_, "Slave " << static_cast<int>(slave_number) <<
              " is lost, trying to recover ...");
          if (ec_recover_slave(slave_number, EC_TIMEOUTRET3))
          {
            RCLCPP_INFO_STREAM(logger_, "Slave " << static_cast<int>(slave_number) << " recovered");
          }
          b_error_cleared = false;
        }
        else if (ec_slave[slave_number].state == SAFE_OP_ERROR)
        {
          RCLCPP_ERROR_STREAM(
            logger_, "Slave " << static_cast<int>(slave_number) <<
              " is on SAFE_OP state with ERROR " <<
              ec_ALstatuscode2string(ec_slave[slave_number].ALstatuscode));
          (void)changeNMTState(slave_number, SAFE_OP_ERROR_ACK);
          b_error_cleared = false;
        }
        else if (ec_slave[slave_number].state == SAFE_OP)
        {
          (void)changeNMTState(slave_number, OPERATIONAL);
        }
        else
        {
          b_error_cleared = false;
          if (ec_reconfig_slave(slave_number, EC_TIMEOUTRET3))
          {
            RCLCPP_INFO_STREAM(
              logger_, "Slave " << static_cast<int>(slave_number) <<
                " reconfigured");
          }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(ERROR_CHECK_DELAY));
        end_time = clock.now();
        duration = end_time - start_time;
      }

      if (interface_timeout_ > duration.seconds())
      {
        slave_number++;
      }
      else
      {
        RCLCPP_ERROR_STREAM(
          logger_, "Interface unresponsive (duration: " << duration.seconds() <<
            " s).");
        b_interface_unresponsive_ = true;
        break;
      }
    }
    if (b_interface_unresponsive_)
    {
      break;
    }
  }

  if (b_interface_unresponsive_)
  {
    // Call shutdown to exit exec.spin();
    rclcpp::shutdown();
  }
  return;
}

void TmcCoeInterpreter::stopInterface()
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");

  /* Set b_exit_thread even with no thread created. To make sure that no threads are stucked in while loop */
  b_exit_threads_ = true;

  /* Closes the thread after it finish cleanly */
  if (processdata_thread_.joinable())
  {
    processdata_thread_.join();
    RCLCPP_INFO_STREAM(logger_, "Process Data Thread closed");
  }
  if (error_check_thread_.joinable())
  {
    error_check_thread_.join();
    RCLCPP_INFO_STREAM(logger_, "Error Check Thread closed");
  }

  if (!b_interface_unresponsive_ && (0 < ec_slavecount))
  {
    RCLCPP_INFO_STREAM(logger_, "Set all slave state to INIT");
    (void) this->changeNMTState(0, INIT);
  }

  /* stop SOEM, close socket */
  RCLCPP_INFO_STREAM(logger_, "Closing socket");
  ec_close();
}

uint8_t TmcCoeInterpreter::getCycleCounter()
{
  return cycle_counter_;
}

void TmcCoeInterpreter::startCycleCounter()
{
  b_start_cycle_count_ = true;
  return;
}

void TmcCoeInterpreter::stopCycleCounter()
{
  b_start_cycle_count_ = false;
  cycle_counter_ = 0;
  return;
}

bool TmcCoeInterpreter::isCycleFinished()
{
  return b_cycle_finished_;
}

bool TmcCoeInterpreter::readSDO(uint8_t slave_number, std::string object_name, std::string * value)
{
  RCLCPP_DEBUG_STREAM(logger_, "[" << __func__ << "] called");
  std::string data_type = "";
  std::string index_str = "";
  std::string sub_index_str = "";
  uint16_t index = 0;
  uint8_t sub_index = 0;
  std::string local_value = "";
  int object_name_index = 0;
  bool b_result = false;

  // Find in object_name in all_obj_name_
  std::vector<std::string>::iterator it_begin_object_name = all_obj_name_[slave_number].begin();
  std::vector<std::string>::iterator it_end_object_name = all_obj_name_[slave_number].end();
  std::vector<std::string>::iterator it_find_object_name = std::find(
    it_begin_object_name,
    it_end_object_name, object_name);
  if (it_find_object_name != it_end_object_name)
  {
    // Found object_name in all_obj_name_;
    object_name_index = std::distance(it_begin_object_name, it_find_object_name);

    index_str = all_index_[slave_number][object_name_index];
    index = std::stoi(index_str, nullptr, 16);

    sub_index_str = all_sub_index_[slave_number][object_name_index];
    sub_index = std::stoi(sub_index_str, nullptr, 16);

    data_type = all_datatype_[slave_number][object_name_index];
    RCLCPP_DEBUG_STREAM(
      logger_, "Object Name: " << object_name << " Index: " << index <<
        " Sub Index: " << sub_index << " Datatype: " << data_type);
    b_result = true;
  }
  else
  {
    RCLCPP_WARN_STREAM(logger_, "Did not find " << object_name);
  }

  if (b_result)
  {
    if ("UINT8" == data_type)
    {
      local_value = readSDO<uint8_t>(slave_number, index, sub_index);
    }
    else if ("UINT16" == data_type)
    {
      local_value = readSDO<uint16_t>(slave_number, index, sub_index);
    }
    else if ("UINT32" == data_type)
    {
      local_value = readSDO<uint32_t>(slave_number, index, sub_index);
    }
    else if ("INT8" == data_type)
    {
      local_value = readSDO<int8_t>(slave_number, index, sub_index);
    }
    else if ("INT16" == data_type)
    {
      local_value = readSDO<int16_t>(slave_number, index, sub_index);
    }
    else if ("INT32" == data_type)
    {
      local_value = readSDO<int32_t>(slave_number, index, sub_index);
    }
    else if ("VISIBLE_STRING" == data_type)
    {
      local_value = readSDO(slave_number, index, sub_index);
    }
    else if ("BOOL" == data_type)
    {
      local_value = readSDO<bool>(slave_number, index, sub_index);
      if (local_value == "1")
      {
        local_value = "true";
      }
      else
      {
        local_value = "false";
      }
    }
    RCLCPP_DEBUG_STREAM(logger_, "readSDO return: " << local_value);
  }

  if (local_value.empty())
  {
    RCLCPP_WARN_STREAM(logger_, "Failed reading " << object_name);
    b_result = false;
  }
  else
  {
    *value = local_value;
    b_result = true;
  }
  return b_result;
}

template<typename T>
std::string TmcCoeInterpreter::readSDO(uint8_t slave_number, uint16_t index_number,
  uint8_t subindex_number)
{
  T value;
  int value_size = sizeof(value);
  bool b_result = false;
  uint8_t n_retries = 0;
  std::string result = "";
  int workcounter;
  RCLCPP_DEBUG_STREAM(logger_, "[" << __func__ << "] (inner) called");

  while (n_retries < SDO_PDO_retries_)
  {
    workcounter = ec_SDOread(
      slave_number, index_number, subindex_number, FALSE, &value_size,
      &value, EC_TIMEOUTSAFE);
    if (workcounter > 0)
    {
      RCLCPP_DEBUG_STREAM(logger_, "SDO Read Success");
      b_result = true;
      break;
    }

    n_retries++;
    RCLCPP_DEBUG_STREAM(logger_, __func__ << " Retry " << n_retries);
    rclcpp::sleep_for(std::chrono::microseconds(6000)); //Safe tested delay time
  }

  if (!b_result)
  {
    RCLCPP_WARN_STREAM(logger_, "SDO read failed");
  }
  else
  {
    result = std::to_string(value);
  }

  return result;
}

std::string TmcCoeInterpreter::readSDO(uint8_t slave_number, uint16_t index_number,
  uint8_t subindex_number)
{
  char value_char[128];
  int value_size = sizeof(value_char) / sizeof(value_char[0]);
  bool b_result = false;
  uint8_t n_retries = 0;
  std::string result = "";
  int workcounter;
  RCLCPP_DEBUG_STREAM(logger_, "[" << __func__ << "] (inner) called");

  while (n_retries < SDO_PDO_retries_)
  {
    workcounter = ec_SDOread(
      slave_number, index_number, subindex_number, FALSE, &value_size,
      &value_char, EC_TIMEOUTSAFE);
    if (workcounter > 0)
    {
      RCLCPP_DEBUG_STREAM(logger_, "SDO Read Success");
      b_result = true;
      break;
    }

    n_retries++;
    RCLCPP_DEBUG_STREAM(logger_, __func__ << " Retry " << n_retries);
    rclcpp::sleep_for(std::chrono::microseconds(6000)); //Safe tested delay time
  }

  if (!b_result)
  {
    RCLCPP_WARN_STREAM(logger_, "SDO read failed");
  }
  else
  {
    result = std::string(value_char);
  }
  return result;
}

bool TmcCoeInterpreter::writeSDO(uint8_t slave_number, std::string object_name, std::string * value)
{
  RCLCPP_DEBUG_STREAM(logger_, "[" << __func__ << "] called");
  std::string data_type = "";
  std::string index_str = "";
  std::string sub_index_str = "";
  uint16_t index = 0;
  uint8_t sub_index = 0;
  std::string local_value = "";
  int object_name_index = 0;
  bool b_result = false;

  // Find in object_name in all_obj_name_
  std::vector<std::string>::iterator it_begin_object_name = all_obj_name_[slave_number].begin();
  std::vector<std::string>::iterator it_end_object_name = all_obj_name_[slave_number].end();
  std::vector<std::string>::iterator it_find_object_name = std::find(
    it_begin_object_name,
    it_end_object_name, object_name);
  if (it_find_object_name != it_end_object_name)
  {
    // Found object_name in all_obj_name_;
    object_name_index = std::distance(it_begin_object_name, it_find_object_name);

    index_str = all_index_[slave_number][object_name_index];
    index = std::stoi(index_str, nullptr, 16);

    sub_index_str = all_sub_index_[slave_number][object_name_index];
    sub_index = std::stoi(sub_index_str, nullptr, 16);

    data_type = all_datatype_[slave_number][object_name_index];
    RCLCPP_DEBUG_STREAM(
      logger_, "Object Name: " << object_name << " Index: " << index <<
        " Sub Index: " << std::to_string(sub_index) << " Datatype: " << data_type);
    b_result = true;
  }
  else
  {
    RCLCPP_WARN_STREAM(logger_, "Did not find " << object_name);
  }

  if (b_result)
  {
    if ("UINT8" == data_type)
    {
      uint8_t val = std::stoi(*value);
      local_value = writeSDO<uint8_t>(slave_number, index, sub_index, val);
    }
    else if ("UINT16" == data_type)
    {
      uint16_t val = std::stoi(*value);
      local_value = writeSDO<uint16_t>(slave_number, index, sub_index, val);
    }
    else if ("UINT32" == data_type)
    {
      uint32_t val = std::stoi(*value);
      local_value = writeSDO<uint32_t>(slave_number, index, sub_index, val);
    }
    else if ("INT8" == data_type)
    {
      int8_t val = std::stoi(*value);
      local_value = writeSDO<int8_t>(slave_number, index, sub_index, val);
    }
    else if ("INT16" == data_type)
    {
      int16_t val = std::stoi(*value);
      local_value = writeSDO<int16_t>(slave_number, index, sub_index, val);
    }
    else if ("INT32" == data_type)
    {
      int32_t val = std::stoi(*value);
      local_value = writeSDO<int32_t>(slave_number, index, sub_index, val);
    }
    RCLCPP_DEBUG_STREAM(logger_, "writeSDO return: " << local_value);
  }

  if (!b_result)
  {
    RCLCPP_WARN_STREAM(logger_, "Failed writing to " << object_name);
  }
  else
  {
    *value = local_value;
  }
  return b_result;
}

template<typename T>
std::string TmcCoeInterpreter::writeSDO(uint8_t slave_number, uint16_t index_number,
  uint8_t subindex_number, T value)
{
  int value_size = sizeof(value);
  bool b_result = false;
  uint8_t n_retries = 0;
  int workcounter;
  std::string result = "";
  RCLCPP_DEBUG_STREAM(logger_, "[" << __func__ << "] (inner) called");
  RCLCPP_DEBUG_STREAM(
    logger_, "writeSDO slave_number: " << std::to_string(slave_number) <<
      " index: " << index_number << " subindex: " << std::to_string(subindex_number) <<
      " value: " <<
      std::to_string(value));
  while (n_retries < SDO_PDO_retries_)
  {
    workcounter = ec_SDOwrite(
      slave_number, index_number, subindex_number, FALSE, value_size,
      &value, EC_TIMEOUTSAFE);
    if (workcounter > 0)
    {
      RCLCPP_DEBUG_STREAM(logger_, "SDO Write Success");
      result = std::to_string(value);
      b_result = true;
      break;
    }

    n_retries++;
    RCLCPP_DEBUG_STREAM(logger_, __func__ << " Retry " << n_retries);
    rclcpp::sleep_for(std::chrono::microseconds(10000)); //Safe tested delay time
  }

  if (!b_result)
  {
    RCLCPP_WARN_STREAM(logger_, "SDO write failed");
  }

  return result;
}


/* Explicit Instantiation of the template */
template std::string TmcCoeInterpreter::readSDO<uint8_t>(uint8_t slave_number,
  uint16_t index_number, uint8_t subindex_number);
template std::string TmcCoeInterpreter::readSDO<uint16_t>(uint8_t slave_number,
  uint16_t index_number, uint8_t subindex_number);
template std::string TmcCoeInterpreter::readSDO<uint32_t>(uint8_t slave_number,
  uint16_t index_number, uint8_t subindex_number);
template std::string TmcCoeInterpreter::readSDO<int8_t>(uint8_t slave_number,
  uint16_t index_number, uint8_t subindex_number);
template std::string TmcCoeInterpreter::readSDO<int16_t>(uint8_t slave_number,
  uint16_t index_number, uint8_t subindex_number);
template std::string TmcCoeInterpreter::readSDO<int32_t>(uint8_t slave_number,
  uint16_t index_number, uint8_t subindex_number);
template std::string TmcCoeInterpreter::readSDO<bool>(uint8_t slave_number, uint16_t index_number,
  uint8_t subindex_number);

template std::string TmcCoeInterpreter::writeSDO<uint8_t>(uint8_t slave_number,
  uint16_t index_number, uint8_t subindex_number, uint8_t value);
template std::string TmcCoeInterpreter::writeSDO<uint16_t>(uint8_t slave_number,
  uint16_t index_number, uint8_t subindex_number, uint16_t value);
template std::string TmcCoeInterpreter::writeSDO<uint32_t>(uint8_t slave_number,
  uint16_t index_number, uint8_t subindex_number, uint32_t value);
template std::string TmcCoeInterpreter::writeSDO<int8_t>(uint8_t slave_number,
  uint16_t index_number, uint8_t subindex_number, int8_t value);
template std::string TmcCoeInterpreter::writeSDO<int16_t>(uint8_t slave_number,
  uint16_t index_number, uint8_t subindex_number, int16_t value);
template std::string TmcCoeInterpreter::writeSDO<int32_t>(uint8_t slave_number,
  uint16_t index_number, uint8_t subindex_number, int32_t value);
