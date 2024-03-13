#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <unistd.h>
#include <functional>
#include <sched.h>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "robotis_controller/robotis_controller.h"

using namespace std::chrono_literals;
using namespace robotis_framework;

RobotisController::RobotisController()
  : is_timer_running_(false),
    is_offset_enabled_(true),
    offset_ratio_(1.0),
    stop_timer_(false),
    init_pose_loaded_(false),
    timer_thread_(),
    controller_mode_(MotionModuleMode),
    DEBUG_PRINT(false),
    robot_(0),
    gazebo_mode_(false),
    gazebo_robot_name_("robotis")
{
  direct_sync_write_.clear();
}


void RobotisController::initializeSyncWrite()
{

  for (auto& it : port_to_bulk_read_)
    it.second->txRxPacket();

  for (auto& it : port_to_bulk_read_)
  {
    int error_count = 0;
    int result = COMM_SUCCESS;
    do
    {
      if (++error_count > 10)
      {
        RCLCPP_ERROR(rclcpp::get_logger("robotis_controller"), "[RobotisController] first bulk read fail!!");
        rclcpp::shutdown();
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      result = it.second->txRxPacket();
    } while (result != COMM_SUCCESS);
  }
  init_pose_loaded_ = true;

  // clear syncwrite param setting
  for (auto& it : port_to_sync_write_position_)
  {
    if (it.second != nullptr)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_position_p_gain_)
  {
    if (it.second != nullptr)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_position_i_gain_)
  {
    if (it.second != nullptr)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_position_d_gain_)
  {
    if (it.second != nullptr)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_velocity_)
  {
    if (it.second != nullptr)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_velocity_p_gain_)
  {
    if (it.second != nullptr)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_velocity_i_gain_)
  {
    if (it.second != nullptr)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_velocity_d_gain_)
  {
    if (it.second != nullptr)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_current_)
  {
    if (it.second != nullptr)
      it.second->clearParam();
  }

  // set init syncwrite param(from data of bulkread)
  for (auto& it : robot_->dxls_)
  {
    std::string joint_name = it.first; 
    Dynamixel *dxl = it.second;

    for (size_t i = 0; i < dxl->bulk_read_items_.size(); ++i)
    {
      uint32_t read_data = 0;
      uint8_t sync_write_data[4];

      if (port_to_bulk_read_[dxl->port_name_]->isAvailable(dxl->id_,
                                                          dxl->bulk_read_items_[i]->address_,
                                                          dxl->bulk_read_items_[i]->data_length_))
      {
        read_data = port_to_bulk_read_[dxl->port_name_]->getData(dxl->id_,
                                                                dxl->bulk_read_items_[i]->address_,
                                                                dxl->bulk_read_items_[i]->data_length_);

        sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(read_data));
        sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(read_data));
        sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(read_data));
        sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(read_data));

        if ((dxl->present_position_item_ != nullptr) &&
            (dxl->bulk_read_items_[i]->item_name_ == dxl->present_position_item_->item_name_))
        {
          dxl->dxl_state_->present_position_ = dxl->convertValue2Radian(read_data) - dxl->dxl_state_->position_offset_ * offset_ratio_;
          dxl->dxl_state_->goal_position_ = dxl->dxl_state_->present_position_;

          port_to_sync_write_position_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);
        }
        else if ((dxl->position_p_gain_item_ != nullptr) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->position_p_gain_item_->item_name_))
        {
          dxl->dxl_state_->position_p_gain_ = read_data;
        }
        else if ((dxl->position_i_gain_item_ != nullptr) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->position_i_gain_item_->item_name_))
        {
          dxl->dxl_state_->position_i_gain_ = read_data;
        }
        else if ((dxl->position_d_gain_item_ != nullptr) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->position_d_gain_item_->item_name_))
        {
          dxl->dxl_state_->position_d_gain_ = read_data;
        }
        else if ((dxl->present_velocity_item_ != nullptr) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->present_velocity_item_->item_name_))
        {
          dxl->dxl_state_->present_velocity_ = dxl->convertValue2Velocity(read_data);
          dxl->dxl_state_->goal_velocity_ = dxl->dxl_state_->present_velocity_;
        }
        else if ((dxl->velocity_p_gain_item_ != nullptr) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->velocity_p_gain_item_->item_name_))
        {
          dxl->dxl_state_->velocity_p_gain_ = read_data;
        }
        else if ((dxl->velocity_i_gain_item_ != nullptr) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->velocity_i_gain_item_->item_name_))
        {
          dxl->dxl_state_->velocity_i_gain_ = read_data;
        }
        else if ((dxl->velocity_d_gain_item_ != nullptr) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->velocity_d_gain_item_->item_name_))
        {
          dxl->dxl_state_->velocity_d_gain_ = read_data;
        }
        else if ((dxl->present_current_item_ != nullptr) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->present_current_item_->item_name_))
        {
          dxl->dxl_state_->present_torque_ = dxl->convertValue2Torque(read_data);
          dxl->dxl_state_->goal_torque_ = dxl->dxl_state_->present_torque_;
        }
      }
    }
  }
}

bool RobotisController::initialize(const std::string robot_file_path, const std::string init_file_path)
{
  std::string dev_desc_dir_path = ament_index_cpp::get_package_share_directory("robotis_device") + "/devices";

  // load robot info : port , device
  robot_ = new Robot(robot_file_path, dev_desc_dir_path);


  for (auto& it : robot_->ports_)
  {
    std::string               port_name           = it.first;
    dynamixel::PortHandler   *port                = it.second;
    dynamixel::PacketHandler *default_pkt_handler = dynamixel::PacketHandler::getPacketHandler(2.0);

    if (port->setBaudRate(port->getBaudRate()) == false)
    {
      RCLCPP_ERROR(rclcpp::get_logger("robotis_controller"), "PORT [%s] SETUP ERROR! (baudrate: %d)", port_name.c_str(), port->getBaudRate());
      exit(-1);
    }

    // get the default device info of the port
    std::string default_device_name = robot_->port_default_device_[port_name];
    auto dxl_it = robot_->dxls_.find(default_device_name);
    auto sensor_it = robot_->sensors_.find(default_device_name);
    if (dxl_it != robot_->dxls_.end())
    {
      Dynamixel *default_device = dxl_it->second;
      default_pkt_handler = dynamixel::PacketHandler::getPacketHandler(default_device->protocol_version_);

      if (default_device->goal_position_item_ != 0)
      {
        port_to_sync_write_position_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->goal_position_item_->address_,
                                            default_device->goal_position_item_->data_length_);
      }

      if (default_device->position_p_gain_item_ != 0)
      {
        port_to_sync_write_position_p_gain_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->position_p_gain_item_->address_,
                                            default_device->position_p_gain_item_->data_length_);
      }

      if (default_device->position_i_gain_item_ != 0)
      {
        port_to_sync_write_position_i_gain_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->position_i_gain_item_->address_,
                                            default_device->position_i_gain_item_->data_length_);
      }

      if (default_device->position_d_gain_item_ != 0)
      {
        port_to_sync_write_position_d_gain_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->position_d_gain_item_->address_,
                                            default_device->position_d_gain_item_->data_length_);
      }

      if (default_device->goal_velocity_item_ != 0)
      {
        port_to_sync_write_velocity_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->goal_velocity_item_->address_,
                                            default_device->goal_velocity_item_->data_length_);
      }

      if (default_device->velocity_p_gain_item_ != 0)
      {
        port_to_sync_write_velocity_p_gain_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->velocity_p_gain_item_->address_,
                                            default_device->velocity_p_gain_item_->data_length_);
      }

      if (default_device->velocity_i_gain_item_ != 0)
      {
        port_to_sync_write_velocity_i_gain_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->velocity_i_gain_item_->address_,
                                            default_device->velocity_i_gain_item_->data_length_);
      }

      if (default_device->velocity_d_gain_item_ != 0)
      {
        port_to_sync_write_velocity_d_gain_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->velocity_d_gain_item_->address_,
                                            default_device->velocity_d_gain_item_->data_length_);
      }

      if (default_device->goal_current_item_ != 0)
      {
        port_to_sync_write_current_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->goal_current_item_->address_,
                                            default_device->goal_current_item_->data_length_);
      }
    }
    else if (sensor_it != robot_->sensors_.end())
    {
      Sensor *_default_device = sensor_it->second;
      default_pkt_handler = dynamixel::PacketHandler::getPacketHandler(_default_device->protocol_version_);
    }

    port_to_bulk_read_[port_name] = new dynamixel::GroupBulkRead(port, default_pkt_handler);
  }

  // (for loop) check all dxls are connected.
  for (auto& it : robot_->dxls_)
  {
    std::string joint_name  = it.first;
    Dynamixel  *dxl         = it.second;

    if (ping(joint_name) != 0)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      if (ping(joint_name) != 0)
        RCLCPP_ERROR(rclcpp::get_logger("robotis_controller"), "JOINT[%s] does NOT respond!!", joint_name.c_str());
    }
  }

  initializeDevice(init_file_path);

  queue_thread_ = std::thread(std::bind(&RobotisController::msgQueueThread, this));
  return true;
}

void RobotisController::initializeDevice(const std::string init_file_path)
{
  // device initialize
  if (DEBUG_PRINT)
    RCLCPP_WARN(rclcpp::get_logger("robotis_controller"), "INIT FILE LOAD");

  YAML::Node doc;
  try
  {
    doc = YAML::LoadFile(init_file_path.c_str());

    for (YAML::const_iterator it_doc = doc.begin(); it_doc != doc.end(); it_doc++)
    {
      std::string joint_name = it_doc->first.as<std::string>();

      YAML::Node joint_node = doc[joint_name];
      if (joint_node.size() == 0)
        continue;

      Dynamixel *dxl = NULL;
      auto dxl_it = robot_->dxls_.find(joint_name);
      if (dxl_it != robot_->dxls_.end())
        dxl = dxl_it->second;

      if (dxl == NULL)
      {
        RCLCPP_WARN(rclcpp::get_logger("robotis_controller"), "Joint [%s] was not found.", joint_name.c_str());
        continue;
      }
      if (DEBUG_PRINT)
        RCLCPP_INFO(rclcpp::get_logger("robotis_controller"), "JOINT_NAME: %s", joint_name.c_str());

      uint8_t torque_enabled = 0;
      read1Byte(joint_name, dxl->torque_enable_item_->address_, &torque_enabled);

      for (YAML::const_iterator it_joint = joint_node.begin(); it_joint != joint_node.end(); it_joint++)
      {
        std::string item_name = it_joint->first.as<std::string>();

        if (DEBUG_PRINT)
          RCLCPP_INFO(rclcpp::get_logger("robotis_controller"), "  ITEM_NAME: %s", item_name.c_str());

        uint32_t value = it_joint->second.as<uint32_t>();

        ControlTableItem *item = dxl->ctrl_table_[item_name];
        if (item == NULL)
        {
          RCLCPP_WARN(rclcpp::get_logger("robotis_controller"), "Control Item [%s] was not found.", item_name.c_str());
          continue;
        }

        if (item->memory_type_ == EEPROM)
        {
          uint8_t   data8 = 0;
          uint16_t  data16 = 0;
          uint32_t  data32 = 0;

          switch (item->data_length_)
          {
          case 1:
            read1Byte(joint_name, item->address_, &data8);
            if (data8 == value)
              continue;
            break;
          case 2:
            read2Byte(joint_name, item->address_, &data16);
            if (data16 == value)
              continue;
            break;
          case 4:
            read4Byte(joint_name, item->address_, &data32);
            if (data32 == value)
              continue;
            break;
          default:
            break;
          }

          if (torque_enabled == 1)
          {
              RCLCPP_ERROR(rclcpp::get_logger("robotis_controller"), "################\nThe initial value of the EEPROM area has been changed. \nTurn off Torque Enable and try again.");
              rclcpp::shutdown();
              return;
          }
        }

        switch (item->data_length_)
        {
        case 1:
          write1Byte(joint_name, item->address_, (uint8_t) value);
          break;
        case 2:
          write2Byte(joint_name, item->address_, (uint16_t) value);
          break;
        case 4:
          write4Byte(joint_name, item->address_, value);
          break;
        default:
          break;
        }

        if (item->memory_type_ == EEPROM)
        {
          // Write to EEPROM -> delay is required (max delay: 55 msec per byte)
          std::this_thread::sleep_for(std::chrono::milliseconds(item->data_length_ * 55));
        }
      }
    }
  } catch (const std::exception& e)
  {
    RCLCPP_INFO(rclcpp::get_logger("robotis_controller"), "Dynamixel Init file not found.");
  }

  // [ BulkRead ] StartAddress : Present Position , Length : 10 ( Position/Velocity/Current )
  for (auto& it : robot_->ports_)
  {
    if (port_to_bulk_read_[it.first] != 0)
      port_to_bulk_read_[it.first]->clearParam();
  }
  for (auto& it : robot_->dxls_)
  {
    std::string joint_name  = it.first;
    Dynamixel  *dxl         = it.second;

    if (dxl == NULL)
      continue;

    int bulkread_start_addr = 0;
    int bulkread_data_length = 0;

    uint8_t torque_enabled = 0;
    read1Byte(joint_name, dxl->torque_enable_item_->address_, &torque_enabled);

    // calculate bulk read start address & data length
    auto indirect_addr_it = dxl->ctrl_table_.find(INDIRECT_ADDRESS_1);
    if (indirect_addr_it != dxl->ctrl_table_.end()) // INDIRECT_ADDRESS_1 exist
    {
      if (dxl->bulk_read_items_.size() != 0)
      {
        uint16_t  data16 = 0;

        bulkread_start_addr = dxl->bulk_read_items_[0]->address_;
        bulkread_data_length = 0;

        // set indirect address
        int indirect_addr = indirect_addr_it->second->address_;
        for (int i = 0; i < dxl->bulk_read_items_.size(); i++)
        {
          int addr_leng = dxl->bulk_read_items_[i]->data_length_;

          bulkread_data_length += addr_leng;
          for (int l = 0; l < addr_leng; l++)
          {
            read2Byte(joint_name, indirect_addr, &data16);
            if (data16 != dxl->ctrl_table_[dxl->bulk_read_items_[i]->item_name_]->address_ + l)
            {
              if (torque_enabled == 1)
              {
                RCLCPP_ERROR(rclcpp::get_logger("my_node"), "################\nThe indirect address of the EEPROM area has been changed.\nTurn off Torque Enable and try again.");
                rclcpp::shutdown();
                return;
              }
              write2Byte(joint_name, indirect_addr, dxl->ctrl_table_[dxl->bulk_read_items_[i]->item_name_]->address_ + l);
            }
            indirect_addr += 2;
          }
        }
      }
    }
    else    // INDIRECT_ADDRESS_1 NOT exist
    {
      if (dxl->bulk_read_items_.size() != 0)
      {
        bulkread_start_addr = dxl->bulk_read_items_[0]->address_;
        bulkread_data_length = 0;

        ControlTableItem *last_item = dxl->bulk_read_items_[0];

        for (int i = 0; i < dxl->bulk_read_items_.size(); i++)
        {
          int addr = dxl->bulk_read_items_[i]->address_;
          if (addr < bulkread_start_addr)
            bulkread_start_addr = addr;
          else if (last_item->address_ < addr)
            last_item = dxl->bulk_read_items_[i];
        }

        bulkread_data_length = last_item->address_ - bulkread_start_addr + last_item->data_length_;
      }
    }

    if (bulkread_start_addr != 0)
      port_to_bulk_read_[dxl->port_name_]->addParam(dxl->id_, bulkread_start_addr, bulkread_data_length);

    // Torque ON
    if (writeCtrlItem(joint_name, dxl->torque_enable_item_->item_name_, 1) != COMM_SUCCESS)
      writeCtrlItem(joint_name, dxl->torque_enable_item_->item_name_, 1);
  }

  for (auto& it : robot_->sensors_)
  {
    std::string sensor_name = it.first;
    Sensor     *sensor      = it.second;

    if (sensor == NULL)
      continue;

    int bulkread_start_addr = 0;
    int bulkread_data_length = 0;

    // calculate bulk read start address & data length
    auto indirect_addr_it = sensor->ctrl_table_.find(INDIRECT_ADDRESS_1);
    if (indirect_addr_it != sensor->ctrl_table_.end()) // INDIRECT_ADDRESS_1 exist
    {
      if (sensor->bulk_read_items_.size() != 0)
      {
        uint16_t  data16 = 0;

        bulkread_start_addr = sensor->bulk_read_items_[0]->address_;
        bulkread_data_length = 0;

        // set indirect address
        int indirect_addr = indirect_addr_it->second->address_;
        for (int i = 0; i < sensor->bulk_read_items_.size(); i++)
        {
          int addr_leng = sensor->bulk_read_items_[i]->data_length_;

          bulkread_data_length += addr_leng;
          for (int l = 0; l < addr_leng; l++)
          {
            read2Byte(sensor_name, indirect_addr, &data16);
            if (data16 != sensor->ctrl_table_[sensor->bulk_read_items_[i]->item_name_]->address_ + l)
            {
              write2Byte(sensor_name,
                         indirect_addr,
                         sensor->ctrl_table_[sensor->bulk_read_items_[i]->item_name_]->address_ + l);
            }
            indirect_addr += 2;
          }
        }
      }
    }
    else    // INDIRECT_ADDRESS_1 NOT exist
    {
      if (sensor->bulk_read_items_.size() != 0)
      {
        bulkread_start_addr = sensor->bulk_read_items_[0]->address_;
        bulkread_data_length = 0;

        ControlTableItem *last_item = sensor->bulk_read_items_[0];

        for (int i = 0; i < sensor->bulk_read_items_.size(); i++)
        {
          int addr = sensor->bulk_read_items_[i]->address_;
          if (addr < bulkread_start_addr)
            bulkread_start_addr = addr;
          else if (last_item->address_ < addr)
            last_item = sensor->bulk_read_items_[i];
        }

        bulkread_data_length = last_item->address_ - bulkread_start_addr + last_item->data_length_;
      }
    }

    if (bulkread_start_addr != 0)
      port_to_bulk_read_[sensor->port_name_]->addParam(sensor->id_, bulkread_start_addr, bulkread_data_length);
  }
}


void RobotisController::msgQueueThread()
{
  auto node = std::make_shared<rclcpp::Node>("robotis_controller");
  
  /* subscriber */
  auto write_control_table_sub = node->create_subscription<robotis_controller_msgs::msg::WriteControlTable>(
    "/robotis/write_control_table", 5,
    std::bind(&RobotisController::writeControlTableCallback, this, std::placeholders::_1));

  auto sync_write_item_sub = node->create_subscription<robotis_controller_msgs::msg::SyncWriteItem>(
    "/robotis/sync_write_item", 10,
    std::bind(&RobotisController::syncWriteItemCallback, this, std::placeholders::_1));

  auto joint_ctrl_modules_sub = node->create_subscription<robotis_controller_msgs::msg::JointCtrlModule>(
    "/robotis/set_joint_ctrl_modules", 10,
    std::bind(&RobotisController::setJointCtrlModuleCallback, this, std::placeholders::_1));

  auto enable_ctrl_module_sub = node->create_subscription<std_msgs::msg::Bool>(
    "/robotis/enable_ctrl_module", 10,
    std::bind(&RobotisController::setCtrlModuleCallback, this, std::placeholders::_1));

  auto control_mode_sub = node->create_subscription<std_msgs::msg::String>(
    "/robotis/set_control_mode", 10,
    std::bind(&RobotisController::setControllerModeCallback, this, std::placeholders::_1));

  auto joint_states_sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "/robotis/set_joint_states", 10,
    std::bind(&RobotisController::setJointStatesCallback, this, std::placeholders::_1));

  auto enable_offset_sub = node->create_subscription<std_msgs::msg::Bool>(
    "/robotis/enable_offset", 10,
    std::bind(&RobotisController::enableOffsetCallback, this, std::placeholders::_1));

  /* publisher */
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr goal_joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr present_joint_state_pub_;
  rclcpp::Publisher<robotis_controller_msgs::msg::JointCtrlModule>::SharedPtr current_module_pub_;
  goal_joint_state_pub_ = node->create_publisher<sensor_msgs::msg::JointState>("/robotis/goal_joint_states", 10);
  present_joint_state_pub_ = node->create_publisher<sensor_msgs::msg::JointState>("/robotis/present_joint_states", 10);
  current_module_pub_ = node->create_publisher<robotis_controller_msgs::msg::JointCtrlModule>(
    "/robotis/present_joint_ctrl_modules", 10);

  /* service */
  auto get_joint_module_server = node->create_service<robotis_controller_msgs::srv::GetJointModule>(
    "/robotis/get_present_joint_ctrl_modules",
    std::bind(&RobotisController::getJointCtrlModuleService, this, std::placeholders::_1, std::placeholders::_2));

  auto set_joint_module_server = node->create_service<robotis_controller_msgs::srv::SetJointModule>(
    "/robotis/set_present_joint_ctrl_modules",
    std::bind(&RobotisController::setJointCtrlModuleService, this, std::placeholders::_1, std::placeholders::_2));

  auto set_module_server = node->create_service<robotis_controller_msgs::srv::SetModule>(
    "/robotis/set_present_ctrl_modules",
    std::bind(&RobotisController::setCtrlModuleService, this, std::placeholders::_1, std::placeholders::_2));

  auto load_offset_server = node->create_service<robotis_controller_msgs::srv::LoadOffset>(
    "/robotis/load_offset",
    std::bind(&RobotisController::loadOffsetService, this, std::placeholders::_1, std::placeholders::_2));

  rclcpp::WallRate loop_rate(1000.0 / robot_->getControlCycle());

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
}

void RobotisController::timerThread()
{
  rclcpp::Time next_time = rclcpp::Clock().now();
  rclcpp::Time curr_time;

  RCLCPP_DEBUG(rclcpp::get_logger("robotis_controller"), "controller::thread_proc started");

  while (rclcpp::ok() && !stop_timer_)
  {
    next_time += rclcpp::Duration(robot_->getControlCycle() * 1000000ns);

    process();

    curr_time = rclcpp::Clock().now();

    auto delta_nsec = (next_time - curr_time).nanoseconds();
    if (delta_nsec < -100000)
    {
      if (DEBUG_PRINT)
      {
        RCLCPP_WARN(rclcpp::get_logger("robotis_controller"),
                    "[RobotisController::ThreadProc] NEXT TIME < CURR TIME.. (%f)[%ld.%09ld / %ld.%09ld]",
                    delta_nsec / 1000000.0, next_time.nanoseconds() / 1000000000, next_time.nanoseconds() % 1000000000,
                    curr_time.nanoseconds() / 1000000000, curr_time.nanoseconds() % 1000000000);
      }

      // next_time = curr_time + 3 msec
      next_time = curr_time + rclcpp::Duration(3ms);
    }

     //sleep for
  }
}


void RobotisController::startTimer()
{
    if (this->is_timer_running_ == true)
        return;

    else
    {
        initializeSyncWrite();
        for (auto& it : port_to_bulk_read_)
        {
            it.second->txPacket();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(8));

        // create and start the thread
        timer_thread_ = std::thread(&RobotisController::timerThread, this);

        // Set the thread priority and scheduler
        sched_param param;
        int policy;
        pthread_getschedparam(timer_thread_.native_handle(), &policy, &param);
        param.sched_priority = sched_get_priority_max(SCHED_RR);
        if (pthread_setschedparam(timer_thread_.native_handle(), SCHED_RR, &param) != 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("robotis_controller"), "Failed to set timer thread priority");
        }
    }

    this->is_timer_running_ = true;
}

void RobotisController::stopTimer()
{
  // set the flag to stop the thread
  if (this->is_timer_running_)
  {
    this->stop_timer_ = true;

    if (!this->gazebo_mode_)
    {
      // wait until the thread is stopped.
      if (this->timer_thread_.joinable())
        this->timer_thread_.join();

      for (auto& it : port_to_bulk_read_)
      {
        if (it.second != NULL)
          it.second->rxPacket();
      }

      for (auto& it : port_to_sync_write_position_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }

      for (auto& it : port_to_sync_write_position_p_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }

      for (auto& it : port_to_sync_write_position_i_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }

      for (auto& it : port_to_sync_write_position_d_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }

      for (auto& it : port_to_sync_write_velocity_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }

      for (auto& it : port_to_sync_write_velocity_p_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }

      for (auto& it : port_to_sync_write_velocity_i_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }

      for (auto& it : port_to_sync_write_velocity_d_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }

      for (auto& it : port_to_sync_write_current_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
    }

    this->stop_timer_ = false;
    this->is_timer_running_ = false;
  }
}

bool RobotisController::isTimerRunning()
{
  return this->is_timer_running_;
}

void RobotisController::loadOffset(const std::string path)
{
  YAML::Node doc;
  try
  {
    doc = YAML::LoadFile(path.c_str());
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(rclcpp::get_logger("robotis_controller"), "Fail to load offset yaml.");
    return;
  }

  YAML::Node offset_node = doc["offset"];
  if (offset_node.size() == 0)
    return;

  RCLCPP_INFO(rclcpp::get_logger("robotis_controller"), "Load offsets...");
  for (YAML::const_iterator it = offset_node.begin(); it != offset_node.end(); it++)
  {
    std::string joint_name = it->first.as<std::string>();
    double offset = it->second.as<double>();

    auto dxl_it = robot_->dxls_.find(joint_name);
    if (dxl_it != robot_->dxls_.end())
      dxl_it->second->dxl_state_->position_offset_ = offset;
  }
}

void RobotisController::process()
{
  // avoid duplicated function call
  static bool is_process_running = false;
  if (is_process_running == true)
    return;
  is_process_running = true;

  // offset ratio
  if (is_offset_enabled_)
  {
    if (offset_ratio_ < 1.0)
      offset_ratio_ += 0.01;
    else
      offset_ratio_ = 1.0;
  }
  else
  {
    if (offset_ratio_ > 0.0)
      offset_ratio_ -= 0.01;
    else
      offset_ratio_ = 0.0;
  }

  rclcpp::Time start_time;
  rclcpp::Duration time_duration(0,0);

  if (DEBUG_PRINT)
    start_time = rclcpp::Clock().now();

  sensor_msgs::msg::JointState goal_state;
  sensor_msgs::msg::JointState present_state;

  present_state.header.stamp = rclcpp::Clock().now();
  goal_state.header.stamp = present_state.header.stamp;

  if (controller_mode_ == MotionModuleMode)
  {
    if (gazebo_mode_ == false)
    {
      // BulkRead Rx
      for (auto& it : port_to_bulk_read_)
      {
        robot_->ports_[it.first]->setPacketTimeout(0.0);
        if (DEBUG_PRINT)
        {
          int result = it.second->rxPacket();
          if (result != COMM_SUCCESS)
            RCLCPP_ERROR(rclcpp::get_logger("robotis_controller"), "Bulk Read Fail : %s", it.first.c_str());
        }
        else
          it.second->rxPacket();
      }

      // -> save to robot->dxls_[]->dxl_state_
      if (robot_->dxls_.size() > 0)
      {
        for (auto& dxl_it : robot_->dxls_)
        {
          Dynamixel* dxl = dxl_it.second;
          std::string port_name = dxl_it.second->port_name_;
          std::string joint_name = dxl_it.first;

          if (dxl->bulk_read_items_.size() > 0)
          {
            bool updated = false;
            uint32_t data = 0;
            for (int i = 0; i < dxl->bulk_read_items_.size(); i++)
            {
              ControlTableItem* item = dxl->bulk_read_items_[i];
              if (port_to_bulk_read_[port_name]->isAvailable(dxl->id_, item->address_, item->data_length_) == true)
              {
                updated = true;
                data = port_to_bulk_read_[port_name]->getData(dxl->id_, item->address_, item->data_length_);

                // change dxl_state
                if (dxl->present_position_item_ != 0 && item->item_name_ == dxl->present_position_item_->item_name_)
                {
                  dxl->dxl_state_->present_position_ = dxl->convertValue2Radian(data) - dxl->dxl_state_->position_offset_ * offset_ratio_;
                }
                else if (dxl->present_velocity_item_ != 0 && item->item_name_ == dxl->present_velocity_item_->item_name_)
                  dxl->dxl_state_->present_velocity_ = dxl->convertValue2Velocity(data);
                else if (dxl->present_current_item_ != 0 && item->item_name_ == dxl->present_current_item_->item_name_)
                  dxl->dxl_state_->present_torque_ = dxl->convertValue2Torque(data);
                else if (dxl->goal_position_item_ != 0 && item->item_name_ == dxl->goal_position_item_->item_name_)
                {
                  dxl->dxl_state_->goal_position_ = dxl->convertValue2Radian(data) - dxl->dxl_state_->position_offset_ * offset_ratio_;
                }
                else if (dxl->goal_velocity_item_ != 0 && item->item_name_ == dxl->goal_velocity_item_->item_name_)
                  dxl->dxl_state_->goal_velocity_ = dxl->convertValue2Velocity(data);
                else if (dxl->goal_current_item_ != 0 && item->item_name_ == dxl->goal_current_item_->item_name_)
                  dxl->dxl_state_->goal_torque_ = dxl->convertValue2Torque(data);

                dxl->dxl_state_->bulk_read_table_[item->item_name_] = data;
              }
            }

            // -> update time stamp to Robot->dxls[]->dynamixel_state.update_time_stamp
            if (updated == true)
              dxl->dxl_state_->update_time_stamp_ = TimeStamp(present_state.header.stamp.sec, present_state.header.stamp.nanosec);
          }
        }
      }

      // -> save to robot->sensors_[]->sensor_state_
      if (robot_->sensors_.size() > 0)
      {
        for (auto& sensor_it : robot_->sensors_)
        {
          Sensor* sensor = sensor_it.second;
          std::string port_name = sensor_it.second->port_name_;
          std::string sensor_name = sensor_it.first;

          if (sensor->bulk_read_items_.size() > 0)
          {
            bool updated = false;
            uint32_t data = 0;
            for (int i = 0; i < sensor->bulk_read_items_.size(); i++)
            {
              ControlTableItem* item = sensor->bulk_read_items_[i];
              if (port_to_bulk_read_[port_name]->isAvailable(sensor->id_, item->address_, item->data_length_) == true)
              {
                updated = true;
                data = port_to_bulk_read_[port_name]->getData(sensor->id_, item->address_, item->data_length_);

                // change sensor_state
                sensor->sensor_state_->bulk_read_table_[item->item_name_] = data;
              }
            }

            // -> update time stamp to Robot->dxls[]->dynamixel_state.update_time_stamp
            if (updated == true)
              sensor->sensor_state_->update_time_stamp_ = TimeStamp(present_state.header.stamp.sec, present_state.header.stamp.nanosec);
          }
        }
      }

      if (DEBUG_PRINT)
      {
        time_duration = rclcpp::Clock().now() - start_time;
        RCLCPP_INFO(rclcpp::get_logger("robotis_controller"), "(%2.6f) BulkRead Rx & update state", time_duration.nanoseconds() * 0.000001);
      }

      // SyncWrite
      queue_mutex_.lock();

      if (direct_sync_write_.size() > 0)
      {
        for (int i = 0; i < direct_sync_write_.size(); i++)
        {
          direct_sync_write_[i]->txPacket();
          direct_sync_write_[i]->clearParam();
        }
        direct_sync_write_.clear();
      }

      if (port_to_sync_write_position_p_gain_.size() > 0)
      {
        for (auto& it : port_to_sync_write_position_p_gain_)
        {
          it.second->txPacket();
          it.second->clearParam();
        }
      }
      if (port_to_sync_write_position_i_gain_.size() > 0)
      {
        for (auto& it : port_to_sync_write_position_i_gain_)
        {
          it.second->txPacket();
          it.second->clearParam();
        }
      }
      if (port_to_sync_write_position_d_gain_.size() > 0)
      {
        for (auto& it : port_to_sync_write_position_d_gain_)
        {
          it.second->txPacket();
          it.second->clearParam();
        }
      }
      if (port_to_sync_write_velocity_p_gain_.size() > 0)
      {
        for (auto& it : port_to_sync_write_velocity_p_gain_)
        {
          it.second->txPacket();
          it.second->clearParam();
        }
      }
      if (port_to_sync_write_velocity_i_gain_.size() > 0)
      {
        for (auto& it : port_to_sync_write_velocity_i_gain_)
        {
          it.second->txPacket();
          it.second->clearParam();
        }
      }
      if (port_to_sync_write_velocity_d_gain_.size() > 0)
      {
        for (auto& it : port_to_sync_write_velocity_d_gain_)
        {
          it.second->txPacket();
          it.second->clearParam();
        }
      }
      for (auto& it : port_to_sync_write_position_)
      {
        if (it.second != NULL)
          it.second->txPacket();
      }
      for (auto& it : port_to_sync_write_velocity_)
      {
        if (it.second != NULL)
          it.second->txPacket();
      }
      for (auto& it : port_to_sync_write_current_)
      {
        if (it.second != NULL)
          it.second->txPacket();
      }

      queue_mutex_.unlock();

      // BulkRead Tx
      for (auto& it : port_to_bulk_read_)
        it.second->txPacket();

      if (DEBUG_PRINT)
      {
        time_duration = rclcpp::Clock().now() - start_time;
        RCLCPP_INFO(rclcpp::get_logger("robotis_controller"), "(%2.6f) SyncWrite & BulkRead Tx", time_duration.nanoseconds() * 0.000001);
      }
    }
  }
  else if (controller_mode_ == DirectControlMode)
  {
    if (gazebo_mode_ == false)
    {
      // BulkRead Rx
      for (auto& it : port_to_bulk_read_)
      {
        robot_->ports_[it.first]->setPacketTimeout(0.0);
        it.second->rxPacket();
      }

      // -> save to robot->dxls_[]->dxl_state_
      if (robot_->dxls_.size() > 0)
      {
        for (auto& dxl_it : robot_->dxls_)
        {
          Dynamixel* dxl = dxl_it.second;
          std::string port_name = dxl_it.second->port_name_;
          std::string joint_name = dxl_it.first;

          if (dxl->bulk_read_items_.size() > 0)
          {
            uint32_t data = 0;
            for (int i = 0; i < dxl->bulk_read_items_.size(); i++)
            {
              ControlTableItem* item = dxl->bulk_read_items_[i];
              if (port_to_bulk_read_[port_name]->isAvailable(dxl->id_, item->address_, item->data_length_) == true)
              {
                data = port_to_bulk_read_[port_name]->getData(dxl->id_, item->address_, item->data_length_);

                // change dxl_state
                if (dxl->present_position_item_ != 0 && item->item_name_ == dxl->present_position_item_->item_name_)
                {
                  dxl->dxl_state_->present_position_ = dxl->convertValue2Radian(data) - dxl->dxl_state_->position_offset_ * offset_ratio_;
                }
                else if (dxl->present_velocity_item_ != 0 && item->item_name_ == dxl->present_velocity_item_->item_name_)
                  dxl->dxl_state_->present_velocity_ = dxl->convertValue2Velocity(data);
                else if (dxl->present_current_item_ != 0 && item->item_name_ == dxl->present_current_item_->item_name_)
                  dxl->dxl_state_->present_torque_ = dxl->convertValue2Torque(data);
                else if (dxl->goal_position_item_ != 0 && item->item_name_ == dxl->goal_position_item_->item_name_)
                {
                  dxl->dxl_state_->goal_position_ = dxl->convertValue2Radian(data) - dxl->dxl_state_->position_offset_ * offset_ratio_;
                }
                else if (dxl->goal_velocity_item_ != 0 && item->item_name_ == dxl->goal_velocity_item_->item_name_)
                  dxl->dxl_state_->goal_velocity_ = dxl->convertValue2Velocity(data);
                else if (dxl->goal_current_item_ != 0 && item->item_name_ == dxl->goal_current_item_->item_name_)
                  dxl->dxl_state_->goal_torque_ = dxl->convertValue2Torque(data);

                dxl->dxl_state_->bulk_read_table_[item->item_name_] = data;
              }
            }

            // -> update time stamp to Robot->dxls[]->dynamixel_state.update_time_stamp
            dxl->dxl_state_->update_time_stamp_ = TimeStamp(present_state.header.stamp.sec, present_state.header.stamp.nanosec);
          }
        }
      }

      queue_mutex_.lock();

      if (direct_sync_write_.size() > 0)
      {
        for (int i = 0; i < direct_sync_write_.size(); i++)
        {
          direct_sync_write_[i]->txPacket();
          direct_sync_write_[i]->clearParam();
        }
        direct_sync_write_.clear();
      }

      queue_mutex_.unlock();

      // BulkRead Tx
      for (auto& it : port_to_bulk_read_)
        it.second->txPacket();
    }
  }

  // Call SensorModule Process()
  // -> for loop : call SensorModule list -> Process()
  if (sensor_modules_.size() > 0)
  {
    for (auto module_it = sensor_modules_.begin(); module_it != sensor_modules_.end(); module_it++)
    {
      (*module_it)->process(robot_->dxls_, robot_->sensors_);

      for (auto& it : (*module_it)->result_)
        sensor_result_[it.first] = it.second;
    }
  }
  
  if (DEBUG_PRINT)
  {
    time_duration = rclcpp::Time() - start_time;
    fprintf(stderr, "(%2.6f) Process() DONE \n", time_duration.seconds());
  }

  is_process_running = false;
}

void RobotisController::addMotionModule(MotionModule* module)
{
  // check whether the module name already exists
  for (auto m_it = motion_modules_.begin(); m_it != motion_modules_.end(); m_it++)
  {
    if ((*m_it)->getModuleName() == module->getModuleName())
    {
      RCLCPP_ERROR(rclcpp::get_logger("robotis_controller"), "Motion Module Name [%s] already exist !!", module->getModuleName().c_str());
      return;
    }
  }

  module->initialize(robot_->getControlCycle(), robot_);
  motion_modules_.push_back(module);
  motion_modules_.unique();
}

void RobotisController::removeMotionModule(MotionModule* module)
{
  motion_modules_.remove(module);
}

void RobotisController::addSensorModule(SensorModule* module)
{
  // check whether the module name already exists
  for (auto m_it = sensor_modules_.begin(); m_it != sensor_modules_.end(); m_it++)
  {
    if ((*m_it)->getModuleName() == module->getModuleName())
    {
      RCLCPP_ERROR(rclcpp::get_logger("robotis_controller"), "Sensor Module Name [%s] already exists !!", module->getModuleName().c_str());
      return;
    }
  }

  module->initialize(robot_->getControlCycle(), robot_);
  sensor_modules_.push_back(module);
  sensor_modules_.unique();
}

void RobotisController::removeSensorModule(SensorModule* module)
{
  sensor_modules_.remove(module);
}

void RobotisController::writeControlTableCallback(const robotis_controller_msgs::msg::WriteControlTable::SharedPtr msg)
{
  Device* device = NULL;

  if (DEBUG_PRINT)
    RCLCPP_INFO(rclcpp::get_logger("robotis_controller"), "[WriteControlTable] led control msg received");

  auto dev_it1 = robot_->dxls_.find(msg->joint_name);
  if (dev_it1 != robot_->dxls_.end())
  {
    device = dev_it1->second;
  }
  else
  {
    auto dev_it2 = robot_->sensors_.find(msg->joint_name);
    if (dev_it2 != robot_->sensors_.end())
    {
      device = dev_it2->second;
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("robotis_controller"), "[WriteControlTable] Unknown device : %s", msg->joint_name.c_str());
      return;
    }
  }

  ControlTableItem* item = NULL;
  auto item_it = device->ctrl_table_.find(msg->start_item_name);
  if (item_it != device->ctrl_table_.end())
  {
    item = item_it->second;
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("robotis_controller"), "[WriteControlTable] Unknown item : %s", msg->start_item_name.c_str());
    return;
  }

  dynamixel::PortHandler* port = robot_->ports_[device->port_name_];
  dynamixel::PacketHandler* packet_handler = dynamixel::PacketHandler::getPacketHandler(device->protocol_version_);

  if (item->access_type_ == Read)
    return;

  queue_mutex_.lock();

  direct_sync_write_.push_back(new dynamixel::GroupSyncWrite(port, packet_handler, item->address_, msg->data_length));
  direct_sync_write_[direct_sync_write_.size() - 1]->addParam(device->id_, (uint8_t*)(msg->data.data()));

  queue_mutex_.unlock();
}

void RobotisController::syncWriteItemCallback(const robotis_controller_msgs::msg::SyncWriteItem::SharedPtr msg)
{
  for (size_t i = 0; i < msg->joint_name.size(); i++)
  {
    Device* device;

    auto d_it1 = robot_->dxls_.find(msg->joint_name[i]);
    if (d_it1 != robot_->dxls_.end())
    {
      device = d_it1->second;
    }
    else
    {
      auto d_it2 = robot_->sensors_.find(msg->joint_name[i]);
      if (d_it2 != robot_->sensors_.end())
      {
        device = d_it2->second;
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger("robotis_controller"), "[SyncWriteItem] Unknown device : %s", msg->joint_name[i].c_str());
        continue;
      }
    }

    ControlTableItem* item = NULL;
    auto item_it = device->ctrl_table_.find(msg->item_name);
    if (item_it != device->ctrl_table_.end())
    {
      item = item_it->second;
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("robotis_controller"), "[SyncWriteItem] Unknown item : %s", msg->item_name.c_str());
      continue;
    }

    dynamixel::PortHandler* port = robot_->ports_[device->port_name_];
    dynamixel::PacketHandler* packet_handler = dynamixel::PacketHandler::getPacketHandler(device->protocol_version_);

    if (item->access_type_ == Read)
      continue;

    queue_mutex_.lock();

    size_t idx = 0;
    if (direct_sync_write_.size() == 0)
    {
      direct_sync_write_.push_back(new dynamixel::GroupSyncWrite(port, packet_handler, item->address_, item->data_length_));
      idx = 0;
    }
    else
    {
      for (idx = 0; idx < direct_sync_write_.size(); idx++)
      {
        if (direct_sync_write_[idx]->getPortHandler() == port && direct_sync_write_[idx]->getPacketHandler() == packet_handler)
          break;
      }

      if (idx == direct_sync_write_.size())
        direct_sync_write_.push_back(new dynamixel::GroupSyncWrite(port, packet_handler, item->address_, item->data_length_));
    }

    uint8_t* data = new uint8_t[item->data_length_];
    if (item->data_length_ == 1)
      data[0] = (uint8_t)msg->value[i];
    else if (item->data_length_ == 2)
    {
      data[0] = DXL_LOBYTE((uint16_t)msg->value[i]);
      data[1] = DXL_HIBYTE((uint16_t)msg->value[i]);
    }
    else if (item->data_length_ == 4)
    {
      data[0] = DXL_LOBYTE(DXL_LOWORD((uint32_t)msg->value[i]));
      data[1] = DXL_HIBYTE(DXL_LOWORD((uint32_t)msg->value[i]));
      data[2] = DXL_LOBYTE(DXL_HIWORD((uint32_t)msg->value[i]));
      data[3] = DXL_HIBYTE(DXL_HIWORD((uint32_t)msg->value[i]));
    }
    direct_sync_write_[idx]->addParam(device->id_, data);
    delete[] data;

    queue_mutex_.unlock();
  }
}

void RobotisController::setControllerModeCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "DirectControlMode")
  {
    for (auto& it : port_to_bulk_read_)
    {
      robot_->ports_[it.first]->setPacketTimeout(0.0);
      it.second->rxPacket();
    }
    controller_mode_ = DirectControlMode;
  }
  else if (msg->data == "MotionModuleMode")
  {
    for (auto& it : port_to_bulk_read_)
    {
      it.second->txPacket();
    }
    controller_mode_ = MotionModuleMode;
  }
}

void RobotisController::setJointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  queue_mutex_.lock();

  for (size_t i = 0; i < msg->name.size(); i++)
  {
    Dynamixel* dxl = robot_->dxls_[msg->name[i]];
    if (dxl == NULL)
      continue;

    if ((controller_mode_ == DirectControlMode) ||
        (controller_mode_ == MotionModuleMode && dxl->ctrl_module_name_ == "none"))
    {
      dxl->dxl_state_->goal_position_ = (double)msg->position[i];

      if (gazebo_mode_ == false)
      {
        // add offset
        uint32_t pos_data;
        pos_data = dxl->convertRadian2Value(dxl->dxl_state_->goal_position_ + dxl->dxl_state_->position_offset_ * offset_ratio_);

        uint8_t sync_write_data[4] = { 0 };
        sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(pos_data));
        sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(pos_data));
        sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(pos_data));
        sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(pos_data));

        if (port_to_sync_write_position_[dxl->port_name_] != NULL)
          port_to_sync_write_position_[dxl->port_name_]->changeParam(dxl->id_, sync_write_data);
      }
    }
  }

  queue_mutex_.unlock();
}

void RobotisController::setCtrlModuleCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (set_module_thread_.joinable())
    set_module_thread_.join();

  std::string _module_name_to_set = msg->data;

  set_module_thread_ = std::thread(std::bind(&RobotisController::setCtrlModuleThread, this, _module_name_to_set));
}

void RobotisController::setCtrlModule(std::string module_name)
{
  if (set_module_thread_.joinable())
    set_module_thread_.join();

  set_module_thread_ = std::thread(std::bind(&RobotisController::setCtrlModuleThread, this, module_name));
}

void RobotisController::setJointCtrlModuleCallback(const robotis_controller_msgs::msg::JointCtrlModule::SharedPtr msg)
{
  if (msg->joint_name.size() != msg->module_name.size())
    return;

  if (set_module_thread_.joinable())
    set_module_thread_.join();

  set_module_thread_ = std::thread(std::bind(&RobotisController::setJointCtrlModuleThread, this, msg));
}

void RobotisController::enableOffsetCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  is_offset_enabled_ = (bool)msg->data;
  if (is_offset_enabled_)
    offset_ratio_ = 0.0;
  else
    offset_ratio_ = 1.0;
}

bool RobotisController::getJointCtrlModuleService(
    robotis_controller_msgs::srv::GetJointModule::Request::SharedPtr req,
    robotis_controller_msgs::srv::GetJointModule::Response::SharedPtr res)
{
    for (size_t idx = 0; idx < req->joint_name.size(); idx++)
    {
        auto d_it = robot_->dxls_.find((std::string)(req->joint_name[idx]));
        if (d_it != robot_->dxls_.end())
        {
            res->joint_name.push_back(req->joint_name[idx]);
            res->module_name.push_back(d_it->second->ctrl_module_name_);
        }
    }
    return true;
}

bool RobotisController::setJointCtrlModuleService(
    robotis_controller_msgs::srv::SetJointModule::Request::SharedPtr req,
    robotis_controller_msgs::srv::SetJointModule::Response::SharedPtr res)
{
    if (set_module_thread_.joinable())
        set_module_thread_.join();

    robotis_controller_msgs::msg::JointCtrlModule modules;
    modules.joint_name = req->joint_name;
    modules.module_name = req->module_name;

    std::shared_ptr<robotis_controller_msgs::msg::JointCtrlModule> msg_ptr =
        std::make_shared<robotis_controller_msgs::msg::JointCtrlModule>(modules);

    if (modules.joint_name.size() != modules.module_name.size())
    {
        res->result = false;
        return false;
    }

    set_module_thread_ = std::thread(std::bind(&RobotisController::setJointCtrlModuleThread, this, msg_ptr));
    set_module_thread_.detach();

    res->result = true;
    return true;
}

bool RobotisController::setCtrlModuleService(
    robotis_controller_msgs::srv::SetModule::Request::SharedPtr req,
    robotis_controller_msgs::srv::SetModule::Response::SharedPtr res)
{
    if (set_module_thread_.joinable())
        set_module_thread_.join();

    std::string _module_name_to_set = req->module_name;

    set_module_thread_ = std::thread(std::bind(&RobotisController::setCtrlModuleThread, this, _module_name_to_set));
    set_module_thread_.detach();

    res->result = true;
    return true;
}

bool RobotisController::loadOffsetService(
    robotis_controller_msgs::srv::LoadOffset::Request::SharedPtr req,
    robotis_controller_msgs::srv::LoadOffset::Response::SharedPtr res)
{
    loadOffset((std::string)req->file_path);
    res->result = true;
    return true;
}

void RobotisController::setJointCtrlModuleThread(const robotis_controller_msgs::msg::JointCtrlModule::SharedPtr msg)
{
  // stop module list
  std::list<MotionModule*> _stop_modules;
  std::list<MotionModule*> _enable_modules;

  for (size_t idx = 0; idx < msg->joint_name.size(); idx++)
  {
    Dynamixel* _dxl = NULL;
    auto _dxl_it = robot_->dxls_.find((std::string)(msg->joint_name[idx]));
    if (_dxl_it != robot_->dxls_.end())
      _dxl = _dxl_it->second;
    else
      continue;

    // enqueue
    if (_dxl->ctrl_module_name_ != msg->module_name[idx])
    {
      for (auto _stop_m_it = motion_modules_.begin(); _stop_m_it != motion_modules_.end(); _stop_m_it++)
      {
        if ((*_stop_m_it)->getModuleName() == _dxl->ctrl_module_name_ && (*_stop_m_it)->getModuleEnable() == true)
          _stop_modules.push_back(*_stop_m_it);
      }
    }
  }

  // stop the module
  _stop_modules.unique();
  for (auto _stop_m_it = _stop_modules.begin(); _stop_m_it != _stop_modules.end(); _stop_m_it++)
  {
    (*_stop_m_it)->stop();
  }

  // wait to stop
  for (auto _stop_m_it = _stop_modules.begin(); _stop_m_it != _stop_modules.end(); _stop_m_it++)
  {
    while ((*_stop_m_it)->isRunning())
      std::this_thread::sleep_for(std::chrono::milliseconds(robot_->getControlCycle()));
  }

  // disable module(s)
  for (auto _stop_m_it = _stop_modules.begin(); _stop_m_it != _stop_modules.end(); _stop_m_it++)
  {
    (*_stop_m_it)->setModuleEnable(false);
  }

  // set ctrl module
  queue_mutex_.lock();

  for (size_t idx = 0; idx < msg->joint_name.size(); idx++)
  {
    std::string ctrl_module = msg->module_name[idx];
    std::string joint_name = msg->joint_name[idx];

    Dynamixel* _dxl = NULL;
    auto _dxl_it = robot_->dxls_.find(joint_name);
    if (_dxl_it != robot_->dxls_.end())
      _dxl = _dxl_it->second;
    else
      continue;

    // none
    if (ctrl_module == "" || ctrl_module == "none")
    {
      _dxl->ctrl_module_name_ = "none";

      uint32_t _pos_data;
      _pos_data = _dxl->convertRadian2Value(_dxl->dxl_state_->goal_position_ + _dxl->dxl_state_->position_offset_ * offset_ratio_);

      uint8_t _sync_write_data[4];
      _sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(_pos_data));
      _sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(_pos_data));
      _sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(_pos_data));
      _sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(_pos_data));

      if (port_to_sync_write_position_[_dxl->port_name_] != NULL)
        port_to_sync_write_position_[_dxl->port_name_]->addParam(_dxl->id_, _sync_write_data);

      if (port_to_sync_write_current_[_dxl->port_name_] != NULL)
        port_to_sync_write_current_[_dxl->port_name_]->removeParam(_dxl->id_);
      if (port_to_sync_write_velocity_[_dxl->port_name_] != NULL)
        port_to_sync_write_velocity_[_dxl->port_name_]->removeParam(_dxl->id_);
    }
    else
    {
      // check whether the module exist
      for (auto _m_it = motion_modules_.begin(); _m_it != motion_modules_.end(); _m_it++)
      {
        // if it exist
        if ((*_m_it)->getModuleName() == ctrl_module)
        {
          auto _result_it = (*_m_it)->result_.find(joint_name);
          if (_result_it == (*_m_it)->result_.end())
            break;

          _dxl->ctrl_module_name_ = ctrl_module;

          // enqueue enable module list
          _enable_modules.push_back(*_m_it);
          ControlMode _mode = (*_m_it)->getControlMode();

          if (_mode == PositionControl)
          {
            uint32_t _pos_data;
            _pos_data = _dxl->convertRadian2Value(_dxl->dxl_state_->goal_position_ + _dxl->dxl_state_->position_offset_ * offset_ratio_);

            uint8_t _sync_write_data[4];
            _sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(_pos_data));
            _sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(_pos_data));
            _sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(_pos_data));
            _sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(_pos_data));

            if (port_to_sync_write_position_[_dxl->port_name_] != NULL)
              port_to_sync_write_position_[_dxl->port_name_]->addParam(_dxl->id_, _sync_write_data);

            if (port_to_sync_write_current_[_dxl->port_name_] != NULL)
              port_to_sync_write_current_[_dxl->port_name_]->removeParam(_dxl->id_);
            if (port_to_sync_write_velocity_[_dxl->port_name_] != NULL)
              port_to_sync_write_velocity_[_dxl->port_name_]->removeParam(_dxl->id_);
          }
          else if (_mode == VelocityControl)
          {
            uint32_t _vel_data = _dxl->convertVelocity2Value(_dxl->dxl_state_->goal_velocity_);
            uint8_t _sync_write_data[4];
            _sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(_vel_data));
            _sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(_vel_data));
            _sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(_vel_data));
            _sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(_vel_data));

            if (port_to_sync_write_velocity_[_dxl->port_name_] != NULL)
              port_to_sync_write_velocity_[_dxl->port_name_]->addParam(_dxl->id_, _sync_write_data);

            if (port_to_sync_write_current_[_dxl->port_name_] != NULL)
              port_to_sync_write_current_[_dxl->port_name_]->removeParam(_dxl->id_);
            if (port_to_sync_write_position_[_dxl->port_name_] != NULL)
              port_to_sync_write_position_[_dxl->port_name_]->removeParam(_dxl->id_);
          }
          else if (_mode == TorqueControl)
          {
            uint32_t _curr_data = _dxl->convertTorque2Value(_dxl->dxl_state_->goal_torque_);
            uint8_t _sync_write_data[4];
            _sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(_curr_data));
            _sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(_curr_data));
            _sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(_curr_data));
            _sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(_curr_data));

            if (port_to_sync_write_current_[_dxl->port_name_] != NULL)
              port_to_sync_write_current_[_dxl->port_name_]->addParam(_dxl->id_, _sync_write_data);

            if (port_to_sync_write_velocity_[_dxl->port_name_] != NULL)
              port_to_sync_write_velocity_[_dxl->port_name_]->removeParam(_dxl->id_);
            if (port_to_sync_write_position_[_dxl->port_name_] != NULL)
              port_to_sync_write_position_[_dxl->port_name_]->removeParam(_dxl->id_);
          }
          break;
        }
      }
    }
  }

  // enable module(s)
  _enable_modules.unique();
  for (auto _m_it = _enable_modules.begin(); _m_it != _enable_modules.end(); _m_it++)
  {
    (*_m_it)->setModuleEnable(true);
  }

  // TODO: set indirect address
  // -> check module's control_mode

  queue_mutex_.unlock();

  // publish current module
  robotis_controller_msgs::msg::JointCtrlModule _current_module_msg;
  rclcpp::Publisher<robotis_controller_msgs::msg::JointCtrlModule>::SharedPtr current_module_pub_;

  for (auto _dxl_iter = robot_->dxls_.begin(); _dxl_iter != robot_->dxls_.end(); ++_dxl_iter)
  {
    _current_module_msg.joint_name.push_back(_dxl_iter->first);
    _current_module_msg.module_name.push_back(_dxl_iter->second->ctrl_module_name_);
  }

  if (_current_module_msg.joint_name.size() == _current_module_msg.module_name.size())
    current_module_pub_->publish(_current_module_msg);
}

void RobotisController::setCtrlModuleThread(std::string ctrl_module)
{
  // stop module
  std::list<MotionModule*> stop_modules;

  if (ctrl_module == "" || ctrl_module == "none")
  {
    // enqueue all modules in order to stop
    for (auto m_it = motion_modules_.begin(); m_it != motion_modules_.end(); m_it++)
    {
      if ((*m_it)->getModuleEnable() == true)
        stop_modules.push_back(*m_it);
    }
  }
  else
  {
    for (auto m_it = motion_modules_.begin(); m_it != motion_modules_.end(); m_it++)
    {
      // if it exist
      if ((*m_it)->getModuleName() == ctrl_module)
      {
        // enqueue the module which lost control of joint in order to stop
        for (auto& result_it : (*m_it)->result_)
        {
          auto d_it = robot_->dxls_.find(result_it.first);

          if (d_it != robot_->dxls_.end())
          {
            // enqueue
            if (d_it->second->ctrl_module_name_ != ctrl_module)
            {
              for (auto stop_m_it = motion_modules_.begin(); stop_m_it != motion_modules_.end(); stop_m_it++)
              {
                if (((*stop_m_it)->getModuleName() == d_it->second->ctrl_module_name_) &&
                    ((*stop_m_it)->getModuleEnable() == true))
                {
                  stop_modules.push_back(*stop_m_it);
                }
              }
            }
          }
        }

        break;
      }
    }
  }

  // stop the module
  stop_modules.unique();
  for (auto stop_m_it = stop_modules.begin(); stop_m_it != stop_modules.end(); stop_m_it++)
  {
    (*stop_m_it)->stop();
  }

  // wait to stop
  for (auto stop_m_it = stop_modules.begin(); stop_m_it != stop_modules.end(); stop_m_it++)
  {
    while ((*stop_m_it)->isRunning())
      std::this_thread::sleep_for(std::chrono::milliseconds(robot_->getControlCycle()));
  }

  // disable module(s)
  for (auto _stop_m_it = stop_modules.begin(); _stop_m_it != stop_modules.end(); _stop_m_it++)
  {
    (*_stop_m_it)->setModuleEnable(false);
  }

  // set ctrl module
  queue_mutex_.lock();

  if (DEBUG_PRINT)
    RCLCPP_INFO(rclcpp::get_logger("robotis_controller"), "set module : %s", ctrl_module.c_str());

  // none
  if ((ctrl_module == "") || (ctrl_module == "none"))
  {
    // set dxl's control module to "none"
    for (auto& d_it : robot_->dxls_)
    {
      Dynamixel* dxl = d_it.second;
      dxl->ctrl_module_name_ = "none";

      uint32_t pos_data;
      pos_data = dxl->convertRadian2Value(dxl->dxl_state_->goal_position_ + dxl->dxl_state_->position_offset_ * offset_ratio_);

      uint8_t sync_write_data[4] = { 0 };
      sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(pos_data));
      sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(pos_data));
      sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(pos_data));
      sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(pos_data));

      if (port_to_sync_write_position_[dxl->port_name_] != NULL)
        port_to_sync_write_position_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);

      if (port_to_sync_write_current_[dxl->port_name_] != NULL)
        port_to_sync_write_current_[dxl->port_name_]->removeParam(dxl->id_);
      if (port_to_sync_write_velocity_[dxl->port_name_] != NULL)
        port_to_sync_write_velocity_[dxl->port_name_]->removeParam(dxl->id_);
    }
  }
  else
  {
    // check whether the module exist
    for (auto m_it = motion_modules_.begin(); m_it != motion_modules_.end(); m_it++)
    {
      // if it exist
      if ((*m_it)->getModuleName() == ctrl_module)
      {
        ControlMode mode = (*m_it)->getControlMode();
        for (auto& result_it : (*m_it)->result_)
        {
          auto d_it = robot_->dxls_.find(result_it.first);
          if (d_it != robot_->dxls_.end())
          {
            Dynamixel* dxl = d_it->second;
            dxl->ctrl_module_name_ = ctrl_module;

            if (mode == PositionControl)
            {
              uint32_t pos_data;
              pos_data = dxl->convertRadian2Value(dxl->dxl_state_->goal_position_ + dxl->dxl_state_->position_offset_ * offset_ratio_);

              uint8_t sync_write_data[4] = { 0 };
              sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(pos_data));
              sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(pos_data));
              sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(pos_data));
              sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(pos_data));

              if (port_to_sync_write_position_[dxl->port_name_] != NULL)
                port_to_sync_write_position_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);

              if (port_to_sync_write_current_[dxl->port_name_] != NULL)
                port_to_sync_write_current_[dxl->port_name_]->removeParam(dxl->id_);
              if (port_to_sync_write_velocity_[dxl->port_name_] != NULL)
                port_to_sync_write_velocity_[dxl->port_name_]->removeParam(dxl->id_);
            }
            else if (mode == VelocityControl)
            {
              uint32_t vel_data = dxl->convertVelocity2Value(dxl->dxl_state_->goal_velocity_);
              uint8_t sync_write_data[4] = { 0 };
              sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(vel_data));
              sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(vel_data));
              sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(vel_data));
              sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(vel_data));

              if (port_to_sync_write_velocity_[dxl->port_name_] != NULL)
                port_to_sync_write_velocity_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);

              if (port_to_sync_write_current_[dxl->port_name_] != NULL)
                port_to_sync_write_current_[dxl->port_name_]->removeParam(dxl->id_);
              if (port_to_sync_write_position_[dxl->port_name_] != NULL)
                port_to_sync_write_position_[dxl->port_name_]->removeParam(dxl->id_);
            }
            else if (mode == TorqueControl)
            {
              uint32_t curr_data = dxl->convertTorque2Value(dxl->dxl_state_->goal_torque_);
              uint8_t sync_write_data[4] = { 0 };
              sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(curr_data));
              sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(curr_data));
              sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(curr_data));
              sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(curr_data));

              if (port_to_sync_write_current_[dxl->port_name_] != NULL)
                port_to_sync_write_current_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);

              if (port_to_sync_write_velocity_[dxl->port_name_] != NULL)
                port_to_sync_write_velocity_[dxl->port_name_]->removeParam(dxl->id_);
              if (port_to_sync_write_position_[dxl->port_name_] != NULL)
                port_to_sync_write_position_[dxl->port_name_]->removeParam(dxl->id_);
            }
          }
        }

        break;
      }
    }
  }

  for (auto m_it = motion_modules_.begin(); m_it != motion_modules_.end(); m_it++)
  {
    // set all used modules -> enable
    for (auto& d_it : robot_->dxls_)
    {
      if (d_it.second->ctrl_module_name_ == (*m_it)->getModuleName())
      {
        (*m_it)->setModuleEnable(true);
        break;
      }
    }
  }

  // TODO: set indirect address
  // -> check module's control_mode

  queue_mutex_.unlock();

  // publish current module
  robotis_controller_msgs::msg::JointCtrlModule current_module_msg;
  rclcpp::Publisher<robotis_controller_msgs::msg::JointCtrlModule>::SharedPtr current_module_pub_;

  for (auto& dxl_iter : robot_->dxls_)
  {
    current_module_msg.joint_name.push_back(dxl_iter.first);
    current_module_msg.module_name.push_back(dxl_iter.second->ctrl_module_name_);
  }

  if (current_module_msg.joint_name.size() == current_module_msg.module_name.size())
    current_module_pub_->publish(current_module_msg);
}

bool RobotisController::isTimerStopped()
{
  if (this->is_timer_running_)
  {
    if (DEBUG_PRINT == true)
      RCLCPP_WARN(rclcpp::get_logger("robotis_controller"), "Process Timer is running.. STOP the timer first.");
    return false;
  }
  return true;
}

int RobotisController::ping(const std::string joint_name, uint8_t* error)
{
  return ping(joint_name, 0, error);
}

int RobotisController::ping(const std::string joint_name, uint16_t* model_number, uint8_t* error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel* dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = robot_->ports_[dxl->port_name_];

  return pkt_handler->ping(port_handler, dxl->id_, model_number, error);
}

int RobotisController::action(const std::string joint_name)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel* dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = robot_->ports_[dxl->port_name_];

  return pkt_handler->action(port_handler, dxl->id_);
}

int RobotisController::reboot(const std::string joint_name, uint8_t* error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel* dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = robot_->ports_[dxl->port_name_];

  return pkt_handler->reboot(port_handler, dxl->id_, error);
}

int RobotisController::factoryReset(const std::string joint_name, uint8_t option, uint8_t* error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel* dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = robot_->ports_[dxl->port_name_];

  return pkt_handler->factoryReset(port_handler, dxl->id_, option, error);
}

int RobotisController::read(const std::string joint_name, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  auto dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler *port_handler = robot_->ports_[dxl->port_name_];

  return pkt_handler->readTxRx(port_handler, dxl->id_, address, length, data, error);
}


int RobotisController::readCtrlItem(const std::string joint_name, const std::string item_name, uint32_t *data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  auto dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  auto item = dxl->ctrl_table_[item_name];
  if (item == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler *port_handler = robot_->ports_[dxl->port_name_];

  int result = COMM_NOT_AVAILABLE;
  switch (item->data_length_)
  {
    case 1:
    {
      uint8_t read_data = 0;
      result = pkt_handler->read1ByteTxRx(port_handler, dxl->id_, item->address_, &read_data, error);
      if (result == COMM_SUCCESS)
        *data = read_data;
      break;
    }
    case 2:
    {
      uint16_t read_data = 0;
      result = pkt_handler->read2ByteTxRx(port_handler, dxl->id_, item->address_, &read_data, error);
      if (result == COMM_SUCCESS)
        *data = read_data;
      break;
    }
    case 4:
    {
      uint32_t read_data = 0;
      result = pkt_handler->read4ByteTxRx(port_handler, dxl->id_, item->address_, &read_data, error);
      if (result == COMM_SUCCESS)
        *data = read_data;
      break;
    }
    default:
      break;
  }
  return result;
}

int RobotisController::read1Byte(const std::string joint_name, uint16_t address, uint8_t *data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  auto dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler *port_handler = robot_->ports_[dxl->port_name_];

  return pkt_handler->read1ByteTxRx(port_handler, dxl->id_, address, data, error);
}


int RobotisController::read2Byte(const std::string joint_name, uint16_t address, uint16_t *data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  auto dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler *port_handler = robot_->ports_[dxl->port_name_];

  return pkt_handler->read2ByteTxRx(port_handler, dxl->id_, address, data, error);
}


int RobotisController::read4Byte(const std::string joint_name, uint16_t address, uint32_t *data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  auto dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler *port_handler = robot_->ports_[dxl->port_name_];

  return pkt_handler->read4ByteTxRx(port_handler, dxl->id_, address, data, error);
}

int RobotisController::write(const std::string joint_name, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  auto dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler *port_handler = robot_->ports_[dxl->port_name_];

  return pkt_handler->writeTxRx(port_handler, dxl->id_, address, length, data, error);
}

int RobotisController::writeCtrlItem(const std::string joint_name, const std::string item_name, uint32_t data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  auto dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  ControlTableItem *item = dxl->ctrl_table_[item_name];
  if (item == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler *port_handler = robot_->ports_[dxl->port_name_];

  int result = COMM_NOT_AVAILABLE;
  uint8_t *write_data = new uint8_t[item->data_length_];

  if (item->data_length_ == 1)
  {
    write_data[0] = (uint8_t)data;
    result = pkt_handler->write1ByteTxRx(port_handler, dxl->id_, item->address_, write_data[0], error);
  }
  else if (item->data_length_ == 2)
  {
    write_data[0] = DXL_LOBYTE((uint16_t)data);
    write_data[1] = DXL_HIBYTE((uint16_t)data);
    result = pkt_handler->write2ByteTxRx(port_handler, dxl->id_, item->address_, DXL_MAKEWORD(write_data[0], write_data[1]), error);
  }
  else if (item->data_length_ == 4)
  {
    write_data[0] = DXL_LOBYTE(DXL_LOWORD((uint32_t)data));
    write_data[1] = DXL_HIBYTE(DXL_LOWORD((uint32_t)data));
    write_data[2] = DXL_LOBYTE(DXL_HIWORD((uint32_t)data));
    write_data[3] = DXL_HIBYTE(DXL_HIWORD((uint32_t)data));
    result = pkt_handler->write4ByteTxRx(port_handler, dxl->id_, item->address_, DXL_MAKEDWORD(DXL_MAKEWORD(write_data[0], write_data[1]), DXL_MAKEWORD(write_data[2], write_data[3])), error);
  }

  delete[] write_data;
  return result;
}


int RobotisController::write1Byte(const std::string joint_name, uint16_t address, uint8_t data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  auto dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler *port_handler = robot_->ports_[dxl->port_name_];

  return pkt_handler->write1ByteTxRx(port_handler, dxl->id_, address, data, error);
}

int RobotisController::write2Byte(const std::string joint_name, uint16_t address, uint16_t data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->write2ByteTxRx(port_handler, dxl->id_, address, data, error);
}

int RobotisController::write4Byte(const std::string joint_name, uint16_t address, uint32_t data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->write4ByteTxRx(port_handler, dxl->id_, address, data, error);
}

int RobotisController::regWrite(const std::string joint_name, uint16_t address, uint16_t length, uint8_t *data,
    uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->regWriteTxRx(port_handler, dxl->id_, address, length, data, error);
}

