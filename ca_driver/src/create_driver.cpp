// Copyright 2015 Jacob Perron (Autonomy Lab, Simon Fraser University)

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>

#include <chrono>
#include <memory>
#include <string>

#include "create_driver/create_driver.hpp"

namespace create_autonomy
{

using CallbackReturn =
 rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CreateDriver::CreateDriver(const std::string & name)
: LifecycleNode(name),
  model_(create::RobotModel::CREATE_2),
  ros_clock_(RCL_ROS_TIME),
  dev_("/dev/ttyUSB0"),
  base_frame_("base_footprint"),
  odom_frame_("odom"),
  latch_duration_(0.2),
  loop_hz_(10.0),
  publish_tf_(true)
{
  std::string robot_model_name = "CREATE_2";

  rclcpp::Parameter parameter;
  if (get_parameter("dev", parameter)) {
    dev_ = parameter.get_value<std::string>();
  }
  if (get_parameter("robot_model", parameter)) {
    robot_model_name = parameter.get_value<std::string>();
  }
  if (get_parameter("base_frame", parameter)) {
    base_frame_ = parameter.get_value<std::string>();
  }
  if (get_parameter("odom_frame", parameter)) {
    odom_frame_ = parameter.get_value<std::string>();
  }
  if (get_parameter("latch_cmd_duration", parameter)) {
    latch_duration_ = parameter.get_value<double>();
  }
  if (get_parameter("loop_hz", parameter)) {
    loop_hz_ = parameter.get_value<double>();
  }
  if (get_parameter("publish_tf", parameter)) {
    publish_tf_ = parameter.get_value<bool>();
  }

  if (robot_model_name == "ROOMBA_400") {
    model_ = create::RobotModel::ROOMBA_400;
  } else if (robot_model_name == "CREATE_1") {
    model_ = create::RobotModel::CREATE_1;
  } else if (robot_model_name == "CREATE_2") {
    model_ = create::RobotModel::CREATE_2;
  } else {
    RCLCPP_FATAL(get_logger(), "[CREATE] Robot model \"%s\" is not known.",
      robot_model_name.c_str());
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(get_logger(), "[CREATE] \"%s\" selected",
    robot_model_name.c_str());

  baud_ = model_.getBaud();
  if (get_parameter("baud", parameter)) {
    baud_ = parameter.get_value<int>();
  }
}

CreateDriver::~CreateDriver()
{
}

CallbackReturn CreateDriver::on_configure(const rclcpp_lifecycle::State &)
{
  using namespace std::chrono_literals;

  robot_ = std::make_unique<create::Create>(model_);

  // Set frame_id's
  mode_msg_.header.frame_id = base_frame_;
  bumper_msg_.header.frame_id = base_frame_;
  charging_state_msg_.header.frame_id = base_frame_;
  tf_odom_.header.frame_id = odom_frame_;
  tf_odom_.child_frame_id = base_frame_;
  odom_msg_.header.frame_id = odom_frame_;
  odom_msg_.child_frame_id = base_frame_;
  joint_state_msg_.name.resize(2);
  joint_state_msg_.position.resize(2);
  joint_state_msg_.velocity.resize(2);
  joint_state_msg_.effort.resize(2);
  joint_state_msg_.name[0] = "left_wheel_joint";
  joint_state_msg_.name[1] = "right_wheel_joint";

  // Populate initial covariances
  for (int i = 0; i < 36; i++) {
    odom_msg_.pose.covariance[i] = COVARIANCE[i];
    odom_msg_.twist.covariance[i] = COVARIANCE[i];
  }

  // Setup subscribers
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&CreateDriver::cmdVelCallback, this, std::placeholders::_1));
  debris_led_sub_ = create_subscription<std_msgs::msg::Bool>(
    "debris_led", 10, std::bind(&CreateDriver::debrisLEDCallback, this, std::placeholders::_1));
  spot_led_sub_ = create_subscription<std_msgs::msg::Bool>(
    "spot_led", 10, std::bind(&CreateDriver::spotLEDCallback, this, std::placeholders::_1));
  dock_led_sub_ = create_subscription<std_msgs::msg::Bool>(
    "dock_led", 10, std::bind(&CreateDriver::dockLEDCallback, this, std::placeholders::_1));
  check_led_sub_ = create_subscription<std_msgs::msg::Bool>(
    "check_led", 10, std::bind(&CreateDriver::checkLEDCallback, this, std::placeholders::_1));
  power_led_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
    "power_led", 10, std::bind(&CreateDriver::powerLEDCallback, this, std::placeholders::_1));
  set_ascii_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
    "set_ascii", 10, std::bind(&CreateDriver::setASCIICallback, this, std::placeholders::_1));
  dock_sub_ = create_subscription<std_msgs::msg::Empty>(
    "dock", 10, std::bind(&CreateDriver::dockCallback, this, std::placeholders::_1));
  undock_sub_ = create_subscription<std_msgs::msg::Empty>(
    "undock", 10, std::bind(&CreateDriver::undockCallback, this, std::placeholders::_1));
  define_song_sub_ = create_subscription<ca_msgs::msg::DefineSong>(
    "define_song", 10, std::bind(&CreateDriver::defineSongCallback, this, std::placeholders::_1));
  play_song_sub_ = create_subscription<ca_msgs::msg::PlaySong>(
    "play_song", 10, std::bind(&CreateDriver::playSongCallback, this, std::placeholders::_1));

  // Setup publishers
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  clean_btn_pub_ = create_publisher<std_msgs::msg::Empty>("clean_button", 10);
  day_btn_pub_ = create_publisher<std_msgs::msg::Empty>("day_button", 10);
  hour_btn_pub_ = create_publisher<std_msgs::msg::Empty>("hour_button", 10);
  min_btn_pub_ = create_publisher<std_msgs::msg::Empty>("minute_button", 10);
  dock_btn_pub_ = create_publisher<std_msgs::msg::Empty>("dock_button", 10);
  spot_btn_pub_ = create_publisher<std_msgs::msg::Empty>("spot_button", 10);
  voltage_pub_ = create_publisher<std_msgs::msg::Float32>("battery/voltage", 10);
  current_pub_ = create_publisher<std_msgs::msg::Float32>("battery/current", 10);
  charge_pub_ = create_publisher<std_msgs::msg::Float32>("battery/charge", 10);
  charge_ratio_pub_ = create_publisher<std_msgs::msg::Float32>("battery/charge_ratio", 10);
  capacity_pub_ = create_publisher<std_msgs::msg::Float32>("battery/capacity", 10);
  temperature_pub_ = create_publisher<std_msgs::msg::Int16>("battery/temperature", 10);
  charging_state_pub_ = create_publisher<ca_msgs::msg::ChargingState>("battery/charging_state", 10);
  omni_char_pub_ = create_publisher<std_msgs::msg::UInt16>("ir_omni", 10);
  mode_pub_ = create_publisher<ca_msgs::msg::Mode>("mode", 10);
  bumper_pub_ = create_publisher<ca_msgs::msg::Bumper>("bumper", 10);
  wheeldrop_pub_ = create_publisher<std_msgs::msg::Empty>("wheeldrop", 10);
  wheel_joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  timer_ = create_wall_timer(100ms, std::bind(&CreateDriver::update, this));
  timer_->cancel();

  RCLCPP_INFO(get_logger(), "[CREATE] Ready.");

  return CallbackReturn::SUCCESS;
}

CallbackReturn CreateDriver::on_activate(const rclcpp_lifecycle::State &)
{
  if (!robot_->connect(dev_, baud_)) {
    RCLCPP_FATAL(get_logger(),
      "[CREATE] Failed to establish serial connection with Create.");
    rclcpp::shutdown();
  }

  RCLCPP_INFO(get_logger(), "[CREATE] Connection established.");

  // Start in full control mode
  robot_->setMode(create::MODE_FULL);

  // Show robot's battery level
  RCLCPP_INFO(get_logger(), "[CREATE] Battery level %.2f %%",
    (robot_->getBatteryCharge() / robot_->getBatteryCapacity()) * 100.0);

  odom_pub_->on_activate();
  clean_btn_pub_->on_activate();
  day_btn_pub_->on_activate();
  hour_btn_pub_->on_activate();
  min_btn_pub_->on_activate();
  dock_btn_pub_->on_activate();
  spot_btn_pub_->on_activate();
  voltage_pub_->on_activate();
  current_pub_->on_activate();
  charge_pub_->on_activate();
  charge_ratio_pub_->on_activate();
  capacity_pub_->on_activate();
  temperature_pub_->on_activate();
  charging_state_pub_->on_activate();
  omni_char_pub_->on_activate();
  mode_pub_->on_activate();
  bumper_pub_->on_activate();
  wheeldrop_pub_->on_activate();
  wheel_joint_pub_->on_activate();

  timer_->reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn CreateDriver::on_deactivate(const rclcpp_lifecycle::State &)
{
  timer_->cancel();

  odom_pub_->on_deactivate();
  clean_btn_pub_->on_deactivate();
  day_btn_pub_->on_deactivate();
  hour_btn_pub_->on_deactivate();
  min_btn_pub_->on_deactivate();
  dock_btn_pub_->on_deactivate();
  spot_btn_pub_->on_deactivate();
  voltage_pub_->on_deactivate();
  current_pub_->on_deactivate();
  charge_pub_->on_deactivate();
  charge_ratio_pub_->on_deactivate();
  capacity_pub_->on_deactivate();
  temperature_pub_->on_deactivate();
  charging_state_pub_->on_deactivate();
  omni_char_pub_->on_deactivate();
  mode_pub_->on_deactivate();
  bumper_pub_->on_deactivate();
  wheeldrop_pub_->on_deactivate();
  wheel_joint_pub_->on_deactivate();

  robot_->disconnect();
  RCLCPP_INFO(get_logger(), "[CREATE] Connection terminated.");

  return CallbackReturn::SUCCESS;
}

CallbackReturn CreateDriver::on_cleanup(const rclcpp_lifecycle::State &)
{
  timer_.reset();

  odom_pub_.reset();
  clean_btn_pub_.reset();
  day_btn_pub_.reset();
  hour_btn_pub_.reset();
  min_btn_pub_.reset();
  dock_btn_pub_.reset();
  spot_btn_pub_.reset();
  voltage_pub_.reset();
  current_pub_.reset();
  charge_pub_.reset();
  charge_ratio_pub_.reset();
  capacity_pub_.reset();
  temperature_pub_.reset();
  charging_state_pub_.reset();
  omni_char_pub_.reset();
  mode_pub_.reset();
  bumper_pub_.reset();
  wheeldrop_pub_.reset();
  wheel_joint_pub_.reset();

  cmd_vel_sub_.reset();
  debris_led_sub_.reset();
  spot_led_sub_.reset();
  dock_led_sub_.reset();
  check_led_sub_.reset();
  power_led_sub_.reset();
  set_ascii_sub_.reset();
  dock_sub_.reset();
  undock_sub_.reset();
  define_song_sub_.reset();
  play_song_sub_.reset();

  robot_.reset();

  return CallbackReturn::SUCCESS;
}

void CreateDriver::cmdVelCallback(const geometry_msgs::msg::Twist::UniquePtr msg)
{
  robot_->drive(msg->linear.x, msg->angular.z);
  last_cmd_vel_time_ = ros_clock_.now();
}

void CreateDriver::debrisLEDCallback(const std_msgs::msg::Bool::UniquePtr msg)
{
  robot_->enableDebrisLED(msg->data);
}

void CreateDriver::spotLEDCallback(const std_msgs::msg::Bool::UniquePtr msg)
{
  robot_->enableSpotLED(msg->data);
}

void CreateDriver::dockLEDCallback(const std_msgs::msg::Bool::UniquePtr msg)
{
  robot_->enableDockLED(msg->data);
}

void CreateDriver::checkLEDCallback(const std_msgs::msg::Bool::UniquePtr msg)
{
  robot_->enableCheckRobotLED(msg->data);
}

void CreateDriver::powerLEDCallback(const std_msgs::msg::UInt8MultiArray::UniquePtr msg)
{
  if (msg->data.empty()) {
    RCLCPP_ERROR(get_logger(), "[CREATE] No values provided to set power LED");
  } else {
    if (msg->data.size() < 2) {
      robot_->setPowerLED(msg->data[0]);
    } else {
      robot_->setPowerLED(msg->data[0], msg->data[1]);
    }
  }
}

void CreateDriver::setASCIICallback(const std_msgs::msg::UInt8MultiArray::UniquePtr msg)
{
  bool result = false;
  if (msg->data.empty()) {
    RCLCPP_ERROR(get_logger(), "[CREATE] No ASCII digits provided");
  } else if (msg->data.size() < 2) {
    result = robot_->setDigitsASCII(msg->data[0], ' ', ' ', ' ');
  } else if (msg->data.size() < 3) {
    result = robot_->setDigitsASCII(msg->data[0], msg->data[1], ' ', ' ');
  } else if (msg->data.size() < 4) {
    result = robot_->setDigitsASCII(msg->data[0], msg->data[1], msg->data[2], ' ');
  } else {
    result = robot_->setDigitsASCII(msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
  }

  if (!result) {
    RCLCPP_ERROR(get_logger(), "[CREATE] ASCII character out of range [32, 126]");
  }
}

void CreateDriver::dockCallback(const std_msgs::msg::Empty::UniquePtr msg)
{
  robot_->setMode(create::MODE_PASSIVE);

  if (model_.getVersion() <= create::V_2) {
    usleep(1000000);  // Create 1 requires a delay (1 sec)
  }

  // Call docking behaviour
  robot_->dock();
}

void CreateDriver::undockCallback(const std_msgs::msg::Empty::UniquePtr msg)
{
  // Switch robot back to FULL mode
  robot_->setMode(create::MODE_FULL);
}

void CreateDriver::defineSongCallback(const ca_msgs::msg::DefineSong::UniquePtr msg)
{
  if (!robot_->defineSong(msg->song, msg->length, &(msg->notes.front()),
    &(msg->durations.front())))
  {
    RCLCPP_ERROR(get_logger(), "[CREATE] Failed to define song %d of length %d",
      msg->song, msg->length);
  }
}

void CreateDriver::playSongCallback(const ca_msgs::msg::PlaySong::UniquePtr msg)
{
  if (!robot_->playSong(msg->song)) {
    RCLCPP_ERROR(get_logger(), "[CREATE] Failed to play song %d", msg->song);
  }
}

void CreateDriver::update()
{
  publishOdom();
  publishJointState();
  publishBatteryInfo();
  publishButtonPresses();
  publishOmniChar();
  publishMode();
  publishBumperInfo();
  publishWheeldrop();

  // If last velocity command was sent longer than latch duration, stop robot
  if (ros_clock_.now() - last_cmd_vel_time_ >= rclcpp::Duration(latch_duration_)) {
    robot_->drive(0, 0);
  }
}

void CreateDriver::publishOdom()
{
  create::Pose pose = robot_->getPose();
  create::Vel vel = robot_->getVel();

  // Populate position info
  tf2::Quaternion quat;
  quat.setRPY(0, 0, pose.yaw);
  odom_msg_.header.stamp = ros_clock_.now();
  odom_msg_.pose.pose.position.x = pose.x;
  odom_msg_.pose.pose.position.y = pose.y;
  odom_msg_.pose.pose.orientation.x = quat.x();
  odom_msg_.pose.pose.orientation.y = quat.y();
  odom_msg_.pose.pose.orientation.z = quat.z();
  odom_msg_.pose.pose.orientation.w = quat.w();

  // Populate velocity info
  odom_msg_.twist.twist.linear.x = vel.x;
  odom_msg_.twist.twist.linear.y = vel.y;
  odom_msg_.twist.twist.angular.z = vel.yaw;

  // Update covariances
  odom_msg_.pose.covariance[0] = static_cast<double>(pose.covariance[0]);
  odom_msg_.pose.covariance[1] = pose.covariance[1];
  odom_msg_.pose.covariance[5] = pose.covariance[2];
  odom_msg_.pose.covariance[6] = pose.covariance[3];
  odom_msg_.pose.covariance[7] = pose.covariance[4];
  odom_msg_.pose.covariance[11] = pose.covariance[5];
  odom_msg_.pose.covariance[30] = pose.covariance[6];
  odom_msg_.pose.covariance[31] = pose.covariance[7];
  odom_msg_.pose.covariance[35] = pose.covariance[8];
  odom_msg_.twist.covariance[0] = vel.covariance[0];
  odom_msg_.twist.covariance[1] = vel.covariance[1];
  odom_msg_.twist.covariance[5] = vel.covariance[2];
  odom_msg_.twist.covariance[6] = vel.covariance[3];
  odom_msg_.twist.covariance[7] = vel.covariance[4];
  odom_msg_.twist.covariance[11] = vel.covariance[5];
  odom_msg_.twist.covariance[30] = vel.covariance[6];
  odom_msg_.twist.covariance[31] = vel.covariance[7];
  odom_msg_.twist.covariance[35] = vel.covariance[8];

  if (publish_tf_) {
    tf_odom_.header.stamp = ros_clock_.now();
    tf_odom_.transform.translation.x = pose.x;
    tf_odom_.transform.translation.y = pose.y;
    tf_odom_.transform.rotation.x = quat.x();
    tf_odom_.transform.rotation.y = quat.y();
    tf_odom_.transform.rotation.z = quat.z();
    tf_odom_.transform.rotation.w = quat.w();
    tf_broadcaster_->sendTransform(tf_odom_);
  }

  odom_pub_->publish(odom_msg_);
}

void CreateDriver::publishJointState()
{
  // Publish joint states
  double wheelRadius = model_.getWheelDiameter() / 2.0;

  joint_state_msg_.header.stamp = ros_clock_.now();
  joint_state_msg_.position[0] = robot_->getLeftWheelDistance() / wheelRadius;
  joint_state_msg_.position[1] = robot_->getRightWheelDistance() / wheelRadius;
  joint_state_msg_.velocity[0] = robot_->getRequestedLeftWheelVel() / wheelRadius;
  joint_state_msg_.velocity[1] = robot_->getRequestedRightWheelVel() / wheelRadius;
  wheel_joint_pub_->publish(joint_state_msg_);
}

void CreateDriver::publishBatteryInfo()
{
  float32_msg_.data = robot_->getVoltage();
  voltage_pub_->publish(float32_msg_);
  float32_msg_.data = robot_->getCurrent();
  current_pub_->publish(float32_msg_);
  float32_msg_.data = robot_->getBatteryCharge();
  charge_pub_->publish(float32_msg_);
  float32_msg_.data = robot_->getBatteryCapacity();
  capacity_pub_->publish(float32_msg_);
  int16_msg_.data = robot_->getTemperature();
  temperature_pub_->publish(int16_msg_);
  float32_msg_.data = robot_->getBatteryCharge() / robot_->getBatteryCapacity();
  charge_ratio_pub_->publish(float32_msg_);

  const create::ChargingState charging_state = robot_->getChargingState();
  charging_state_msg_.header.stamp = ros_clock_.now();
  switch (charging_state) {
    case create::CHARGE_NONE:
      charging_state_msg_.state = ca_msgs::msg::ChargingState::CHARGE_NONE;
      break;
    case create::CHARGE_RECONDITION:
      charging_state_msg_.state = ca_msgs::msg::ChargingState::CHARGE_RECONDITION;
      break;

    case create::CHARGE_FULL:
      charging_state_msg_.state = ca_msgs::msg::ChargingState::CHARGE_FULL;
      break;

    case create::CHARGE_TRICKLE:
      charging_state_msg_.state = ca_msgs::msg::ChargingState::CHARGE_TRICKLE;
      break;

    case create::CHARGE_WAITING:
      charging_state_msg_.state = ca_msgs::msg::ChargingState::CHARGE_WAITING;
      break;

    case create::CHARGE_FAULT:
      charging_state_msg_.state = ca_msgs::msg::ChargingState::CHARGE_FAULT;
      break;
  }
  charging_state_pub_->publish(charging_state_msg_);
}

void CreateDriver::publishButtonPresses() const
{
  if (robot_->isCleanButtonPressed()) {
    clean_btn_pub_->publish(empty_msg_);
  }
  if (robot_->isDayButtonPressed()) {
    day_btn_pub_->publish(empty_msg_);
  }
  if (robot_->isHourButtonPressed()) {
    hour_btn_pub_->publish(empty_msg_);
  }
  if (robot_->isMinButtonPressed()) {
    min_btn_pub_->publish(empty_msg_);
  }
  if (robot_->isDockButtonPressed()) {
    dock_btn_pub_->publish(empty_msg_);
  }
  if (robot_->isSpotButtonPressed()) {
    spot_btn_pub_->publish(empty_msg_);
  }
}

void CreateDriver::publishOmniChar()
{
  uint8_t ir_char = robot_->getIROmni();
  uint16_msg_.data = ir_char;
  omni_char_pub_->publish(uint16_msg_);
  // TODO(jacobperron): Publish info based on character, such as dock in sight
}

void CreateDriver::publishMode()
{
  const create::CreateMode mode = robot_->getMode();
  mode_msg_.header.stamp = ros_clock_.now();
  switch (mode) {
    case create::MODE_OFF:
      mode_msg_.mode = mode_msg_.MODE_OFF;
      break;
    case create::MODE_PASSIVE:
      mode_msg_.mode = mode_msg_.MODE_PASSIVE;
      break;
    case create::MODE_SAFE:
      mode_msg_.mode = mode_msg_.MODE_SAFE;
      break;
    case create::MODE_FULL:
      mode_msg_.mode = mode_msg_.MODE_FULL;
      break;
    default:
      RCLCPP_ERROR(get_logger(), "[CREATE] Unknown mode detected");
      break;
  }
  mode_pub_->publish(mode_msg_);
}

void CreateDriver::publishBumperInfo()
{
  bumper_msg_.header.stamp = ros_clock_.now();
  bumper_msg_.is_left_pressed = robot_->isLeftBumper();
  bumper_msg_.is_right_pressed = robot_->isRightBumper();

  if (model_.getVersion() >= create::V_3) {
    bumper_msg_.is_light_left = robot_->isLightBumperLeft();
    bumper_msg_.is_light_front_left = robot_->isLightBumperFrontLeft();
    bumper_msg_.is_light_center_left = robot_->isLightBumperCenterLeft();
    bumper_msg_.is_light_right = robot_->isLightBumperRight();
    bumper_msg_.is_light_front_right = robot_->isLightBumperFrontRight();
    bumper_msg_.is_light_center_right = robot_->isLightBumperCenterRight();

    bumper_msg_.light_signal_left = robot_->getLightSignalLeft();
    bumper_msg_.light_signal_front_left = robot_->getLightSignalFrontLeft();
    bumper_msg_.light_signal_center_left = robot_->getLightSignalCenterLeft();
    bumper_msg_.light_signal_right = robot_->getLightSignalRight();
    bumper_msg_.light_signal_front_right = robot_->getLightSignalFrontRight();
    bumper_msg_.light_signal_center_right = robot_->getLightSignalCenterRight();
  }

  bumper_pub_->publish(bumper_msg_);
}

void CreateDriver::publishWheeldrop()
{
  if (robot_->isWheeldrop()) {
    wheeldrop_pub_->publish(empty_msg_);
  }
}

}  // namespace create_autonomy

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<create_autonomy::CreateDriver>("ca_driver");
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
