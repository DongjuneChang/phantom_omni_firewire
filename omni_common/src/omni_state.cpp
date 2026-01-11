/**
 * Phantom Omni ROS2 Driver - FireWire Version
 * 
 * Ported from fsuarez6/phantom_omni (ROS1 Hydro)
 * Original author: Francisco Suárez Ruiz
 * ROS2 port: Dongjune Chang
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sstream>
#include <thread>
#include <atomic>
#include <mutex>

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduQuaternion.h>

#include "omni_msgs/msg/omni_button_event.hpp"
#include "omni_msgs/msg/omni_feedback.hpp"
#include "omni_msgs/msg/omni_state.hpp"

int calibrationStyle;
std::atomic<bool> running(true);

struct OmniState {
  hduVector3Dd position;
  hduVector3Dd velocity;
  hduVector3Dd inp_vel1;
  hduVector3Dd inp_vel2;
  hduVector3Dd inp_vel3;
  hduVector3Dd out_vel1;
  hduVector3Dd out_vel2;
  hduVector3Dd out_vel3;
  hduVector3Dd pos_hist1;
  hduVector3Dd pos_hist2;
  hduQuaternion rot;
  hduVector3Dd joints;
  hduVector3Dd force;
  float thetas[7];
  float thetas_offset[7];  // Joint offset for calibration
  int buttons[2];
  int buttons_prev[2];
  bool lock;
  bool close_gripper;
  hduVector3Dd lock_pos;
  double units_ratio;
  std::mutex offset_mutex;
  bool calibrated;
};

class PhantomOmniNode : public rclcpp::Node {
public:
  PhantomOmniNode(OmniState* state) : Node("omni_haptic_node"), state_(state) {
    // Declare parameters
    this->declare_parameter<std::string>("omni_name", "phantom");
    this->declare_parameter<std::string>("reference_frame", "map");
    this->declare_parameter<std::string>("units", "mm");
    this->declare_parameter<int>("publish_rate", 1000);

    // Joint scales and offsets (configurable per device)
    // Default values from sawSensablePhantom for older Omni devices
    this->declare_parameter<std::vector<double>>("joint_scales",
        {1.0, 1.0, 1.0, 1.0, -1.0, 1.0});
    this->declare_parameter<std::vector<double>>("joint_offsets",
        {0.0, 0.0, 0.0, M_PI, -3*M_PI/4, -M_PI});

    omni_name_ = this->get_parameter("omni_name").as_string();
    ref_frame_ = this->get_parameter("reference_frame").as_string();
    std::string units = this->get_parameter("units").as_string();
    publish_rate_ = this->get_parameter("publish_rate").as_int();

    // Get joint scales and offsets
    joint_scales_ = this->get_parameter("joint_scales").as_double_array();
    joint_offsets_ = this->get_parameter("joint_offsets").as_double_array();

    if (joint_scales_.size() != 6 || joint_offsets_.size() != 6) {
      RCLCPP_WARN(this->get_logger(),
          "joint_scales and joint_offsets must have 6 elements. Using defaults.");
      joint_scales_ = {1.0, 1.0, 1.0, 1.0, -1.0, 1.0};
      joint_offsets_ = {0.0, 0.0, 0.0, M_PI, -3*M_PI/4, -M_PI};
    }

    RCLCPP_INFO(this->get_logger(), "Joint offsets: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
        joint_offsets_[0], joint_offsets_[1], joint_offsets_[2],
        joint_offsets_[3], joint_offsets_[4], joint_offsets_[5]);

    // Set units ratio
    if (units == "mm")
      state_->units_ratio = 1.0;
    else if (units == "cm")
      state_->units_ratio = 10.0;
    else if (units == "dm")
      state_->units_ratio = 100.0;
    else if (units == "m")
      state_->units_ratio = 1000.0;
    else {
      state_->units_ratio = 1.0;
      RCLCPP_WARN(this->get_logger(), "Unknown units [%s] using [mm]", units.c_str());
      units = "mm";
    }
    RCLCPP_INFO(this->get_logger(), "PHaNTOM position given in [%s], ratio [%.1f]", 
                units.c_str(), state_->units_ratio);

    // Create publishers
    button_pub_ = this->create_publisher<omni_msgs::msg::OmniButtonEvent>(
        omni_name_ + "/button", 100);
    state_pub_ = this->create_publisher<omni_msgs::msg::OmniState>(
        omni_name_ + "/state", 1);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        omni_name_ + "/pose", 1);
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        omni_name_ + "/joint_states", 1);

    // Create subscriber for force feedback
    force_sub_ = this->create_subscription<omni_msgs::msg::OmniFeedback>(
        omni_name_ + "/force_feedback", 1,
        std::bind(&PhantomOmniNode::force_callback, this, std::placeholders::_1));

    // Create calibration service
    calibrate_srv_ = this->create_service<std_srvs::srv::Trigger>(
        omni_name_ + "/calibrate",
        std::bind(&PhantomOmniNode::calibrate_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Initialize state
    state_->buttons[0] = 0;
    state_->buttons[1] = 0;
    state_->buttons_prev[0] = 0;
    state_->buttons_prev[1] = 0;
    hduVector3Dd zeros(0, 0, 0);
    state_->velocity = zeros;
    state_->inp_vel1 = zeros;
    state_->inp_vel2 = zeros;
    state_->inp_vel3 = zeros;
    state_->out_vel1 = zeros;
    state_->out_vel2 = zeros;
    state_->out_vel3 = zeros;
    state_->pos_hist1 = zeros;
    state_->pos_hist2 = zeros;
    state_->lock = false;
    state_->close_gripper = false;
    state_->lock_pos = zeros;
    state_->force = zeros;
    state_->calibrated = false;
    for (int i = 0; i < 7; i++) {
      state_->thetas_offset[i] = 0.0f;
    }

    // Create timer for publishing
    auto period = std::chrono::microseconds(1000000 / publish_rate_);
    timer_ = this->create_wall_timer(period, 
        std::bind(&PhantomOmniNode::publish_omni_state, this));

    RCLCPP_INFO(this->get_logger(), "Publishing PHaNTOM state at [%d] Hz", publish_rate_);
  }

private:
  void force_callback(const omni_msgs::msg::OmniFeedback::SharedPtr msg) {
    state_->force[0] = msg->force.x - 0.001 * state_->velocity[0];
    state_->force[1] = msg->force.y - 0.001 * state_->velocity[1];
    state_->force[2] = msg->force.z - 0.001 * state_->velocity[2];

    state_->lock_pos[0] = msg->position.x;
    state_->lock_pos[1] = msg->position.y;
    state_->lock_pos[2] = msg->position.z;
  }

  void calibrate_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    std::lock_guard<std::mutex> lock(state_->offset_mutex);

    // Store current joint positions as offset
    for (int i = 0; i < 7; i++) {
      state_->thetas_offset[i] = state_->thetas[i];
    }
    state_->calibrated = true;

    RCLCPP_INFO(this->get_logger(), "Calibration complete. Current position set as zero.");
    RCLCPP_INFO(this->get_logger(), "Offsets: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                state_->thetas_offset[0], state_->thetas_offset[1], state_->thetas_offset[2],
                state_->thetas_offset[3], state_->thetas_offset[4], state_->thetas_offset[5],
                state_->thetas_offset[6]);

    response->success = true;
    response->message = "Calibration complete. Current position set as zero.";
  }

  void publish_omni_state() {
    auto now = this->now();

    // Build the state msg
    omni_msgs::msg::OmniState state_msg;
    state_msg.header.stamp = now;
    state_msg.locked = state_->lock;
    state_msg.close_gripper = state_->close_gripper;
    // Position
    state_msg.pose.position.x = state_->position[0];
    state_msg.pose.position.y = state_->position[1];
    state_msg.pose.position.z = state_->position[2];
    // Orientation
    state_msg.pose.orientation.x = state_->rot.v()[0];
    state_msg.pose.orientation.y = state_->rot.v()[1];
    state_msg.pose.orientation.z = state_->rot.v()[2];
    state_msg.pose.orientation.w = state_->rot.s();
    // Velocity
    state_msg.velocity.x = state_->velocity[0];
    state_msg.velocity.y = state_->velocity[1];
    state_msg.velocity.z = state_->velocity[2];
    state_pub_->publish(state_msg);

    // Publish the JointState msg with calibration offset applied
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = now;
    joint_state.name = {"waist", "shoulder", "elbow", "yaw", "pitch", "roll"};

    // Apply calibration offset (from /phantom/calibrate service)
    float t1 = state_->thetas[1] - state_->thetas_offset[1];
    float t2 = state_->thetas[2] - state_->thetas_offset[2];
    float t3 = state_->thetas[3] - state_->thetas_offset[3];
    float t4 = state_->thetas[4] - state_->thetas_offset[4];
    float t5 = state_->thetas[5] - state_->thetas_offset[5];
    float t6 = state_->thetas[6] - state_->thetas_offset[6];

    // Apply joint scales and offsets (from parameters - device-specific)
    // joint_scales: sign correction for each joint
    // joint_offsets: zero position offset for each joint (especially gimbal)
    joint_state.position = {
      joint_scales_[0] * t1 + joint_offsets_[0],  // waist
      joint_scales_[1] * t2 + joint_offsets_[1],  // shoulder
      joint_scales_[2] * t3 + joint_offsets_[2],  // elbow
      joint_scales_[3] * t4 + joint_offsets_[3],  // yaw
      joint_scales_[4] * t5 + joint_offsets_[4],  // pitch
      joint_scales_[5] * t6 + joint_offsets_[5]   // roll
    };
    joint_pub_->publish(joint_state);

    // Build the pose msg
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = now;
    pose_msg.header.frame_id = ref_frame_;
    pose_msg.pose = state_msg.pose;
    pose_msg.pose.position.x /= 1000.0;
    pose_msg.pose.position.y /= 1000.0;
    pose_msg.pose.position.z /= 1000.0;
    pose_pub_->publish(pose_msg);

    // Button events
    if ((state_->buttons[0] != state_->buttons_prev[0]) ||
        (state_->buttons[1] != state_->buttons_prev[1])) {
      if (state_->buttons[0] == 1) {
        state_->close_gripper = !(state_->close_gripper);
      }
      if (state_->buttons[1] == 1) {
        state_->lock = !(state_->lock);
      }
      omni_msgs::msg::OmniButtonEvent button_event;
      button_event.grey_button = state_->buttons[0];
      button_event.white_button = state_->buttons[1];
      state_->buttons_prev[0] = state_->buttons[0];
      state_->buttons_prev[1] = state_->buttons[1];
      button_pub_->publish(button_event);
    }
  }

  OmniState* state_;
  std::string omni_name_;
  std::string ref_frame_;
  int publish_rate_;
  std::vector<double> joint_scales_;
  std::vector<double> joint_offsets_;

  rclcpp::Publisher<omni_msgs::msg::OmniButtonEvent>::SharedPtr button_pub_;
  rclcpp::Publisher<omni_msgs::msg::OmniState>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<omni_msgs::msg::OmniFeedback>::SharedPtr force_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

HDCallbackCode HDCALLBACK omni_state_callback(void *pUserData) {
  OmniState *omni_state = static_cast<OmniState *>(pUserData);
  
  if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
    hdUpdateCalibration(calibrationStyle);
  }
  
  hdBeginFrame(hdGetCurrentDevice());
  
  // Get transform and angles
  hduMatrix transform;
  hdGetDoublev(HD_CURRENT_TRANSFORM, transform);
  hdGetDoublev(HD_CURRENT_JOINT_ANGLES, omni_state->joints);
  hduVector3Dd gimbal_angles;
  hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal_angles);
  
  // Position (inverting Z and swapping Y<-->Z)
  omni_state->position = hduVector3Dd(transform[3][0], -transform[3][2], transform[3][1]);
  omni_state->position /= omni_state->units_ratio;
  
  // Orientation (quaternion)
  hduMatrix rotation(transform);
  rotation.getRotationMatrix(rotation);
  hduMatrix rotation_offset(0.0, -1.0, 0.0, 0.0,
                            1.0,  0.0, 0.0, 0.0,
                            0.0,  0.0, 1.0, 0.0,
                            0.0,  0.0, 0.0, 1.0);
  rotation_offset.getRotationMatrix(rotation_offset);
  omni_state->rot = hduQuaternion(rotation_offset * rotation);
  
  // Velocity estimation
  hduVector3Dd vel_buff(0, 0, 0);
  vel_buff = (omni_state->position * 3 - 4 * omni_state->pos_hist1
      + omni_state->pos_hist2) / 0.002;
  omni_state->velocity = (.2196 * (vel_buff + omni_state->inp_vel3)
      + .6588 * (omni_state->inp_vel1 + omni_state->inp_vel2)) / 1000.0
      - (-2.7488 * omni_state->out_vel1 + 2.5282 * omni_state->out_vel2
          - 0.7776 * omni_state->out_vel3);
  omni_state->pos_hist2 = omni_state->pos_hist1;
  omni_state->pos_hist1 = omni_state->position;
  omni_state->inp_vel3 = omni_state->inp_vel2;
  omni_state->inp_vel2 = omni_state->inp_vel1;
  omni_state->inp_vel1 = vel_buff;
  omni_state->out_vel3 = omni_state->out_vel2;
  omni_state->out_vel2 = omni_state->out_vel1;
  omni_state->out_vel1 = omni_state->velocity;
  
  // Set forces
  hduVector3Dd feedback;
  feedback[0] = omni_state->force[0];
  feedback[1] = omni_state->force[2];
  feedback[2] = -omni_state->force[1];
  hdSetDoublev(HD_CURRENT_FORCE, feedback);

  // Get buttons
  int nButtons = 0;
  hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
  omni_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
  omni_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

  hdEndFrame(hdGetCurrentDevice());

  HDErrorInfo error;
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    hduPrintError(stderr, &error, "Error during main scheduler callback");
    if (hduIsSchedulerError(&error))
      return HD_CALLBACK_DONE;
  }

  float t[7] = {0., omni_state->joints[0], omni_state->joints[1],
      omni_state->joints[2] - omni_state->joints[1], gimbal_angles[0],
      gimbal_angles[1], gimbal_angles[2]};
  for (int i = 0; i < 7; i++)
    omni_state->thetas[i] = t[i];
    
  return HD_CALLBACK_CONTINUE;
}

void HHD_Auto_Calibration() {
  int supportedCalibrationStyles;
  HDErrorInfo error;

  hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
  if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET) {
    calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
    printf("HD_CALIBRATION_ENCODER_RESET..\n");
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL) {
    calibrationStyle = HD_CALIBRATION_INKWELL;
    printf("HD_CALIBRATION_INKWELL..\n");
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_AUTO) {
    calibrationStyle = HD_CALIBRATION_AUTO;
    printf("HD_CALIBRATION_AUTO..\n");
  }
  
  if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET) {
    do {
      hdUpdateCalibration(calibrationStyle);
      printf("Calibrating.. (put stylus in well)\n");
      if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Reset encoders reset failed.");
        break;
      }
    } while (hdCheckCalibration() != HD_CALIBRATION_OK);
    printf("Calibration complete.\n");
  }
  
  while (hdCheckCalibration() != HD_CALIBRATION_OK) {
    usleep(1e6);
    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT)
      printf("Please place the device into the inkwell for calibration\n");
    else if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
      printf("Calibration updated successfully\n");
      hdUpdateCalibration(calibrationStyle);
    }
    else
      printf("Unknown calibration status\n");
  }
}

int main(int argc, char** argv) {
  // Init Phantom
  HDErrorInfo error;
  HHD hHD;
  hHD = hdInitDevice(HD_DEFAULT_DEVICE);
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    fprintf(stderr, "Failed to initialize haptic device\n");
    return -1;
  }

  printf("Found %s.\n", hdGetString(HD_DEVICE_MODEL_TYPE));
  hdEnable(HD_FORCE_OUTPUT);
  hdStartScheduler();
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    fprintf(stderr, "Failed to start the scheduler\n");
    return -1;
  }
  HHD_Auto_Calibration();

  // Init ROS2
  rclcpp::init(argc, argv);
  OmniState state;
  auto node = std::make_shared<PhantomOmniNode>(&state);

  // Schedule haptic callback
  hdScheduleAsynchronous(omni_state_callback, &state, HD_MAX_SCHEDULER_PRIORITY);

  // Spin
  rclcpp::spin(node);

  // Cleanup
  printf("Ending Session....\n");
  running = false;
  hdStopScheduler();
  hdDisableDevice(hHD);
  rclcpp::shutdown();

  return 0;
}

