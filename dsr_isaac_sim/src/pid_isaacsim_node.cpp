#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "dsr_isaac_sim/pid_controller.hpp"
#include <vector>
#include <string>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <chrono>
#include <mutex>
#include <algorithm>

using namespace std::chrono_literals;

class PidIsaacsimNode : public rclcpp::Node
{
public:
  PidIsaacsimNode()
  : Node("pid_isaacsim_node")
  {
    // PID & rate
    this->declare_parameter<double>("kp", 0.02);
    this->declare_parameter<double>("ki", 0.001);
    this->declare_parameter<double>("kd", 0.03);
    this->declare_parameter<double>("min_output", -3.14);
    this->declare_parameter<double>("max_output", 3.14);
    this->declare_parameter<int>("rate", 50);

    // 허용 조인트 목록 (기본: 6축 + 그리퍼)
    this->declare_parameter<std::vector<std::string>>(
      "allowed_joints",
      std::vector<std::string>{
        "joint_1","joint_2","joint_3","joint_4","joint_5","joint_6",
        "robotiq_85_left_knuckle_joint"
      });

    // 별칭(오타/선호 대응): allow_joints 가 설정되면 allowed_joints 를 대체
    this->declare_parameter<std::vector<std::string>>("allow_joints", std::vector<std::string>{});

    // 그리퍼 등 패스스루(입력값 그대로 출력) 목록
    this->declare_parameter<std::vector<std::string>>(
      "passthrough_joints",
      std::vector<std::string>{"robotiq_85_left_knuckle_joint"});

    // Get params
    kp_ = this->get_parameter("kp").as_double();
    ki_ = this->get_parameter("ki").as_double();
    kd_ = this->get_parameter("kd").as_double();
    min_output_ = this->get_parameter("min_output").as_double();
    max_output_ = this->get_parameter("max_output").as_double();
    int rate = this->get_parameter("rate").as_int();
    if (rate <= 0) { RCLCPP_WARN(get_logger(), "rate must be > 0; forcing 100 Hz"); rate = 100; }

    allowed_joints_ = this->get_parameter("allowed_joints").as_string_array();
    {
      auto alias = this->get_parameter("allow_joints").as_string_array();
      if (!alias.empty()) {
        allowed_joints_ = alias;
        RCLCPP_INFO(get_logger(), "Using allow_joints (alias) to override allowed_joints.");
      }
      if (allowed_joints_.empty()) {
        allowed_joints_ = {"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","robotiq_85_left_knuckle_joint"};
        RCLCPP_WARN(get_logger(), "allowed_joints empty; using default 6+gripper.");
      }
    }

    auto passthrough_vec = this->get_parameter("passthrough_joints").as_string_array();
    passthrough_joints_.insert(passthrough_vec.begin(), passthrough_vec.end());

    // 빠른 인덱싱용 맵
    index_of_allowed_.clear();
    for (size_t i = 0; i < allowed_joints_.size(); ++i)
      index_of_allowed_[allowed_joints_[i]] = i;

    RCLCPP_INFO(get_logger(), "PID: kp=%.5f ki=%.5f kd=%.5f, rate=%d Hz", kp_, ki_, kd_, rate);
    RCLCPP_INFO(get_logger(), "Allowed joints (%zu): %s",
                allowed_joints_.size(), vec_to_csv(allowed_joints_).c_str());
    RCLCPP_INFO(get_logger(), "Passthrough joints (%zu): %s",
                passthrough_joints_.size(), set_to_csv(passthrough_joints_).c_str());

    // I/O
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&PidIsaacsimNode::joint_state_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states_to_isaac", 10);

    // 내부 버퍼: allowed_joints 기준
    current_positions_.name = allowed_joints_;
    current_positions_.position.assign(allowed_joints_.size(), 0.0);
    current_positions_.velocity.assign(allowed_joints_.size(), 0.0);
    current_positions_.effort.assign(allowed_joints_.size(), 0.0);

    target_positions_.name = allowed_joints_;
    target_positions_.position.assign(allowed_joints_.size(), 0.0);
    target_positions_.velocity.assign(allowed_joints_.size(), 0.0);
    target_positions_.effort.assign(allowed_joints_.size(), 0.0);

    // 타이머
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / rate),
      std::bind(&PidIsaacsimNode::pid_timer_callback, this));
  }

private:
  static std::string vec_to_csv(const std::vector<std::string>& v) {
    std::string s; for (size_t i=0;i<v.size();++i){ s+=v[i]; if(i+1<v.size()) s+=','; } return s;
  }
  static std::string set_to_csv(const std::unordered_set<std::string>& st) {
    std::string s; size_t i=0; for (auto &e: st){ s+=e; if(++i<st.size()) s+=','; } return s;
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // name -> position 매핑
    std::unordered_map<std::string, double> incoming_pos;
    const size_t n = std::min(msg->name.size(), msg->position.size());
    for (size_t i = 0; i < n; ++i) incoming_pos[msg->name[i]] = msg->position[i];

    const bool first_time = pid_controllers_.empty();

    // allowed 순서대로만 갱신
    for (size_t i = 0; i < allowed_joints_.size(); ++i) {
      const auto& jn = allowed_joints_[i];

      // 패스스루가 아니면 PID 준비
      if (passthrough_joints_.find(jn) == passthrough_joints_.end()) {
        if (pid_controllers_.find(jn) == pid_controllers_.end()) {
          pid_controllers_[jn] = std::make_unique<PIDController>(kp_, ki_, kd_, min_output_, max_output_);
          RCLCPP_INFO(this->get_logger(), "Initialized PID for joint: %s", jn.c_str());
        }
      }

      auto it = incoming_pos.find(jn);
      if (it != incoming_pos.end()) {
        // 타겟 갱신
        target_positions_.position[i] = it->second;

        // 첫 메시지면 현재도 맞춰 초기화 (점프 방지)
        if (first_time) current_positions_.position[i] = it->second;
      } else if (first_time) {
        // 첫 메시지인데 입력에 없으면 그대로 유지
        current_positions_.position[i] = current_positions_.position[i];
      }
    }
  }

  void pid_timer_callback()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // 패스스루만 있는 경우도 허용 (pid_controllers_ 비어있어도 퍼블리시 가능)
    bool has_any_controller_or_passthrough = !allowed_joints_.empty();
    if (!has_any_controller_or_passthrough) return;

    sensor_msgs::msg::JointState out;
    out.header.stamp = this->get_clock()->now();
    out.name = allowed_joints_;
    out.position.resize(allowed_joints_.size());
    out.velocity.assign(allowed_joints_.size(), 0.0);
    out.effort.assign(allowed_joints_.size(), 0.0);

    for (size_t i = 0; i < allowed_joints_.size(); ++i) {
      const auto& jn = allowed_joints_[i];

      // 그리퍼 등 패스스루: 입력(target)을 그대로 출력
      if (passthrough_joints_.find(jn) != passthrough_joints_.end()) {
        const double passthrough = target_positions_.position[i];
        out.position[i] = passthrough;
        current_positions_.position[i] = passthrough; // 내부 상태 동기화
        continue;
      }

      // 일반 조인트: PID 적용
      auto it = pid_controllers_.find(jn);
      if (it == pid_controllers_.end()) {
        RCLCPP_WARN_ONCE(this->get_logger(), "PID for '%s' missing; passing current.", jn.c_str());
        out.position[i] = current_positions_.position[i];
        continue;
      }
      const double target = target_positions_.position[i];
      const double current = current_positions_.position[i];
      const double new_pos = it->second->compute(target, current);

      out.position[i] = new_pos;
      current_positions_.position[i] = new_pos;
    }

    publisher_->publish(out);
  }

  // ROS I/O
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 내부 상태(항상 allowed_joints 순서)
  sensor_msgs::msg::JointState target_positions_;
  sensor_msgs::msg::JointState current_positions_;

  // 필터/색인
  std::vector<std::string> allowed_joints_;
  std::map<std::string, size_t> index_of_allowed_;
  std::unordered_set<std::string> passthrough_joints_;

  // PID
  std::map<std::string, std::unique_ptr<PIDController>> pid_controllers_;

  std::mutex mutex_;
  double kp_, ki_, kd_;
  double min_output_, max_output_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PidIsaacsimNode>());
  rclcpp::shutdown();
  return 0;
}
