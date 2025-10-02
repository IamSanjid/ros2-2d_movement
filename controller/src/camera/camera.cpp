#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <interfaces/msg/point.hpp>
#include <interfaces/msg/pov.hpp>
#include <interfaces/msg/camera_view.hpp>
#include <interfaces/srv/get_pos.hpp>
#include <interfaces/action/move.hpp>

#include "fake_view.h"

using namespace std::chrono_literals;

// class CameraTest {
// public:
//   CameraTest()
//     : node_("Camera") {

//   }

// public:
//   auto AsSharedNode() {
//     return std::shared_ptr<rclcpp::Node>(&this->node_, [](auto) {});
//   }

// private:
//   rclcpp::Node node_;
// };

class Camera : public rclcpp::Node {
public:
  Camera()
    : Node("Camera") {
      // this->declare_parameter("distance", 10.0);
      // this->declare_parameter("angle_x", 45.0);
      // this->declare_parameter("angle_y", 45.0);
      // this->declare_parameter("rotation", 0.0);
      this->declare_parameter("view_step", 0.3);

      const auto qos = rclcpp::QoS(1)
        .best_effort()
        .durability_volatile();
      view_publisher_ = this->create_publisher<interfaces::msg::CameraView>("ViewBumps", qos);

      get_pos_client_ = this->create_client<interfaces::srv::GetPos>("GetPos");
      while (!get_pos_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Will exit soon...");
          return;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
      }
      SyncWithBodyPos();

      pos_update_subscription_ = this->create_subscription<interfaces::msg::Point>("PosUpdate",
                                                                                   qos,
                                                                                   std::bind(&Camera::OnBodyPosUpdate, this, std::placeholders::_1));

      view_publish_timer_ = this->create_wall_timer(1000ms, [this]() {
        this->PublishViewBumps();
      });

      parameter_set_callback_ = this->add_on_set_parameters_callback(std::bind(
                                                                      &Camera::OnParamSet,
                                                                      this,
                                                                      std::placeholders::_1));
    }
  
public:
  // inline interfaces::msg::POV POV() const {
  //   interfaces::msg::Point angle{};
  //   angle.x = static_cast<float>(this->get_parameter("angle_x").as_double());
  //   angle.y = static_cast<float>(this->get_parameter("angle_y").as_double());

  //   interfaces::msg::POV pov{};
  //   pov.distance = static_cast<float>(this->get_parameter("distance").as_double());
  //   pov.angle = angle;
  //   pov.rotation = static_cast<float>(this->get_parameter("rotation").as_double());

  //   return pov;
  // }

  inline float ViewStep() const {
    auto view_step = static_cast<float>(this->get_parameter("view_step").as_double());
    return view_step;
  }

  void SyncWithBodyPos() {
    auto request = std::make_shared<interfaces::srv::GetPos::Request>();
    auto future = get_pos_client_->async_send_request(request);

    rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
    if (future.valid()) {
      pos_ = future.get()->pos;
      RCLCPP_INFO(this->get_logger(), "Synced Pos with Body!");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service GetPos");
    }
  }

private:
  void PublishViewBumps() {
    // RCLCPP_INFO(this->get_logger(), "Publishing View Bumps");

    auto data = FakeView::GetInstance().Get(this->pos_, ViewStep());

    interfaces::msg::CameraView msg{};
    msg.x_cords = data.x_cords;
    msg.y_cords = data.y_cords;
    msg.bumps = data.bumps;

    view_publisher_->publish(msg);
  }

  rcl_interfaces::msg::SetParametersResult OnParamSet(const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result{};
    result.successful = true;
    result.reason = "";

    for (auto& param: parameters) {
      if (param.get_name() == "view_step") {
        auto value = param.as_double();
        if (value > 1.0 || value < 0.1) {
          result.successful = false;
          result.reason = "View step must be between 0.1 and 1.0";
          RCLCPP_INFO(this->get_logger(), "Rejecting param set");
        }
      }
    }

    return result;
  }

  void OnBodyPosUpdate(const interfaces::msg::Point::UniquePtr pos) {
    this->pos_ = *pos;
    PublishViewBumps();
  }

private:
  rclcpp::TimerBase::SharedPtr view_publish_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_set_callback_;
  rclcpp::Publisher<interfaces::msg::CameraView>::SharedPtr view_publisher_;
  rclcpp::Client<interfaces::srv::GetPos>::SharedPtr get_pos_client_;
  rclcpp::Subscription<interfaces::msg::Point>::SharedPtr pos_update_subscription_;

  interfaces::msg::Point pos_{};
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<Camera>());

  // CameraTest ct{};
  // rclcpp::spin(ct.AsSharedNode());

  rclcpp::shutdown();
  return 0;
}
