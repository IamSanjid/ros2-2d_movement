#include <chrono>
#include <functional>
#include <mutex>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <interfaces/msg/point.hpp>
#include <interfaces/srv/get_pos.hpp>
#include <interfaces/action/move.hpp>

using Point = interfaces::msg::Point;
using GetPos = interfaces::srv::GetPos;
using Move = interfaces::action::Move;
using MoveGoalHandle = rclcpp_action::ServerGoalHandle<Move>;

using PositionChangedCallback = std::function<void(Point)>;

class MoveProcessor {
  enum class Direction {
    UP = 0,
    DOWN = 1,
    LEFT = 2,
    RIGHT = 3,
  };

public:
  static MoveProcessor& GetInstance() {
    static MoveProcessor instance;
    return instance;
  }

  bool IsIdle() const {
    if (!move_mutex_.try_lock()) {
      return false;
    }

    std::lock_guard<std::mutex> guard(move_mutex_, std::adopt_lock);
    if (move_ != nullptr) {
      return false;
    }

    return true;
  }

  void Move(const std::shared_ptr<MoveGoalHandle> handle) {
    std::scoped_lock lock{move_mutex_};
    move_ = new std::shared_ptr<MoveGoalHandle>(handle);
    stop_source_ = std::stop_source{};
  }

  void StopMoving() {
    stop_source_.request_stop();
  }

  void AddPositionChangedCallback(PositionChangedCallback callback) {
    std::scoped_lock lock{move_mutex_};
    pos_changed_callbacks_.push_back(callback);
  }

  Point GetPos() {
    std::scoped_lock lock{move_mutex_};
    return pos_;
  }

private:
  MoveProcessor() {
    pos_.set__x(50.f).set__y(50.f);
    worker_ = std::jthread(std::bind(&MoveProcessor::Process, this, std::placeholders::_1));
    worker_.detach();
  }

  void Process(std::stop_token stoken) {
    while (!stoken.stop_requested()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (move_mutex_.try_lock()) {
        std::lock_guard<std::mutex> guard(move_mutex_, std::adopt_lock);
        if (move_ != nullptr) {
          MoveInternal(*move_, stop_source_.get_token());
          delete move_;
          move_ = nullptr;
        }
      }
    }
  }

private:
  void MoveInternal(const std::shared_ptr<MoveGoalHandle>& handle, std::stop_token stoken) {
    const auto& move_param = handle->get_goal();
    auto dir = static_cast<Direction>(move_param->direction);
    
    auto feedback = std::make_shared<Move::Feedback>();
    auto result = std::make_shared<Move::Result>();

    feedback->new_pos = pos_;

    auto& new_pos_x = feedback->new_pos.x;
    auto& new_pos_y = feedback->new_pos.y;
    
    for (auto cur_step = 1; cur_step <= move_param->step && !stoken.stop_requested(); cur_step++) {
      switch (dir)
      {
      case Direction::UP: new_pos_y -= 1; break;
      case Direction::DOWN: new_pos_y += 1; break;
      case Direction::LEFT: new_pos_x -= 1; break;
      case Direction::RIGHT: new_pos_x += 1; break;
      default: goto CANCEL;
      }

      if (new_pos_x < 0 || new_pos_y < 0) {
        goto CANCEL;
      }

      pos_.x = new_pos_x;
      pos_.y = new_pos_y;

      handle->publish_feedback(feedback);
      for (auto& callback : pos_changed_callbacks_) {
        callback(pos_);
      }
      // simulating delay
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
    result->new_pos = pos_;
    handle->succeed(result);
    return;

    CANCEL:
    result->new_pos = pos_;
    handle->canceled(result);
  }

private:
  mutable std::mutex move_mutex_{};
  std::shared_ptr<MoveGoalHandle>* move_{nullptr};
  std::stop_source stop_source_{};
  std::jthread worker_;
  Point pos_;

  std::vector<PositionChangedCallback> pos_changed_callbacks_{};
};

class Body : public rclcpp::Node {
public:
  Body()
    : Node("Body") {
      pos_update_publisher_ = this->create_publisher<Point>("PosUpdate",
                                                            rclcpp::QoS(1)
                                                            .best_effort()
                                                            .durability_volatile());

      get_pos_srv_ = this->create_service<GetPos>("GetPos",
                                                  std::bind(&Body::OnGetPos, this, std::placeholders::_1, std::placeholders::_2));

      move_server_ = rclcpp_action::create_server<Move>(this,
                                                        "Move",
                                                        std::bind(&Body::HandleMoveGoal, this, std::placeholders::_1, std::placeholders::_2),
                                                        std::bind(&Body::HandleMoveCancel, this, std::placeholders::_1),
                                                        std::bind(&Body::HandleMoveAccepted, this, std::placeholders::_1));

      MoveProcessor::GetInstance().AddPositionChangedCallback(std::bind(&Body::OnPosChanged, this, std::placeholders::_1));
    }
private:
  void OnGetPos(const std::shared_ptr<GetPos::Request> request,
          std::shared_ptr<GetPos::Response> response) {
    (void)request;
    response->pos = MoveProcessor::GetInstance().GetPos();
    RCLCPP_INFO(this->get_logger(), "GetPos was called!");
  }

  rclcpp_action::GoalResponse HandleMoveGoal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Move::Goal> goal) {
    const auto pos = MoveProcessor::GetInstance().GetPos();
    const bool is_idle = MoveProcessor::GetInstance().IsIdle();
    if (goal->from_pos.x != pos.x || goal->from_pos.y != pos.y || !is_idle) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "Received goal request from %f, %f", goal->from_pos.x, goal->from_pos.y);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };

  rclcpp_action::CancelResponse HandleMoveCancel(const std::shared_ptr<MoveGoalHandle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    MoveProcessor::GetInstance().StopMoving();
    return rclcpp_action::CancelResponse::ACCEPT;
  };

  void HandleMoveAccepted(const std::shared_ptr<MoveGoalHandle> goal_handle) {
    MoveProcessor::GetInstance().Move(goal_handle);
  };

  void OnPosChanged(Point new_pos) {
    pos_update_publisher_->publish(new_pos);
  }

private:
  rclcpp::Publisher<Point>::SharedPtr pos_update_publisher_;
  rclcpp::Service<GetPos>::SharedPtr get_pos_srv_;
  rclcpp_action::Server<Move>::SharedPtr move_server_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<Body>());

  rclcpp::shutdown();
  return 0;
}
