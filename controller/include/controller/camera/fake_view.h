#pragma once

#include <vector>

#include <interfaces/msg/point.hpp>
#include <interfaces/msg/pov.hpp>

class FastNoiseLite;

class FakeView {
public:
  struct Data {
    std::vector<float> x_cords;
    std::vector<float> y_cords;
    std::vector<float> bumps;
  };
public:
  static FakeView& GetInstance() {
    static FakeView instance;
    return instance;
  }

public:
  FakeView(const FakeView&) = delete;
  FakeView& operator=(const FakeView&) = delete;
  FakeView(FakeView&&) = delete;
  FakeView& operator=(FakeView&&) = delete;

public:
  Data Get(const interfaces::msg::Point& pos,
    const interfaces::msg::POV& pov,
    float view_step
  ) const;

  Data Get(const interfaces::msg::Point& pos, float view_step) const;

private:
  FakeView();
  ~FakeView() = default;

private:
  FastNoiseLite* noise_{nullptr};
};