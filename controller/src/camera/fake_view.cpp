#include "fake_view.h"

#include "FastNoiseLite.h"

#include <iostream>

constexpr uint32_t kTileSize = 10;
constexpr uint32_t kScaleViewStep = 10;

static float Deg2Rad(float deg) {
  return deg * (static_cast<float>(std::numbers::pi) / 180.0f);
}

FakeView::FakeView() : noise_(new FastNoiseLite()) {
  noise_->SetNoiseType(FastNoiseLite::NoiseType_OpenSimplex2);
}

FakeView::Data FakeView::Get(const interfaces::msg::Point &pos,
  const interfaces::msg::POV &pov,
  float view_step
) const {
  const float tile_step = view_step * kTileSize;  
  if (pov.distance <= tile_step) {
    const auto x_cords = std::vector<float>({pos.x});
    const auto y_cords = std::vector<float>({pos.y});
    const auto bumps = std::vector<float>({noise_->GetNoise(pos.x, pos.y)});
    return Data{ x_cords, y_cords, bumps };
  }

  std::vector<float> x_cords{};
  std::vector<float> y_cords{};
  std::vector<float> bumps{};

  float distance = pov.distance;
  while (distance > tile_step) {
    const float hyp_x = distance / std::cos(Deg2Rad(90.0 - pov.angle.x));
    const float hyp_y = distance / std::cos(Deg2Rad(90.0 - pov.angle.y));

    const float opp_x = hyp_x * std::sin(Deg2Rad(90.0 - pov.angle.x));
    const float opp_y = hyp_y * std::sin(Deg2Rad(90.0 - pov.angle.y));

    // TODO: Use Tile size to get bumps for all the tiles in current smaller pov.
    const float x = std::floor(pos.x + distance);
    const float y1 = std::floor(pos.y - opp_x);
    const float y2 = std::floor(pos.y);
    const float y3 = std::floor(pos.y + opp_y);

    x_cords.push_back(x);
    x_cords.push_back(x);
    x_cords.push_back(x);

    y_cords.push_back(y1);
    y_cords.push_back(y2);
    y_cords.push_back(y3);

    bumps.push_back(noise_->GetNoise(x, y1));
    bumps.push_back(noise_->GetNoise(x, y2));
    bumps.push_back(noise_->GetNoise(x, y3));

    distance -= tile_step;
  }
  
  return Data{ x_cords, y_cords, bumps };
}

// Race Condition
FakeView::Data FakeView::Get(const interfaces::msg::Point &pos, float view_step) const {
  const auto pov_radius = (kScaleViewStep * kTileSize) * view_step;
  const auto pov_radius_in_tile = std::floor(pov_radius / kTileSize);
  const auto pov_radius_in_tile2 = pov_radius_in_tile * pov_radius_in_tile;

  std::vector<float> x_cords{};
  std::vector<float> y_cords{};
  std::vector<float> bumps{};

  const float min_x = std::floor((pos.x - pov_radius_in_tile));
  const float max_x = std::floor((pos.x + pov_radius_in_tile));
  const float min_y = std::floor((pos.y - pov_radius_in_tile));
  const float max_y = std::floor((pos.y + pov_radius_in_tile));

  for (float tile_x = min_x; tile_x <= max_x; tile_x++) {
    for (float tile_y = min_y; tile_y <= max_y; tile_y++) {
      float dx = pos.x - tile_x;
      float dy = pos.y - tile_y;
      float dist2 = dx * dx + dy * dy;
      if (dist2 > pov_radius_in_tile2) continue;

      x_cords.push_back(tile_x);
      y_cords.push_back(tile_y);
      bumps.push_back(noise_->GetNoise(tile_x, tile_y));
    }
  }

  return Data { x_cords, y_cords, bumps };
}
