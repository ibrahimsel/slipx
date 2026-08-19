// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0

#include "slipx/scene/broadphase.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "ray_intersect.hpp"

namespace slipx {
namespace scene {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

// How far every bounding box is fattened. A ray through the exact corner of
// a wall (which is what a diagonal from integer coordinates produces, and
// what the emitter pose of a corner-mounted sensor is) can pass the exact
// segment test while the zero-slack slab test rejects the box by an ulp:
// the two computations round differently. A nanometre of padding is a
// thousand times wider than the arithmetic can wobble and a thousand times
// narrower than anything physical, the same argument the grid's corner
// margin makes; padding can only add tests, never change an answer, because
// the exact test stays the authority.
constexpr double kPad = 1e-9;

// As in raycast.cpp, and for the same measured reason: fmin/fmax are libm
// calls because of their NaN contract, and nothing compared here is NaN
// unless the caller's pose already is.
constexpr double smaller(double a, double b) { return a < b ? a : b; }
constexpr double larger(double a, double b) { return a > b ? a : b; }

// Entry distance of a ray into an axis-aligned box, or +inf for a miss
// within [0, limit). The parallel case is branched explicitly rather than
// left to IEEE division: a ray running exactly along a box edge would
// otherwise compute 0/0 and reject a box it touches.
inline double aabb_entry(double x, double y, double dx, double dy,
                         double min_x, double min_y, double max_x,
                         double max_y, double limit) {
  double enter = 0.0;
  double leave = limit;

  if (dx == 0.0) {
    if (x < min_x || x > max_x) return kInf;
  } else {
    double t0 = (min_x - x) / dx;
    double t1 = (max_x - x) / dx;
    if (t0 > t1) std::swap(t0, t1);
    enter = larger(enter, t0);
    leave = smaller(leave, t1);
  }
  if (dy == 0.0) {
    if (y < min_y || y > max_y) return kInf;
  } else {
    double t0 = (min_y - y) / dy;
    double t1 = (max_y - y) / dy;
    if (t0 > t1) std::swap(t0, t1);
    enter = larger(enter, t0);
    leave = smaller(leave, t1);
  }
  return leave < enter ? kInf : enter;
}

}  // namespace

// ------------------------------------------------------------------ SceneBvh

SceneBvh::SceneBvh(const Walls& walls) {
  // The same span enumeration Walls uses, over its own mitred polylines, so
  // the geometry is byte-identical to what the grid accelerates.
  const std::vector<double>& lx = walls.left_x();
  const std::vector<double>& ly = walls.left_y();
  const std::vector<double>& rx = walls.right_x();
  const std::vector<double>& ry = walls.right_y();
  const std::size_t count = lx.size();
  if (count < 2) return;
  const std::size_t spans = walls.closed() ? count : count - 1;

  struct Item {
    Segment segment;
    std::uint8_t left;
    double min_x, min_y, max_x, max_y;
    double cx, cy;
  };
  std::vector<Item> input;
  input.reserve(spans * 2);
  const auto add = [&input](double ax, double ay, double bx, double by,
                            bool left) {
    Item item;
    item.segment = Segment{ax, ay, bx - ax, by - ay};
    item.left = left ? 1u : 0u;
    item.min_x = smaller(ax, bx) - kPad;
    item.max_x = larger(ax, bx) + kPad;
    item.min_y = smaller(ay, by) - kPad;
    item.max_y = larger(ay, by) + kPad;
    item.cx = 0.5 * (ax + bx);
    item.cy = 0.5 * (ay + by);
    input.push_back(item);
  };
  for (std::size_t i = 0; i < spans; ++i) {
    const std::size_t j = (i + 1) % count;
    add(lx[i], ly[i], lx[j], ly[j], true);
    add(rx[i], ry[i], rx[j], ry[j], false);
  }

  std::vector<std::uint32_t> items(input.size());
  for (std::uint32_t i = 0; i < items.size(); ++i) items[i] = i;

  nodes_.reserve(2 * input.size());

  // Recursive median build over [begin, end) of `items`. The split is fully
  // specified: a stable criterion (centroid on the wider axis, then the
  // segment index) through std::sort, so every standard library builds the
  // same tree; nth_element would be faster and leaves the order of equals
  // to the implementation, which is a per-platform tree for no benefit at
  // a few thousand segments built once.
  constexpr std::size_t kLeafSize = 4;
  const auto build = [&](const auto& self, std::size_t begin,
                         std::size_t end) -> std::uint32_t {
    Node node;
    node.min_x = kInf;
    node.min_y = kInf;
    node.max_x = -kInf;
    node.max_y = -kInf;
    for (std::size_t k = begin; k < end; ++k) {
      const Item& item = input[items[k]];
      node.min_x = smaller(node.min_x, item.min_x);
      node.min_y = smaller(node.min_y, item.min_y);
      node.max_x = larger(node.max_x, item.max_x);
      node.max_y = larger(node.max_y, item.max_y);
    }

    const std::uint32_t index = static_cast<std::uint32_t>(nodes_.size());
    nodes_.push_back(node);

    if (end - begin <= kLeafSize) {
      nodes_[index].first = static_cast<std::uint32_t>(segments_.size());
      nodes_[index].count = static_cast<std::uint32_t>(end - begin);
      for (std::size_t k = begin; k < end; ++k) {
        segments_.push_back(input[items[k]].segment);
        segment_left_.push_back(input[items[k]].left);
      }
      return index;
    }

    const bool split_x =
        (node.max_x - node.min_x) >= (node.max_y - node.min_y);
    std::sort(items.begin() + static_cast<std::ptrdiff_t>(begin),
              items.begin() + static_cast<std::ptrdiff_t>(end),
              [&input, split_x](std::uint32_t a, std::uint32_t b) {
                const double ca = split_x ? input[a].cx : input[a].cy;
                const double cb = split_x ? input[b].cx : input[b].cy;
                if (ca != cb) return ca < cb;
                return a < b;
              });

    const std::size_t middle = begin + (end - begin) / 2;
    // Pre-order: the left child always lands at index + 1, so the node only
    // needs to store where the right child ended up (the left subtree's
    // size is not one, which is exactly the mistake this comment is here to
    // stop being reintroduced).
    self(self, begin, middle);
    const std::uint32_t right_child = self(self, middle, end);
    nodes_[index].first = right_child;
    nodes_[index].count = 0;
    return index;
  };
  build(build, 0, items.size());
}

RayHit SceneBvh::cast(double x, double y, double bearing,
                      double max_range) const {
  RayHit best;
  double best_range = max_range;
  if (nodes_.empty()) return best;

  const double dx = std::cos(bearing);
  const double dy = std::sin(bearing);

  // An explicit stack of (node, entry distance). Depth is bounded by the
  // median build: ceil(log2(n / kLeafSize)) + 1 levels, so 64 covers any
  // track that fits in memory several times over.
  struct Frame {
    std::uint32_t node;
    double entry;
  };
  Frame stack[64];
  int top = 0;
  {
    const Node& root = nodes_[0];
    const double entry = aabb_entry(x, y, dx, dy, root.min_x, root.min_y,
                                    root.max_x, root.max_y, best_range);
    if (entry < kInf) stack[top++] = Frame{0, entry};
  }

  while (top > 0) {
    const Frame frame = stack[--top];
    // Pruned against the CURRENT best, which may have improved since the
    // frame was pushed.
    if (frame.entry >= best_range) continue;
    const Node& node = nodes_[frame.node];

    if (node.count > 0) {
      for (std::uint32_t k = node.first; k < node.first + node.count; ++k) {
        const Segment& s = segments_[k];
        const double t = detail::ray_segment(x, y, dx, dy, s.ax, s.ay, s.ex,
                                             s.ey, best_range);
        if (t >= 0.0) {
          best_range = t;
          best.hit = true;
          best.left_wall = segment_left_[k] != 0u;
        }
      }
      continue;
    }

    const std::uint32_t left = frame.node + 1;   // pre-order: always next
    const std::uint32_t right = node.first;
    const Node& ln = nodes_[left];
    const Node& rn = nodes_[right];
    double lt = aabb_entry(x, y, dx, dy, ln.min_x, ln.min_y, ln.max_x,
                           ln.max_y, best_range);
    double rt = aabb_entry(x, y, dx, dy, rn.min_x, rn.min_y, rn.max_x,
                           rn.max_y, best_range);
    // Nearer child on top of the stack, so it is visited first and its hit
    // can prune the farther one. On an exact tie the left child is visited
    // first, deterministically.
    std::uint32_t first_node = left, second_node = right;
    double first_t = lt, second_t = rt;
    if (rt < lt) {
      first_node = right;
      second_node = left;
      first_t = rt;
      second_t = lt;
    }
    if (second_t < kInf) stack[top++] = Frame{second_node, second_t};
    if (first_t < kInf) stack[top++] = Frame{first_node, first_t};
  }

  best.range = best.hit ? best_range : 0.0;
  return best;
}

// -------------------------------------------------------------- AgentOverlay

void AgentOverlay::resize(std::size_t count) {
  boxes_.assign(count, Box{});
  order_.resize(count);
}

void AgentOverlay::set(std::size_t i, double x, double y, double yaw,
                       double half_length, double half_width, bool active) {
  Box& box = boxes_.at(i);
  box.x = x;
  box.y = y;
  box.c = std::cos(yaw);
  box.s = std::sin(yaw);
  box.hl = half_length;
  box.hw = half_width;
  const double extent_x =
      half_length * std::fabs(box.c) + half_width * std::fabs(box.s) + kPad;
  const double extent_y =
      half_length * std::fabs(box.s) + half_width * std::fabs(box.c) + kPad;
  box.min_x = x - extent_x;
  box.max_x = x + extent_x;
  box.min_y = y - extent_y;
  box.max_y = y + extent_y;
  box.active = active;
}

OverlayHit AgentOverlay::cast(double x, double y, double bearing,
                              double max_range, std::size_t skip) const {
  const double dx = std::cos(bearing);
  const double dy = std::sin(bearing);

  OverlayHit best;
  double best_range = max_range;
  for (std::size_t i = 0; i < boxes_.size(); ++i) {
    const Box& box = boxes_[i];
    if (!box.active || i == skip) continue;
    // The bounds prefilter is what makes this the accelerated query; it
    // must never cull a box the exact test would hit, and the brute-force
    // agreement tests hold it to that.
    if (aabb_entry(x, y, dx, dy, box.min_x, box.min_y, box.max_x, box.max_y,
                   best_range) == kInf) {
      continue;
    }

    // Into the box frame (rotate by -yaw about the centre).
    const double px = x - box.x;
    const double py = y - box.y;
    const double ox = box.c * px + box.s * py;
    const double oy = -box.s * px + box.c * py;
    const double ldx = box.c * dx + box.s * dy;
    const double ldy = -box.s * dx + box.c * dy;
    const double t = aabb_entry(ox, oy, ldx, ldy, -box.hl, -box.hw, box.hl,
                                box.hw, best_range);
    if (t < best_range) {
      best_range = t;
      best.hit = true;
      best.index = i;
    }
  }
  best.range = best.hit ? best_range : 0.0;
  return best;
}

OverlayHit AgentOverlay::cast_brute_force(double x, double y, double bearing,
                                          double max_range,
                                          std::size_t skip) const {
  const double dx = std::cos(bearing);
  const double dy = std::sin(bearing);

  OverlayHit best;
  double best_range = max_range;
  for (std::size_t i = 0; i < boxes_.size(); ++i) {
    const Box& box = boxes_[i];
    if (!box.active || i == skip) continue;
    const double px = x - box.x;
    const double py = y - box.y;
    const double ox = box.c * px + box.s * py;
    const double oy = -box.s * px + box.c * py;
    const double ldx = box.c * dx + box.s * dy;
    const double ldy = -box.s * dx + box.c * dy;
    const double t = aabb_entry(ox, oy, ldx, ldy, -box.hl, -box.hw, box.hl,
                                box.hw, best_range);
    if (t < best_range) {
      best_range = t;
      best.hit = true;
      best.index = i;
    }
  }
  best.range = best.hit ? best_range : 0.0;
  return best;
}

void AgentOverlay::overlapping_pairs(
    std::vector<std::pair<std::uint32_t, std::uint32_t>>& pairs) const {
  pairs.clear();

  // Sweep along x: sort the active boxes by their lower x bound (index as
  // the tie-break, so the order is fully specified), then each box only
  // looks ahead while lower bounds still reach its own upper bound.
  std::size_t active = 0;
  for (std::uint32_t i = 0; i < boxes_.size(); ++i) {
    if (boxes_[i].active) order_[active++] = i;
  }
  std::sort(order_.begin(), order_.begin() + static_cast<std::ptrdiff_t>(active),
            [this](std::uint32_t a, std::uint32_t b) {
              if (boxes_[a].min_x != boxes_[b].min_x) {
                return boxes_[a].min_x < boxes_[b].min_x;
              }
              return a < b;
            });

  for (std::size_t oi = 0; oi < active; ++oi) {
    const std::uint32_t i = order_[oi];
    const Box& a = boxes_[i];
    for (std::size_t oj = oi + 1; oj < active; ++oj) {
      const std::uint32_t j = order_[oj];
      const Box& b = boxes_[j];
      if (b.min_x > a.max_x) break;   // sorted: nothing further reaches back
      if (b.min_y > a.max_y || a.min_y > b.max_y) continue;
      pairs.emplace_back(i < j ? i : j, i < j ? j : i);
    }
  }
  // Canonical output order, so two implementations (and two runs) can be
  // compared with operator== rather than as sets.
  std::sort(pairs.begin(), pairs.end());
}

void AgentOverlay::overlapping_pairs_brute_force(
    std::vector<std::pair<std::uint32_t, std::uint32_t>>& pairs) const {
  pairs.clear();
  for (std::uint32_t i = 0; i < boxes_.size(); ++i) {
    if (!boxes_[i].active) continue;
    for (std::uint32_t j = i + 1; j < boxes_.size(); ++j) {
      if (!boxes_[j].active) continue;
      const Box& a = boxes_[i];
      const Box& b = boxes_[j];
      if (a.min_x <= b.max_x && b.min_x <= a.max_x && a.min_y <= b.max_y &&
          b.min_y <= a.max_y) {
        pairs.emplace_back(i, j);
      }
    }
  }
}

}  // namespace scene
}  // namespace slipx
