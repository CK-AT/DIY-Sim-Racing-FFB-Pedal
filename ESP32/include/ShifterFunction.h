#pragma once
#include <vector>

#include "Arduino.h"
#include "CommManager.fwd.h"
#include "ConfigManager.h"
#include "IFunction.h"
#include "Physics.h"

enum class AxisRole {
    Unknown,
    X,
    Y
};

struct Interval { float lo, hi; };

struct GateSegPre {
  bool horizontal;  // true=H, false=V
  float hw;         // half width (mm)
  float line;       // H: yLine, V: xLine
  float a0_cap;     // along-range min with caps (H:xMinCap, V:yMinCap)
  float a1_cap;     // along-range max with caps
};

struct DetentPre {
  float x_mm;
  float y_mm;
  float radius_mm;
  float spring_N_per_mm;

  int8_t laneV; // assigned vertical lane index (0..vCount-1) or -1
  int8_t laneH; // assigned horizontal lane index (0..hCount-1) or -1
};

static inline float clampf(float v, float lo, float hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }
static inline float absf(float v) { return (v >= 0.0f) ? v : -v; }
static inline void order2(float& a, float& b) { if (a > b) { float t=a; a=b; b=t; } }

template<int MAX_SEGS, int MAX_DETS, int MAX_LANES, int MAX_DETS_PER_LANE, int MAX_UNIONS>
struct ShifterGateRuntime {
  // Precomputed segments
  GateSegPre segs[MAX_SEGS];
  int segCount = 0;

  // Lane subsets: indices into segs[]
  int segIndexV[MAX_LANES]; int vCount = 0;
  int segIndexH[MAX_LANES]; int hCount = 0;
  // Fast lookup: seg index -> lane list index
  int8_t segToLaneV[MAX_SEGS];
  int8_t segToLaneH[MAX_SEGS];

  // Detents and per-lane grouping
  DetentPre dets[MAX_DETS];
  int detCount = 0;

  int detByVlane[MAX_LANES][MAX_DETS_PER_LANE];
  uint8_t detByVlaneCount[MAX_LANES];

  int detByHlane[MAX_LANES][MAX_DETS_PER_LANE];
  uint8_t detByHlaneCount[MAX_LANES];

  // Global travel bounds (mm)
  float xMin=0, xMax=0, yMin=0, yMax=0;

  // ---- Lane hysteresis state (distance-based) ----
  struct LaneHold {
    int8_t laneSeg;     // index into segs[] of held lane segment, -1 if none
  };

  LaneHold holdX{ -1 };
  LaneHold holdY{ -1 };

  // ---- Tunables for hysteresis (mm) ----
  float membershipMargin_mm = 0.2f;   // widen corridor membership tests slightly
  float switchMargin_mm     = 0.6f;   // require new lane to be closer by this much
  float releaseExtra_mm     = 0.4f;   // release held lane only after you're this far outside its core

  // -----------------------------
  // Fast corridor membership
  // -----------------------------
  static inline bool inside(const GateSegPre& s, float x_mm, float y_mm, float margin_mm) {
    const float hw = s.hw + margin_mm;
    if (s.horizontal) {
      // |y - yLine| <= hw  AND  x in [a0_cap - margin, a1_cap + margin]
      return (absf(y_mm - s.line) <= hw) &&
             (x_mm >= (s.a0_cap - margin_mm)) &&
             (x_mm <= (s.a1_cap + margin_mm));
    } else {
      return (absf(x_mm - s.line) <= hw) &&
             (y_mm >= (s.a0_cap - margin_mm)) &&
             (y_mm <= (s.a1_cap + margin_mm));
    }
  }

  // Perpendicular distance to centerline (mm) (no sqrt)
  static inline float perpDist(const GateSegPre& s, float x_mm, float y_mm) {
    return s.horizontal ? absf(y_mm - s.line) : absf(x_mm - s.line);
  }

  // Interval candidate for one axis from one segment (cap-extended already)
  static inline Interval intervalFromSeg(const GateSegPre& s, AxisRole axis, float gMin, float gMax) {
    Interval iv;
    if (axis == AxisRole::X) {
      if (s.horizontal) iv = { s.a0_cap, s.a1_cap };
      else              iv = { s.line - s.hw, s.line + s.hw };
      iv.lo = (iv.lo < gMin) ? gMin : iv.lo;
      iv.hi = (iv.hi > gMax) ? gMax : iv.hi;
    } else { // AXIS_Y
      if (!s.horizontal) iv = { s.a0_cap, s.a1_cap };
      else               iv = { s.line - s.hw, s.line + s.hw };
      iv.lo = (iv.lo < gMin) ? gMin : iv.lo;
      iv.hi = (iv.hi > gMax) ? gMax : iv.hi;
    }
    return iv;
  }

  // Union insert (no sort). MAX_UNIONS should be small (e.g. 8).
  static inline int unionInsert(Interval* u, int uCount, Interval in) {
    if (in.lo > in.hi) return uCount;

    // Merge overlaps by repeated scan (uCount tiny)
    for (int i=0; i<uCount; ) {
      if (in.hi < u[i].lo || in.lo > u[i].hi) {
        i++;
        continue;
      }
      // merge & remove u[i]
      in.lo = (in.lo < u[i].lo) ? in.lo : u[i].lo;
      in.hi = (in.hi > u[i].hi) ? in.hi : u[i].hi;
      u[i] = u[uCount - 1];
      uCount--;
      i = 0; // restart scan
    }

    if (uCount < MAX_UNIONS) {
      u[uCount++] = in;
    } else {
      // If ever exceeded, conservatively expand the first interval (rare in H-gate)
      u[0].lo = (in.lo < u[0].lo) ? in.lo : u[0].lo;
      u[0].hi = (in.hi > u[0].hi) ? in.hi : u[0].hi;
    }
    return uCount;
  }

  static inline Interval pickUnionBest(const Interval* u, int uCount, float coord, Interval fallback) {
    if (uCount <= 0) return fallback;
    int best = 0;
    float bestD = 1e30f;
    for (int i=0;i<uCount;i++){
      float d = 0.0f;
      if (coord < u[i].lo) d = u[i].lo - coord;
      else if (coord > u[i].hi) d = coord - u[i].hi;
      else d = 0.0f;
      if (d < bestD) { bestD = d; best = i; }
    }
    return u[best];
  }

  // -----------------------------
  // Axis context result
  // -----------------------------
  struct AxisContext {
    Interval soft;
    int8_t laneSeg; // chosen/held lane segment index into segs[], -1 if none
  };

  // -----------------------------
  // Distance-based lane selection with hysteresis
  //
  // Rules:
  //  - Only consider segments that contain point.
  //  - Prefer orientation: for Y use vertical lanes only; for X use horizontal bands only
  //    (in H-gate overlap, this is what you want).
  //  - Hold the previous lane as long as you're not clearly closer to a different lane.
  //  - Switch when:
  //      newDist + switchMargin < heldDist   AND
  //      heldDist > (held.hw + releaseExtra)
  //
  // This avoids flipping at intersections while still allowing lane changes when you move over.
  // -----------------------------
  int8_t chooseLaneDistanceBased(float x_mm, float y_mm, AxisRole axis, uint32_t insideMask) {
    LaneHold& hold = (axis == AxisRole::X) ? holdX : holdY;

    // Build candidate set: only correct orientation
    const int* idxList = (axis == AxisRole::Y) ? segIndexV : segIndexH;
    const int  idxCount= (axis == AxisRole::Y) ? vCount    : hCount;

    int bestSeg = -1;
    float bestD = 1e30f;

    for (int li=0; li<idxCount; li++){
      int si = idxList[li];
      const GateSegPre& s = segs[si];
      if ((insideMask & (1u << si)) == 0u) continue;

      float d = perpDist(s, x_mm, y_mm);
      if (d < bestD) { bestD = d; bestSeg = si; }
    }

    // If no oriented segment contains point, drop hold.
    if (bestSeg < 0) {
      hold.laneSeg = -1;
      return -1;
    }

    // If no previous hold, take best.
    if (hold.laneSeg < 0) {
      hold.laneSeg = (int8_t)bestSeg;
      return hold.laneSeg;
    }

    // If held lane still contains point, compare distances
    const GateSegPre& heldS = segs[hold.laneSeg];
    const bool heldInside = (insideMask & (1u << hold.laneSeg)) != 0u;

    if (!heldInside) {
      // You left it: immediately adopt best
      hold.laneSeg = (int8_t)bestSeg;
      return hold.laneSeg;
    }

    float heldD = perpDist(heldS, x_mm, y_mm);
    const GateSegPre& bestS = segs[bestSeg];

    // Decide if switching is allowed (distance-based)
    const float heldRelease = heldS.hw + releaseExtra_mm;

    if ((bestD + switchMargin_mm < heldD) && (heldD > heldRelease)) {
      hold.laneSeg = (int8_t)bestSeg;
    }
    return hold.laneSeg;
  }

  // -----------------------------
  // 1) Determine current soft limits for one axis
  //
  // Optimization: we union intervals from ALL segments that contain point (both orientations),
  // because overlap defines allowable motion envelope.
  // But laneSeg comes from distance-based oriented selection above.
  // -----------------------------
  AxisContext updateAxisContext(float x_mm, float y_mm, AxisRole axis) {
    Interval unions[MAX_UNIONS];
    int uCount = 0;
    uint32_t insideMask = 0u;

    const float gMin = (axis == AxisRole::X) ? xMin : yMin;
    const float gMax = (axis == AxisRole::X) ? xMax : yMax;
    const float coord= (axis == AxisRole::X) ? x_mm : y_mm;

    // Gather soft limit candidates from all containing segments (both H and V)
    for (int i=0;i<segCount;i++){
      const GateSegPre& s = segs[i];
      const bool isInside = inside(s, x_mm, y_mm, membershipMargin_mm);
      if (isInside) insideMask |= (1u << i);
      if (!isInside) continue;

      Interval iv = intervalFromSeg(s, axis, gMin, gMax);
      uCount = unionInsert(unions, uCount, iv);
    }

    AxisContext out;
    out.soft = pickUnionBest(unions, uCount, coord, Interval{gMin, gMax});
    out.laneSeg = (int8_t)chooseLaneDistanceBased(x_mm, y_mm, axis, insideMask);
    return out;
  }

  // -----------------------------
  // 2) Reachable detents: detents assigned to the current lane
  // -----------------------------
  struct DetentSpan { const int* indices; uint8_t count; };

  // Find lane list index for seg index (O(1) via lookup table)
  int8_t findLaneListIndexForSeg(int segIndex, AxisRole axis) const {
    if (segIndex < 0 || segIndex >= segCount) return -1;
    return (axis == AxisRole::Y) ? segToLaneV[segIndex] : segToLaneH[segIndex];
  }

  DetentSpan detentsForLane(const AxisContext& ctx, AxisRole axis) const {
    DetentSpan s{nullptr, 0};
    if (ctx.laneSeg < 0) return s;

    int8_t li = findLaneListIndexForSeg(ctx.laneSeg, axis);
    if (li < 0) return s;

    if (axis == AxisRole::Y) {
      s.indices = detByVlane[li];
      s.count   = detByVlaneCount[li];
    } else {
      s.indices = detByHlane[li];
      s.count   = detByHlaneCount[li];
    }
    return s;
  }

  // -----------------------------
  // CONFIG CHANGE PRECOMPUTE
  // -----------------------------
  void clearDetentLists() {
    for (int i=0;i<MAX_LANES;i++){
      detByVlaneCount[i] = 0;
      detByHlaneCount[i] = 0;
    }
  }

  // Build from already-prepared arrays (no allocations)
  void buildPrecompute(const GateSegPre* inSegs, int inSegCount,
                       const DetentPre*  inDets, int inDetCount,
                       float xMin_mm, float xMax_mm,
                       float yMin_mm, float yMax_mm,
                       float detLaneTol_mm = 0.8f) {
    xMin = xMin_mm; xMax = xMax_mm;
    yMin = yMin_mm; yMax = yMax_mm;

    // Copy segs
    segCount = (inSegCount > MAX_SEGS) ? MAX_SEGS : inSegCount;
    vCount = hCount = 0;
    for (int i=0;i<MAX_SEGS;i++){
      segToLaneV[i] = -1;
      segToLaneH[i] = -1;
    }

    for (int i=0;i<segCount;i++){
      segs[i] = inSegs[i];
      if (segs[i].horizontal) {
        if (hCount < MAX_LANES) {
          segIndexH[hCount] = i;
          segToLaneH[i] = (int8_t)hCount;
          hCount++;
        }
      } else {
        if (vCount < MAX_LANES) {
          segIndexV[vCount] = i;
          segToLaneV[i] = (int8_t)vCount;
          vCount++;
        }
      }
    }

    // Copy detents
    detCount = (inDetCount > MAX_DETS) ? MAX_DETS : inDetCount;
    for (int i=0;i<detCount;i++){
      dets[i] = inDets[i];
      dets[i].laneV = -1;
      dets[i].laneH = -1;
    }

    clearDetentLists();

    // Assign detents to nearest vertical/horizontal lane (distance-based)
    for (int di=0; di<detCount; di++){
      // vertical lane assignment by x distance
      int bestV = -1;
      float bestDx = 1e30f;
      for (int li=0; li<vCount; li++){
        const GateSegPre& lane = segs[segIndexV[li]];
        float dx = absf(dets[di].x_mm - lane.line);
        if (dx < bestDx) { bestDx = dx; bestV = li; }
      }
      if (bestV >= 0 && bestDx <= detLaneTol_mm) {
        dets[di].laneV = (int8_t)bestV;
        uint8_t& cnt = detByVlaneCount[bestV];
        if (cnt < MAX_DETS_PER_LANE) detByVlane[bestV][cnt++] = di;
      }

      // horizontal lane assignment by y distance (optional)
      int bestH = -1;
      float bestDy = 1e30f;
      for (int li=0; li<hCount; li++){
        const GateSegPre& lane = segs[segIndexH[li]];
        float dy = absf(dets[di].y_mm - lane.line);
        if (dy < bestDy) { bestDy = dy; bestH = li; }
      }
      if (bestH >= 0 && bestDy <= detLaneTol_mm) {
        dets[di].laneH = (int8_t)bestH;
        uint8_t& cnt = detByHlaneCount[bestH];
        if (cnt < MAX_DETS_PER_LANE) detByHlane[bestH][cnt++] = di;
      }
    }

    // Reset lane holds on config change
    holdX.laneSeg = -1;
    holdY.laneSeg = -1;
  }
};

static inline float mm01_to_mm(int32_t v01) { return 0.1f * (float)v01; }

static inline GateSegPre makeSegPre(int32_t x0_01, int32_t y0_01,
                                    int32_t x1_01, int32_t y1_01,
                                    uint32_t halfW_01) {
  float x0 = mm01_to_mm(x0_01);
  float y0 = mm01_to_mm(y0_01);
  float x1 = mm01_to_mm(x1_01);
  float y1 = mm01_to_mm(y1_01);
  float hw = 0.1f * (float)halfW_01;

  GateSegPre s{};
  s.hw = hw;

  const bool horizontal = (std::fabs(y1 - y0) < 1e-6f);
  s.horizontal = horizontal;

  if (horizontal) {
    s.line = y0;
    float xmin = x0, xmax = x1; order2(xmin, xmax);
    s.a0_cap = xmin - hw;
    s.a1_cap = xmax + hw;
  } else {
    s.line = x0;
    float ymin = y0, ymax = y1; order2(ymin, ymax);
    s.a0_cap = ymin - hw;
    s.a1_cap = ymax + hw;
  }
  return s;
}

static inline DetentPre makeDetPre(int32_t x_01, int32_t y_01,
                                   uint32_t r_01, float spring_N_per_mm) {
  DetentPre d{};
  d.x_mm = mm01_to_mm(x_01);
  d.y_mm = mm01_to_mm(y_01);
  d.radius_mm = 0.1f * (float)r_01;
  d.spring_N_per_mm = spring_N_per_mm;
  d.laneV = d.laneH = -1;
  return d;
}

struct CenterPointMm {
  float x_mm;
  float y_mm;
  bool valid;
};

// Pick the "neutral horizontal gate" as the horizontal segment with the largest along span.
// This works well for your config (the long y=30mm bar).
static inline int findBestHorizontalGate(const GateSegPre* segs, int segCount) {
  int best = -1;
  float bestSpan = -1.0f;

  for (int i = 0; i < segCount; i++) {
    const GateSegPre& s = segs[i];
    if (!s.horizontal) continue;

    float span = s.a1_cap - s.a0_cap; // already cap-extended, but fine for ranking
    if (span > bestSpan) {
      bestSpan = span;
      best = i;
    }
  }
  return best;
}

static inline CenterPointMm computeCenteringAnchorMm(
    const ShifterDetectConfig& detect,
    const GateSegPre* segs, int segCount
) {
  CenterPointMm out{0.0f, 0.0f, false};

  // Look for neutral slot first (authoritative for BOTH x and y)
  for (int i = 0; i < detect.gear_slots_count; i++) {
    const ShifterGearSlot& s = detect.gear_slots[i];
    if (s.gear == ShifterGear_SHIFTER_GEAR_NEUTRAL) {
      out.x_mm = mm01_to_mm(s.center_x);
      out.y_mm = mm01_to_mm(s.center_y);
      out.valid = true;
      return out;
    }
  }

  // Neutral not defined -> need Gear 3 for X and best horizontal gate for Y
  bool haveGear3 = false;
  float x3 = 0.0f;

  for (int i = 0; i < detect.gear_slots_count; i++) {
    const ShifterGearSlot& s = detect.gear_slots[i];
    if (s.gear == ShifterGear_SHIFTER_GEAR_3) {
      haveGear3 = true;
      x3 = mm01_to_mm(s.center_x);
      break;
    }
  }

  if (!haveGear3) {
    // No fallback gears by requirement
    return out; // valid=false
  }

  int hIdx = findBestHorizontalGate(segs, segCount);
  if (hIdx < 0) {
    return out; // valid=false
  }

  out.x_mm = x3;
  out.y_mm = segs[hIdx].line; // y centerline (mm)
  out.valid = true;
  return out;
}

class ShifterFunction : public IFunction {
    public:
        ShifterFunction(void);
        void update_config(const ShifterConfig &config, const ShifterDetectConfig &detect_config, CommManager &comm_manager, const AxisID *linked_axes);
        float get_x_contact_point_min(void) override;
        float get_x_contact_point_max(void) override;
        void on_ffb_action(const FFBAction &ffb_action) override;
        void update(const SimState &state, SimAccumulators &accum) override;

    private:
        void rebuild_map(void);
        AxisRole resolve_axis_role(const AxisID *linked_axes);

        CommManager *_comm_manager = nullptr;
        AxisRole _axis_role = AxisRole::Unknown;
        AxisID _axis_id_x = AxisID_AXIS_UNDEFINED;
        AxisID _axis_id_y = AxisID_AXIS_UNDEFINED;
        bool _invert_x = false;
        bool _invert_y = false;
        float _x_min = 0.0f;
        float _x_max = 0.0f;
        ShifterGateRuntime<12, 16, 8, 8, 8> gateRt;

        Damper _damper = Damper(1.0f);
        Spring _centering_spring = Spring(0.0f, 5.0f);
        // ForceMap _detents = ForceMap({-20.0f, -18.0, -16.0f, 16.0f, 18.0f, 20.0f}, {0.0f, 50.0f, 0.0f, 0.0f, -50.0f, 0.0f});
};
