#pragma once

#include <cmath>
#include <cstdint>

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

struct Interval {
    float lo, hi;
};

struct GateSegPre {
    bool  horizontal;
    float hw;      // half width in mm
    float line;    // horizontal: y line, vertical: x line
    float a0_cap;  // along-range min with endcaps expanded by hw
    float a1_cap;  // along-range max with endcaps expanded by hw
};

struct DetentPre {
    float x_mm;
    float y_mm;
    float radius_mm;
    float spring_N_per_mm;

    int8_t laneV;  // assigned vertical lane index (0..vCount-1) or -1
    int8_t laneH;  // assigned horizontal lane index (0..hCount-1) or -1
};

static inline float clampf(float v, float lo, float hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }
static inline float absf(float v) { return (v >= 0.0f) ? v : -v; }
static inline void  order2(float& a, float& b) { if (a > b) { float t=a; a=b; b=t; } }

// Gate runtime implementing the "1-2 active lanes" model.
// At any position, we keep at most one active horizontal segment and one active vertical segment.
// Soft limits are the union of the interval contributions from those up to two segments.
//
// No STL / no heap usage; intended for fast periodic calls.
struct ShifterGateRuntime {
    static const int MAX_SEGS  = 12;
    static const int MAX_DETS  = 16;
    static const int MAX_LANES = 8;
    static const int MAX_DETS_PER_LANE = 8;

    // Precomputed segments
    GateSegPre segs[MAX_SEGS];
    int segCount = 0;

    // Lane subsets: indices into segs[]
    int segIndexV[MAX_LANES];
    int vCount = 0;
    int segIndexH[MAX_LANES];
    int hCount = 0;

    // Map seg index -> lane id (O(1) detent lookup)
    int8_t segToVlaneId[MAX_SEGS];
    int8_t segToHlaneId[MAX_SEGS];

    // Detents and per-lane grouping
    DetentPre dets[MAX_DETS];
    int detCount = 0;

    int detByVlane[MAX_LANES][MAX_DETS_PER_LANE];
    uint8_t detByVlaneCount[MAX_LANES];

    int detByHlane[MAX_LANES][MAX_DETS_PER_LANE];
    uint8_t detByHlaneCount[MAX_LANES];

    // Global travel bounds (mm)
    float xMin=0, xMax=0, yMin=0, yMax=0;

    // Small widening of corridor membership to avoid chatter on borders (mm)
    float membershipMargin_mm = 0.15f;

    struct ActiveLanes {
        int8_t hSeg = -1;  // index into segs[]
        int8_t vSeg = -1;  // index into segs[]
        int8_t hLaneId = -1; // 0..hCount-1
        int8_t vLaneId = -1; // 0..vCount-1
    };

    struct AxisContext {
        Interval soft;
        ActiveLanes lanes;
    };

    struct DetentSpan {
        const int* indices;
        uint8_t count;
    };

    static inline float perpDist(const GateSegPre& s, float x_mm, float y_mm) {
        return s.horizontal ? absf(y_mm - s.line) : absf(x_mm - s.line);
    }

    static inline bool insideCorridor(const GateSegPre& s, float x_mm, float y_mm, float margin_mm) {
        const float hw = s.hw + margin_mm;
        if (s.horizontal) {
            return (absf(y_mm - s.line) <= hw) && (x_mm >= (s.a0_cap - margin_mm)) && (x_mm <= (s.a1_cap + margin_mm));
        } else {
            return (absf(x_mm - s.line) <= hw) && (y_mm >= (s.a0_cap - margin_mm)) && (y_mm <= (s.a1_cap + margin_mm));
        }
    }

    static inline Interval intervalFromSeg(const GateSegPre& s, AxisRole axis, float gMin, float gMax) {
        Interval iv;
        if (axis == AxisRole::X) {
            if (s.horizontal) iv = {s.a0_cap, s.a1_cap};
            else              iv = {s.line - s.hw, s.line + s.hw};
        } else { // AxisRole::Y
            if (!s.horizontal) iv = {s.a0_cap, s.a1_cap};
            else               iv = {s.line - s.hw, s.line + s.hw};
        }
        if (iv.lo < gMin) iv.lo = gMin;
        if (iv.hi > gMax) iv.hi = gMax;
        return iv;
    }

    static inline bool overlaps(const Interval& a, const Interval& b) {
        return !(b.hi < a.lo || b.lo > a.hi);
    }

    static inline Interval merge2(const Interval& a, const Interval& b) {
        Interval o;
        o.lo = (a.lo < b.lo) ? a.lo : b.lo;
        o.hi = (a.hi > b.hi) ? a.hi : b.hi;
        return o;
    }

    // Pick active horizontal + vertical corridors containing the point.
    // If multiple contain, pick the closest to the centerline (perpendicular distance).
    ActiveLanes findActiveLanes(float x_mm, float y_mm) const {
        ActiveLanes out;

        float bestHd = 1e30f;
        float bestVd = 1e30f;

        for (int i=0; i<segCount; i++) {
            const GateSegPre& s = segs[i];
            if (!insideCorridor(s, x_mm, y_mm, membershipMargin_mm)) continue;

            const float d = perpDist(s, x_mm, y_mm);
            if (s.horizontal) {
                if (d < bestHd) { bestHd = d; out.hSeg = (int8_t)i; }
            } else {
                if (d < bestVd) { bestVd = d; out.vSeg = (int8_t)i; }
            }
        }

        if (out.hSeg >= 0) out.hLaneId = segToHlaneId[(int)out.hSeg];
        if (out.vSeg >= 0) out.vLaneId = segToVlaneId[(int)out.vSeg];
        return out;
    }

    // Compute soft limits as union of up to two intervals (active H and/or active V).
    AxisContext updateAxisContext(float x_mm, float y_mm, AxisRole axis) const {
        AxisContext ctx;
        ctx.lanes = findActiveLanes(x_mm, y_mm);

        const float gMin = (axis == AxisRole::X) ? xMin : yMin;
        const float gMax = (axis == AxisRole::X) ? xMax : yMax;
        const float coord = (axis == AxisRole::X) ? x_mm : y_mm;

        bool have = false;
        Interval a{gMin, gMax}, b{gMin, gMax};
        int count = 0;

        if (ctx.lanes.hSeg >= 0) {
            a = intervalFromSeg(segs[(int)ctx.lanes.hSeg], axis, gMin, gMax);
            have = true;
            count = 1;
        }
        if (ctx.lanes.vSeg >= 0) {
            if (!have) {
                a = intervalFromSeg(segs[(int)ctx.lanes.vSeg], axis, gMin, gMax);
                have = true;
                count = 1;
            } else {
                b = intervalFromSeg(segs[(int)ctx.lanes.vSeg], axis, gMin, gMax);
                count = 2;
            }
        }

        if (!have) {
            ctx.soft = {gMin, gMax};
            return ctx;
        }

        if (count == 1) {
            ctx.soft = a;
            return ctx;
        }

        // count == 2 -> union of two intervals (may be 1 or 2 disjoint pieces).
        if (overlaps(a, b)) {
            ctx.soft = merge2(a, b);
            return ctx;
        }

        // Disjoint: pick the interval that contains coord; else nearest.
        const bool inA = (coord >= a.lo && coord <= a.hi);
        const bool inB = (coord >= b.lo && coord <= b.hi);
        if (inA) { ctx.soft = a; return ctx; }
        if (inB) { ctx.soft = b; return ctx; }

        float dA = (coord < a.lo) ? (a.lo - coord) : (coord - a.hi);
        float dB = (coord < b.lo) ? (b.lo - coord) : (coord - b.hi);
        ctx.soft = (dA <= dB) ? a : b;
        return ctx;
    }

    DetentSpan detentsForAxis(const AxisContext& ctx, AxisRole axis) const {
        DetentSpan s{nullptr, 0};

        if (axis == AxisRole::Y) {
            const int8_t lane = ctx.lanes.vLaneId;
            if (lane < 0) return s;
            s.indices = detByVlane[lane];
            s.count   = detByVlaneCount[lane];
            return s;
        } else {
            const int8_t lane = ctx.lanes.hLaneId;
            if (lane < 0) return s;
            s.indices = detByHlane[lane];
            s.count   = detByHlaneCount[lane];
            return s;
        }
    }

    void clearDetentLists() {
        for (int i=0;i<MAX_LANES;i++) {
            detByVlaneCount[i] = 0;
            detByHlaneCount[i] = 0;
        }
    }

    void buildPrecompute(const GateSegPre* inSegs, int inSegCount,
                         const DetentPre* inDets, int inDetCount,
                         float xMin_mm, float xMax_mm,
                         float yMin_mm, float yMax_mm,
                         float detLaneTol_mm = 0.8f) {
        xMin = xMin_mm; xMax = xMax_mm;
        yMin = yMin_mm; yMax = yMax_mm;

        segCount = (inSegCount > MAX_SEGS) ? MAX_SEGS : inSegCount;
        vCount = hCount = 0;

        for (int i=0;i<MAX_SEGS;i++) {
            segToVlaneId[i] = -1;
            segToHlaneId[i] = -1;
        }

        for (int i=0;i<segCount;i++) {
            segs[i] = inSegs[i];
            if (segs[i].horizontal) {
                if (hCount < MAX_LANES) {
                    segIndexH[hCount] = i;
                    segToHlaneId[i] = (int8_t)hCount;
                    hCount++;
                }
            } else {
                if (vCount < MAX_LANES) {
                    segIndexV[vCount] = i;
                    segToVlaneId[i] = (int8_t)vCount;
                    vCount++;
                }
            }
        }

        detCount = (inDetCount > MAX_DETS) ? MAX_DETS : inDetCount;
        for (int i=0;i<detCount;i++) {
            dets[i] = inDets[i];
            dets[i].laneV = -1;
            dets[i].laneH = -1;
        }

        clearDetentLists();

        // Assign detents to nearest vertical/horizontal lane centerlines.
        for (int di=0; di<detCount; di++) {
            // vertical assignment by x distance
            int bestV = -1;
            float bestDx = 1e30f;
            for (int li=0; li<vCount; li++) {
                const GateSegPre& lane = segs[segIndexV[li]];
                float dx = absf(dets[di].x_mm - lane.line);
                if (dx < bestDx) { bestDx = dx; bestV = li; }
            }
            if (bestV >= 0 && bestDx <= detLaneTol_mm) {
                dets[di].laneV = (int8_t)bestV;
                uint8_t& cnt = detByVlaneCount[bestV];
                if (cnt < MAX_DETS_PER_LANE) detByVlane[bestV][cnt++] = di;
            }

            // horizontal assignment by y distance (optional/useful for X-axis if you ever want it)
            int bestH = -1;
            float bestDy = 1e30f;
            for (int li=0; li<hCount; li++) {
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
    }
};

static inline float mm01_to_mm(int32_t v01) { return 0.1f * (float)v01; }

static inline GateSegPre makeSegPre(int32_t x0_01, int32_t y0_01, int32_t x1_01, int32_t y1_01, uint32_t halfW_01) {
    float x0 = mm01_to_mm(x0_01);
    float y0 = mm01_to_mm(y0_01);
    float x1 = mm01_to_mm(x1_01);
    float y1 = mm01_to_mm(y1_01);
    float hw = mm01_to_mm(halfW_01);

    GateSegPre s{};
    s.hw = hw;

    const bool horizontal = (std::fabs(y1 - y0) < 1e-6f);
    s.horizontal = horizontal;

    if (horizontal) {
        s.line = y0;
        float xmin = x0, xmax = x1;
        order2(xmin, xmax);
        s.a0_cap = xmin - hw;
        s.a1_cap = xmax + hw;
    } else {
        s.line = x0;
        float ymin = y0, ymax = y1;
        order2(ymin, ymax);
        s.a0_cap = ymin - hw;
        s.a1_cap = ymax + hw;
    }
    return s;
}

static inline DetentPre makeDetPre(int32_t x_01, int32_t y_01, uint32_t r_01, float spring_N_per_mm) {
    DetentPre d{};
    d.x_mm = mm01_to_mm(x_01);
    d.y_mm = mm01_to_mm(y_01);
    d.radius_mm = mm01_to_mm(r_01);
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
static inline int findBestHorizontalGate(const GateSegPre* segs, int segCount) {
    int best = -1;
    float bestSpan = -1.0f;

    for (int i = 0; i < segCount; i++) {
        const GateSegPre& s = segs[i];
        if (!s.horizontal) continue;
        float span = s.a1_cap - s.a0_cap;
        if (span > bestSpan) {
            bestSpan = span;
            best = i;
        }
    }
    return best;
}

// If a NEUTRAL slot exists: use its (center_x, center_y) directly.
// If NEUTRAL is missing: x comes from GEAR_3 slot, y comes from best horizontal gate.
static inline CenterPointMm computeCenteringAnchorMm(const ShifterDetectConfig& detect, const GateSegPre* segs, int segCount) {
    CenterPointMm out{0.0f, 0.0f, false};

    for (int i = 0; i < detect.gear_slots_count; i++) {
        const ShifterGearSlot& s = detect.gear_slots[i];
        if (s.gear == ShifterGear_SHIFTER_GEAR_NEUTRAL) {
            out.x_mm = mm01_to_mm(s.center_x);
            out.y_mm = mm01_to_mm(s.center_y);
            out.valid = true;
            return out;
        }
    }

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
    if (!haveGear3) return out;

    int hIdx = findBestHorizontalGate(segs, segCount);
    if (hIdx < 0) return out;

    out.x_mm = x3;
    out.y_mm = segs[hIdx].line;
    out.valid = true;
    return out;
}

class ShifterFunction : public IFunction {
public:
    ShifterFunction(void);
    void update_config(const ShifterConfig& config, const ShifterDetectConfig& detect_config, CommManager& comm_manager,
                       const AxisID* linked_axes);
    float get_x_contact_point_min(void) override;
    float get_x_contact_point_max(void) override;
    void on_ffb_action(const FFBAction& ffb_action) override;
    void update(Sim* sim, float& f_sum) override;

private:
    void rebuild_map(void);
    AxisRole resolve_axis_role(const AxisID* linked_axes);

    CommManager* _comm_manager = nullptr;
    AxisRole _axis_role = AxisRole::Unknown;
    AxisID _axis_id_x = AxisID_AXIS_UNDEFINED;
    AxisID _axis_id_y = AxisID_AXIS_UNDEFINED;
    bool _invert_x = false;
    bool _invert_y = false;
    float _x_min = 0.0f;
    float _x_max = 0.0f;

    ShifterGateRuntime gateRt;

    Damper _damper = Damper(1.0f);
    Spring _centering_spring = Spring(0.0f, 5.0f);
};
