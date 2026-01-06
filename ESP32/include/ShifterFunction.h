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

struct Interval {
        float lo, hi;
};

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

        int8_t laneV;  // assigned vertical lane index (0..vCount-1) or -1
        int8_t laneH;  // assigned horizontal lane index (0..hCount-1) or -1
};

static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}
static inline float absf(float v) {
    return (v >= 0.0f) ? v : -v;
}
static inline void order2(float& a, float& b) {
    if (a > b) {
        float t = a;
        a = b;
        b = t;
    }
}

template <int MAX_SEGS, int MAX_DETS, int MAX_LANES, int MAX_DETS_PER_LANE, int MAX_UNIONS>
struct ShifterGateRuntime {
        // Precomputed segments
        GateSegPre segs[MAX_SEGS];
        int segCount = 0;

        // Lane subsets: indices into segs[]
        int segIndexV[MAX_LANES];
        int vCount = 0;
        int segIndexH[MAX_LANES];
        int hCount = 0;

        // Detents and per-lane grouping
        DetentPre dets[MAX_DETS];
        int detCount = 0;

        int detByVlane[MAX_LANES][MAX_DETS_PER_LANE];
        uint8_t detByVlaneCount[MAX_LANES];

        int detByHlane[MAX_LANES][MAX_DETS_PER_LANE];
        uint8_t detByHlaneCount[MAX_LANES];

        // Global travel bounds (mm)
        float xMin = 0, xMax = 0, yMin = 0, yMax = 0;

        // ---- Lane hysteresis state (distance-based) ----
        struct LaneHold {
                int8_t laneSeg;  // index into segs[] of held lane segment, -1 if none
        };

        LaneHold holdX{-1};
        LaneHold holdY{-1};

        // Sticky "last known inside" segments. Updated only when the current
        // (x,y) is inside a corridor. If membership is temporarily empty
        // (common when other-axis position is slightly stale over CAN), we keep
        // using these to avoid dropping into a "void".
        int8_t lastInsideHSeg = -1;
        int8_t lastInsideVSeg = -1;

        // ---- Tunables for hysteresis (mm) ----
        float membershipMargin_mm = 0.0f;  // widen corridor membership tests slightly
        float switchMargin_mm = 0.0f;      // require new lane to be closer by this much
        float releaseExtra_mm = 0.0f;      // release held lane only after you're this far outside its core

        // -----------------------------
        // Fast corridor membership
        // -----------------------------
        static inline bool inside(const GateSegPre& s, float x_mm, float y_mm, float margin_mm) {
            const float hw = s.hw + margin_mm;
            if (s.horizontal) {
                // |y - yLine| <= hw  AND  x in [a0_cap - margin, a1_cap + margin]
                return (absf(y_mm - s.line) <= hw) && (x_mm >= (s.a0_cap - margin_mm)) && (x_mm <= (s.a1_cap + margin_mm));
            } else {
                return (absf(x_mm - s.line) <= hw) && (y_mm >= (s.a0_cap - margin_mm)) && (y_mm <= (s.a1_cap + margin_mm));
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
                if (s.horizontal)
                    iv = {s.a0_cap, s.a1_cap};
                else
                    iv = {s.line - s.hw, s.line + s.hw};
                iv.lo = (iv.lo < gMin) ? gMin : iv.lo;
                iv.hi = (iv.hi > gMax) ? gMax : iv.hi;
            } else {  // AXIS_Y
                if (!s.horizontal)
                    iv = {s.a0_cap, s.a1_cap};
                else
                    iv = {s.line - s.hw, s.line + s.hw};
                iv.lo = (iv.lo < gMin) ? gMin : iv.lo;
                iv.hi = (iv.hi > gMax) ? gMax : iv.hi;
            }
            return iv;
        }

        // Union insert (no sort). MAX_UNIONS should be small (e.g. 8).
        static inline int unionInsert(Interval* u, int uCount, Interval in) {
            if (in.lo > in.hi) return uCount;

            // Merge overlaps by repeated scan (uCount tiny)
            for (int i = 0; i < uCount;) {
                if (in.hi < u[i].lo || in.lo > u[i].hi) {
                    i++;
                    continue;
                }
                // merge & remove u[i]
                in.lo = (in.lo < u[i].lo) ? in.lo : u[i].lo;
                in.hi = (in.hi > u[i].hi) ? in.hi : u[i].hi;
                u[i] = u[uCount - 1];
                uCount--;
                i = 0;  // restart scan
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
            for (int i = 0; i < uCount; i++) {
                float d = 0.0f;
                if (coord < u[i].lo)
                    d = u[i].lo - coord;
                else if (coord > u[i].hi)
                    d = coord - u[i].hi;
                else
                    d = 0.0f;
                if (d < bestD) {
                    bestD = d;
                    best = i;
                }
            }
            return u[best];
        }

        // -----------------------------
        // Axis context result
        // -----------------------------
        struct AxisContext {
                Interval soft;
                int8_t laneSeg;  // chosen/held lane segment index into segs[], -1 if none
        };

        // Choose nearest lane by distance to centerline, with hysteresis.
        // DOES NOT require being inside the lane corridor in X.
        int8_t chooseVerticalLaneByDistance(float x_mm, float y_mm) {
            // candidates are vertical lanes only (precomputed list segIndexV[])
            int bestSeg = -1;
            float bestD = 1e30f;

            // Find nearest vertical lane centerline among lanes that span current y.
            for (int li = 0; li < vCount; li++) {
                int si = segIndexV[li];
                const GateSegPre& s = segs[si];  // vertical lane => horizontal==false

                // Require y within the lane's along-range (cap-extended) with a small margin.
                if (y_mm < (s.a0_cap - membershipMargin_mm) || y_mm > (s.a1_cap + membershipMargin_mm)) continue;

                float d = absf(x_mm - s.line);  // distance to lane centerline in mm
                if (d < bestD) {
                    bestD = d;
                    bestSeg = si;
                }
            }

            // If no lane spans this y, keep the previous lane if we had one.
            // This avoids transient "no lane" states when the other axis
            // position is slightly stale.
            if (bestSeg < 0) {
                return holdY.laneSeg;
            }

            // If no hold yet, take nearest.
            if (holdY.laneSeg < 0) {
                holdY.laneSeg = (int8_t)bestSeg;
                return holdY.laneSeg;
            }

            // Hysteresis: only switch if new lane is clearly closer than held lane.
            const GateSegPre& heldS = segs[holdY.laneSeg];
            float heldD = absf(x_mm - heldS.line);

            if (bestSeg != holdY.laneSeg) {
                if (bestD + switchMargin_mm < heldD) {
                    holdY.laneSeg = (int8_t)bestSeg;
                }
            }

            return holdY.laneSeg;
        }

        int8_t chooseHorizontalLaneByDistance(float x_mm, float y_mm) {
            int bestSeg = -1;
            float bestD = 1e30f;

            for (int li = 0; li < hCount; li++) {
                int si = segIndexH[li];
                const GateSegPre& s = segs[si];  // horizontal lane => horizontal==true

                // Require x within along-range for that horizontal band.
                if (x_mm < (s.a0_cap - membershipMargin_mm) || x_mm > (s.a1_cap + membershipMargin_mm)) continue;

                float d = absf(y_mm - s.line);
                if (d < bestD) {
                    bestD = d;
                    bestSeg = si;
                }
            }

            if (bestSeg < 0) {
                return holdX.laneSeg;
            }
            if (holdX.laneSeg < 0) {
                holdX.laneSeg = (int8_t)bestSeg;
                return holdX.laneSeg;
            }

            const GateSegPre& heldS = segs[holdX.laneSeg];
            float heldD = absf(y_mm - heldS.line);

            if (bestSeg != holdX.laneSeg) {
                if (bestD + switchMargin_mm < heldD) holdX.laneSeg = (int8_t)bestSeg;
            }
            return holdX.laneSeg;
        }

        int8_t chooseLaneDistanceBased(float x_mm, float y_mm, AxisRole axis) {
            if (axis == AxisRole::Y) {
                return chooseVerticalLaneByDistance(x_mm, y_mm);
            }
            // For AXIS_X you can do the symmetric version (nearest horizontal lane by |y - yLine|)
            // or keep your old "inside-corridor" logic if you prefer.
            // Recommended symmetry:
            return chooseHorizontalLaneByDistance(x_mm, y_mm);
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

            const float gMin = (axis == AxisRole::X) ? xMin : yMin;
            const float gMax = (axis == AxisRole::X) ? xMax : yMax;
            const float coord = (axis == AxisRole::X) ? x_mm : y_mm;

            // Gather soft limit candidates from all containing segments (both H and V).
            // Also track the "best" containing horizontal and vertical segment so we can
            // update sticky membership and fall back if membership becomes empty.
            int bestHSeg = -1;
            int bestVSeg = -1;
            float bestHd = 1e30f;
            float bestVd = 1e30f;

            for (int i = 0; i < segCount; i++) {
                const GateSegPre& s = segs[i];
                if (!inside(s, x_mm, y_mm, membershipMargin_mm)) continue;

                Interval iv = intervalFromSeg(s, axis, gMin, gMax);
                uCount = unionInsert(unions, uCount, iv);

                const float d = perpDist(s, x_mm, y_mm);
                if (s.horizontal) {
                    if (d < bestHd) {
                        bestHd = d;
                        bestHSeg = i;
                    }
                } else {
                    if (d < bestVd) {
                        bestVd = d;
                        bestVSeg = i;
                    }
                }
            }

            // Update sticky membership on positive evidence.
            if (bestHSeg >= 0) lastInsideHSeg = (int8_t)bestHSeg;
            if (bestVSeg >= 0) lastInsideVSeg = (int8_t)bestVSeg;

            // If we are temporarily "in the void" (no containing corridors), fall back to the
            // last known inside corridors instead of widening to global bounds.
            if (uCount == 0) {
                if (lastInsideHSeg >= 0) {
                    Interval iv = intervalFromSeg(segs[(int)lastInsideHSeg], axis, gMin, gMax);
                    uCount = unionInsert(unions, uCount, iv);
                }
                if (lastInsideVSeg >= 0) {
                    Interval iv = intervalFromSeg(segs[(int)lastInsideVSeg], axis, gMin, gMax);
                    uCount = unionInsert(unions, uCount, iv);
                }
            }

            AxisContext out;
            out.soft = pickUnionBest(unions, uCount, coord, Interval{gMin, gMax});
            out.laneSeg = (int8_t)chooseLaneDistanceBased(x_mm, y_mm, axis);
            return out;
        }

        // -----------------------------
        // 2) Reachable detents: detents assigned to the current lane
        // -----------------------------
        struct DetentSpan {
                const int* indices;
                uint8_t count;
        };

        // Find lane list index for seg index (linear scan; lane count small)
        int8_t findLaneListIndexForSeg(int segIndex, AxisRole axis) const {
            if (axis == AxisRole::Y) {
                for (int li = 0; li < vCount; li++)
                    if (segIndexV[li] == segIndex) return (int8_t)li;
            } else {
                for (int li = 0; li < hCount; li++)
                    if (segIndexH[li] == segIndex) return (int8_t)li;
            }
            return -1;
        }

        DetentSpan detentsForLane(const AxisContext& ctx, AxisRole axis) const {
            DetentSpan s{nullptr, 0};
            if (ctx.laneSeg < 0) return s;

            int8_t li = findLaneListIndexForSeg(ctx.laneSeg, axis);
            if (li < 0) return s;

            if (axis == AxisRole::Y) {
                s.indices = detByVlane[li];
                s.count = detByVlaneCount[li];
            } else {
                s.indices = detByHlane[li];
                s.count = detByHlaneCount[li];
            }
            return s;
        }

        // -----------------------------
        // CONFIG CHANGE PRECOMPUTE
        // -----------------------------
        void clearDetentLists() {
            for (int i = 0; i < MAX_LANES; i++) {
                detByVlaneCount[i] = 0;
                detByHlaneCount[i] = 0;
            }
        }

        // Build from already-prepared arrays (no allocations)
        void buildPrecompute(const GateSegPre* inSegs, int inSegCount, const DetentPre* inDets, int inDetCount, float xMin_mm, float xMax_mm,
                             float yMin_mm, float yMax_mm, float detLaneTol_mm = 0.8f) {
            xMin = xMin_mm;
            xMax = xMax_mm;
            yMin = yMin_mm;
            yMax = yMax_mm;

            // Copy segs
            segCount = (inSegCount > MAX_SEGS) ? MAX_SEGS : inSegCount;
            vCount = hCount = 0;

            for (int i = 0; i < segCount; i++) {
                segs[i] = inSegs[i];
                if (segs[i].horizontal) {
                    if (hCount < MAX_LANES) segIndexH[hCount++] = i;
                } else {
                    if (vCount < MAX_LANES) segIndexV[vCount++] = i;
                }
            }

            // Copy detents
            detCount = (inDetCount > MAX_DETS) ? MAX_DETS : inDetCount;
            for (int i = 0; i < detCount; i++) {
                dets[i] = inDets[i];
                dets[i].laneV = -1;
                dets[i].laneH = -1;
            }

            clearDetentLists();

            // Assign detents to nearest vertical/horizontal lane (distance-based)
            for (int di = 0; di < detCount; di++) {
                // vertical lane assignment by x distance
                int bestV = -1;
                float bestDx = 1e30f;
                for (int li = 0; li < vCount; li++) {
                    const GateSegPre& lane = segs[segIndexV[li]];
                    float dx = absf(dets[di].x_mm - lane.line);
                    if (dx < bestDx) {
                        bestDx = dx;
                        bestV = li;
                    }
                }
                if (bestV >= 0 && bestDx <= detLaneTol_mm) {
                    dets[di].laneV = (int8_t)bestV;
                    uint8_t& cnt = detByVlaneCount[bestV];
                    if (cnt < MAX_DETS_PER_LANE) detByVlane[bestV][cnt++] = di;
                }

                // horizontal lane assignment by y distance (optional)
                int bestH = -1;
                float bestDy = 1e30f;
                for (int li = 0; li < hCount; li++) {
                    const GateSegPre& lane = segs[segIndexH[li]];
                    float dy = absf(dets[di].y_mm - lane.line);
                    if (dy < bestDy) {
                        bestDy = dy;
                        bestH = li;
                    }
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

            // Reset sticky "last inside" segments
            lastInsideHSeg = -1;
            lastInsideVSeg = -1;
        }
};

static inline float mm01_to_mm(int32_t v01) {
    return 0.1f * (float)v01;
}

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
// This works well for your config (the long y=30mm bar).
static inline int findBestHorizontalGate(const GateSegPre* segs, int segCount) {
    int best = -1;
    float bestSpan = -1.0f;

    for (int i = 0; i < segCount; i++) {
        const GateSegPre& s = segs[i];
        if (!s.horizontal) continue;

        float span = s.a1_cap - s.a0_cap;  // already cap-extended, but fine for ranking
        if (span > bestSpan) {
            bestSpan = span;
            best = i;
        }
    }
    return best;
}

static inline CenterPointMm computeCenteringAnchorMm(const ShifterDetectConfig& detect, const GateSegPre* segs, int segCount) {
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
        return out;  // valid=false
    }

    int hIdx = findBestHorizontalGate(segs, segCount);
    if (hIdx < 0) {
        return out;  // valid=false
    }

    out.x_mm = x3;
    out.y_mm = segs[hIdx].line;  // y centerline (mm)
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
        ShifterGateRuntime<12, 16, 8, 8, 8> gateRt;

        Damper _damper = Damper(1.0f);
        Spring _centering_spring = Spring(0.0f, 5.0f);
        // ForceMap _detents = ForceMap({-20.0f, -18.0, -16.0f, 16.0f, 18.0f, 20.0f}, {0.0f, 50.0f, 0.0f, 0.0f, -50.0f, 0.0f});
};
