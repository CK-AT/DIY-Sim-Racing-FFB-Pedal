#include <unity.h>

#include "TopologyCache.h"

namespace {

constexpr size_t N = 4;

void run_compute(
    AxisID own,
    const AxisID *linked,
    PositionMode &mode,
    AxisID &primary,
    ForceFetchEntry *fetch,
    uint8_t &count
) {
    compute_topology(own, linked, N, mode, primary, fetch, count);
}

}  // namespace

void setUp(void) {
}

void tearDown(void) {
}

// Independent: own entry carries AXIS_INDEPENDENT.
// Expected: USE_OWN, not subtractive, fetch list contains only own with +1.
void test_topology_independent(void) {
    const AxisID own = AxisID_AXIS_ID_1;
    AxisID linked[N] = {
        AxisID(AxisID_AXIS_ID_1 | AxisID_AXIS_INDEPENDENT),
        AxisID_AXIS_UNDEFINED,
        AxisID_AXIS_UNDEFINED,
        AxisID_AXIS_UNDEFINED,
    };
    PositionMode mode;
    AxisID primary;
    ForceFetchEntry fetch[TOPOLOGY_MAX_FETCH];
    uint8_t count;
    run_compute(own, linked, mode, primary, fetch, count);

    TEST_ASSERT_EQUAL(POSITION_MODE_USE_OWN, mode);
    TEST_ASSERT_EQUAL(1, count);
    TEST_ASSERT_EQUAL(AxisID_AXIS_ID_1, fetch[0].axis_id);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, fetch[0].sign);
}

// Independent with non-own entries also present in the list. The independent
// own axis must ignore non-own entries entirely.
void test_topology_independent_filters_others(void) {
    const AxisID own = AxisID_AXIS_ID_1;
    AxisID linked[N] = {
        AxisID_AXIS_ID_2,
        AxisID(AxisID_AXIS_ID_1 | AxisID_AXIS_INDEPENDENT),
        AxisID_AXIS_ID_3,
        AxisID_AXIS_UNDEFINED,
    };
    PositionMode mode;
    AxisID primary;
    ForceFetchEntry fetch[TOPOLOGY_MAX_FETCH];
    uint8_t count;
    run_compute(own, linked, mode, primary, fetch, count);

    TEST_ASSERT_EQUAL(POSITION_MODE_USE_OWN, mode);
    TEST_ASSERT_EQUAL(1, count);
    TEST_ASSERT_EQUAL(AxisID_AXIS_ID_1, fetch[0].axis_id);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, fetch[0].sign);
}

// Primary: linked_axes[0] == own, no INDEPENDENT flag.
// Expected: USE_OWN, not subtractive, fetch list = all linked entries with their signs.
void test_topology_primary(void) {
    const AxisID own = AxisID_AXIS_ID_1;
    AxisID linked[N] = {
        AxisID_AXIS_ID_1,
        AxisID_AXIS_ID_2,
        AxisID(AxisID_AXIS_ID_3 | AxisID_AXIS_SUBTRACTIVE),
        AxisID_AXIS_UNDEFINED,
    };
    PositionMode mode;
    AxisID primary;
    ForceFetchEntry fetch[TOPOLOGY_MAX_FETCH];
    uint8_t count;
    run_compute(own, linked, mode, primary, fetch, count);

    TEST_ASSERT_EQUAL(POSITION_MODE_USE_OWN, mode);
    TEST_ASSERT_EQUAL(3, count);
    TEST_ASSERT_EQUAL(AxisID_AXIS_ID_1, fetch[0].axis_id);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, fetch[0].sign);
    TEST_ASSERT_EQUAL(AxisID_AXIS_ID_2, fetch[1].axis_id);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, fetch[1].sign);
    TEST_ASSERT_EQUAL(AxisID_AXIS_ID_3, fetch[2].axis_id);
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, fetch[2].sign);
}

// Linked additive: linked_axes[0] != own; own appears in entry 1..n without
// AXIS_SUBTRACTIVE.
// Expected: FETCH_PRIMARY, not subtractive.
void test_topology_linked_additive(void) {
    const AxisID own = AxisID_AXIS_ID_2;
    AxisID linked[N] = {
        AxisID_AXIS_ID_1,
        AxisID_AXIS_ID_2,
        AxisID_AXIS_UNDEFINED,
        AxisID_AXIS_UNDEFINED,
    };
    PositionMode mode;
    AxisID primary;
    ForceFetchEntry fetch[TOPOLOGY_MAX_FETCH];
    uint8_t count;
    run_compute(own, linked, mode, primary, fetch, count);

    TEST_ASSERT_EQUAL(POSITION_MODE_FETCH_PRIMARY, mode);
    TEST_ASSERT_EQUAL(AxisID_AXIS_ID_1, primary);
    TEST_ASSERT_EQUAL(2, count);
    TEST_ASSERT_EQUAL(AxisID_AXIS_ID_1, fetch[0].axis_id);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, fetch[0].sign);
    TEST_ASSERT_EQUAL(AxisID_AXIS_ID_2, fetch[1].axis_id);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, fetch[1].sign);
}

// Linked subtractive: same as linked_additive but with AXIS_SUBTRACTIVE on
// the own entry (mirror case — left flight pedal).
// Expected: FETCH_PRIMARY_MIRRORED, is_subtractive == true.
void test_topology_linked_subtractive(void) {
    const AxisID own = AxisID_AXIS_ID_2;
    AxisID linked[N] = {
        AxisID_AXIS_ID_1,
        AxisID(AxisID_AXIS_ID_2 | AxisID_AXIS_SUBTRACTIVE),
        AxisID_AXIS_UNDEFINED,
        AxisID_AXIS_UNDEFINED,
    };
    PositionMode mode;
    AxisID primary;
    ForceFetchEntry fetch[TOPOLOGY_MAX_FETCH];
    uint8_t count;
    run_compute(own, linked, mode, primary, fetch, count);

    TEST_ASSERT_EQUAL(POSITION_MODE_FETCH_PRIMARY_MIRRORED, mode);
    TEST_ASSERT_EQUAL(AxisID_AXIS_ID_1, primary);
    TEST_ASSERT_EQUAL(2, count);
    TEST_ASSERT_EQUAL(AxisID_AXIS_ID_1, fetch[0].axis_id);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, fetch[0].sign);
    TEST_ASSERT_EQUAL(AxisID_AXIS_ID_2, fetch[1].axis_id);
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, fetch[1].sign);
}

// Primary with AXIS_INDEPENDENT bit on entry 0 — should resolve to USE_OWN
// regardless of own's presence in the rest of the list (matches the original
// `linked_axes[0] & AXIS_INDEPENDENT` short-circuit in calc_final_position).
void test_topology_primary_independent_bit(void) {
    const AxisID own = AxisID_AXIS_ID_2;
    AxisID linked[N] = {
        AxisID(AxisID_AXIS_ID_1 | AxisID_AXIS_INDEPENDENT),
        AxisID_AXIS_ID_2,
        AxisID_AXIS_UNDEFINED,
        AxisID_AXIS_UNDEFINED,
    };
    PositionMode mode;
    AxisID primary;
    ForceFetchEntry fetch[TOPOLOGY_MAX_FETCH];
    uint8_t count;
    run_compute(own, linked, mode, primary, fetch, count);

    TEST_ASSERT_EQUAL(POSITION_MODE_USE_OWN, mode);
}

// Own axis not in linked_axes at all — original code returns false from
// calc_final_position. NOT_MEMBER preserves that semantic.
void test_topology_not_member(void) {
    const AxisID own = AxisID_AXIS_ID_3;
    AxisID linked[N] = {
        AxisID_AXIS_ID_1,
        AxisID_AXIS_ID_2,
        AxisID_AXIS_UNDEFINED,
        AxisID_AXIS_UNDEFINED,
    };
    PositionMode mode;
    AxisID primary;
    ForceFetchEntry fetch[TOPOLOGY_MAX_FETCH];
    uint8_t count;
    run_compute(own, linked, mode, primary, fetch, count);

    TEST_ASSERT_EQUAL(POSITION_MODE_NOT_MEMBER, mode);
    // Force-sum still iterates non-independent entries, matching the old
    // calc_input_force_sum behaviour for this corner case.
    TEST_ASSERT_EQUAL(2, count);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_topology_independent);
    RUN_TEST(test_topology_independent_filters_others);
    RUN_TEST(test_topology_primary);
    RUN_TEST(test_topology_linked_additive);
    RUN_TEST(test_topology_linked_subtractive);
    RUN_TEST(test_topology_primary_independent_bit);
    RUN_TEST(test_topology_not_member);
    return UNITY_END();
}
