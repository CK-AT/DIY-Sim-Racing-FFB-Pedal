import math

import diy_ffb_protocol_pb2 as ffb_protocol
import general_kinematics


def _build_triangle_config(travel_negative, travel_positive):
    config = ffb_protocol.GeneralKinematicConfig()
    config.rail_travel_negative = travel_negative
    config.rail_travel_positive = travel_positive

    config.pins.add(pin_id=1, x=0.0, y=0.0, grounded=True)
    config.pins.add(pin_id=2, x=100.0, y=0.0, is_rail_interface=True)
    config.pins.add(pin_id=3, x=50.0, y=50.0, is_contact_point=True)

    metering = config.bars.add()
    metering.pin_ids.extend([2, 3])
    metering.is_metering = True

    link = config.bars.add()
    link.pin_ids.extend([1, 3])
    return config


def _build_collinear_config(travel_negative, travel_positive):
    config = ffb_protocol.GeneralKinematicConfig()
    config.rail_travel_negative = travel_negative
    config.rail_travel_positive = travel_positive

    config.pins.add(pin_id=1, x=0.0, y=0.0, grounded=True)
    config.pins.add(pin_id=2, x=100.0, y=0.0, is_rail_interface=True)
    config.pins.add(pin_id=3, x=50.0, y=50.0)
    config.pins.add(pin_id=4, x=100.0, y=100.0, is_contact_point=True)

    metering = config.bars.add()
    metering.pin_ids.extend([2, 3])
    metering.is_metering = True

    collinear = config.bars.add()
    collinear.pin_ids.extend([1, 3, 4])
    return config


def _assert_near(expected, actual, tol, message):
    if abs(expected - actual) > tol:
        raise AssertionError(f"{message} Expected {expected:.3f}, got {actual:.3f}.")


def _assert_raises(func, message):
    try:
        func()
    except ValueError:
        return
    except Exception as exc:
        raise AssertionError(f"{message} Raised {type(exc).__name__} instead of ValueError.") from exc
    raise AssertionError(message)


def _assert_finite(values, message):
    for value in values:
        if not math.isfinite(value):
            raise AssertionError(message)


def test_centered_contact_zero():
    travel_negative = 15.0
    travel_positive = 25.0
    config = _build_triangle_config(travel_negative, travel_positive)
    params = general_kinematics.calc_kinematic_parameters(config)

    if not params.coeffs_sled_pos_over_contact_point_pos:
        raise AssertionError("Missing sled position coefficients.")

    sled_at_zero = params.coeffs_sled_pos_over_contact_point_pos[0]
    _assert_near(travel_negative, sled_at_zero, 1.0, "Contact position zero should map to rail center.")

    if params.contact_point_pos_min_abs >= 0:
        raise AssertionError("ContactPointPosMinAbs should be negative after centering.")
    if params.contact_point_pos_max_abs <= 0:
        raise AssertionError("ContactPointPosMaxAbs should be positive after centering.")


def test_missing_contact_throws():
    config = ffb_protocol.GeneralKinematicConfig()
    config.rail_travel_negative = 5.0
    config.rail_travel_positive = 5.0
    config.pins.add(pin_id=1, x=0.0, y=0.0, grounded=True)
    config.pins.add(pin_id=2, x=10.0, y=0.0, is_rail_interface=True)
    bar = config.bars.add()
    bar.pin_ids.extend([1, 2])
    bar.is_metering = True

    try:
        general_kinematics.calc_kinematic_parameters(config)
    except ValueError:
        return
    raise AssertionError("Missing contact point should raise ValueError.")


def test_collinear_bar_solves():
    config = _build_collinear_config(15.0, 25.0)
    params = general_kinematics.calc_kinematic_parameters(config)

    if not params.coeffs_sled_pos_over_contact_point_pos:
        raise AssertionError("Missing sled position coefficients for collinear bar.")
    if params.contact_point_pos_min_abs >= 0:
        raise AssertionError("ContactPointPosMinAbs should be negative after centering.")
    if params.contact_point_pos_max_abs <= 0:
        raise AssertionError("ContactPointPosMaxAbs should be positive after centering.")


def test_negative_travel_throws():
    config = _build_triangle_config(-1.0, 5.0)
    _assert_raises(lambda: general_kinematics.calc_kinematic_parameters(config),
                   "Negative rail travel should raise ValueError.")


def test_duplicate_pin_id_throws():
    config = ffb_protocol.GeneralKinematicConfig()
    config.rail_travel_negative = 5.0
    config.rail_travel_positive = 5.0
    config.pins.add(pin_id=1, x=0.0, y=0.0, grounded=True)
    config.pins.add(pin_id=1, x=100.0, y=0.0, is_rail_interface=True)
    config.pins.add(pin_id=2, x=50.0, y=50.0, is_contact_point=True)
    bar = config.bars.add()
    bar.pin_ids.extend([1, 2])
    bar.is_metering = True
    _assert_raises(lambda: general_kinematics.calc_kinematic_parameters(config),
                   "Duplicate pin_id should raise ValueError.")


def test_contact_grounded_throws():
    config = ffb_protocol.GeneralKinematicConfig()
    config.rail_travel_negative = 5.0
    config.rail_travel_positive = 5.0
    config.pins.add(pin_id=1, x=0.0, y=0.0, grounded=True, is_contact_point=True)
    config.pins.add(pin_id=2, x=100.0, y=0.0, is_rail_interface=True)
    bar = config.bars.add()
    bar.pin_ids.extend([1, 2])
    bar.is_metering = True
    _assert_raises(lambda: general_kinematics.calc_kinematic_parameters(config),
                   "Grounded contact pin should raise ValueError.")


def test_rail_grounded_throws():
    config = ffb_protocol.GeneralKinematicConfig()
    config.rail_travel_negative = 5.0
    config.rail_travel_positive = 5.0
    config.pins.add(pin_id=1, x=0.0, y=0.0, grounded=True, is_rail_interface=True)
    config.pins.add(pin_id=2, x=50.0, y=50.0, is_contact_point=True)
    bar = config.bars.add()
    bar.pin_ids.extend([1, 2])
    bar.is_metering = True
    _assert_raises(lambda: general_kinematics.calc_kinematic_parameters(config),
                   "Grounded rail pin should raise ValueError.")


def test_contact_rail_same_throws():
    config = ffb_protocol.GeneralKinematicConfig()
    config.rail_travel_negative = 5.0
    config.rail_travel_positive = 5.0
    config.pins.add(pin_id=1, x=0.0, y=0.0, is_contact_point=True, is_rail_interface=True)
    config.pins.add(pin_id=2, x=50.0, y=50.0, grounded=True)
    bar = config.bars.add()
    bar.pin_ids.extend([1, 2])
    bar.is_metering = True
    _assert_raises(lambda: general_kinematics.calc_kinematic_parameters(config),
                   "Contact point pin cannot be rail interface pin.")


def test_metering_bar_pin_count_throws():
    config = ffb_protocol.GeneralKinematicConfig()
    config.rail_travel_negative = 5.0
    config.rail_travel_positive = 5.0
    config.pins.add(pin_id=1, x=0.0, y=0.0, grounded=True)
    config.pins.add(pin_id=2, x=100.0, y=0.0, is_rail_interface=True)
    config.pins.add(pin_id=3, x=50.0, y=50.0, is_contact_point=True)
    config.pins.add(pin_id=4, x=25.0, y=25.0)
    bar = config.bars.add()
    bar.pin_ids.extend([1, 2, 3])
    bar.is_metering = True
    _assert_raises(lambda: general_kinematics.calc_kinematic_parameters(config),
                   "Metering bar with more than two pins should raise ValueError.")


def test_multiple_metering_bars_throws():
    config = ffb_protocol.GeneralKinematicConfig()
    config.rail_travel_negative = 5.0
    config.rail_travel_positive = 5.0
    config.pins.add(pin_id=1, x=0.0, y=0.0, grounded=True)
    config.pins.add(pin_id=2, x=100.0, y=0.0, is_rail_interface=True)
    config.pins.add(pin_id=3, x=50.0, y=50.0, is_contact_point=True)
    bar = config.bars.add()
    bar.pin_ids.extend([1, 2])
    bar.is_metering = True
    bar = config.bars.add()
    bar.pin_ids.extend([2, 3])
    bar.is_metering = True
    _assert_raises(lambda: general_kinematics.calc_kinematic_parameters(config),
                   "Multiple metering bars should raise ValueError.")


def test_missing_metering_throws():
    config = ffb_protocol.GeneralKinematicConfig()
    config.rail_travel_negative = 5.0
    config.rail_travel_positive = 5.0
    config.pins.add(pin_id=1, x=0.0, y=0.0, grounded=True)
    config.pins.add(pin_id=2, x=100.0, y=0.0, is_rail_interface=True)
    config.pins.add(pin_id=3, x=50.0, y=50.0, is_contact_point=True)
    bar = config.bars.add()
    bar.pin_ids.extend([1, 3])
    _assert_raises(lambda: general_kinematics.calc_kinematic_parameters(config),
                   "Missing metering bar should raise ValueError.")


def test_non_collinear_bar_throws():
    config = ffb_protocol.GeneralKinematicConfig()
    config.rail_travel_negative = 5.0
    config.rail_travel_positive = 5.0
    config.pins.add(pin_id=1, x=0.0, y=0.0, grounded=True)
    config.pins.add(pin_id=2, x=100.0, y=0.0, is_rail_interface=True)
    config.pins.add(pin_id=3, x=50.0, y=50.0, is_contact_point=True)
    config.pins.add(pin_id=4, x=60.0, y=10.0)
    metering = config.bars.add()
    metering.pin_ids.extend([2, 3])
    metering.is_metering = True
    bar = config.bars.add()
    bar.pin_ids.extend([1, 3, 4])
    _assert_raises(lambda: general_kinematics.calc_kinematic_parameters(config),
                   "Non-collinear bar should raise ValueError.")


def test_shared_collinear_bar_throws():
    config = ffb_protocol.GeneralKinematicConfig()
    config.rail_travel_negative = 5.0
    config.rail_travel_positive = 5.0
    config.pins.add(pin_id=1, x=0.0, y=0.0, grounded=True)
    config.pins.add(pin_id=2, x=100.0, y=0.0, is_rail_interface=True)
    config.pins.add(pin_id=3, x=50.0, y=0.0)
    config.pins.add(pin_id=4, x=150.0, y=0.0)
    config.pins.add(pin_id=5, x=200.0, y=0.0, is_contact_point=True)
    metering = config.bars.add()
    metering.pin_ids.extend([2, 5])
    metering.is_metering = True
    bar = config.bars.add()
    bar.pin_ids.extend([1, 3, 4])
    bar = config.bars.add()
    bar.pin_ids.extend([3, 4, 5])
    _assert_raises(lambda: general_kinematics.calc_kinematic_parameters(config),
                   "Pins in multiple collinear bars should raise ValueError.")


def test_coefficients_finite():
    config = _build_triangle_config(10.0, 10.0)
    params = general_kinematics.calc_kinematic_parameters(config)
    _assert_finite(params.coeffs_sled_pos_over_contact_point_pos, "Sled coefficients contain NaN/inf.")
    _assert_finite(params.coeffs_force_factor_over_contact_point_pos, "Force coefficients contain NaN/inf.")


if __name__ == "__main__":
    tests = [
        ("centered_contact_zero", test_centered_contact_zero),
        ("missing_contact_throws", test_missing_contact_throws),
        ("collinear_bar_solves", test_collinear_bar_solves),
        ("negative_travel_throws", test_negative_travel_throws),
        ("duplicate_pin_id_throws", test_duplicate_pin_id_throws),
        ("contact_grounded_throws", test_contact_grounded_throws),
        ("rail_grounded_throws", test_rail_grounded_throws),
        ("contact_rail_same_throws", test_contact_rail_same_throws),
        ("metering_bar_pin_count_throws", test_metering_bar_pin_count_throws),
        ("multiple_metering_bars_throws", test_multiple_metering_bars_throws),
        ("missing_metering_throws", test_missing_metering_throws),
        ("non_collinear_bar_throws", test_non_collinear_bar_throws),
        ("shared_collinear_bar_throws", test_shared_collinear_bar_throws),
        ("coefficients_finite", test_coefficients_finite),
    ]

    failures = 0
    for name, test in tests:
        try:
            test()
            print(f"[PASS] {name}")
        except Exception as exc:
            failures += 1
            print(f"[FAIL] {name}: {exc}")

    print(f"Tests run: {len(tests)}, Failures: {failures}")
    if failures:
        raise SystemExit(1)
