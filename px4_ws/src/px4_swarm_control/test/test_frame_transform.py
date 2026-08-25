from px4_swarm_control.frame_transform import field_delta_to_ned_delta


def test_fixed_field_mapping_converts_forward_left_up_to_px4_ned():
    assert field_delta_to_ned_delta(
        field_x=1.0,
        field_y=0.0,
        field_up=0.0,
    ) == (0.0, 1.0, 0.0)
    assert field_delta_to_ned_delta(
        field_x=0.0,
        field_y=1.0,
        field_up=0.0,
    ) == (1.0, 0.0, 0.0)
    assert field_delta_to_ned_delta(
        field_x=0.0,
        field_y=0.0,
        field_up=1.0,
    ) == (0.0, 0.0, -1.0)


def test_fixed_mapping_keeps_signed_combined_deltas_in_ned():
    assert field_delta_to_ned_delta(
        field_x=-0.5,
        field_y=0.25,
        field_up=-0.75,
    ) == (0.25, -0.5, 0.75)
