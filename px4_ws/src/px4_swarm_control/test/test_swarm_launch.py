from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def private_node_field(node, field_name):
    # launch_ros 沒有穩定 getter 可在不執行 launch 時讀 namespace/parameters，集中存取降低測試脆弱性。
    return getattr(node, f'_Node__{field_name}')


def load_launch_module(file_name):
    launch_path = Path(__file__).parents[1] / 'launch' / file_name
    spec = spec_from_file_location('swarm_nodes_launch', launch_path)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_swarm_launch_module():
    return load_launch_module('swarm_nodes.launch.py')


def plain_node_parameters(node):
    result = {}
    for parameter_set in private_node_field(node, 'parameters'):
        if not isinstance(parameter_set, dict):
            continue
        for key_substitutions, value in parameter_set.items():
            key = ''.join(substitution.text for substitution in key_substitutions)
            result[key] = plain_launch_value(value)
    return result


def plain_launch_value(value):
    if isinstance(value, tuple):
        if len(value) == 1 and isinstance(value[0], LaunchConfiguration):
            return launch_configuration_name(value[0])
        if len(value) == 1 and isinstance(value[0], list):
            return value[0]
        if all(isinstance(item, list) for item in value):
            return [
                ''.join(substitution.text for substitution in item)
                .strip()
                .removesuffix('...')
                .strip()
                .strip("'")
                for item in value
            ]
        text = ''.join(substitution.text for substitution in value).strip()
        return text.removesuffix('...').strip()
    return value


def launch_configuration_name(value):
    return ''.join(
        substitution.text
        for substitution in value.variable_name
    )


def launch_argument_names(launch_description):
    return [
        entity.name for entity in launch_description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]


def test_swarm_launch_starts_only_first_version_swarm_nodes():
    module = load_swarm_launch_module()

    launch_description = module.generate_launch_description()
    nodes = [
        entity for entity in launch_description.entities
        if isinstance(entity, Node)
    ]

    assert [(node.node_package, node.node_executable) for node in nodes] == [
        ('px4_swarm_control', 'vehicle_node'),
        ('px4_swarm_control', 'vehicle_node'),
        ('px4_swarm_control', 'vehicle_node'),
        ('px4_swarm_control', 'ground_station_node'),
    ]
    assert [
        private_node_field(node, 'node_namespace')
        for node in nodes[:3]
    ] == ['/MAV1', '/MAV2', '/MAV3']
    assert private_node_field(nodes[3], 'node_namespace') is None
    assert all(
        node.node_executable not in {
            'operator_console',
            'px4_speed_profile',
            'MicroXRCEAgent',
            'px4',
            'gz',
        }
        for node in nodes
    )


def test_swarm_launch_uses_three_vehicle_yaml_as_parameter_source():
    module = load_swarm_launch_module()

    launch_description = module.generate_launch_description()
    vehicle_nodes = [
        entity for entity in launch_description.entities
        if isinstance(entity, Node) and entity.node_executable == 'vehicle_node'
    ]

    assert [
        plain_node_parameters(node)['vehicle_id'] for node in vehicle_nodes
    ] == ['MAV1', 'MAV2', 'MAV3']
    assert [
        plain_node_parameters(node)['px4_namespace'] for node in vehicle_nodes
    ] == ['/MAV1', '/MAV2', '/MAV3']
    assert [
        plain_node_parameters(node)['slot'] for node in vehicle_nodes
    ] == ['leader', 'follower_left', 'follower_right']
    assert [
        plain_node_parameters(node)['px4_target_system'] for node in vehicle_nodes
    ] == [1, 2, 3]
    assert all(
        plain_node_parameters(node)['coordinate_profile'] == 'gazebo_enu_common_world'
        for node in vehicle_nodes
    )
    assert plain_node_parameters(nodes := [
        entity for entity in launch_description.entities
        if isinstance(entity, Node) and entity.node_executable == 'ground_station_node'
    ][0])['vertical_axis_up'] is True


def test_real_vehicle_launches_start_one_vehicle_node_each():
    cases = (
        ('real_mav1_vehicle.launch.py', '/MAV1', 'MAV1', 'leader'),
        ('real_mav2_vehicle.launch.py', '/MAV2', 'MAV2', 'follower_left'),
        ('real_mav3_vehicle.launch.py', '/MAV3', 'MAV3', 'follower_right'),
    )

    for file_name, namespace, vehicle_id, slot in cases:
        module = load_launch_module(file_name)
        launch_description = module.generate_launch_description()
        nodes = [
            entity for entity in launch_description.entities
            if isinstance(entity, Node)
        ]

        assert [(node.node_package, node.node_executable) for node in nodes] == [
            ('px4_swarm_control', 'vehicle_node'),
        ]
        assert private_node_field(nodes[0], 'node_namespace') == namespace
        parameters = plain_node_parameters(nodes[0])
        assert parameters['vehicle_id'] == vehicle_id
        assert parameters['px4_namespace'] == namespace
        assert parameters['slot'] == slot
        assert parameters['coordinate_profile'] == 'raw_px4_local'
        assert parameters['common_origin_e_m'] == 0.0
        assert parameters['common_origin_n_m'] == 0.0
        assert parameters['common_origin_u_m'] == 0.0


def test_real_ground_station_launch_starts_only_ground_station_node():
    module = load_launch_module('real_ground_station.launch.py')

    launch_description = module.generate_launch_description()
    nodes = [
        entity for entity in launch_description.entities
        if isinstance(entity, Node)
    ]

    assert [(node.node_package, node.node_executable) for node in nodes] == [
        ('px4_swarm_control', 'ground_station_node'),
    ]
    assert private_node_field(nodes[0], 'node_namespace') is None
    assert 'formation_line_abreast_lateral_spacing_m' in plain_node_parameters(nodes[0])
    assert plain_node_parameters(nodes[0])['vertical_axis_up'] is False


def test_swarm_and_real_ground_station_launches_expose_formation_tolerance_override():
    for file_name in ('swarm_nodes.launch.py', 'real_ground_station.launch.py'):
        module = load_launch_module(file_name)
        launch_description = module.generate_launch_description()
        nodes = [
            entity for entity in launch_description.entities
            if isinstance(entity, Node)
            and entity.node_executable == 'ground_station_node'
        ]

        assert 'formation_position_tolerance_m' in launch_argument_names(
            launch_description,
        )
        assert (
            plain_node_parameters(nodes[0])['formation_position_tolerance_m']
            == 'formation_position_tolerance_m'
        )


def test_operator_console_launch_starts_only_console_with_settle_overrides():
    module = load_launch_module('operator_console.launch.py')

    launch_description = module.generate_launch_description()
    nodes = [
        entity for entity in launch_description.entities
        if isinstance(entity, Node)
    ]

    assert [(node.node_package, node.node_executable) for node in nodes] == [
        ('px4_swarm_control', 'operator_console'),
    ]
    assert set(launch_argument_names(launch_description)) >= {
        'settle_position_tolerance_m',
        'settle_stable_duration_s',
    }
    parameters = plain_node_parameters(nodes[0])
    assert parameters['demo_commands'] == [
        '1',
        '2',
        'settle',
        '5',
        'settle',
        '7',
        'settle',
        '6',
        'settle',
        'home_yaw',
        'settle',
        'home',
        'settle',
        '8',
    ]
    assert parameters['settle_position_tolerance_m'] == 'settle_position_tolerance_m'
    assert parameters['settle_stable_duration_s'] == 'settle_stable_duration_s'
