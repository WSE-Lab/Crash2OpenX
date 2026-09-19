import os
import math
import re
import traceback
import xml.etree.ElementTree as ET
from datetime import datetime
import sys
import argparse
from collections import deque
from html import escape

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scenario_runner'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'PCLA'))

try:
    from carla_compat import ensure_carla_importable
except ImportError:
    from src.carla_compat import ensure_carla_importable

ensure_carla_importable()

VERSION = '0.9.16'
DEFAULT_INPUT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'input'))


def import_runtime_dependencies():
    global carla, DataCollector, ScenarioRunner

    import carla as carla_module
    from data_collector import DataCollector as data_collector_class
    from scenario_runner_local import ScenarioRunner as scenario_runner_class

    carla = carla_module
    DataCollector = data_collector_class
    ScenarioRunner = scenario_runner_class


def create_carla_waypoint_from_xyz(x, y, z, pitch=0.0, yaw=0.0, roll=0.0):
    location = carla.Location(x=float(x), y=float(y), z=float(z))
    rotation = carla.Rotation(pitch=float(pitch), yaw=float(yaw), roll=float(roll))
    return carla.Transform(location, rotation)


def parse_arguments():
    parser = argparse.ArgumentParser(description='CARLA Demo Script')
    parser.add_argument(
        'case_arg',
        nargs='?',
        default=None,
        help='可选：场景编号/简称/文件名；例如 1、001、zoox april。未指定时使用 input 中排序第一的场景。'
    )
    parser.add_argument('-v', '--version', action='version', version='%(prog)s ' + VERSION)
    parser.add_argument('--timeout', default='10.0', help='Set the CARLA client timeout value in seconds')
    parser.add_argument('--trafficManagerPort', default='8000', help='Port to use for the TrafficManager (default: 8000)')
    parser.add_argument('--trafficManagerSeed', default='0', help='Seed used by the TrafficManager (default: 0)')
    parser.add_argument('--sync', default=True, action='store_true', help='Forces the simulation to run synchronously')
    parser.add_argument('--reloadWorld', default=True, action='store_true', help='Reload the CARLA world before starting a scenario (default=True)')
    parser.add_argument('--host', default='localhost', help='CARLA服务器主机地址 (默认: localhost)')
    parser.add_argument('--port', type=int, default=2000, help='CARLA服务器端口号 (默认: 2000)')
    parser.add_argument('--record', action='store_true', help='是否启用录制功能')
    parser.add_argument('--output', default='route.xml', help='输出路径文件名 (默认: route.xml)')
    parser.add_argument('--scenario', default=None, help='OpenSCENARIO文件路径；未指定时默认使用 input/xosc_skill 中排序第一的场景')
    parser.add_argument(
        '--case',
        default=None,
        help='可选：input 目录中的场景编号/名称，例如 001_Zoox_April_11_2025；会自动匹配同名 .xosc 和 .xodr。'
    )
    parser.add_argument(
        '--input-root',
        default=DEFAULT_INPUT_ROOT,
        help='--case 的输入根目录，默认使用仓库 input 目录。'
    )
    parser.add_argument('--agent', default='if_if', help='PCLA agent 名称，格式如 if_if / tfv6_regnet / carl_roach')
    parser.add_argument(
        '--sync-route',
        action='store_true',
        help='启用旧的 PCLA agent 路径同步逻辑：生成 route.xml、写入 hero controller、同步 ActEnd。回放模式默认关闭。'
    )
    parser.add_argument(
        '--replay-control',
        choices=['kinematic', 'npc'],
        default='kinematic',
        help='回放模式控制方式：kinematic 按 XOSC 轨迹逐帧贴位姿；npc 使用 CARLA PID/LocalPlanner 追踪轨迹。'
    )
    parser.add_argument(
        '--pcla-sut',
        action='store_true',
        help='在 replay 场景中保留一个被测车辆由 PCLA/ADS 控制，其他轨迹 actor 继续 replay。'
    )
    parser.add_argument(
        '--sut-actor',
        default='hero',
        help='PCLA/ADS 被测车辆的 OpenSCENARIO entityRef，默认 hero。'
    )
    parser.add_argument(
        '--sut-spawn-z',
        type=float,
        default=0.5,
        help='PCLA/ADS 被测车辆 Init TeleportAction 的最低 z，默认 0.5。'
    )
    parser.add_argument(
        '--sut-start-offset',
        type=float,
        default=0.0,
        help='将 PCLA/ADS 被测车辆沿其原始轨迹向前裁剪的距离，单位米；用于避开 OpenDRIVE 边界外的起点。'
    )
    parser.add_argument(
        '--sut-route-mode',
        choices=['endpoints', 'full'],
        default='endpoints',
        help='PCLA/ADS route 生成方式：endpoints 只给起点和终点；full 保留原始轨迹顶点。默认 endpoints。'
    )
    parser.add_argument(
        '--sut-route-spacing',
        type=float,
        default=5.0,
        help='PCLA/ADS route waypoint 最大间距，单位米；0 表示不插值。默认 5m。'
    )
    parser.add_argument('--disable-traffic-lights', default=True, help='全局禁用红绿灯，统一冻结为绿灯')
    parser.add_argument('--collect-data', default=True, action='store_true', help='是否启用数据收集功能')
    parser.add_argument('--continue-after-collision', action='store_true',
                        help='保留多次碰撞序列，首个碰撞后继续到场景结束')
    parser.add_argument('--collision-tail-seconds', type=float, default=2.0,
                        help='物理碰撞后继续采集的仿真秒数，默认 2 秒')
    parser.add_argument('--data-output', default=None, help='数据输出文件路径（默认自动生成）')
    parser.add_argument(
        '--xodr',
        '--map',
        dest='xodr',
        default=None,
        help='可选：自定义 OpenDRIVE .xodr 文件路径。指定后会先加载该地图，再加载/运行 --scenario 指定的 .xosc。'
    )
    parser.add_argument('--mid-model-path', default=None, help='可选：生成该 XOSC 的中间模型 JSON 路径，写入 sim_feedback.json')
    parser.add_argument('--real-time-factor', type=float, default=1.0, help='同步模式仿真实时倍率，1.0 表示按真实时间播放，2.0 表示两倍速')
    parser.add_argument('--post-run-hold', type=float, default=0.0, help='场景结束后销毁 actor 前停留秒数，方便观察最终状态')
    parser.add_argument('--no-rendering', action='store_true', help='启用 CARLA no_rendering_mode，无窗口/无画面渲染时也可跑测试')
    parser.add_argument('--record-video', action='store_true', help='根据采集到的轨迹生成可下载的俯视视频')
    parser.add_argument('--video-fps', type=int, default=10, help='轨迹视频帧率，默认 10')
    parser.add_argument('--video-frame-stride', type=int, default=2, help='轨迹视频采样间隔，默认每 2 个采集帧取 1 帧')
    args = parser.parse_args()
    if args.case_arg and not args.case and not args.scenario:
        args.case = args.case_arg
    return args


def _candidate_file_names(identifier, extension):
    base_name = os.path.basename(identifier)
    if base_name.lower().endswith(extension):
        return [base_name]
    return [base_name + extension]


def _normalize_case_token(value):
    return ''.join(ch.lower() for ch in value if ch.isalnum())


def _collect_input_files(input_dir, extension):
    matches = []
    input_dir = os.path.abspath(input_dir)
    for root, _, files in os.walk(input_dir):
        for file_name in files:
            if file_name.lower().endswith(extension):
                matches.append(os.path.abspath(os.path.join(root, file_name)))
    return sorted(matches)


def resolve_input_file(identifier, input_dir, extension, label):
    if not identifier:
        return None

    direct_path = os.path.abspath(identifier)
    if os.path.isfile(direct_path):
        return direct_path

    if os.path.splitext(identifier)[1].lower() == extension and os.path.isfile(os.path.abspath(identifier)):
        return os.path.abspath(identifier)

    input_dir = os.path.abspath(input_dir)
    candidate_names = _candidate_file_names(identifier, extension)
    direct_candidates = [os.path.join(input_dir, name) for name in candidate_names]
    for candidate in direct_candidates:
        if os.path.isfile(candidate):
            return os.path.abspath(candidate)

    all_files = _collect_input_files(input_dir, extension)
    matches = [path for path in all_files if os.path.basename(path) in candidate_names]

    if not matches:
        query = _normalize_case_token(os.path.splitext(os.path.basename(identifier))[0])
        for path in all_files:
            stem = os.path.splitext(os.path.basename(path))[0]
            normalized_stem = _normalize_case_token(stem)
            numeric_prefix = stem.split('_', 1)[0]
            if numeric_prefix.isdigit() and query.isdigit() and int(query) == int(numeric_prefix):
                matches.append(path)
            elif not query.isdigit() and (normalized_stem.startswith(query) or query in normalized_stem):
                matches.append(path)

    if not matches:
        raise FileNotFoundError(f'未找到 {label}: {identifier}，搜索目录: {input_dir}')
    if len(matches) > 1:
        match_list = '\n  '.join(matches)
        raise ValueError(f'{label} 匹配到多个文件，请传入更具体路径:\n  {match_list}')
    return matches[0]


def resolve_first_input_case(input_dir):
    candidates = _collect_input_files(input_dir, '.xosc')
    if not candidates:
        raise FileNotFoundError(f'input 中没有找到任何 OpenSCENARIO .xosc 文件: {os.path.abspath(input_dir)}')
    return sorted(candidates)[0]


def resolve_demo_inputs(args):
    input_root = os.path.abspath(args.input_root)
    xosc_input_dir = os.path.join(input_root, 'xosc_skill')
    xodr_input_dir = os.path.join(input_root, 'generated_maps_skill')

    if args.case:
        scenario_file = resolve_input_file(args.case, xosc_input_dir, '.xosc', 'OpenSCENARIO')
        xodr_file = resolve_input_file(args.case, xodr_input_dir, '.xodr', 'OpenDRIVE')
        args.scenario = scenario_file
        if not args.xodr:
            args.xodr = xodr_file
        return scenario_file, args.xodr

    if args.scenario:
        scenario_file = os.path.abspath(args.scenario)
    else:
        scenario_file = resolve_first_input_case(xosc_input_dir)
        args.scenario = scenario_file
        args.case = os.path.splitext(os.path.basename(scenario_file))[0]

    if not os.path.isfile(scenario_file):
        scenario_file = resolve_input_file(args.scenario, xosc_input_dir, '.xosc', 'OpenSCENARIO')
        args.scenario = scenario_file

    if args.xodr:
        args.xodr = os.path.abspath(args.xodr)
        return scenario_file, args.xodr

    scenario_stem = os.path.splitext(os.path.basename(scenario_file))[0]
    try:
        xodr_file = resolve_input_file(scenario_stem, xodr_input_dir, '.xodr', 'OpenDRIVE')
    except FileNotFoundError:
        xodr_file = None
    if xodr_file:
        args.xodr = xodr_file
    return scenario_file, args.xodr


def build_episode_output_dir(scenario_file, requested_output):
    if requested_output:
        output_path = os.path.abspath(requested_output)
        file_like_extensions = {
            '.csv',
            '.gif',
            '.json',
            '.jsonl',
            '.mp4',
            '.txt',
            '.xml',
        }
        if os.path.splitext(os.path.basename(output_path))[1].lower() in file_like_extensions:
            return os.path.dirname(output_path) or os.getcwd()
        return output_path

    scenario_name = os.path.splitext(os.path.basename(scenario_file))[0]
    return os.path.abspath(os.path.join(os.getcwd(), 'runs', scenario_name))


def build_runtime_scenario_path(source_scenario_file, episode_output_dir):
    scenario_name = os.path.splitext(os.path.basename(source_scenario_file))[0]
    runtime_dir = episode_output_dir or os.path.join(os.getcwd(), 'eval', '_runtime')
    os.makedirs(runtime_dir, exist_ok=True)
    return os.path.abspath(os.path.join(runtime_dir, f'{scenario_name}.runtime.xosc'))


def extract_hero_world_position(xml_tree, carla_map=None):
    for private in xml_tree.iter('Private'):
        if private.attrib.get('entityRef') != 'hero':
            continue
        for position in private.findall('./PrivateAction/TeleportAction/Position'):
            world_position = position.find('WorldPosition')
            if world_position is not None:
                x = float(world_position.attrib.get('x', 0.0))
                y = float(world_position.attrib.get('y', 0.0))
                z = float(world_position.attrib.get('z', 0.2))
                h = float(world_position.attrib.get('h', 0.0))
                yaw_deg = math.degrees(h)
                return carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw_deg))

            lane_position = position.find('LanePosition')
            if lane_position is not None:
                if carla_map is None:
                    raise ValueError('hero 使用 LanePosition 起点，需要已加载的 CARLA map 才能生成 route')
                transform = _position_to_carla_transform(position, carla_map)
                if transform is not None:
                    return transform

    raise ValueError('未能在 xosc 中找到 hero 的 WorldPosition 或 LanePosition')


def get_scenario_town(xml_tree):
    logic_file = xml_tree.find('.//RoadNetwork/LogicFile')
    return logic_file.attrib.get('filepath', '_') if logic_file is not None else '_'


def set_scenario_logic_file(xml_tree, logic_file_path):
    road_network = xml_tree.find('RoadNetwork')
    if road_network is None:
        root = xml_tree.getroot()
        entities = root.find('Entities')
        insert_index = list(root).index(entities) if entities is not None else len(list(root))
        road_network = ET.Element('RoadNetwork')
        root.insert(insert_index, road_network)

    logic_file = road_network.find('LogicFile')
    if logic_file is None:
        logic_file = ET.Element('LogicFile')
        road_network.insert(0, logic_file)

    logic_file.set('filepath', logic_file_path)


def normalize_compiled_seed_world_positions(xml_tree):
    header = xml_tree.find('FileHeader')
    if header is not None and header.attrib.get('description', '').startswith('CARLA:'):
        return 0

    waypoint_property_names = {
        'spawn_waypoint_id',
        'spawn_trigger_sut_waypoint_id',
        'sut_goal_waypoint_id',
        'sut_start_waypoint_id',
    }
    is_compiled_seed = any(
        prop.attrib.get('name') in waypoint_property_names
        for prop in xml_tree.findall('.//Property')
    )
    if not is_compiled_seed:
        return 0

    updated_count = 0
    for world_position in xml_tree.findall('.//WorldPosition'):
        if 'y' in world_position.attrib:
            world_position.set('y', str(-float(world_position.attrib['y'])))
        if 'h' in world_position.attrib:
            world_position.set('h', str(-float(world_position.attrib['h'])))
        updated_count += 1
    return updated_count


def resolve_scenario_xodr_path(xml_tree, scenario_file):
    town = get_scenario_town(xml_tree)
    if not town or not town.lower().endswith('.xodr') or os.path.isabs(town):
        return None

    xodr_path = os.path.abspath(os.path.join(os.path.dirname(scenario_file), town))
    if not os.path.isfile(xodr_path):
        raise FileNotFoundError(f'OpenSCENARIO 中引用的相对 OpenDRIVE 文件不存在: {xodr_path}')
    set_scenario_logic_file(xml_tree, xodr_path)
    return xodr_path


def normalize_yaw_delta(source_yaw, target_yaw):
    return (target_yaw - source_yaw + 180.0) % 360.0 - 180.0


def choose_branch_candidate(current_wp, next_candidates):
    current_yaw = current_wp.transform.rotation.yaw
    ranked_candidates = []
    for index, candidate in enumerate(next_candidates):
        candidate_yaw = candidate.transform.rotation.yaw
        delta_yaw = normalize_yaw_delta(current_yaw, candidate_yaw)
        lane_continuity = 1 if candidate.lane_id == current_wp.lane_id else 0
        ranked_candidates.append((lane_continuity, -abs(delta_yaw), -index, delta_yaw, candidate))
    ranked_candidates.sort(key=lambda item: item[:3], reverse=True)
    _, _, _, chosen_delta, chosen_wp = ranked_candidates[0]
    branch_side = 'right-turn/branch' if chosen_delta > 5.0 else 'left-turn/branch' if chosen_delta < -5.0 else 'lane-continuation'
    return chosen_wp, chosen_delta, branch_side


def find_first_branch(start_wp, step_distance=2.0, branch_lookahead=8.0, scan_steps=30):
    current_wp = start_wp
    for scan_step in range(scan_steps):
        lookahead_candidates = current_wp.next(branch_lookahead)
        if len(lookahead_candidates) > 1:
            chosen_wp, chosen_delta, branch_side = choose_branch_candidate(current_wp, lookahead_candidates)
            return current_wp, chosen_wp, chosen_delta, branch_side, scan_step
        next_candidates = current_wp.next(step_distance)
        if not next_candidates:
            break
        current_wp = next_candidates[0]
    return None, None, None, None, None


def build_lane_route(start_wp, step_distance=2.0, branch_lookahead=8.0, scan_steps=30, branch_extension_steps=28):
    route_waypoints = [start_wp]
    branch_origin_wp, chosen_wp, chosen_delta, branch_side, scan_step = find_first_branch(
        start_wp, step_distance=step_distance, branch_lookahead=branch_lookahead, scan_steps=scan_steps)

    if chosen_wp is None:
        current_wp = start_wp
        for _ in range(scan_steps):
            next_candidates = current_wp.next(step_distance)
            if not next_candidates:
                break
            next_wp = next_candidates[0]
            if next_wp.transform.location.distance(route_waypoints[-1].transform.location) >= 0.05:
                route_waypoints.append(next_wp)
            current_wp = next_wp
        destination = route_waypoints[-1].transform.location
        return route_waypoints, destination, 'no branch found, fallback=lane continuation preview only'

    current_wp = start_wp
    while current_wp.transform.location.distance(branch_origin_wp.transform.location) > (step_distance * 0.6):
        next_candidates = current_wp.next(step_distance)
        if not next_candidates:
            break
        next_wp = next_candidates[0]
        if next_wp.transform.location.distance(route_waypoints[-1].transform.location) >= 0.05:
            route_waypoints.append(next_wp)
        current_wp = next_wp

    if chosen_wp.transform.location.distance(route_waypoints[-1].transform.location) >= 0.05:
        route_waypoints.append(chosen_wp)
    current_wp = chosen_wp

    for _ in range(branch_extension_steps):
        next_candidates = current_wp.next(step_distance)
        if not next_candidates:
            break
        if len(next_candidates) > 1:
            next_wp, _, _ = choose_branch_candidate(current_wp, next_candidates)
        else:
            next_wp = next_candidates[0]
        if next_wp.transform.location.distance(route_waypoints[-1].transform.location) >= 0.05:
            route_waypoints.append(next_wp)
        current_wp = next_wp

    destination = route_waypoints[-1].transform.location
    decision = (
        f'branch step={scan_step}, chose road={chosen_wp.road_id}, lane={chosen_wp.lane_id}, '
        f'delta_yaw={chosen_delta:.1f}°, decision={branch_side}'
    )
    return route_waypoints, destination, decision


def set_actor_external_control_property(xml_tree, actor_name, property_name, property_value):
    for private in xml_tree.iter('Private'):
        if private.attrib.get('entityRef') != actor_name:
            continue
        for controller_action in private.iter('ControllerAction'):
            assign_action = controller_action.find('AssignControllerAction')
            if assign_action is None:
                continue
            controller = assign_action.find('Controller')
            if controller is None:
                continue
            properties = controller.find('Properties')
            if properties is None:
                properties = ET.SubElement(controller, 'Properties')
            target_property = None
            module_value = None
            for prop in properties.findall('Property'):
                if prop.attrib.get('name') == 'module':
                    module_value = prop.attrib.get('value')
                if prop.attrib.get('name') == property_name:
                    target_property = prop
            if module_value != 'external_control':
                continue
            if target_property is None:
                target_property = ET.SubElement(properties, 'Property')
                target_property.set('name', property_name)
            target_property.set('value', property_value)
            return True
    return False


def set_hero_external_control_property(xml_tree, property_name, property_value):
    return set_actor_external_control_property(xml_tree, 'hero', property_name, property_value)


def set_hero_route_file_property(xml_tree, route_file_path):
    return set_hero_external_control_property(xml_tree, 'route_file', route_file_path)


def set_hero_agent_property(xml_tree, agent_name):
    return set_hero_external_control_property(xml_tree, 'pcla_agent', agent_name)


def remove_external_control_controller_actions(xml_tree, keep_entities=None):
    keep_entities = set(keep_entities or [])
    removed_count = 0
    for private in xml_tree.iter('Private'):
        if private.attrib.get('entityRef') in keep_entities:
            continue
        for private_action in list(private.findall('PrivateAction')):
            controller_action = private_action.find('ControllerAction')
            if controller_action is None:
                continue

            assign_action = controller_action.find('AssignControllerAction')
            if assign_action is None:
                continue

            controller = assign_action.find('Controller')
            properties = controller.find('Properties') if controller is not None else None
            if properties is None:
                continue

            module_value = None
            for prop in properties.findall('Property'):
                if prop.attrib.get('name') == 'module':
                    module_value = prop.attrib.get('value')
                    break

            if module_value == 'external_control':
                private.remove(private_action)
                removed_count += 1
    return removed_count


def get_follow_trajectory_entities(xml_tree):
    entity_refs = set()
    for maneuver_group in xml_tree.findall('.//ManeuverGroup'):
        if maneuver_group.find('.//FollowTrajectoryAction') is None:
            continue
        actors = maneuver_group.find('Actors')
        if actors is None:
            continue
        for entity_ref in actors.findall('EntityRef'):
            ref_name = entity_ref.attrib.get('entityRef')
            if ref_name:
                entity_refs.add(ref_name)

    for private in xml_tree.findall('.//Storyboard/Init/Actions/Private'):
        if private.find('.//FollowTrajectoryAction') is not None:
            ref_name = private.attrib.get('entityRef')
            if ref_name:
                entity_refs.add(ref_name)
    return entity_refs


def ensure_scenario_actor_exists(xml_tree, actor_name):
    for scenario_object in xml_tree.findall('.//Entities/ScenarioObject'):
        if scenario_object.attrib.get('name') == actor_name:
            if scenario_object.find('Vehicle') is None:
                raise ValueError(f'PCLA SUT 必须是 Vehicle，当前 {actor_name} 不是车辆')
            return
    raise ValueError(f'未找到 PCLA SUT actor: {actor_name}')


def ensure_init_private(xml_tree, entity_ref):
    actions = xml_tree.find('.//Storyboard/Init/Actions')
    if actions is None:
        raise ValueError('未找到 Storyboard/Init/Actions，无法配置 replay controller')

    for private in actions.findall('Private'):
        if private.attrib.get('entityRef') == entity_ref:
            return private

    private = ET.Element('Private')
    private.set('entityRef', entity_ref)
    actions.append(private)
    return private


def set_controller_module(controller_action, module_name):
    override_action = controller_action.find('OverrideControllerValueAction')
    if override_action is None:
        override_action = ET.Element('OverrideControllerValueAction')
        for tag, attrs in (
                ('Throttle', {'active': 'false', 'value': '0.0'}),
                ('Brake', {'active': 'false', 'value': '0.0'}),
                ('Clutch', {'active': 'false', 'value': '0.0'}),
                ('ParkingBrake', {'active': 'false', 'value': '0.0'}),
                ('SteeringWheel', {'active': 'false', 'value': '0.0'}),
                ('Gear', {'active': 'false', 'number': '0.0'})):
            ET.SubElement(override_action, tag, attrs)
        controller_action.insert(0, override_action)

    assign_action = controller_action.find('AssignControllerAction')
    if assign_action is None:
        assign_action = ET.SubElement(controller_action, 'AssignControllerAction')

    controller = assign_action.find('Controller')
    if controller is None:
        controller = ET.SubElement(assign_action, 'Controller')
        controller.set('name', f'{module_name}_controller')

    properties = controller.find('Properties')
    if properties is None:
        properties = ET.SubElement(controller, 'Properties')

    module_property = None
    for prop in properties.findall('Property'):
        if prop.attrib.get('name') == 'module':
            module_property = prop
            break
    if module_property is None:
        module_property = ET.SubElement(properties, 'Property')
        module_property.set('name', 'module')
    module_property.set('value', module_name)


def configure_kinematic_replay_controllers(xml_tree, exclude_entities=None):
    moving_entities = get_follow_trajectory_entities(xml_tree)
    exclude_entities = set(exclude_entities or [])
    configured_count = 0

    for entity_ref in sorted(moving_entities):
        if entity_ref in exclude_entities:
            continue
        private = ensure_init_private(xml_tree, entity_ref)
        controller_action = None
        for private_action in private.findall('PrivateAction'):
            candidate = private_action.find('ControllerAction')
            if candidate is not None:
                controller_action = candidate
                break

        if controller_action is None:
            private_action = ET.Element('PrivateAction')
            controller_action = ET.SubElement(private_action, 'ControllerAction')
            private.append(private_action)

        set_controller_module(controller_action, 'kinematic_replay_control')
        configured_count += 1

    return configured_count


def _set_property(properties, name, value):
    target_property = None
    for prop in properties.findall('Property'):
        if prop.attrib.get('name') == name:
            target_property = prop
            break
    if target_property is None:
        target_property = ET.SubElement(properties, 'Property')
        target_property.set('name', name)
    target_property.set('value', value)


def set_unique_ego_vehicle(xml_tree, sut_actor_name):
    ensure_scenario_actor_exists(xml_tree, sut_actor_name)
    updated = {}

    for scenario_object in xml_tree.findall('.//Entities/ScenarioObject'):
        actor_name = scenario_object.attrib.get('name')
        vehicle = scenario_object.find('Vehicle')
        if vehicle is None:
            continue

        properties = vehicle.find('Properties')
        if properties is None:
            properties = ET.SubElement(vehicle, 'Properties')

        target_type = 'ego_vehicle' if actor_name == sut_actor_name else 'simulation'
        _set_property(properties, 'type', target_type)
        updated[actor_name] = target_type

    return updated


def lift_actor_init_teleport_z(xml_tree, actor_name, min_z):
    updated_count = 0
    for private in xml_tree.findall('.//Storyboard/Init/Actions/Private'):
        if private.attrib.get('entityRef') != actor_name:
            continue
        for world_position in private.findall('./PrivateAction/TeleportAction/Position/WorldPosition'):
            current_z = float(world_position.get('z', 0.0))
            if current_z < min_z:
                world_position.set('z', f'{min_z:.3f}')
                updated_count += 1
    return updated_count


def set_actor_init_teleport_from_transform(xml_tree, actor_name, transform, min_z=None):
    updated_count = 0
    osc_y = -transform.location.y
    osc_h = -math.radians(transform.rotation.yaw)
    target_z = transform.location.z
    if min_z is not None:
        target_z = max(target_z, min_z)

    for private in xml_tree.findall('.//Storyboard/Init/Actions/Private'):
        if private.attrib.get('entityRef') != actor_name:
            continue
        for world_position in private.findall('./PrivateAction/TeleportAction/Position/WorldPosition'):
            world_position.set('x', f'{transform.location.x:.6f}')
            world_position.set('y', f'{osc_y:.6f}')
            world_position.set('z', f'{target_z:.6f}')
            world_position.set('h', f'{osc_h:.12f}')
            updated_count += 1
    return updated_count


def configure_pcla_sut_controller(xml_tree, actor_name, agent_name, route_file_path=None):
    ensure_scenario_actor_exists(xml_tree, actor_name)
    private = ensure_init_private(xml_tree, actor_name)
    controller_action = None
    for private_action in private.findall('PrivateAction'):
        candidate = private_action.find('ControllerAction')
        if candidate is not None:
            controller_action = candidate
            break

    if controller_action is None:
        private_action = ET.Element('PrivateAction')
        controller_action = ET.SubElement(private_action, 'ControllerAction')
        private.append(private_action)

    set_controller_module(controller_action, 'external_control')

    assign_action = controller_action.find('AssignControllerAction')
    controller = assign_action.find('Controller') if assign_action is not None else None
    if controller is None:
        raise ValueError(f'无法为 {actor_name} 配置 external_control controller')
    controller.set('name', f'{actor_name}_pcla_sut')

    properties = controller.find('Properties')
    if properties is None:
        properties = ET.SubElement(controller, 'Properties')
    _set_property(properties, 'pcla_agent', agent_name)
    if route_file_path:
        _set_property(properties, 'route_file', route_file_path)
    return True


def _osc_world_position_to_carla_transform(world_position):
    x = float(world_position.get('x', 0.0))
    y = -float(world_position.get('y', 0.0))
    z = float(world_position.get('z', 0.5))
    yaw = -math.degrees(float(world_position.get('h', 0.0)))
    pitch = math.degrees(float(world_position.get('p', 0.0)))
    roll = math.degrees(float(world_position.get('r', 0.0)))
    return carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(pitch=pitch, yaw=yaw, roll=roll))


def _position_to_carla_transform(position, carla_map=None):
    world_position = position.find('WorldPosition')
    if world_position is not None:
        return _osc_world_position_to_carla_transform(world_position)

    lane_position = position.find('LanePosition')
    if lane_position is not None:
        if carla_map is None:
            raise ValueError('LanePosition route extraction requires a loaded CARLA map')
        road_id = int(lane_position.get('roadId', 0))
        lane_id = int(lane_position.get('laneId', 0))
        s = float(lane_position.get('s', 0.0))
        offset = float(lane_position.get('offset', 0.0))
        waypoint = carla_map.get_waypoint_xodr(road_id, lane_id, s)
        if waypoint is None:
            raise ValueError(f'LanePosition roadId={road_id}, laneId={lane_id}, s={s} does not exist')

        transform = carla.Transform(waypoint.transform.location, waypoint.transform.rotation)
        orientation = lane_position.find('Orientation')
        if orientation is not None:
            transform.rotation.yaw += -math.degrees(float(orientation.get('h', 0.0)))
            transform.rotation.pitch += math.degrees(float(orientation.get('p', 0.0)))
            transform.rotation.roll += math.degrees(float(orientation.get('r', 0.0)))

        if offset:
            forward = transform.rotation.get_forward_vector()
            transform.location.x += offset * -forward.y
            transform.location.y += offset * forward.x
        return transform

    return None


def extract_actor_follow_trajectory_transforms(xml_tree, actor_name, carla_map=None):
    transforms = []
    for maneuver_group in xml_tree.findall('.//ManeuverGroup'):
        actors = maneuver_group.find('Actors')
        if actors is None:
            continue
        actor_refs = {entity.attrib.get('entityRef') for entity in actors.findall('EntityRef')}
        if actor_name not in actor_refs:
            continue
        for position in maneuver_group.findall('.//FollowTrajectoryAction//Vertex/Position'):
            transform = _position_to_carla_transform(position, carla_map)
            if transform is not None:
                transforms.append(transform)
    return transforms


def extract_actor_init_teleport_transform(xml_tree, actor_name, carla_map=None):
    for private in xml_tree.findall('.//Storyboard/Init/Actions/Private'):
        if private.attrib.get('entityRef') != actor_name:
            continue
        position = private.find('./PrivateAction/TeleportAction/Position')
        if position is not None:
            transform = _position_to_carla_transform(position, carla_map)
            if transform is not None:
                return transform
    return None


def extract_actor_acquire_position_transforms(xml_tree, actor_name, carla_map=None):
    transforms = []
    for maneuver_group in xml_tree.findall('.//ManeuverGroup'):
        actors = maneuver_group.find('Actors')
        if actors is None:
            continue
        actor_refs = {entity.attrib.get('entityRef') for entity in actors.findall('EntityRef')}
        if actor_name not in actor_refs:
            continue
        for position in maneuver_group.findall('.//AcquirePositionAction/Position'):
            transform = _position_to_carla_transform(position, carla_map)
            if transform is not None:
                transforms.append(transform)
    return transforms


def extract_actor_assigned_route_transforms(xml_tree, actor_name, carla_map=None):
    """Read a concrete OSC route without replanning its junction choices."""
    routes = []
    for private in xml_tree.findall('.//Storyboard/Init/Actions/Private'):
        if private.get('entityRef') == actor_name:
            routes.extend(private.findall('.//AssignRouteAction/Route'))
    for group in xml_tree.findall('.//ManeuverGroup'):
        actors = group.find('Actors')
        if actors is not None and any(ref.get('entityRef') == actor_name for ref in actors.findall('EntityRef')):
            routes.extend(group.findall('.//AssignRouteAction/Route'))
    if not routes:
        return []
    if len(routes) != 1:
        raise ValueError(f'{actor_name} has multiple assigned routes; a static PCLA route cannot preserve their triggers')
    transforms = []
    for position in routes[0].findall('./Waypoint/Position'):
        transform = _position_to_carla_transform(position, carla_map)
        if transform is None:
            raise ValueError(f'{actor_name} assigned route contains an unsupported waypoint position')
        transforms.append(transform)
    if len(transforms) < 2:
        raise ValueError(f'{actor_name} assigned route needs at least two waypoints')
    return transforms


def _get_actor_vehicle_property(xml_tree, actor_name, property_name):
    for scenario_object in xml_tree.findall('.//Entities/ScenarioObject'):
        if scenario_object.attrib.get('name') != actor_name:
            continue
        vehicle = scenario_object.find('Vehicle')
        properties = vehicle.find('Properties') if vehicle is not None else None
        if properties is None:
            return None
        for prop in properties.findall('Property'):
            if prop.attrib.get('name') == property_name:
                return prop.attrib.get('value')
    return None


def _resolve_carla_waypoint_id(carla_map, waypoint_id):
    match = re.fullmatch(r'r(-?\d+):sec(-?\d+):l(-?\d+):s(-?\d+(?:\.\d+)?)', waypoint_id or '')
    if match is None:
        raise ValueError(f'Invalid CARLA waypoint id: {waypoint_id!r}')

    road_id, section_id, lane_id, s = match.groups()
    waypoint = carla_map.get_waypoint_xodr(int(road_id), int(lane_id), float(s))
    if waypoint is None:
        raise ValueError(f'CARLA waypoint does not exist: {waypoint_id}')
    if int(waypoint.section_id) != int(section_id):
        raise ValueError(
            f'CARLA waypoint section mismatch: requested={waypoint_id}, '
            f'resolved_section={waypoint.section_id}'
        )
    return waypoint


def _carla_waypoint_key(waypoint):
    return (
        int(waypoint.road_id),
        int(waypoint.section_id),
        int(waypoint.lane_id),
        round(float(waypoint.s), 2),
    )


def _trace_carla_waypoint_route(start_waypoint, goal_waypoint, step_distance=2.0, max_waypoints=10000):
    goal_key = _carla_waypoint_key(goal_waypoint)
    queue = deque([start_waypoint])
    start_key = _carla_waypoint_key(start_waypoint)
    parents = {start_key: None}
    waypoints = {start_key: start_waypoint}
    resolved_goal_key = None

    while queue and len(parents) <= max_waypoints:
        current_waypoint = queue.popleft()
        current_key = _carla_waypoint_key(current_waypoint)
        if current_key == goal_key:
            resolved_goal_key = current_key
            break

        for next_waypoint in current_waypoint.next(step_distance):
            next_key = _carla_waypoint_key(next_waypoint)
            if next_key in parents:
                continue
            parents[next_key] = current_key
            waypoints[next_key] = next_waypoint
            queue.append(next_waypoint)

    if resolved_goal_key is None:
        return []

    route = []
    current_key = resolved_goal_key
    while current_key is not None:
        route.append(waypoints[current_key])
        current_key = parents[current_key]
    route.reverse()
    return route


def extract_actor_waypoint_property_route_transforms(xml_tree, actor_name, carla_map, client):
    start_id = _get_actor_vehicle_property(xml_tree, actor_name, 'sut_start_waypoint_id')
    goal_id = _get_actor_vehicle_property(xml_tree, actor_name, 'sut_goal_waypoint_id')
    if not start_id and not goal_id:
        return []
    if not start_id or not goal_id:
        raise ValueError(
            f'{actor_name} must define both sut_start_waypoint_id and sut_goal_waypoint_id'
        )

    start_waypoint = _resolve_carla_waypoint_id(carla_map, start_id)
    goal_waypoint = _resolve_carla_waypoint_id(carla_map, goal_id)
    route_waypoints = _trace_carla_waypoint_route(start_waypoint, goal_waypoint)
    if not route_waypoints:
        from PCLA import location_to_waypoint
        route_waypoints = location_to_waypoint(
            client,
            start_waypoint.transform.location,
            goal_waypoint.transform.location
        )
    if len(route_waypoints) < 2:
        raise ValueError(
            f'{actor_name} waypoint property route could not be planned: {start_id} -> {goal_id}'
        )
    return [waypoint.transform for waypoint in route_waypoints]


def _normalize_yaw_delta_degrees(source_yaw, target_yaw):
    return (target_yaw - source_yaw + 180.0) % 360.0 - 180.0


def _interpolate_transform(first, second, ratio):
    ratio = max(0.0, min(1.0, ratio))
    location = carla.Location(
        x=first.location.x + (second.location.x - first.location.x) * ratio,
        y=first.location.y + (second.location.y - first.location.y) * ratio,
        z=first.location.z + (second.location.z - first.location.z) * ratio,
    )
    yaw_delta = _normalize_yaw_delta_degrees(first.rotation.yaw, second.rotation.yaw)
    rotation = carla.Rotation(
        pitch=first.rotation.pitch + (second.rotation.pitch - first.rotation.pitch) * ratio,
        yaw=first.rotation.yaw + yaw_delta * ratio,
        roll=first.rotation.roll + (second.rotation.roll - first.rotation.roll) * ratio,
    )
    return carla.Transform(location, rotation)


def trim_transforms_by_distance(transforms, start_offset):
    if start_offset <= 0.0 or len(transforms) < 2:
        return transforms

    remaining_offset = start_offset
    for index in range(len(transforms) - 1):
        current_tf = transforms[index]
        next_tf = transforms[index + 1]
        segment_distance = current_tf.location.distance(next_tf.location)
        if segment_distance <= 1e-6:
            continue
        if remaining_offset <= segment_distance:
            trimmed_start = _interpolate_transform(current_tf, next_tf, remaining_offset / segment_distance)
            return [trimmed_start] + transforms[index + 1:]
        remaining_offset -= segment_distance

    return [transforms[-1]]


def densify_transforms_by_distance(transforms, max_spacing):
    if max_spacing <= 0.0 or len(transforms) < 2:
        return transforms

    dense_transforms = [transforms[0]]
    for index in range(len(transforms) - 1):
        current_tf = transforms[index]
        next_tf = transforms[index + 1]
        segment_distance = current_tf.location.distance(next_tf.location)
        if segment_distance <= 1e-6:
            continue

        segment_count = max(1, int(math.ceil(segment_distance / max_spacing)))
        for step in range(1, segment_count + 1):
            dense_transforms.append(_interpolate_transform(current_tf, next_tf, step / segment_count))

    return dense_transforms


def align_transforms_to_start_driving_lane(carla_map, transforms):
    if not transforms:
        return transforms

    start_waypoint = carla_map.get_waypoint(
        transforms[0].location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )
    if start_waypoint is None:
        return transforms

    projected_start = start_waypoint.transform
    offset = carla.Location(
        x=projected_start.location.x - transforms[0].location.x,
        y=projected_start.location.y - transforms[0].location.y,
        z=0.0,
    )
    aligned_transforms = []
    yaw_delta = _normalize_yaw_delta_degrees(transforms[0].rotation.yaw, projected_start.rotation.yaw)
    for transform in transforms:
        aligned_transforms.append(carla.Transform(
            carla.Location(
                x=transform.location.x + offset.x,
                y=transform.location.y + offset.y,
                z=projected_start.location.z,
            ),
            carla.Rotation(
                pitch=transform.rotation.pitch,
                yaw=transform.rotation.yaw + yaw_delta,
                roll=transform.rotation.roll,
            )
        ))
    return aligned_transforms


def _heading_degrees_between_locations(start_location, end_location):
    return math.degrees(math.atan2(
        end_location.y - start_location.y,
        end_location.x - start_location.x
    ))


def _choose_straightest_waypoint(candidates, target_yaw):
    ranked_candidates = []
    for index, candidate in enumerate(candidates):
        yaw_delta = abs(_normalize_yaw_delta_degrees(target_yaw, candidate.transform.rotation.yaw))
        ranked_candidates.append((yaw_delta, index, candidate))
    ranked_candidates.sort(key=lambda item: item[:2])
    return ranked_candidates[0][2]


def build_straight_lane_route_from_endpoints(carla_map, transforms, step_distance):
    if len(transforms) < 2:
        return transforms

    start_transform = transforms[0]
    end_transform = transforms[-1]
    start_waypoint = carla_map.get_waypoint(
        start_transform.location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )
    if start_waypoint is None:
        return align_transforms_to_start_driving_lane(carla_map, transforms)

    target_yaw = _heading_degrees_between_locations(start_transform.location, end_transform.location)
    route_distance = start_transform.location.distance(end_transform.location)
    route_transforms = [start_waypoint.transform]
    current_waypoint = start_waypoint
    travelled = 0.0
    step_distance = max(step_distance, 1.0)
    max_steps = max(2, int(math.ceil(route_distance / step_distance)) + 4)

    for _ in range(max_steps):
        if travelled >= route_distance:
            break
        candidates = current_waypoint.next(min(step_distance, route_distance - travelled))
        if not candidates:
            break
        next_waypoint = _choose_straightest_waypoint(candidates, target_yaw)
        travelled += current_waypoint.transform.location.distance(next_waypoint.transform.location)
        route_transforms.append(next_waypoint.transform)
        current_waypoint = next_waypoint

    return route_transforms


def write_route_from_transforms(transforms, output_file):
    from xml.dom import minidom

    root = minidom.Document()
    xml = root.createElement('route')
    xml.setAttribute('id', '_')
    xml.setAttribute('town', '_')
    root.appendChild(xml)

    for tf in transforms:
        waypoint = root.createElement('waypoint')
        waypoint.setAttribute('pitch', str(tf.rotation.pitch))
        waypoint.setAttribute('roll', str(tf.rotation.roll))
        waypoint.setAttribute('x', str(tf.location.x))
        waypoint.setAttribute('y', str(tf.location.y))
        waypoint.setAttribute('yaw', str(tf.rotation.yaw))
        waypoint.setAttribute('z', str(tf.location.z))
        xml.appendChild(waypoint)

    with open(output_file, 'w', encoding='utf-8') as route_output:
        route_output.write(root.toprettyxml(indent='\t'))


def build_pcla_sut_route_from_xosc(
        xml_tree,
        actor_name,
        client,
        output_file,
        start_offset=0.0,
        route_mode='endpoints',
        route_spacing=5.0):
    world = ensure_world_matches_scenario_town(client, xml_tree)
    carla_map = world.get_map()
    transforms = extract_actor_follow_trajectory_transforms(xml_tree, actor_name, carla_map)
    preserve_topology = False
    if len(transforms) < 2:
        transforms = extract_actor_assigned_route_transforms(xml_tree, actor_name, carla_map)
        if transforms:
            preserve_topology = True
            print(f'PCLA SUT route: preserving {len(transforms)} generated AssignRouteAction waypoints for {actor_name}.')
    if len(transforms) < 2:
        init_transform = extract_actor_init_teleport_transform(xml_tree, actor_name, carla_map)
        acquire_transforms = extract_actor_acquire_position_transforms(xml_tree, actor_name, carla_map)
        if init_transform is not None and acquire_transforms:
            transforms = [init_transform, acquire_transforms[-1]]
            print(f'PCLA SUT route fallback: 使用 {actor_name} 的 Init TeleportAction -> AcquirePositionAction 生成 route。')
    if len(transforms) < 2:
        transforms = extract_actor_waypoint_property_route_transforms(xml_tree, actor_name, carla_map, client)
        if transforms:
            preserve_topology = True
            print(f'PCLA SUT route fallback: 使用 {actor_name} 的 sut_start_waypoint_id -> sut_goal_waypoint_id 生成 route。')
    if len(transforms) < 2:
        raise ValueError(
            f'{actor_name} 没有足够的 FollowTrajectory、AssignRoute、AcquirePosition 或 waypoint property 顶点，'
            '无法为 PCLA SUT 生成 route'
        )
    transforms = trim_transforms_by_distance(transforms, start_offset)
    if len(transforms) < 2:
        raise ValueError(f'{actor_name} start offset={start_offset:.2f}m 后 route 少于 2 个点，无法为 PCLA SUT 生成 route')
    if route_mode == 'endpoints' and not preserve_topology:
        lane_transforms = build_straight_lane_route_from_endpoints(
            carla_map, transforms, route_spacing
        )
        transforms = lane_transforms if len(lane_transforms) >= 2 else [transforms[0], transforms[-1]]
    original_waypoint_count = len(transforms)
    if route_mode == 'endpoints' and not preserve_topology:
        transforms = align_transforms_to_start_driving_lane(carla_map, transforms)
    else:
        print('PCLA full route: preserving supplied start position, heading and route geometry.')
    transforms = densify_transforms_by_distance(transforms, route_spacing)

    write_route_from_transforms(transforms, output_file)
    print(f'PCLA SUT route 已从 {actor_name} 原始轨迹生成: {output_file}')
    print(
        f'PCLA SUT route mode={route_mode}, spacing={route_spacing:.2f}m, '
        f'waypoints={len(transforms)} (before densify={original_waypoint_count}), '
        f'PCLA SUT route start=({transforms[0].location.x:.2f}, {transforms[0].location.y:.2f}) '
        f'end=({transforms[-1].location.x:.2f}, {transforms[-1].location.y:.2f})'
    )
    return transforms[0]


def remove_actor_follow_trajectory_maneuver_groups(xml_tree, actor_name):
    removed_count = 0
    for act in xml_tree.findall('.//Act'):
        for maneuver_group in list(act.findall('ManeuverGroup')):
            actors = maneuver_group.find('Actors')
            if actors is None:
                continue
            actor_refs = {entity.attrib.get('entityRef') for entity in actors.findall('EntityRef')}
            if actor_name not in actor_refs:
                continue
            if maneuver_group.find('.//FollowTrajectoryAction') is None:
                continue
            act.remove(maneuver_group)
            removed_count += 1
    return removed_count


def set_act_end_reach_position(xml_tree, destination, tolerance=1.0):
    act_end_condition = xml_tree.find(".//Storyboard/Story/Act/StopTrigger/ConditionGroup/Condition[@name='ActEnd']")
    if act_end_condition is None:
        act_end_condition = xml_tree.find(".//Storyboard/Story/Act/StopTrigger/ConditionGroup/Condition[@name='act_stop']")
    if act_end_condition is None:
        act_end_condition = xml_tree.find(".//Storyboard/Story/Act/StopTrigger/ConditionGroup/Condition")
    if act_end_condition is None:
        act = xml_tree.find(".//Storyboard/Story/Act")
        if act is None:
            raise ValueError('未找到 Storyboard/Story/Act，无法同步终点结束条件')
        stop_trigger = act.find('StopTrigger')
        if stop_trigger is None:
            stop_trigger = ET.SubElement(act, 'StopTrigger')
        condition_group = stop_trigger.find('ConditionGroup')
        if condition_group is None:
            condition_group = ET.SubElement(stop_trigger, 'ConditionGroup')
        act_end_condition = ET.SubElement(condition_group, 'Condition')
        act_end_condition.set('name', 'ActEnd')
        act_end_condition.set('delay', '0.0')
        act_end_condition.set('conditionEdge', 'rising')

    by_entity = act_end_condition.find('ByEntityCondition')
    if by_entity is None:
        by_entity = ET.SubElement(act_end_condition, 'ByEntityCondition')
        triggering_entities = ET.SubElement(by_entity, 'TriggeringEntities')
        triggering_entities.set('triggeringEntitiesRule', 'any')
        entity_ref = ET.SubElement(triggering_entities, 'EntityRef')
        entity_ref.set('entityRef', 'hero')

    entity_condition = by_entity.find('EntityCondition')
    if entity_condition is None:
        entity_condition = ET.SubElement(by_entity, 'EntityCondition')
    else:
        for child in list(entity_condition):
            entity_condition.remove(child)

    reach_position = ET.SubElement(entity_condition, 'ReachPositionCondition')
    reach_position.set('tolerance', f'{tolerance:.1f}')
    position = ET.SubElement(reach_position, 'Position')
    world_position = ET.SubElement(position, 'WorldPosition')
    world_position.set('x', f'{destination.x:.6f}')
    world_position.set('y', f'{destination.y:.6f}')
    world_position.set('z', f'{destination.z:.6f}')


def extract_act_end_world_position(xml_tree):
    world_position = xml_tree.find(".//Storyboard/Story/Act/StopTrigger/ConditionGroup/Condition[@name='ActEnd']//WorldPosition")
    if world_position is None:
        raise ValueError('未在 xosc 中找到 ActEnd 的 WorldPosition 终点坐标')
    return carla.Location(
        x=float(world_position.get('x', 0.0)),
        y=float(world_position.get('y', 0.0)),
        z=float(world_position.get('z', 0.0)),
    )


def _read_xodr_file(xodr_path):
    return ScenarioRunner._read_opendrive_file(xodr_path)


def _generate_opendrive_world(client, xodr_path):
    xodr_data = _read_xodr_file(xodr_path)
    print(f'Route sync: generating OpenDRIVE world from {xodr_path}')
    return client.generate_opendrive_world(
        xodr_data,
        carla.OpendriveGenerationParameters(
            vertex_distance=2.0,
            wall_height=0.0,
            additional_width=0.6,
            smooth_junctions=True,
            enable_mesh_visibility=True,
        )
    )


def ensure_world_matches_scenario_town(client, xml_tree):
    town = get_scenario_town(xml_tree)
    world = client.get_world()
    current_town = world.get_map().name.split('/')[-1]
    if town and town.lower().endswith('.xodr'):
        xodr_path = town if os.path.isabs(town) else os.path.abspath(town)
        world = _generate_opendrive_world(client, xodr_path)
    elif town and current_town != town:
        print(f'Route sync: loading world {town} before computing route (current={current_town})')
        world = client.load_world(town)
    return world


def sync_route_from_xosc(xml_tree, client, output_file):
    world = ensure_world_matches_scenario_town(client, xml_tree)
    carla_map = world.get_map()
    hero_transform = extract_hero_world_position(xml_tree, carla_map)
    start_wp = carla_map.get_waypoint(hero_transform.location, project_to_road=True, lane_type=carla.LaneType.Driving)
    if start_wp is None:
        raise ValueError('无法将 hero 起点投影到 Driving 车道，不能生成统一 route')

    from PCLA import location_to_waypoint, route_maker
    route_waypoints, destination, decision = build_lane_route(start_wp)
    if len(route_waypoints) < 2:
        fallback_destination = extract_act_end_world_position(xml_tree)
        route_waypoints = location_to_waypoint(client, start_wp.transform.location, fallback_destination)
        if len(route_waypoints) < 2:
            raise ValueError('道路拓扑预览和起点终点全局规划都未生成足够的路径点，无法生成统一 route')
        route_maker(route_waypoints, output_file)
        decision = 'fallback=xosc ActEnd destination'
    else:
        route_maker(route_waypoints, output_file)

    route_tree = ET.parse(output_file)
    route_points = []
    for waypoint in route_tree.getroot().findall('.//waypoint'):
        route_points.append(carla.Location(
            x=float(waypoint.get('x', 0.0)),
            y=float(waypoint.get('y', 0.0)),
            z=float(waypoint.get('z', 0.5))
        ))
    if not route_points:
        raise ValueError('生成的 route.xml 不包含 waypoint，无法同步终点')
    destination = route_points[-1]

    if not set_hero_route_file_property(xml_tree, output_file):
        raise ValueError('未找到 hero 的 external_control route_file 配置，无法同步 route 文件')
    set_act_end_reach_position(xml_tree, destination, tolerance=1.0)
    print(f'Runtime ActEnd target: ({destination.x:.2f}, {destination.y:.2f}, {destination.z:.2f}), tolerance=1.0m')

    start_location = start_wp.transform.location
    print(
        f'统一 route 已生成: 起点({start_location.x:.2f}, {start_location.y:.2f}) '
        f'-> 终点({destination.x:.2f}, {destination.y:.2f})'
    )
    print(f'Route endpoint selection: {decision}')
    print(f'统一 route 文件: {output_file}')
    print(f'ActEnd 已同步为到达终点 1.0m 范围内结束: ({destination.x:.2f}, {destination.y:.2f})')


def export_route_debug_artifacts(route_file_path, preview_file_path, max_labels=20):
    route_tree = ET.parse(route_file_path)
    route_points = []
    for waypoint in route_tree.getroot().findall('.//waypoint'):
        route_points.append((
            float(waypoint.get('x', 0.0)),
            float(waypoint.get('y', 0.0)),
            float(waypoint.get('z', 0.0)),
            float(waypoint.get('yaw', 0.0)),
        ))

    if len(route_points) < 2:
        print('Route preview skipped: route has fewer than 2 waypoints')
        return

    xs = [point[0] for point in route_points]
    ys = [point[1] for point in route_points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    width = 900
    height = 900

    def project(x, y):
        span_x = max(max_x - min_x, 1.0)
        span_y = max(max_y - min_y, 1.0)
        scale = min((width - 80) / span_x, (height - 80) / span_y)
        px = 40 + (x - min_x) * scale
        py = height - (40 + (y - min_y) * scale)
        return px, py

    polyline = ' '.join(f'{project(x, y)[0]:.1f},{project(x, y)[1]:.1f}' for x, y, _, _ in route_points)
    label_indices = sorted(set([0, len(route_points) - 1] + [int(i * (len(route_points) - 1) / max(1, max_labels - 1)) for i in range(max_labels)]))

    circles = []
    labels = []
    for idx in label_indices:
        x, y, _, yaw = route_points[idx]
        px, py = project(x, y)
        color = '#1d4ed8' if idx == 0 else '#b91c1c' if idx == len(route_points) - 1 else '#d97706'
        circles.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="4" fill="{color}" />')
        labels.append(
            f'<text x="{px + 6:.1f}" y="{py - 6:.1f}" font-size="12" fill="#111827">'
            f'{idx}: ({x:.1f}, {y:.1f}) yaw={yaw:.1f}</text>'
        )

    svg_parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f8fafc" />',
        f'<polyline fill="none" stroke="#0f766e" stroke-width="3" points="{escape(polyline)}" />',
        *circles,
        *labels,
        f'<text x="24" y="28" font-size="18" fill="#111827">Route Preview: {escape(str(route_file_path))}</text>',
        '</svg>',
    ]

    with open(preview_file_path, 'w', encoding='utf-8') as svg_file:
        svg_file.write('\n'.join(svg_parts) + '\n')
    print(f'Route preview saved to: {preview_file_path}')
    print('Route waypoint preview:')
    preview_head = route_points[:8]
    preview_tail = route_points[-5:] if len(route_points) > 8 else []
    for idx, (x, y, z, yaw) in enumerate(preview_head):
        print(f'  head[{idx}]: x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={yaw:.1f}')
    if preview_tail:
        print('  ...')
        offset = len(route_points) - len(preview_tail)
        for idx, (x, y, z, yaw) in enumerate(preview_tail, start=offset):
            print(f'  tail[{idx}]: x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={yaw:.1f}')


def persist_synced_scenario(xml_tree, scenario_file):
    xml_tree.write(scenario_file, encoding='utf-8', xml_declaration=True)
    print(f'已将同步后的 OpenSCENARIO 写入运行时文件: {scenario_file}')


def save_route(tf):
    from xml.dom import minidom

    root = minidom.Document()
    root.toxml(encoding='utf-8')

    xml = root.createElement('route')
    xml.setAttribute('id', '_')
    xml.setAttribute('town', '_')
    root.appendChild(xml)

    product_child = root.createElement('waypoint')
    product_child.setAttribute('pitch', str(tf.rotation.pitch))
    product_child.setAttribute('roll', str(tf.rotation.roll))
    product_child.setAttribute('x', str(tf.location.x))
    product_child.setAttribute('y', str(tf.location.y))
    product_child.setAttribute('yaw', str(tf.rotation.yaw))
    product_child.setAttribute('z', str(tf.location.z))
    xml.appendChild(product_child)

    xml_str = root.toprettyxml(indent='\t')
    with open(args.output, 'w', encoding='utf-8') as route_output:
        route_output.write(xml_str)
    print(f'路径文件已保存到: {args.output}')


if __name__ == '__main__':
    args = parse_arguments()
    source_scenario_file, _ = resolve_demo_inputs(args)

    print(f'CARLA Host: {args.host}')
    print(f'CARLA Port: {args.port}')
    print(f'Record Mode: {args.record}')
    print(f'Output File: {args.output}')
    print(f'Case: {args.case or "disabled"}')
    print(f'Input Root: {args.input_root}')
    print(f'Scenario File: {args.scenario}')
    print(f'Custom XODR: {args.xodr or "disabled"}')
    print(f'PCLA Agent: {args.agent if args.sync_route or args.pcla_sut else "disabled (replay mode)"}')
    print(f'PCLA SUT: {args.sut_actor if args.pcla_sut else "disabled"}')
    print(f'Sync Route: {args.sync_route}')
    print(f'Replay Control: {args.replay_control}')
    print(f'Disable Traffic Lights: {args.disable_traffic_lights}')
    print(f'Data Collection: {args.collect_data}')
    print(f'Real Time Factor: {args.real_time_factor}')
    print(f'Post Run Hold: {args.post_run_hold}s')
    print(f'No Rendering: {args.no_rendering}')
    print(f'Record Video: {args.record_video}')
    if args.collect_data:
        print(f'Data Output: {args.data_output or "自动生成"}')
    print('-' * 50)

    scenario_file = source_scenario_file
    xml_tree = ET.parse(source_scenario_file)
    normalized_world_positions = normalize_compiled_seed_world_positions(xml_tree)
    if normalized_world_positions:
        print(
            'Compiled seed coordinate normalization: converted '
            f'{normalized_world_positions} WorldPosition entries from CARLA x/yaw to OpenSCENARIO coordinates.'
        )

    if args.xodr:
        xodr_path = os.path.abspath(args.xodr)
        if not os.path.isfile(xodr_path):
            raise FileNotFoundError(f'自定义 OpenDRIVE 文件不存在: {xodr_path}')
        set_scenario_logic_file(xml_tree, xodr_path)
        print(f'已将运行时 OpenSCENARIO 的 RoadNetwork/LogicFile 指向自定义地图: {xodr_path}')
    else:
        resolved_xodr_path = resolve_scenario_xodr_path(xml_tree, source_scenario_file)
        if resolved_xodr_path:
            print(f'已将 OpenSCENARIO 中的相对 OpenDRIVE 路径解析为绝对路径: {resolved_xodr_path}')

    episode_output_dir = None
    if args.collect_data:
        episode_output_dir = build_episode_output_dir(scenario_file, args.data_output)
        os.makedirs(episode_output_dir, exist_ok=True)
        print(f'Episode Output Dir: {episode_output_dir}')

    import_runtime_dependencies()

    route_output_path = None
    if args.sync_route:
        if not set_hero_agent_property(xml_tree, args.agent):
            raise ValueError('未找到 hero 的 external_control controller，无法写入 pcla_agent 配置')

        if episode_output_dir:
            route_output_path = os.path.join(episode_output_dir, os.path.basename(args.output))
        else:
            route_output_path = args.output if os.path.isabs(args.output) else os.path.abspath(args.output)
        client = carla.Client(args.host, args.port)
        client.set_timeout(float(args.timeout))
        sync_route_from_xosc(xml_tree, client, route_output_path)
        export_route_debug_artifacts(route_output_path, os.path.join(os.path.dirname(route_output_path), 'route_preview.svg'))
    else:
        configured_count = 0
        if args.replay_control == 'kinematic':
            excluded_entities = {args.sut_actor} if args.pcla_sut else set()
            configured_count = configure_kinematic_replay_controllers(xml_tree, exclude_entities=excluded_entities)
        if args.pcla_sut:
            ego_type_updates = set_unique_ego_vehicle(xml_tree, args.sut_actor)
            if episode_output_dir:
                route_output_path = os.path.join(episode_output_dir, f'{args.sut_actor}_pcla_route.xml')
            else:
                route_output_path = os.path.abspath(f'{args.sut_actor}_pcla_route.xml')
            client = carla.Client(args.host, args.port)
            client.set_timeout(float(args.timeout))
            sut_start_transform = build_pcla_sut_route_from_xosc(
                xml_tree,
                args.sut_actor,
                client,
                route_output_path,
                start_offset=args.sut_start_offset,
                route_mode=args.sut_route_mode,
                route_spacing=args.sut_route_spacing
            )
            updated_sut_init_count = set_actor_init_teleport_from_transform(
                xml_tree,
                args.sut_actor,
                sut_start_transform,
                min_z=args.sut_spawn_z
            )
            configure_pcla_sut_controller(xml_tree, args.sut_actor, args.agent, route_output_path)
            removed_sut_trajectories = remove_actor_follow_trajectory_maneuver_groups(xml_tree, args.sut_actor)
            removed_count = remove_external_control_controller_actions(xml_tree, keep_entities={args.sut_actor})
        else:
            removed_sut_trajectories = 0
            removed_count = remove_external_control_controller_actions(xml_tree)
        print('Replay mode: 跳过 PCLA route 生成、hero controller 写入和 ActEnd 终点同步。')
        if args.pcla_sut:
            print(f'Replay mode: 已为 {args.sut_actor} 启用 PCLA external_control，agent={args.agent}。')
            print(f'Replay mode: ego vehicle 已切换为 {args.sut_actor}: {ego_type_updates}')
            print(f'Replay mode: 已将 {args.sut_actor} 的 {updated_sut_init_count} 个 Init TeleportAction 同步到 PCLA route 起点，start_offset={args.sut_start_offset:.2f}m, route_mode={args.sut_route_mode}, route_spacing={args.sut_route_spacing:.2f}m, min_z={args.sut_spawn_z:.2f}。')
            print(f'Replay mode: 已移除 {args.sut_actor} 的 {removed_sut_trajectories} 个 FollowTrajectory ManeuverGroup，避免覆盖 ADS 控制。')
            print(f'Replay mode: 已移除其他 actor 的 {removed_count} 个 external_control ControllerAction。')
        else:
            print(f'Replay mode: 已移除 {removed_count} 个 external_control ControllerAction，避免启动 PCLA/carlaagent。')
        if args.replay_control == 'kinematic':
            print(f'Replay mode: 已为 {configured_count} 个轨迹 actor 启用 kinematic_replay_control 高保真回放。')

    runtime_scenario_path = build_runtime_scenario_path(source_scenario_file, episode_output_dir)
    persist_synced_scenario(xml_tree, runtime_scenario_path)
    args.scenario = runtime_scenario_path

    data_collector = None
    if args.collect_data:
        scenario_name = os.path.basename(source_scenario_file).replace('.xosc', '')
        data_collector = DataCollector(
            scenario_name=scenario_name,
            output_dir=episode_output_dir,
            scenario_path=source_scenario_file,
            mid_model_path=args.mid_model_path,
            route_file_path=route_output_path,
            record_video=args.record_video,
            video_fps=args.video_fps,
            video_frame_stride=args.video_frame_stride,
            stop_on_collision=not args.continue_after_collision,
            collision_tail_seconds=args.collision_tail_seconds,
        )
        print(f'数据收集器已创建，场景名称: {scenario_name}')

    if xml_tree:
        print('开始运行评估...')
        scenario_runner = None
        try:
            scenario_runner = ScenarioRunner(args, xml_tree)
            if data_collector:
                scenario_runner.set_data_collector(data_collector)
                print('数据收集器已传递给ScenarioRunner')
            scenario_runner.run()
        except Exception:
            traceback.print_exc()
        finally:
            if scenario_runner is not None:
                scenario_runner.destroy()
                del scenario_runner

            if data_collector:
                data_collector.save_data()
                summary = data_collector.get_summary()
                print('数据收集摘要:')
                for key, value in summary.items():
                    print(f'  {key}: {value}')
