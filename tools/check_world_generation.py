#!/usr/bin/env python3
"""Regenerate in isolation and check that the accepted runtime world survives."""
import argparse
import hashlib
import json
from pathlib import Path
import runpy
import tempfile
import xml.etree.ElementTree as E

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    source = ROOT / 'src/community_demo/worlds/community.sdf'
    layout = source.with_name('layout.json')
    module = runpy.run_path(str(ROOT / 'tools/generate_world.py'))
    checks = {}
    with tempfile.TemporaryDirectory(prefix='aic-world-regression-') as directory:
        generated = Path(directory) / 'community.sdf'
        generated.with_name('people_reference.json').write_bytes(
            source.with_name('people_reference.json').read_bytes())
        module['main'].__globals__['OUT'] = generated
        module['main']()
        first = generated.read_bytes()
        generated_layout = json.loads(generated.with_name('layout.json').read_text())
        module['main']()
        checks['regeneration_is_deterministic'] = generated.read_bytes() == first
        checks['regenerated_world_matches_accepted_runtime_bytes'] = first == source.read_bytes()
        checks['regenerated_layout_matches_accepted_layout'] = generated_layout == json.loads(layout.read_text())
        world = E.fromstring(first).find('world')
        task=json.loads((ROOT/'src/community_demo/config/patrol_task.json').read_text())
        spawn=list(map(float,world.findtext("model[@name='patrol_bot']/pose").split()))
        checks['spawn_and_localization_task_agree']=spawn[:2]==task['start'][:2] and abs(spawn[5]-task['start'][2])<1e-9
        checks['start_finish_inside_supplied_upper_right_square']=(task['start'][:2]==task['finish'][:2]==[1.7,1.7] and task['start_bay']==[1.4,1.4,2.,2.])
        checks['final_waypoint_returns_inside_square']=task['waypoints'][-1]['position']==task['start'][:2]
        checks['arrival_follows_right_turn_into_square']=task['finish'][2]==0. and task['waypoints'][-1]['heading']==0.
        plugins = [p for p in world.findall('plugin')
                   if p.get('name') == 'gz::sim::systems::Imu']
        checks['exactly_one_world_imu_system'] = (
            len(plugins) == 1 and plugins[0].get('filename') == 'gz-sim-imu-system')
        sensors = world.findall("model[@name='patrol_bot']/link/sensor[@type='imu']")
        checks['localization_imu_source_present_at_50hz'] = (
            len(sensors) == 1 and sensors[0].findtext('topic') == '/imu/data'
            and float(sensors[0].findtext('update_rate', '0')) == 50
            and sensors[0].findtext('always_on') == 'true')
        wheel = world.find("model[@name='patrol_bot']/plugin[@name='gz::sim::systems::DiffDrive']")
        truth = world.find("model[@name='patrol_bot']/plugin[@name='gz::sim::systems::OdometryPublisher']")
        checks['wheel_odometry_50hz_with_independent_truth_20hz'] = (
            wheel is not None and truth is not None
            and float(wheel.findtext('odom_publish_frequency', '0')) == 50
            and float(truth.findtext('odom_publish_frequency', '0')) == 20)
        recording = E.parse(source.with_name('community_recording.sdf')).getroot().find('world')
        cameras = recording.findall("model[@name='recording_camera']")
        checks['recording_world_has_one_observer_camera'] = len(cameras) == 1
        for camera in cameras:
            recording.remove(camera)
        # Compare XML structure, ignoring serialization whitespace only.
        def structure(element):
            return (element.tag, element.attrib, (element.text or '').strip(),
                    [structure(child) for child in element])
        checks['recording_world_preserves_all_runtime_components'] = structure(recording) == structure(world)
        dim_module=runpy.run_path(str(ROOT/'tools/prepare_dim_world.py'))
        dim_path=generated.with_name('community_dim.sdf')
        dim_module['prepare'](source,dim_path)
        checks['dim_world_regenerated_from_current_source']=dim_path.read_bytes()==source.with_name('community_dim.sdf').read_bytes()
        dim=E.parse(dim_path).getroot().find('world')
        checks['dim_lighting_is_selected_condition']=(dim.findtext('scene/ambient')=='.45 .45 .45 1' and dim.findtext('light/diffuse')=='.65 .65 .65 1')
        dim.find('scene/ambient').text=world.findtext('scene/ambient')
        dim.find('light/diffuse').text=world.findtext('light/diffuse')
        checks['dim_world_preserves_every_nonlighting_component']=structure(dim)==structure(world)
    result = {'passed': all(checks.values()), 'checks': checks,
              'world_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
              'scope': 'Isolated full generation; includes geometry, physics, sensors and recording-world equivalence. Live navigation is checked separately.'}
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
