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
        plugins = [p for p in world.findall('plugin')
                   if p.get('name') == 'gz::sim::systems::Imu']
        checks['exactly_one_world_imu_system'] = (
            len(plugins) == 1 and plugins[0].get('filename') == 'gz-sim-imu-system')
        sensors = world.findall("model[@name='patrol_bot']/link/sensor[@type='imu']")
        checks['localization_imu_source_present_at_50hz'] = (
            len(sensors) == 1 and sensors[0].findtext('topic') == '/imu/data'
            and float(sensors[0].findtext('update_rate', '0')) == 50
            and sensors[0].findtext('always_on') == 'true')
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
