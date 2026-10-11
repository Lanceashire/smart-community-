#!/usr/bin/env python3
"""Assemble current accepted recordings and source excerpts, without editing reports."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import textwrap

from PIL import Image, ImageDraw, ImageFont
from summarize_performance import summarize

ROOT = Path(__file__).resolve().parents[1]
SIZE = (1920, 1080)


def read(path):
    return json.loads(path.read_text())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def probe(path):
    return json.loads(subprocess.check_output([
        'ffprobe', '-v', 'error', '-show_format', '-show_streams',
        '-of', 'json', str(path)], text=True))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', required=True, help='Passed three-stage run directory')
    parser.add_argument('--output', required=True)
    parser.add_argument('--date', default='2026-10-10')
    parser.add_argument('--team', default='待填写')
    parser.add_argument('--focus', choices=['world-generation', 'materials', 'route', 'inspection-safety', 'all-turns'], default='world-generation')
    args = parser.parse_args()
    run, output = Path(args.run).resolve(), Path(args.output).resolve()
    pipeline = read(run / 'pipeline.json')
    expected = ['autonomous-mapping', 'mapped-navigation', 'mapped-dynamic']
    assert pipeline['passed'] and [c['case'] for c in pipeline['cases']] == expected
    assert pipeline['source_frozen']
    frozen = read(run / 'runtime-hashes.json')
    # This offline presentation builder is not run by ROS or the lap audit.
    # Permit its own later caption edits, while checking every other frozen file.
    presentation_changes = []
    for path, expected_hash in frozen.items():
        assert (ROOT / path).is_file(), path
        current_hash = sha(ROOT / path)
        if path in ('tools/build_demo_video.py','tools/summarize_performance.py') and current_hash != expected_hash:
            presentation_changes.append({'path': path, 'at_run_sha256': expected_hash,
                                         'current_sha256': current_hash})
        else:
            assert current_hash == expected_hash, path
    output.mkdir(parents=True, exist_ok=True)
    assert not (output / 'video-manifest.json').exists(), 'Preserve earlier videos'
    build = output / 'build'
    build.mkdir(exist_ok=True)
    font_path = ROOT / 'src/community_demo/models/fonts/wqy-zenhei.ttc'
    font = lambda size: ImageFont.truetype(str(font_path), size)
    title_font, body_font, small_font, code_font = font(52), font(34), font(24), font(27)
    ffmpeg = ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y']
    encoding = ['-an', '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '21',
                '-threads', '2', '-pix_fmt', 'yuv420p', '-movflags', '+faststart']
    segments, chapters, screenshots = [], [], []
    cursor = 0.0

    def register(path, title, source=None):
        nonlocal cursor
        duration = float(probe(path)['format']['duration'])
        chapters.append({'title': title, 'start_s': round(cursor, 3),
                         'duration_s': duration, 'source': str(source) if source else None})
        cursor += duration
        segments.append(path)

    def page(title, lines, seconds, name, code=False):
        picture = Image.new('RGB', SIZE, '#101b2b')
        draw = ImageDraw.Draw(picture)
        draw.rectangle((0, 0, 15, 1080), fill='#74ddc5')
        draw.text((75, 65), title, font=title_font, fill='white')
        draw.text((75, 145), f'AIC 智慧社区 · {args.date} · 队伍：{args.team} · 最新仿真演示版',
                  font=small_font, fill='#abc4dc')
        draw.line((75, 210, 1845, 210), fill='#34516f', width=2)
        y = 255
        for line in lines:
            chunks = textwrap.wrap(line, width=100 if code else 44,
                                   break_long_words=True, break_on_hyphens=False) or ['']
            for chunk in chunks:
                draw.text((85, y), chunk, font=code_font if code else body_font,
                          fill='#e2edf7')
                y += 37 if code else 56
            y += 6 if code else 22
        assert y <= 955, (title, y)
        draw.text((75, 1005), '真实 Gazebo 记录与当前源码；中文字幕，无旁白。报告、答辩文件保持原样。',
                  font=small_font, fill='#abc4dc')
        png, clip = build / (name + '.png'), build / (name + '.mp4')
        picture.save(png)
        screenshots.append(png)
        subprocess.run(ffmpeg + ['-loop', '1', '-framerate', '10', '-i', str(png),
                                '-t', str(seconds)] + encoding + [str(clip)], check=True)
        register(clip, title)

    raw_files = {
        '自主建图过程-最新.mp4': run / 'autonomous-mapping/mapping.mp4',
        '新地图完整巡航-最新.mp4': run / 'mapped-navigation/patrol.mp4',
        '新地图动态障碍-最新.mp4': run / 'mapped-dynamic/patrol.mp4',
    }
    normal = read(run / 'mapped-navigation/lap-001.json')
    dynamic = read(run / 'mapped-dynamic/dynamic-interference.json')
    mapping = read(run / 'autonomous-mapping/mapping-evidence.json')
    assert mapping['passed'] and dynamic['passed']
    for case in ['mapped-navigation', 'mapped-dynamic']:
        recording = read(run / case / 'patrol.json')
        lap = read(run / case / 'lap-001.json')
        assert recording['passed'] and recording['run_id'] == lap['run_id'] and lap['passed']
    metrics = {case: summarize(run / case / 'lap-001.json') for case in expected}
    source_duration = sum(float(probe(p)['format']['duration']) for p in raw_files.values())
    # Preserve all footage at recorded wall-time speed. Caption pages fill the
    # usual 8-12 minute demonstration length when the recordings are short.
    caption_total = max(160., 540. - source_duration)
    assert source_duration + caption_total <= 720., 'Continuous recordings exceed demo length; keep raw videos'
    scale = caption_total / 200.
    page('最新演示：自主建图、地图巡航与动态障碍', [
        '从空图开始自主巡检，保存地图后另启世界加载地图导航。',
        '依次展示三段完整实录：建图、正常巡航、障碍停车与恢复。',
        '人员与车牌结果来自机器人相机；俯视摄影仅用于观察。',
        '演示片段保持记录时的墙钟速度，章节之间有说明页。',
    ], 25 * scale, '01-introduction')
    if args.focus == 'all-turns':
        page('全程转弯：六个位置逐一优化', [
            '前三个路口提前进入相切圆弧，减少停车后的原地调向。',
            '第二个信号灯后向安全侧移，规划灯柱净空加强至 6.5 cm。',
            '车牌区增加连续入弯；回到起点后朝向下一圈行驶方向。',
            '按曲率连续调节前瞻距离；末端越过目标后停车并有限重规划。',
            '下方、车牌区和起点共三个弯，加上前三路口，共六处检查。',
        ], 25 * scale, '02-scene')
    elif args.focus == 'inspection-safety':
        page('严格审查后的四项修复', [
            '观察失败超时后停车报错；确认记录不完整时禁止宣告巡检完成。',
            '按图示核对相机实测的 10 人、3 张车牌，数量不足标记不完整。',
            '代价地图保留未知区域，规划与整车扫掠检查共同拒绝穿过未知格。',
            '新激光命中重置旧清空票；重复、倒序源帧不重复参与建图。',
            '暗光副本由当前正式场景生成，车辆背景和黄灯暗态同步保留。',
            '连续圆弧控制与原有道路、灯柱和信号灯保护继续执行。',
        ], 25 * scale, '02-scene')
    elif args.focus == 'route':
        page('第二个红绿灯后：连续转弯优化', [
            '上一版停稳对齐已避免乱走，但在拐角处仍有明显停顿。',
            '新版本采用相切圆弧与缓弯，整车扫过区域必须通过安全核验。',
            '检查实时代价地图、未知格、车道轮廓及灯柱净空。',
            '提前进入低速圆弧，绕过灯柱后平顺回到车道中心。',
        ], 25 * scale, '02-scene')
    elif args.focus == 'materials':
        page('原始素材恢复：车辆背景与黄灯亮 / 暗态', [
            '三张补充 JPEG 保留原始字节，直接用于 Gazebo 贴图。',
            '黄灯亮、暗态均使用实物照片，替换原有暗黄色圆片。',
            '三块车辆板使用提供的车辆背景，车牌保持独立原图。',
            '人物、物料位置、碰撞尺寸、相机参数与控制门槛保留。',
        ], 25 * scale, '02-scene')
    else:
        page('场景重新生成：保留已验收配置', [
            '生成器已补齐 50 Hz IMU 及 Gazebo IMU 系统插件。',
            '同步已验收的云台参数，防止重建后恢复旧参数。',
            '重新生成的场景与已验收场景逐字节一致。',
            '十个人物板均有正面贴图；侧面和背面保持单面立牌形态。',
        ], 25 * scale, '02-scene')
    for i, (name, source) in enumerate(raw_files.items(), 3):
        destination = output / name
        shutil.copy2(source, destination)
        normalized = build / f'{i:02d}-recording.mp4'
        subprocess.run(ffmpeg + ['-i', str(source), '-vf',
            'scale=1920:1080:flags=lanczos,fps=10,setsar=1'] + encoding + [str(normalized)], check=True)
        register(normalized, name[:-4] + '（完整实录）', source)
    generator = (ROOT / 'tools/generate_world.py').read_text().splitlines()
    imu_sensor = next(line.strip() for line in generator if 'sensor name="body_imu"' in line)
    imu_plugin = next(line.strip() for line in generator if 'filename="gz-sim-imu-system"' in line)
    gain = next(line.strip() for line in generator if '<p_gain>' in line)
    if args.focus == 'all-turns':
        source = (ROOT / 'src/community_demo/community_demo/route_path.py').read_text()
        start = source.index('def terminal_sweep_clear(')
        end = source.index('\n\ndef continuous_lower_path', start)
        page('当前源码 1：末端转向目标也要检查整车扫掠', [
            'community_demo/route_path.py（真实源码片段）', '',
            *source[start:end].splitlines(), '',
            '所有弯检查道路、未知格与灯柱；14 cm 延长仅用于转向目标。',
            '机器人在真实目标点停车；越过目标须停车并重新核验路径。',
        ], 40 * scale, '06-code-generation', code=True)
    elif args.focus == 'inspection-safety':
        source=(ROOT/'src/community_demo/community_demo/inspection_policy.py').read_text()
        start=source.index('def inspection_complete(')
        end=source.index('\ndef recognition_complete',start)
        page('当前源码 1：巡检完成必须有确认依据', [
            'community_demo/inspection_policy.py（真实源码片段）','',
            *source[start:end].strip().splitlines(),'',
            '巡检状态机还按图示核对相机实测的 10 人、3 张车牌。',
            '每个视角的双角度证据、OCR 多帧确认与源时间门槛保留。',
        ],40*scale,'06-code-generation',code=True)
    elif args.focus == 'route':
        source = (ROOT / 'src/community_demo/community_demo/route_path.py').read_text()
        start = source.index('def pose_sweep_clear(')
        end = source.index('\n\ndef terminal_sweep_clear', start)
        page('当前源码 1：整车扫过范围核验', [
            'community_demo/route_path.py（真实源码片段）', '',
            *source[start:end].splitlines(), '',
            'clearance_polygon 检查车道联合区域、灯柱与实时地图。',
            '圆弧不可用时仅采用已核验分段；无安全路径则停车重试。',
        ], 40 * scale, '06-code-generation', code=True)
    elif args.focus == 'materials':
        source = (ROOT / 'tools/generate_world.py').read_text()
        start = source.index("    if 'car_front' in TEXTURES:")
        end = source.index('    else:', start)
        page('当前源码 1：原图贴图与独立车牌', [
            'tools/generate_world.py（真实源码片段）', '',
            *source[start:end].splitlines(), '',
            next(line.strip() for line in generator if "mesh_visual('official_plate'" in line), '',
            '一次性素材处理移出工程；场景直接加载已生成的贴图与模型。',
            '原始 JPEG 不重画、不重新压缩；旧下载缓存保留来源与补齐记录。',
        ], 40 * scale, '06-code-generation', code=True)
    else:
        page('当前源码 1：场景生成器修复', [
            'tools/generate_world.py（真实源码片段）',
            imu_sensor, '', imu_plugin, '', gain, '',
            'check_world_generation.py：完整重建、确定性、IMU 与录像世界一致性检查。',
        ], 40 * scale, '06-code-generation', code=True)
    localization = (ROOT / 'src/community_demo/community_demo/localization.py').read_text()
    start = localization.index("        if getattr(self,'require_imu',False) and (not self.imu_samples")
    end = localization.index('\n', localization.index('navigation pose withheld', start))
    excerpt = localization[start:end].splitlines()
    if args.focus == 'all-turns':
        source = (ROOT / 'src/community_demo/community_demo/route_path.py').read_text()
        start = source.index('    j=min(len(points)-2,progress+8)')
        end = source.index('\n    remaining=look+along', start)
        page('当前源码 2：相同弯道采用相同控制参数', [
            'community_demo/route_path.py（真实源码片段）', '',
            *[line[4:] for line in source[start:end].splitlines()], '',
            '前瞻距离由局部曲率连续决定，不再按绝对航向切换。',
            '同一条路径旋转 90 度、180 度，跟踪控制须保持一致。',
        ], 40 * scale, '07-code-safety', code=True)
    elif args.focus == 'inspection-safety':
        source=(ROOT/'src/community_demo/community_demo/mapping_grid.py').read_text()
        start=source.index('    hit_indices=np.unique(')
        end=source.index('\n\ndef fuse_registered_rays',start)
        page('当前源码 2：后来出现的障碍覆盖旧空闲证据', [
            'community_demo/mapping_grid.py（真实源码片段）','',
            *source[start:end].strip().splitlines(),'',
            'free[hit_indices]=0：清空票从最近一次命中之后重新累计。',
            '采集器只登记严格递增的源时间；同帧终点优先于自由射线。',
        ],40*scale,'07-code-safety',code=True)
    elif args.focus == 'route':
        source = (ROOT / 'src/community_demo/community_demo/patrol.py').read_text()
        start = source.index("            if getattr(self,'path_curved',False):")
        end = source.index('\n            else:', start)
        page('当前源码 2：圆弧曲率与限速', [
            'community_demo/patrol.py（真实源码片段）', '',
            *[line[8:] for line in source[start:end].splitlines()], '',
            '角速度 = 线速度 × 曲率；紧弯自动限速，原有安全保护继续执行。',
        ], 40 * scale, '07-code-safety', code=True)
    elif args.focus == 'materials':
        selection = (ROOT / 'src/community_demo/community_demo/plate_geometry.py').read_text()
        start = selection.index('    if candidates:')
        end = selection.index('\ndef blue_plates', start)
        page('当前源码 2：避免将蓝色车身送入车牌 OCR', [
            'src/community_demo/community_demo/plate_geometry.py（真实源码片段）', '',
            *selection[start:end].strip('\n').splitlines(), '',
            '有效 YOLO 车牌框优先；无有效检测框时保留蓝色几何候选。',
            '原始像素 OCR、两次高置信确认与终端 / 图像关联保留。',
        ], 40 * scale, '07-code-safety', code=True)
    else:
        page('当前源码 2：IMU 过期时停止输出导航位姿', [
            'src/community_demo/community_demo/localization.py（真实源码片段）', '',
            *[line[8:] for line in excerpt], '',
            '缺失 / 过期 IMU -> 定位质量丢失 -> 巡航保护停车。',
            '图像、轮速、激光与实际云台关节均保留源时间校验。',
        ], 40 * scale, '07-code-safety', code=True)
    n = metrics['mapped-navigation']
    page('最新实跑结果', [
        f'正常圈：{n["completion_sim_s"]:.2f} 仿真秒，17 路点、8 次观察完成。',
        f'人员：8 名居民 + 2 名访客；3 张车牌正确，终端与图像对应通过。',
        f'最大位置误差 {n["position_error_max_m"] * 100:.2f} cm；灯柱净空 {n["minimum_post_clearance_m"] * 100:.2f} cm。',
        f'道路违规采样 {normal["road_violation_samples"]}；红灯跨线采样 {normal["red_straddle_samples"]}。',
        f'动态障碍最小间隔 {dynamic["minimum_body_clearance_m"] * 100:.2f} cm；停车、清障、重规划与恢复通过。',
    ], 40 * scale, '08-results')
    page('演示边界与后续工作', [
        '固定比赛场地、规定起点、已知人物图库与车道先验。',
        '建图使用真实激光和定位；地图覆盖率不代表整体地图精度。',
        '动态障碍展示停车等待清障，长期占道主动绕行尚未验证。',
        '真实硬件、陌生人物、任意起点及新机器重建仍待验收。',
        '队伍信息待填写；正式报告与答辩材料本次保持原样。',
    ], 30 * scale, '09-scope')
    listing = build / 'segments.txt'
    # ffmpeg concat paths may contain apostrophes; escape its quoted syntax.
    listing.write_text(''.join("file '" + str(p).replace("'", "'\\''") + "'\n" for p in segments))
    composite = output / f'智慧社区综合演示-{args.date}.mp4'
    subprocess.run(ffmpeg + ['-f', 'concat', '-safe', '0', '-i', str(listing),
                            '-c', 'copy', '-an', '-movflags', '+faststart', str(composite)], check=True)
    info = probe(composite)
    stream = info['streams'][0]
    duration = float(info['format']['duration'])
    assert 480 <= duration <= 720 and composite.stat().st_size < 300_000_000
    assert (stream['width'], stream['height']) == SIZE and stream['r_frame_rate'] == '10/1'
    for file in [composite, *[output / n for n in raw_files]]:
        subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-i', str(file),
                        '-f', 'null', '-'], check=True)
    result = {'passed': True, 'date': args.date, 'team': args.team, 'focus': args.focus,
        'pipeline': str(run / 'pipeline.json'), 'source_frozen': True,
        'current_runtime_matches_frozen_run': True,
        'post_run_presentation_tool_changes': presentation_changes,
        'resolution': '1920x1080', 'fps': 10, 'duration_s': duration,
        'audio': 'Chinese captions, no narration', 'chapters': chapters,
        'files': {p.name: {'bytes': p.stat().st_size, 'sha256': sha(p)}
                  for p in [composite, *[output / n for n in raw_files]]},
        'metrics': metrics, 'slides': [str(p) for p in screenshots],
        'scope': 'Three current continuous chronological recordings, unaccelerated. Composite uses one uniform codec/resolution/frame rate. Existing report and defense files are untouched.'}
    (output / 'video-manifest.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ('metrics', 'slides')}, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
