#!/usr/bin/env python3
"""Run the actual YOLO/OCR callback on saved raw camera pixels, without sim controls."""
import json
from pathlib import Path
import cv2,rclpy
from std_msgs.msg import Header,String
from community_demo.perception import Perception

def main():
    rclpy.init(args=['--ros-args','-p','port:=8876']);n=Perception();n.out=Path('log/reliability/perception-smoke');n.out.mkdir(parents=True,exist_ok=True)
    source=Path('log/inspection/plate_1.png');frame=cv2.imread(str(source))
    report={'passed':False,'source':str(source),'scope':'Real GPU detector, real OCR and actual perception callback on saved raw pixels; not a lap accuracy test'}
    try:
        if frame is None:raise RuntimeError('No saved raw plate camera image')
        n.on_lap(String(data=json.dumps({'run_id':'offline-pixel-smoke','lap_id':1,'phase':'started'})))
        n.on_observation(String(data=json.dumps({'run_id':n.run_id,'lap_id':1,'view':'plate_1','attempt':0,'sample':0,'pan_rad':0.})))
        for _ in range(5):
            n.frame=frame.copy();n.last_stamp=Header(stamp=n.get_clock().now().to_msg());n.last_ocr=0.;n.tick()
        report.update(passed=bool(n.plates) and len(n.jpeg)>1000 and isinstance(n.latest['observation_feedback']['pending_person_ids'],list),plate_texts=list(n.plates),frames=n.frame_id,ocr_votes_recorded=bool(n.view_votes))
    except Exception as e:report['error']=str(e)
    finally:
        n.server.shutdown();n.destroy_node();rclpy.try_shutdown()
    Path('validation/reliability-2026-10-04/perception-smoke.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False));return 0 if report['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
