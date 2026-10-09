#!/usr/bin/env python3
"""Counterexamples for early observation completion and readable score semantics."""
import argparse,copy,json,math,os,tempfile
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from PIL import Image,ImageFont
from rclpy.time import Time
from community_demo.patrol import Patrol,OBSERVATION_HEADING
from community_demo.visual_overlay import draw_detection_labels
from community_demo.inspection_policy import signal_hold_reason

def observe(feedback,elapsed=2.,age=.1):
    now=Time(seconds=20.);sink=SimpleNamespace(publish=lambda m:None)
    state=SimpleNamespace(observation_align=True,observation_start=Time(seconds=20.-elapsed),
        observation_index=0,observation_attempt=0,observation_sample=1,sample_start=Time(seconds=19.),
        pose=(-.74,1.7,OBSERVATION_HEADING['observe_A_north']),observation_feedback=feedback,
        feedback_update=Time(seconds=20.-age),pan_pub=sink,observation_pub=sink,inspection_pub=sink,
        publish_command=lambda *args:None,log_status=lambda value:None,camera_frame=np.zeros((48,64,3),np.uint8),
        run_id='counterexample',lap_id=1,observation_outcomes=[],saved_views=[])
    cwd=os.getcwd()
    with tempfile.TemporaryDirectory() as folder:
        try:os.chdir(folder);return Patrol.observe(state,'observe_A_north',now)
        finally:os.chdir(cwd)

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args();checks={}
    feedback={'view':'A_north','attempt':0,'sample':1,'ready':True,'two_angles_verified':True,
              'person_ids':['resident_03'],'pending_person_ids':[],'sample_frames':{'0':5,'1':3},
              'sample_angles':{'0':1.57,'1':1.65},'quiet_s':.9}
    checks['confirmed_two_angle_view_can_finish_before_old_3_8s_timer']=observe(feedback)
    checks['minimum_1_8s_still_required']=not observe(feedback,elapsed=1.7)
    checks['stale_feedback_never_finishes']=not observe(feedback,age=.8)
    for name,key,value in [('wrong_view','view','B_north'),('wrong_sample','sample',0),('pending_identity','pending_person_ids',['visitor_01']),
                           ('no_angle_verification','two_angles_verified',False),('no_person_seen','person_ids',[]),('not_quiet','quiet_s',.3),
                           ('insufficient_frames','sample_frames',{'0':5,'1':2}),('same_angle','sample_angles',{'0':1.57,'1':1.58}),
                           ('nonfinite_angle','sample_angles',{'0':1.57,'1':math.nan})]:
        bad=copy.deepcopy(feedback);bad[key]=value;checks[name+'_cannot_finish']=not observe(bad)
    font=ImageFont.truetype('src/community_demo/models/fonts/wqy-zenhei.ttc',22)
    detections=[{'name':'resident_01','category':'resident','confidence':.87,'confidence_type':'YOLO probability','confirmed':True,'bbox':[40,180,90,300]},
                {'name':'visitor_02','category':'visitor','confidence':.71,'confidence_type':'YOLO probability','confirmed':False,'bbox':[100,180,150,300]},
                {'name':'plate','category':'plate','confidence':.64,'confidence_type':'YOLO probability','bbox':[200,200,300,250]},
                {'name':'blue plate','category':'plate','confidence':.82,'confidence_type':'geometry score, not YOLO probability','ocr_confidence':None,'bbox':[400,200,500,250]}]
    labels=draw_detection_labels(Image.new('RGB',(1280,960)),detections,font)
    checks['yolo_plate_stays_visible_without_overlapping_ocr_proposal']=any(x['name']=='plate' for x in labels)
    checks['geometry_score_not_presented_as_yolo_confidence']=any('几何评分' in x['label'] for x in labels)
    checks['confirmed_and_pending_persons_are_distinguished']=any('已确认' in x['label'] for x in labels) and any('待确认' in x['label'] for x in labels)
    checks['necessary_red_wait_separate_from_green_confirmation']=signal_hold_reason(True,'red',False,None,3.2)=='red' and signal_hold_reason(True,'green',False,12.,3.2)=='confirming_green_frames'
    report={'passed':all(checks.values()),'checks':checks,'scope':'Actual observation method counterexamples; physical patrol and dynamic obstacles checked separately'}
    Path(a.output).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report));return 0 if report['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
