#!/usr/bin/env python3
"""Plate ROI counterexamples from the supplied blue vehicle camera frames."""
import argparse
import json
import math
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/community_demo'))
from community_demo.plate_geometry import select_plate_target

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);args=parser.parse_args()
    # The old nearest-blue-region rule picked this roof region instead of the
    # real plate in the first failed, unaltered Gazebo inspection frame.
    blue=[{'bbox':[526,655,604,672],'geometry_score':.715},
          {'bbox':[602,438,659,465],'geometry_score':.523}]
    neural=[{'bbox':[515.9668,608.2628,611.7653,637.8728],'confidence':.9084}]
    old=min(blue,key=lambda p:abs((p['bbox'][0]+p['bbox'][2])/2-640))
    selected=select_plate_target(blue,neural,1280,960)
    checks={
        'recorded_blue_roof_false_proposal_is_not_sent_to_ocr':old['bbox']==blue[1]['bbox'] and selected['bbox']==[515,608,612,638],
        'detected_plate_works_when_colour_region_merges_with_car':select_plate_target([],neural,1280,960)['bbox']==selected['bbox'],
        'confidence_retains_neural_probability':selected['confidence_type']=='YOLO probability' and selected['confidence']==.9084,
        'original_colour_only_fallback_retained':select_plate_target(blue,[],1280,960)['confidence_type']=='geometry score, not YOLO probability',
        'no_image_evidence_produces_no_ocr_target':select_plate_target([],[],1280,960) is None,
        'out_of_frame_detection_rejected':select_plate_target([],[{'bbox':[1400,50,1500,90],'confidence':.99}],1280,960) is None,
        'nonfinite_detection_rejected':select_plate_target([],[{'bbox':[100,100,200,130],'confidence':math.nan}],1280,960) is None,
        'invalid_box_shape_rejected':select_plate_target([],[{'bbox':[100,100,200],'confidence':.99}],1280,960) is None,
    }
    report={'passed':all(checks.values()),'checks':checks,
            'scope':'Image-space ROI and confidence counterexamples. Real pixel OCR and three-stage autonomous patrol are verified separately.'}
    output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    return 0 if report['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
