#!/usr/bin/env python3
"""Independent final-result comparison, never imported by perception runtime."""
import json,argparse
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--report',default='log/perception/report.json');p.add_argument('--output',default='validation/perception-audit.json');a=p.parse_args()
root=Path(__file__).resolve().parents[1];layout=json.loads((root/'src/community_demo/worlds/layout.json').read_text());actual=json.loads((root/a.report).read_text());expected_people={person['asset'] for person in layout['people']};seen=set(actual['people']);events=actual['events'];missing=sorted(expected_people-seen);extra=sorted(seen-expected_people)
# Plate reference transcribed from user-provided source images, evaluation ONLY.
expected_plates={'苏AB8Q62','鄂D7B5Q2','苏APL12A'};plates=set(actual['plates']);images=all((root/'log/perception'/e['image']).exists() for e in events);sequence=[e['sequence'] for e in events]==list(range(1,len(events)+1))
passed=not missing and not extra and plates==expected_plates and images and sequence and actual['status'].startswith('COMPLETE')
report={'passed':passed,'status':actual['status'],'counts':actual['counts'],'expected_people':sorted(expected_people),'seen_people':sorted(seen),'missing_people':missing,'unexpected_people':extra,'plates':sorted(plates),'missing_plates':sorted(expected_plates-plates),'unexpected_plates':sorted(plates-expected_plates),'events_have_images':images,'recognition_sequence_valid':sequence,'runtime_truth_subscription':False,'limits':'One fixed supplied-gallery simulation layout; no real-world result asserted'}
(root/a.output).write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False,indent=2));raise SystemExit(0 if passed else 1)
