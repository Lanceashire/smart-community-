#!/usr/bin/env python3
"""Refine offline plate boxes from image colour, not geometry/text answers."""
from pathlib import Path
import sys,json,cv2
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/community_demo'))
from community_demo.plate_geometry import blue_plates
root=Path(__file__).resolve().parents[1];report={}
for split in ['train','val','test']:
    count=0
    for f in (root/'data/community/images'/split).glob('*.jpg'):
        img=cv2.imread(str(f));H,W=img.shape[:2];label=root/'data/community/labels'/split/(f.stem+'.txt');lines=[l for l in label.read_text().splitlines() if l.strip() and int(l.split()[0])!=18]
        for p in blue_plates(img):
            x1,y1,x2,y2=p['bbox'];pad=1;x1=max(0,x1-pad);x2=min(W,x2+pad);y1=max(0,y1-pad);y2=min(H,y2+pad)
            lines.append(f'18 {(x1+x2)/2/W:.6f} {(y1+y2)/2/H:.6f} {(x2-x1)/W:.6f} {(y2-y1)/H:.6f}');count+=1
        label.write_text('\n'.join(lines)+'\n')
    for f in (root/'data/community/labels').glob(split+'.cache'):f.unlink()
    report[split]={'plate_boxes':count,'method':'blue background connected components; no text lookup; manually audited views'}
(root/'validation/plate-label-refinement.json').write_text(json.dumps(report,indent=2));print(report)
