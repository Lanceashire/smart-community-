#!/usr/bin/env python3
"""Freeze shared comparison splits; reject exact and very close perceptual duplicates."""
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
NAMES=[f'resident_{i:02}' for i in range(1,17)]+['visitor_01','visitor_02','plate','light_red','light_yellow','light_green']
GROUPS={'full':({i:i for i in range(22)},NAMES), 'person':({i:0 for i in range(18)},['person']),
    'plate':({18:0},['plate']), 'lights':({19:0,20:1,21:2},['red','yellow','green'])}

def phash(path):
    image=cv2.imread(str(path),cv2.IMREAD_GRAYSCALE)
    small=cv2.resize(image,(32,32),interpolation=cv2.INTER_AREA).astype(np.float32)
    dct=cv2.dct(small)[:8,:8].reshape(-1)
    bits=dct>np.median(dct[1:]);bits[0]=False
    return sum(int(b)<<i for i,b in enumerate(bits))

def visible_labels(image,lines):
    """Clip projected objects against the robot's connected yellow foreground hood."""
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV);h,w=image.shape[:2]
    mask=cv2.inRange(hsv,np.array([20,150,60]),np.array([40,255,255]))
    count,components,stats,_=cv2.connectedComponentsWithStats(mask)
    hood=np.zeros((h,w),dtype=bool)
    for i in range(1,count):
        x,y,bw,bh,area=stats[i]
        if y+bh>=h-2 and bw>w*.55 and area>h*w*.035:hood|=components==i
    result=[]
    for line in lines:
        fields=line.split()
        if not fields:continue
        cls,x,y,bw,bh=map(float,fields);x1=max(0,int((x-bw/2)*w));x2=min(w,int((x+bw/2)*w))
        y1=max(0,(y-bh/2)*h);y2=min(h,(y+bh/2)*h)
        if x2>x1:
            region=hood[:,x1:x2];columns=region.any(axis=0)
            if columns.mean()>.5:
                edge=float(np.median(region[:,columns].argmax(axis=0)))
                y2=min(y2,edge)
        if y2-y1<6 or x2-x1<6:continue
        result.append(f'{int(cls)} {(x1+x2)/2/w:.6f} {(y1+y2)/2/h:.6f} {(x2-x1)/w:.6f} {(y2-y1)/h:.6f}')
    return result

def main():
    original=ROOT/'data/community';new=ROOT/'data/comparison';out=ROOT/'data/comparison_prepared'
    if out.exists():raise RuntimeError('Comparison dataset already frozen; use a new version for another experiment')
    kept={s:[] for s in ['train','val','test']};excluded=[];previous={};all_phashes=[]
    for split in kept:
        paths=sorted((original/'images'/split).glob('*.jpg'))
        if split!='train':paths+=sorted((new/'images'/split).glob('*.jpg'))
        within=set();this_phashes=[]
        for p in paths:
            sha=hashlib.sha256(p.read_bytes()).hexdigest();ph=phash(p)
            match=previous.get(sha)
            reason='exact_cross_split' if match else None
            if split!='train' and sha in within:reason='exact_within_split'
            if split!='train' and reason is None:
                near=next((name for code,name in all_phashes if (code^ph).bit_count()<=2),None)
                if near:reason='perceptual_cross_split_distance_le_2';match=near
            if reason:
                excluded.append({'image':str(p.relative_to(ROOT)),'split':split,'reason':reason,'matched':match});continue
            labels=p.parent.parent.parent/'labels'/split/p.with_suffix('.txt').name
            kept[split].append((p,labels,sha,ph));within.add(sha);this_phashes.append((ph,str(p.relative_to(ROOT))))
        for p,_,sha,_ in kept[split]:previous.setdefault(sha,str(p.relative_to(ROOT)))
        all_phashes+=this_phashes
    for group,(mapping,names) in GROUPS.items():
        base=out/group
        for split,items in kept.items():
            for kind in ['images','labels']:(base/kind/split).mkdir(parents=True,exist_ok=True)
            for image,labels,_,_ in items:
                (base/'images'/split/image.name).symlink_to(image)
                lines=[]
                corrected=visible_labels(cv2.imread(str(image)),labels.read_text().splitlines())
                for line in corrected:
                    fields=line.split()
                    if fields and int(fields[0]) in mapping:lines.append(str(mapping[int(fields[0])])+' '+' '.join(fields[1:]))
                (base/'labels'/split/labels.name).write_text('\n'.join(lines)+('\n' if lines else ''))
        (base/'dataset.yaml').write_text(f'path: {base}\ntrain: images/train\nval: images/val\ntest: images/test\nnames:\n'+''.join(f'  {i}: {name}\n' for i,name in enumerate(names)))
    report={'protocol':'Same frozen images per model; original training samples retained; old and expanded validation/test deduplicated without reference to model predictions',
        'perceptual_hash':'64-bit DCT pHash; exclude cross-split Hamming distance <=2; not a guarantee of scene independence',
        'counts':{s:len(v) for s,v in kept.items()},'excluded':excluded,
        'samples':{s:[{'image':str(p.relative_to(ROOT)),'sha256':sha,'phash':format(ph,'016x')} for p,_,sha,ph in v] for s,v in kept.items()},
        'label_correction':'Copied labels are clipped against the connected yellow robot hood; original image/label files remain unchanged',
        'limitations':['Same supplied identities and simulated environment','Automatic geometric labels are not exhaustively manually audited','Validation is for checkpoint selection; test is for final comparison; never train on these images']}
    (ROOT/'validation/comparison-dataset.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({'counts':report['counts'],'excluded':len(excluded)}),flush=True)

if __name__=='__main__':main()
