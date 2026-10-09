#!/usr/bin/env python3
"""Reproduce the actual dim-light failure; reject blank/noisy/chromatic false evidence."""
import argparse,json
from pathlib import Path
import cv2,numpy as np
from community_demo.paint_features import white_paint_mask
def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args();checks={}
    source=Path('validation/simulation-refinement-2026-10-06/final-01/dim-light/stalled-frame.jpg')
    image=cv2.imread(str(source));assert image is not None
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV);old=(hsv[:,:,1]<32)&(hsv[:,:,2]>205);old[:int(image.shape[0]*.59)]=False;old[int(image.shape[0]*.88):]=False;old[:,::2]=False
    mask,threshold,contrast=white_paint_mask(image)
    checks['actual_failed_frame_has_no_old_white_paint']=np.count_nonzero(old)==0
    checks['actual_failed_frame_recovers_over_50_paint_pixels']=np.count_nonzero(mask)>50 and threshold<205
    for value in [0,100,198,255]:
        frame=np.full((960,1280,3),value,np.uint8)
        checks['uniform_'+str(value)+'_never_refreshes_paint']=not np.any(white_paint_mask(frame)[0])
    rng=np.random.default_rng(20261006);noise=np.clip(198+rng.normal(0,3,(960,1280,1)),0,255).astype(np.uint8).repeat(3,axis=2)
    checks['gray_sensor_noise_is_not_road_paint']=not np.any(white_paint_mask(noise)[0])
    synthetic=np.zeros((960,1280,3),np.uint8);synthetic[650:700,:,]=255
    normal,t,c=white_paint_mask(synthetic)
    expected=np.zeros((960,1280),bool);expected[650:700,1::2]=True
    checks['bright_scene_keeps_original_205_threshold']=t==205 and np.array_equal(normal,expected)
    synthetic[650:700,:]=[0,0,255]
    checks['bright_coloured_object_not_accepted_as_white_paint']=not np.any(white_paint_mask(synthetic)[0])
    result={'passed':all(checks.values()),'checks':checks,'failed_scene_new_threshold':threshold,'failed_scene_contrast':contrast,
            'old_failed_scene_pixels':int(np.count_nonzero(old)),'new_failed_scene_pixels':int(np.count_nonzero(mask)),
            'scope':'Actual stalled JPEG reproduces brightness failure; live normal/dim laps still required; blank evidence cannot bypass localization quality'}
    Path(a.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result));return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
