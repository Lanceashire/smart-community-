#!/usr/bin/env python3
"""Save the real SLAM occupancy map and a separate lane planning grid."""
import json,time
from pathlib import Path
import cv2,numpy as np,rclpy
from nav_msgs.msg import OccupancyGrid
from community_demo.road_rules import in_road
root=Path(__file__).resolve().parents[1]/'src/community_demo/maps';root.mkdir(exist_ok=True)
rclpy.init();n=rclpy.create_node('save_community_maps');state={}
def cb(m):state['map']=m
n.create_subscription(OccupancyGrid,'/map',cb,10);end=time.monotonic()+8
while 'map' not in state and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.2)
if 'map' not in state:raise RuntimeError('No real SLAM map received')
m=state['map'];grid=np.asarray(m.data,dtype=np.int8).reshape(m.info.height,m.info.width);image=np.where(grid<0,205,np.where(grid>65,0,254)).astype(np.uint8);cv2.imwrite(str(root/'slam.pgm'),np.flipud(image))
q=m.info.origin.orientation;yaw=2*np.arctan2(q.z,q.w)
(root/'slam.yaml').write_text(f'image: slam.pgm\nmode: trinary\nresolution: {m.info.resolution}\norigin: [{m.info.origin.position.x}, {m.info.origin.position.y}, {yaw}]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
resolution=.02;size=210;road=np.zeros((size,size),np.uint8)
for row in range(size):
    for col in range(size):road[size-1-row,col]=254 if in_road(-2.1+(col+.5)*resolution,-2.1+(row+.5)*resolution,.012) else 0
cv2.imwrite(str(root/'roads.pgm'),road);(root/'roads.yaml').write_text('image: roads.pgm\nmode: trinary\nresolution: 0.02\norigin: [-2.1, -2.1, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
(root/'map-evidence.json').write_text(json.dumps({'source':'actual /map from SLAM Toolbox, not floor texture','frame':m.header.frame_id,'size':[m.info.width,m.info.height],'resolution':m.info.resolution,'occupied_cells':int((grid>65).sum()),'known_cells':int((grid>=0).sum()),'road_grid':'provided schematic drivable polygons; planning constraint, not physical walls'},indent=2));print((root/'map-evidence.json').read_text());n.destroy_node();rclpy.shutdown()
