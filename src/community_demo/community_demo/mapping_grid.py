"""Project a measured SLAM grid; supplied lanes constrain permitted travel."""
import math
import numpy as np
import cv2
from community_demo.road_rules import in_road

RESOLUTION = .02
SIZE = 210
ORIGIN = (-2.1, -2.1)

def field_grid(message, field_to_slam):
    """field_to_slam is a measured/static TF (x,y,yaw), not simulator truth."""
    info=message.info
    data=np.asarray(message.data,dtype=np.int16).reshape(info.height,info.width)
    if not math.isfinite(info.resolution) or info.resolution<=0:
        raise ValueError('Invalid SLAM grid resolution')
    q=info.origin.orientation
    oyaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    y,x=np.mgrid[:SIZE,:SIZE]
    x=ORIGIN[0]+(x+.5)*RESOLUTION;y=ORIGIN[1]+(y+.5)*RESOLUTION
    tx,ty,a=field_to_slam;c,s=math.cos(a),math.sin(a)
    sx=tx+c*x-s*y-info.origin.position.x;sy=ty+s*x+c*y-info.origin.position.y
    c,s=math.cos(oyaw),math.sin(oyaw)
    u=np.floor((c*sx+s*sy)/info.resolution).astype(int)
    v=np.floor((-s*sx+c*sy)/info.resolution).astype(int)
    valid=(u>=0)&(v>=0)&(u<info.width)&(v<info.height)
    output=np.full((SIZE,SIZE),-1,dtype=np.int16);output[valid]=data[v[valid],u[valid]]
    return output

def navigation_grid(measured):
    """Keep unknown road cells unknown; never fill unseen space as free."""
    road=np.array([[in_road(ORIGIN[0]+(x+.5)*RESOLUTION,
                            ORIGIN[1]+(y+.5)*RESOLUTION,.012)
                    for x in range(SIZE)] for y in range(SIZE)])
    occupied=measured>65
    # One measured grid cell of padding around finite physical laser hits.
    padded=cv2.dilate(occupied.astype(np.uint8),np.ones((3,3),np.uint8))>0
    result=np.full((SIZE,SIZE),100,dtype=np.int16)
    result[road]=-1;result[road&(measured>=0)&(measured<=65)]=0
    result[road&padded]=100
    evidence={'road_cells':int(road.sum()),'known_road_fraction':float((road&(measured>=0)).sum()/road.sum()),
              'measured_occupied_road_cells':int((road&occupied).sum()),
              'padded_occupied_road_cells':int((road&padded).sum()),
              'unknown_road_cells':int((road&(result<0)).sum()),
              'source':'Actual SLAM OccupancyGrid + supplied lane polygons; no SDF collision coordinates',
              'unknown_is_free':False,'resolution_m':RESOLUTION,'physical_padding_m':RESOLUTION}
    return result,evidence

def image(grid):
    return np.flipud(np.where(grid<0,205,np.where(grid>65,0,254)).astype(np.uint8))
