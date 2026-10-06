"""Associate standee boxes across a small camera sweep and reject identity flips."""
import math

def overlap(a,b):
    area=max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
    union=(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-area
    return area/union if union>0 else 0.

class PersonEvidence:
    def __init__(self,frame_window=10):self.tracks=[];self.next_id=1;self.frame_window=frame_window

    def update(self,detections,frame_id,pan,width):
        self.tracks=[t for t in self.tracks if frame_id-t['frame']<=self.frame_window]
        # Different YOLO identity classes can overlap the same physical standee.
        boxes=[]
        for d in sorted(detections,key=lambda x:x['confidence'],reverse=True):
            if all(overlap(d['bbox'],b['bbox'])<.6 for b in boxes):boxes.append(d)
        used=set();results=[];f=width/(2*math.tan(.8))
        for d in boxes:
            box=d['bbox'];cx=(box[0]+box[2])/2;cy=(box[1]+box[3])/2
            choices=[]
            for t in self.tracks:
                if t['id'] in used:continue
                old=t['bbox'];ocx=(old[0]+old[2])/2;ocy=(old[1]+old[3])/2
                # Image u increases to the right, opposite to camera-local y.
                # Positive camera yaw therefore moves a fixed target rightward.
                expected=width/2+f*math.tan(math.atan((ocx-width/2)/f)+(pan-t['pan']))
                dx=abs(cx-expected)/max(12.,old[2]-old[0]);dy=abs(cy-ocy)/max(15.,old[3]-old[1])
                if dx<1.1 and dy<.45:choices.append((dx+dy,t))
            if choices:t=min(choices,key=lambda x:x[0])[1]
            else:
                t={'id':self.next_id,'votes':[]};self.next_id+=1;self.tracks.append(t)
            used.add(t['id']);t.update(bbox=box,pan=pan,frame=frame_id)
            t['votes']=[v for v in t['votes'] if frame_id-v[0]<self.frame_window]+[(frame_id,d['name'],d['confidence'])]
            weights={};counts={}
            for _,name,confidence in t['votes']:
                weights[name]=weights.get(name,0.)+confidence;counts[name]=counts.get(name,0)+1
            winner=max(weights,key=weights.get);mean=weights[winner]/counts[winner]
            share=weights[winner]/sum(weights.values())
            confirmed=counts[winner]>=3 and mean>.60 and share>=.75 and d['name']==winner and t.get('identity',winner)==winner
            if confirmed:t.setdefault('identity',winner)
            results.append({**d,'track_id':t['id'],'winner':winner,'confirmed':confirmed,
                            'mean_confidence':mean,'vote_share':share,'votes':counts[winner],
                            'identity_resolved':'identity' in t})
        return results
