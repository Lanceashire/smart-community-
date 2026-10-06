"""YOLO11 + RapidOCR on live ROS camera frames; synchronized visual/JSON output.

This node never reads layout geometry, ground truth, or simulator identities.
The identity classes describe the supplied gallery, not unknown real people.
"""
import json,re,time,threading
from collections import Counter,deque
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import cv2,numpy as np,rclpy,torch
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy,DurabilityPolicy
from sensor_msgs.msg import Image,JointState,CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ultralytics import YOLO
from rapidocr import RapidOCR
from community_demo.plate_geometry import blue_plates
from community_demo.visual_overlay import draw_detection_labels
from community_demo.person_evidence import PersonEvidence
from community_demo.sensor_validation import valid_rgb_image,pan_sample,stamp_ns
from PIL import Image as PILImage,ImageDraw,ImageFont

class Perception(Node):
    def __init__(self):
        super().__init__('community_yolo_ocr')
        cv2.setNumThreads(2);torch.set_num_threads(2)
        self.declare_parameter('model','src/community_demo/models/perception/yolo11s-comparison.pt');self.declare_parameter('port',8765)
        self.declare_parameter('processing_hz',10.)
        self.processing_hz=float(self.get_parameter('processing_hz').value)
        if not 5<=self.processing_hz<=15:raise ValueError('processing_hz must be between 5 and 15')
        self.model=YOLO(self.get_parameter('model').value)
        self.ocr=RapidOCR(params={'Global.model_root_dir':'src/community_demo/models/ocr','EngineConfig.onnxruntime.intra_op_num_threads':2,'EngineConfig.onnxruntime.inter_op_num_threads':2})
        self.out=Path('log/perception');self.out.mkdir(parents=True,exist_ok=True)
        try:self.previous_report=json.loads((self.out/'report.json').read_text())
        except (OSError,ValueError):self.previous_report={}
        self.create_subscription(Image,'/camera/image',self.on_camera,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(String,'/patrol/status',self.on_status,qos_profile_sensor_data)
        self.create_subscription(String,'/patrol/lap',self.on_lap,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String,'/inspection/observation',self.on_observation,qos_profile_sensor_data)
        self.results=self.create_publisher(String,'/perception/results',10);self.light=self.create_publisher(String,'/perception/traffic_light',10);self.stamped_light=self.create_publisher(String,'/perception/traffic_light_stamped',10)
        self.images=self.create_publisher(Image,'/perception/image',qos_profile_sensor_data)
        self.compressed_images=self.create_publisher(CompressedImage,'/perception/image/compressed',qos_profile_sensor_data)
        self.last_raw_image_time=-1e9;self.previous_cycle_ms=None
        self.frame=None;self.last_stamp=None;self.processed=None;self.jpeg=b'';self.people={};self.plates={};self.events=[];self.votes={};self.light_history=deque(maxlen=3);self.last_ocr=0.;self.status='WAIT';self.view='transit';self.completed=False;self.last_light_event='unknown';self.frame_id=0;self.latest={}
        self.run_id='pending';self.lap_id=0;self.lap_active=False
        self.view_people={};self.view_plates={};self.view_votes={};self.last_observation_outcomes=[]
        self.person_evidence=PersonEvidence(frame_window=round(self.processing_hz*2));self.pan_samples=deque(maxlen=256)
        self.view_sample=0;self.view_pan=0.;self.sample_frames={};self.sample_angles={}
        self.create_subscription(JointState,'/world/community/model/patrol_bot/joint_state',self.on_joint,qos_profile_sensor_data)
        self.create_subscription(String,'/inspection/outcome',self.on_outcome,qos_profile_sensor_data)
        fonts=['src/community_demo/models/fonts/wqy-zenhei.ttc','/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc','/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc']
        self.font=next((ImageFont.truetype(f,22) for f in fonts if Path(f).exists()),ImageFont.load_default())
        self.view_key=None;self.view_started=0.;self.view_frames=0;self.evidence_signature=None;self.evidence_changed=0.
        self.create_timer(1./self.processing_hz,self.tick)
        self.restart_client=self.create_client(Trigger,'/patrol/restart')
        node=self
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path.startswith('/frame.jpg'):
                    data=node.jpeg;typ='image/jpeg'
                elif self.path.startswith('/state'):
                    data=json.dumps(node.latest,ensure_ascii=False).encode();typ='application/json; charset=utf-8'
                else:
                    data=('''<!doctype html><meta charset="utf-8"><title>AIC 机器人实时验收</title><style>body{background:#111827;color:#eee;font:18px sans-serif;margin:24px}main{display:grid;grid-template-columns:3fr 1fr;gap:24px}img{width:100%;border-radius:12px}pre{white-space:pre-wrap;background:#1f2937;padding:20px;border-radius:12px}h1{font-size:26px}</style><h1>AIC 智慧社区 · 机器人真实相机</h1><button onclick="restart()" style="font-size:18px;padding:10px 18px;margin-bottom:18px">再巡航一圈</button><span id="restart-message"></span><main><img id="camera"><pre id="state"></pre></main><script>async function restart(){let r=await(await fetch('/restart',{method:'POST'})).json();document.getElementById('restart-message').textContent=r.message;}setInterval(async()=>{document.getElementById('camera').src='/frame.jpg?t='+Date.now();let s=await(await fetch('/state')).json();if(!s.counts){document.getElementById('state').textContent='等待机器人相机';return;}document.getElementById('state').textContent=s.status+'\\n\\n社区人员：'+s.counts.residents+'\\n非社区人员：'+s.counts.visitors+'\\n去重总人数：'+s.counts.total+'\\n\\n车牌：\\n'+Object.keys(s.plates).join('\\n')+'\\n\\n未确认观察点：'+((s.unconfirmed_views||[]).join('、')||'无')+'\\n\\n灯色：'+s.traffic_light+'\\n画面编号：'+s.frame_id;},350)</script>''').encode();typ='text/html; charset=utf-8'
                self.send_response(200);self.send_header('Content-Type',typ);self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(data)
            def do_POST(self):
                if self.path!='/restart':self.send_error(404);return
                result={'success':False,'message':'巡航服务尚未就绪'}
                if node.restart_client.wait_for_service(timeout_sec=2):
                    future=node.restart_client.call_async(Trigger.Request());deadline=time.monotonic()+8
                    while not future.done() and time.monotonic()<deadline:time.sleep(.05)
                    if future.done():result={'success':future.result().success,'message':future.result().message}
                data=json.dumps(result,ensure_ascii=False).encode();self.send_response(200);self.send_header('Content-Type','application/json; charset=utf-8');self.end_headers();self.wfile.write(data)
            def log_message(self,*args):pass
        self.server=ThreadingHTTPServer(('127.0.0.1',int(self.get_parameter('port').value)),Handler);threading.Thread(target=self.server.serve_forever,daemon=True).start()
        self.get_logger().info('Live GPU YOLO / pixel OCR ready: http://127.0.0.1:8765')
    def on_camera(self,m):
        if not valid_rgb_image(m):return
        source_ns=stamp_ns(m.header.stamp);age_ns=self.get_clock().now().nanoseconds-source_ns
        if age_ns < -50_000_000 or age_ns > 500_000_000:return
        if self.last_stamp is not None and source_ns<=stamp_ns(self.last_stamp.stamp):return
        a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
        self.frame=cv2.cvtColor(a,cv2.COLOR_RGB2BGR) if m.encoding=='rgb8' else a.copy();self.last_stamp=m.header
    def on_joint(self,m):
        sample=pan_sample(m)
        if sample is None:return
        if self.pan_samples and sample[0]<=self.pan_samples[-1][0]:return
        self.pan_samples.append(sample)

    def on_status(self,m):
        # Lifecycle messages delimit laps even if old status messages are queued.
        if self.lap_active and m.data.startswith('COMPLETE'):return
        self.status=m.data
        if m.data.startswith('COMPLETE'):self.completed=True
    def on_lap(self,m):
        data=json.loads(m.data)
        key=(data['run_id'],int(data['lap_id']))
        if key!=(self.run_id,self.lap_id):
            self.run_id,self.lap_id=key
            self.people={};self.plates={};self.events=[];self.votes={};self.light_history.clear()
            self.last_light_event='unknown';self.last_ocr=0.;self.view='transit';self.completed=False
            self.view_people={};self.view_plates={};self.view_votes={};self.last_observation_outcomes=[]
            self.person_evidence=PersonEvidence(frame_window=round(self.processing_hz*2));self.sample_frames={};self.sample_angles={}
            self.status=f'START lap={self.lap_id}'
            if data['phase']=='completed':
                try:
                    saved=self.previous_report
                    if (saved.get('run_id'),saved.get('lap_id'))==key and saved.get('status','').startswith('COMPLETE'):
                        self.people=saved['people'];self.plates=saved['plates'];self.events=saved['events']
                        self.frame_id=saved['frame_id'];self.status=saved['status']
                        self.last_observation_outcomes=saved.get('observation_outcomes',[])
                except (OSError,ValueError,KeyError):pass
        self.lap_active=data['phase']=='started'
        if not self.lap_active:self.completed=True
    def on_observation(self,m):
        try:
            data=json.loads(m.data)
            if data.get('view')!='transit' and (data.get('run_id'),data.get('lap_id'))!=(self.run_id,self.lap_id):return
            view=data.get('view','transit')
            key=(view,data.get('attempt',0),self.run_id,self.lap_id)
            if key!=self.view_key:
                self.view_people={};self.view_plates={};self.view_votes={}
                self.view_key=key;self.view_frames=0;self.evidence_signature=None
                self.person_evidence=PersonEvidence(frame_window=round(self.processing_hz*2));self.sample_frames={};self.sample_angles={}
                self.view_started=self.get_clock().now().nanoseconds/1e9;self.evidence_changed=self.view_started
            self.view=view
            self.view_sample=int(data.get('sample',0));self.view_pan=float(data.get('pan_rad',0.))
        except ValueError:pass
    def on_outcome(self,m):
        try:data=json.loads(m.data)
        except ValueError:return
        if (data.get('run_id'),data.get('lap_id'))!=(self.run_id,self.lap_id):return
        self.last_observation_outcomes.append(data)
    def event(self,kind,key,confidence,bbox):
        item={'run_id':self.run_id,'lap_id':self.lap_id,'sequence':len(self.events)+1,'frame_id':self.frame_id,'stamp':self.last_stamp.stamp.sec+self.last_stamp.stamp.nanosec*1e-9,'category':kind,'content':key,'confidence':round(float(confidence),4),'bbox':list(map(int,bbox)),'image':f'{self.run_id}-lap-{self.lap_id:03d}-event-{len(self.events)+1:03d}.jpg'}
        self.events.append(item);self.get_logger().info(json.dumps(item,ensure_ascii=False));return item
    def tick(self):
        if self.frame is None or self.last_stamp==self.processed:return
        header=self.last_stamp;frame=self.frame.copy();self.processed=header;self.frame_id+=1
        image_time=header.stamp.sec+header.stamp.nanosec/1e9
        if not 0<=self.get_clock().now().nanoseconds-stamp_ns(header.stamp)<=500_000_000:
            self.light_history.clear();self.light.publish(String(data='unknown'));self.stamped_light.publish(String(data=json.dumps({'colour':'unknown','source_stamp_ns':stamp_ns(header.stamp),'frame_id':self.frame_id})));return
        begin=time.perf_counter();result=self.model.predict(frame,imgsz=960,conf=.35,iou=.45,device=0,verbose=False)[0];detections=[];light_candidates=[];ocr_due=time.monotonic()-self.last_ocr>.6;new_events=[]
        encoder=min(self.pan_samples,key=lambda x:abs(x[0]-image_time)) if self.pan_samples else None
        pan=encoder[1] if encoder else self.view_pan
        stable_pan=encoder is not None and abs(encoder[0]-image_time)<=.10 and abs(encoder[2])<.15 and abs(pan-self.view_pan)<.035
        person_boxes=[]
        for box in result.boxes:
            name=result.names[int(box.cls.item())]
            if name.startswith(('resident_','visitor_')):
                coords=list(map(int,box.xyxy[0].cpu().numpy()));coords=[max(0,coords[0]),max(0,coords[1]),min(frame.shape[1],coords[2]),min(frame.shape[0],coords[3])]
                person_boxes.append({'name':name,'confidence':float(box.conf.item()),'bbox':coords})
        # Reject identity votes while the camera sweeps between observation views.
        person_tracks=(self.person_evidence.update(person_boxes,self.frame_id,pan,frame.shape[1])
                       if self.view=='transit' or stable_pan else [])
        tracked={(e['name'],tuple(e['bbox'])):e for e in person_tracks}
        pixel_plates=blue_plates(frame,min_width=42)
        plate_boxes=[b for b in result.boxes if result.names[int(b.cls.item())]=='plate']
        chosen_plate=min(plate_boxes,key=lambda b:abs(float((b.xyxy[0,0]+b.xyxy[0,2])/2)-frame.shape[1]/2),default=None)
        for box in result.boxes:
            cls=int(box.cls.item());name=result.names[cls];conf=float(box.conf.item());coords=box.xyxy[0].cpu().numpy().astype(int);x1,y1,x2,y2=map(int,coords);H,W=frame.shape[:2];x1=max(0,x1);y1=max(0,y1);x2=min(W,x2);y2=min(H,y2)
            if x2<=x1 or y2<=y1:continue
            text=name;kind='object';color=(255,190,30)
            if name.startswith(('resident_','visitor_')):
                kind='resident' if name.startswith('resident') else 'visitor';color=(60,220,80) if kind=='resident' else (60,110,255)
                # Require repeated visual identity evidence, not one-frame classes.
                track=tracked.get((name,(x1,y1,x2,y2)))
                confirmed=track is not None and track['confirmed']
                if self.view!='transit' and stable_pan and confirmed:self.view_people[name]=self.frame_id
                if self.lap_active and confirmed and name not in self.people:
                    event=self.event(kind,name,track['mean_confidence'],[x1,y1,x2,y2]);self.people[name]=event;new_events.append(event)
                    conf=float(event['confidence'])
            elif name=='plate':
                kind='plate';color=(255,160,60)
                # Pixel-boundary refinement and OCR are performed once below.
            elif name.startswith('light_'):
                kind='traffic_light';colour=name[6:];text=colour;light_candidates.append(((x2-x1)*(y2-y1),colour,conf,[x1,y1,x2,y2]))
            detections.append({'category':kind,'name':text,'confidence':round(conf,4),'bbox':[x1,y1,x2,y2]});cv2.rectangle(frame,(x1,y1),(x2,y2),color,2)
        if self.view.startswith('plate_') and pixel_plates:
            target=min(pixel_plates,key=lambda p:abs((p['bbox'][0]+p['bbox'][2])/2-frame.shape[1]/2))
            x1,y1,x2,y2=target['bbox'];text='blue plate';ocr_score=None
            if ocr_due:
                H,W=frame.shape[:2];crop=self.frame[max(0,y1-2):min(H,y2+2),max(0,x1-2):min(W,x2+2)]
                crop=cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC);read=self.ocr(crop,use_det=False,use_cls=False)
                if read and read.txts:
                    candidate=re.sub(r'[\s·•.\-]','',read.txts[0]).upper();score=float(read.scores[0]);text=candidate;ocr_score=score
                    if re.fullmatch(r'[\u4e00-\u9fff][A-Z][A-Z0-9]{5}',candidate) and score>.93:
                        history=self.votes.setdefault('plate:'+candidate,deque(maxlen=8));history.append((self.frame_id,score))
                        recent=[c for f,c in history if self.frame_id-f<round(self.processing_hz*4)]
                        evidence=self.view_votes.setdefault('plate:'+candidate,deque(maxlen=8));evidence.append((self.frame_id,score))
                        if len([c for f,c in evidence if self.frame_id-f<round(self.processing_hz*4)])>=2:self.view_plates[candidate]=self.frame_id
                        if self.lap_active and len(recent)>=2 and candidate not in self.plates:
                            event=self.event('plate',candidate,score,[x1,y1,x2,y2]);self.plates[candidate]=event;new_events.append(event)
            detections.append({'category':'plate','name':text,'confidence':target['geometry_score'],'confidence_type':'geometry score, not YOLO probability','ocr_confidence':ocr_score,'bbox':target['bbox'],'method':'YOLO plus blue-boundary proposals / pixel OCR'})
            cv2.rectangle(frame,(x1,y1),(x2,y2),(70,240,255),2)
        if ocr_due:self.last_ocr=time.monotonic()
        state=max(light_candidates,default=(0,'unknown',0))[1];self.light_history.append(state);stable=state if len(self.light_history)==3 and len(set(self.light_history))==1 else 'unknown';age=self.get_clock().now().nanoseconds/1e9-(header.stamp.sec+header.stamp.nanosec*1e-9)
        if age>.5:stable='unknown'
        self.light.publish(String(data=stable))
        self.stamped_light.publish(String(data=json.dumps({'colour':stable,'source_stamp_ns':stamp_ns(header.stamp),'frame_id':self.frame_id})))
        if self.lap_active and stable!='unknown' and stable!=self.last_light_event:
            selected=max((c for c in light_candidates if c[1]==stable),key=lambda c:c[0])
            new_events.append(self.event('traffic_light',stable,selected[2],selected[3]))
            self.last_light_event=stable
        counts={'residents':sum(k.startswith('resident') for k in self.people),'visitors':sum(k.startswith('visitor') for k in self.people),'total':len(self.people)}
        pending=[e['winner'] for e in person_tracks if not e['identity_resolved'] and e['votes']>=2 and e['mean_confidence']>.45]
        view_confirmed=bool(self.view_plates) if self.view.startswith('plate_') else bool(self.view_people) and not pending
        now_sim=self.get_clock().now().nanoseconds/1e9
        self.view_frames+=1
        if stable_pan and age<=.5 and self.view!='transit':
            self.sample_frames[self.view_sample]=self.sample_frames.get(self.view_sample,0)+1
            self.sample_angles[self.view_sample]=pan
        two_angles=(self.sample_frames.get(0,0)>=5 and self.sample_frames.get(1,0)>=3 and
                    abs(self.sample_angles.get(1,0.)-self.sample_angles.get(0,0.))>=.05)
        signature=(tuple(sorted(self.view_people)),tuple(sorted(self.view_plates)),tuple(sorted(pending)),self.view_sample)
        if signature!=self.evidence_signature:
            self.evidence_signature=signature;self.evidence_changed=now_sim
        stable_evidence=self.view_frames>=6 and now_sim-self.view_started>=1.8 and now_sim-self.evidence_changed>=.8
        feedback={'view':self.view,'attempt':self.view_key[1] if self.view_key else 0,'sample':self.view_sample,'ready':bool(self.view!='transit' and age<=.5 and view_confirmed and stable_evidence and (self.view.startswith('plate_') or (stable_pan and two_angles))),'stable_frames':self.view_frames,'quiet_s':round(now_sim-self.evidence_changed,2),'person_ids':list(self.view_people),'plate_texts':list(self.view_plates),'pending_person_ids':pending,'sample_frames':self.sample_frames,'sample_angles':self.sample_angles,'two_angles_verified':two_angles,'criterion':'Track-consistent identities and two measured camera angles; unseen/occluded people are not guaranteed'}
        canvas=PILImage.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB));draw=ImageDraw.Draw(canvas)
        overlay_labels=draw_detection_labels(canvas,detections,self.font)
        draw.rectangle((0,0,canvas.width,64),fill=(15,23,42));draw.text((12,4),f"Frame {self.frame_id} | 居民 {counts['residents']} / 访客 {counts['visitors']} / 总计 {counts['total']} | 灯 {stable}",font=self.font,fill=(255,255,255));draw.text((12,32),self.status[:95],font=self.font,fill=(180,210,255));frame=cv2.cvtColor(np.asarray(canvas),cv2.COLOR_RGB2BGR)
        self.latest={'run_id':self.run_id,'lap_id':self.lap_id,'lap_active':self.lap_active,'frame_id':self.frame_id,'stamp':header.stamp.sec+header.stamp.nanosec*1e-9,'status':self.status,'view':self.view,'traffic_light':stable,'counts':counts,'people':self.people,'plates':self.plates,'detections':detections,'overlay_labels':overlay_labels,'events':self.events,'observation_feedback':feedback,'observation_outcomes':self.last_observation_outcomes,'unconfirmed_views':[x['view'] for x in self.last_observation_outcomes if not x['confirmed']],'inference_and_ocr_ms':round((time.perf_counter()-begin)*1000,1),'previous_cycle_total_ms':self.previous_cycle_ms}
        self.results.publish(String(data=json.dumps(self.latest,ensure_ascii=False)))
        # Full-rate compressed ROS/HTTP view; raw only for events or 2 Hz preview.
        if new_events or time.monotonic()-self.last_raw_image_time>=.5:
            image=Image();image.header=header;image.height,image.width=frame.shape[:2];image.encoding='bgr8';image.step=image.width*3;image.data=frame.tobytes()
            self.images.publish(image);self.last_raw_image_time=time.monotonic()
        self.jpeg=cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,88])[1].tobytes();compressed=CompressedImage();compressed.header=header;compressed.format='jpeg';compressed.data=self.jpeg;self.compressed_images.publish(compressed);(self.out/'latest.jpg').write_bytes(self.jpeg);(self.out/'report.tmp').write_text(json.dumps(self.latest,ensure_ascii=False,indent=2));(self.out/'report.tmp').replace(self.out/'report.json')
        for e in new_events:(self.out/e['image']).write_bytes(self.jpeg)
        if self.view!='transit':(self.out/(self.view+'.jpg')).write_bytes(self.jpeg)
        self.previous_cycle_ms=round((time.perf_counter()-begin)*1000,1)

def main():
    rclpy.init();node=Perception()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.server.shutdown();node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
