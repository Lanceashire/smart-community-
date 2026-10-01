"""YOLO11 + RapidOCR on live ROS camera frames; synchronized visual/JSON output.

This node never reads layout geometry, ground truth, or simulator identities.
The identity classes describe the supplied gallery, not unknown real people.
"""
import json,re,time,threading
from collections import Counter,deque
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import cv2,numpy as np,rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ultralytics import YOLO
from rapidocr import RapidOCR
from community_demo.plate_geometry import blue_plates
from PIL import Image as PILImage,ImageDraw,ImageFont

class Perception(Node):
    def __init__(self):
        super().__init__('community_yolo_ocr')
        self.declare_parameter('model','src/community_demo/models/perception/best.pt');self.declare_parameter('port',8765)
        self.model=YOLO(self.get_parameter('model').value)
        self.ocr=RapidOCR(params={'Global.model_root_dir':'src/community_demo/models/ocr','EngineConfig.onnxruntime.intra_op_num_threads':2,'EngineConfig.onnxruntime.inter_op_num_threads':2})
        self.out=Path('log/perception');self.out.mkdir(parents=True,exist_ok=True)
        self.create_subscription(Image,'/camera/image',self.on_camera,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(String,'/patrol/status',self.on_status,10)
        self.create_subscription(String,'/inspection/observation',self.on_observation,10)
        self.results=self.create_publisher(String,'/perception/results',10);self.light=self.create_publisher(String,'/perception/traffic_light',10)
        self.images=self.create_publisher(Image,'/perception/image',10)
        self.frame=None;self.last_stamp=None;self.processed=None;self.jpeg=b'';self.people={};self.plates={};self.events=[];self.votes={};self.light_history=deque(maxlen=3);self.last_ocr=0.;self.status='WAIT';self.view='transit';self.completed=False;self.last_light_event='unknown';self.frame_id=0;self.latest={}
        fonts=['src/community_demo/models/fonts/wqy-zenhei.ttc','/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc','/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc']
        self.font=next((ImageFont.truetype(f,22) for f in fonts if Path(f).exists()),ImageFont.load_default())
        self.create_timer(.2,self.tick)
        self.restart_client=self.create_client(Trigger,'/patrol/restart')
        node=self
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path.startswith('/frame.jpg'):
                    data=node.jpeg;typ='image/jpeg'
                elif self.path.startswith('/state'):
                    data=json.dumps(node.latest,ensure_ascii=False).encode();typ='application/json; charset=utf-8'
                else:
                    data=('''<!doctype html><meta charset="utf-8"><title>AIC 机器人实时验收</title><style>body{background:#111827;color:#eee;font:18px sans-serif;margin:24px}main{display:grid;grid-template-columns:3fr 1fr;gap:24px}img{width:100%;border-radius:12px}pre{white-space:pre-wrap;background:#1f2937;padding:20px;border-radius:12px}h1{font-size:26px}</style><h1>AIC 智慧社区 · 机器人真实相机</h1><button onclick="restart()" style="font-size:18px;padding:10px 18px;margin-bottom:18px">再巡航一圈</button><span id="restart-message"></span><main><img id="camera"><pre id="state"></pre></main><script>async function restart(){let r=await(await fetch('/restart',{method:'POST'})).json();document.getElementById('restart-message').textContent=r.message;}setInterval(async()=>{document.getElementById('camera').src='/frame.jpg?t='+Date.now();let s=await(await fetch('/state')).json();if(!s.counts){document.getElementById('state').textContent='等待机器人相机';return;}document.getElementById('state').textContent=s.status+'\\n\\n社区人员：'+s.counts.residents+'\\n非社区人员：'+s.counts.visitors+'\\n去重总人数：'+s.counts.total+'\\n\\n车牌：\\n'+Object.keys(s.plates).join('\\n')+'\\n\\n灯色：'+s.traffic_light+'\\n画面编号：'+s.frame_id;},350)</script>''').encode();typ='text/html; charset=utf-8'
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
        if m.encoding not in ('rgb8','bgr8'):return
        a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
        self.frame=cv2.cvtColor(a,cv2.COLOR_RGB2BGR) if m.encoding=='rgb8' else a.copy();self.last_stamp=m.header
    def on_status(self,m):
        if self.completed and m.data.startswith('GO top_approach'):
            self.people={};self.plates={};self.events=[];self.votes={};self.completed=False
        self.status=m.data
        if m.data.startswith('COMPLETE'):self.completed=True
    def on_observation(self,m):
        try:self.view=json.loads(m.data).get('view','transit')
        except ValueError:pass
    def event(self,kind,key,confidence):
        item={'sequence':len(self.events)+1,'frame_id':self.frame_id,'stamp':self.last_stamp.stamp.sec+self.last_stamp.stamp.nanosec*1e-9,'category':kind,'content':key,'confidence':round(float(confidence),4),'image':f'event-{len(self.events)+1:03}.jpg'}
        self.events.append(item);self.get_logger().info(json.dumps(item,ensure_ascii=False));return item
    def tick(self):
        if self.frame is None or self.last_stamp==self.processed:return
        header=self.last_stamp;frame=self.frame.copy();self.processed=header;self.frame_id+=1
        begin=time.perf_counter();result=self.model.predict(frame,imgsz=960,conf=.35,iou=.45,device=0,verbose=False)[0];detections=[];light_candidates=[];ocr_due=time.monotonic()-self.last_ocr>1.0;new_events=[]
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
                history=self.votes.setdefault(name,deque(maxlen=8));history.append((self.frame_id,conf))
                recent=[c for f,c in history if self.frame_id-f<15]
                if len(recent)>=3 and np.mean(recent)>.60 and name not in self.people:
                    event=self.event(kind,name,np.mean(recent));self.people[name]=event;new_events.append(event)
            elif name=='plate':
                kind='plate';color=(255,160,60)
                # Pixel-boundary refinement and OCR are performed once below.
            elif name.startswith('light_'):
                kind='traffic_light';colour=name[6:];text=colour;light_candidates.append(((x2-x1)*(y2-y1),colour,conf))
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
                        if len(history)>=2 and candidate not in self.plates:
                            event=self.event('plate',candidate,score);self.plates[candidate]=event;new_events.append(event)
            detections.append({'category':'plate','name':text,'confidence':target['geometry_score'],'confidence_type':'geometry score, not YOLO probability','ocr_confidence':ocr_score,'bbox':target['bbox'],'method':'YOLO plus blue-boundary proposals / pixel OCR'})
            cv2.rectangle(frame,(x1,y1),(x2,y2),(70,240,255),2)
        if ocr_due:self.last_ocr=time.monotonic()
        state=max(light_candidates,default=(0,'unknown',0))[1];self.light_history.append(state);stable=state if len(self.light_history)==3 and len(set(self.light_history))==1 else 'unknown';age=self.get_clock().now().nanoseconds/1e9-(header.stamp.sec+header.stamp.nanosec*1e-9)
        if age>.5:stable='unknown'
        self.light.publish(String(data=stable))
        if stable!='unknown' and stable!=self.last_light_event:
            new_events.append(self.event('traffic_light',stable,max((c[2] for c in light_candidates),default=0.)))
            self.last_light_event=stable
        counts={'residents':sum(k.startswith('resident') for k in self.people),'visitors':sum(k.startswith('visitor') for k in self.people),'total':len(self.people)}
        canvas=PILImage.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB));draw=ImageDraw.Draw(canvas)
        for d in detections:draw.text((d['bbox'][0],max(0,d['bbox'][1]-25)),f"{d['name']} {'geom ' if 'confidence_type' in d else ''}{d['confidence']:.2f}",font=self.font,fill=(255,230,80),stroke_width=1,stroke_fill=(0,0,0))
        draw.rectangle((0,0,canvas.width,64),fill=(15,23,42));draw.text((12,4),f"Frame {self.frame_id} | 居民 {counts['residents']} / 访客 {counts['visitors']} / 总计 {counts['total']} | 灯 {stable}",font=self.font,fill=(255,255,255));draw.text((12,32),self.status[:95],font=self.font,fill=(180,210,255));frame=cv2.cvtColor(np.asarray(canvas),cv2.COLOR_RGB2BGR)
        self.latest={'frame_id':self.frame_id,'stamp':header.stamp.sec+header.stamp.nanosec*1e-9,'status':self.status,'view':self.view,'traffic_light':stable,'counts':counts,'people':self.people,'plates':self.plates,'detections':detections,'events':self.events,'inference_and_ocr_ms':round((time.perf_counter()-begin)*1000,1)}
        self.results.publish(String(data=json.dumps(self.latest,ensure_ascii=False)));image=Image();image.header=header;image.height,image.width=frame.shape[:2];image.encoding='bgr8';image.step=image.width*3;image.data=frame.tobytes();self.images.publish(image)
        self.jpeg=cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,88])[1].tobytes();(self.out/'latest.jpg').write_bytes(self.jpeg);(self.out/'report.tmp').write_text(json.dumps(self.latest,ensure_ascii=False,indent=2));(self.out/'report.tmp').replace(self.out/'report.json')
        for e in new_events:(self.out/e['image']).write_bytes(self.jpeg)
        if self.view!='transit':(self.out/(self.view+'.jpg')).write_bytes(self.jpeg)

def main():
    rclpy.init();node=Perception()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.server.shutdown();node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
