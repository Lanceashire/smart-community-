#!/usr/bin/env python3
"""Film actual Gazebo overhead + exact annotated camera and synchronized event excerpts.

No driving or signal service calls. Wall-time 10fps, with repeated frames documented.
"""
import argparse,json,subprocess,time
from pathlib import Path
from collections import OrderedDict
import cv2,numpy as np
from PIL import Image as PILImage,ImageDraw,ImageFont
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CompressedImage
from std_msgs.msg import String
ROOT=Path(__file__).resolve().parents[1]
FONT=ROOT/'src/community_demo/models/fonts/wqy-zenhei.ttc'
class Recorder(Node):
    def __init__(self,a):
        super().__init__('patrol_video_recorder');self.a=a;self.overview=None;self.overview_stamp=None
        self.images=OrderedDict();self.reports=OrderedDict();self.latest=None
        self.frames=0;self.start=None;self.finish=None;self.encoder=None;self.run_id=None
        self.font=ImageFont.truetype(str(FONT),22);self.small=ImageFont.truetype(str(FONT),18)
        self.title=ImageFont.truetype(str(FONT),29);self.events=[];self.last_status=None
        self.overview_seen=0;self.paired=0;self.repeated=0;self.prev_frame_id=None
        self.source_ages=[];self.frame_log=[];self.error=None;self.done=False
        self.create_subscription(Image,'/recording/overview',self.on_overview,qos_profile_sensor_data)
        self.create_subscription(CompressedImage,'/perception/image/compressed',self.on_image,qos_profile_sensor_data)
        self.create_subscription(String,'/perception/results',self.on_report,10)
        (ROOT/a.output).parent.mkdir(parents=True,exist_ok=True)
        (ROOT/a.ready).write_text(json.dumps({'ready':True,'read_only':True})+'\n')
    def on_overview(self,m):
        if m.encoding not in ('rgb8','bgr8') or len(m.data)!=m.step*m.height:return
        a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
        self.overview=a.copy() if m.encoding=='rgb8' else cv2.cvtColor(a,cv2.COLOR_BGR2RGB)
        self.overview_stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9;self.overview_seen+=1
    def on_image(self,m):
        t=round(m.header.stamp.sec+m.header.stamp.nanosec/1e9,6)
        a=cv2.imdecode(np.frombuffer(m.data,np.uint8),cv2.IMREAD_COLOR)
        if a is None:return
        self.images[t]=cv2.cvtColor(a,cv2.COLOR_BGR2RGB)
        while len(self.images)>40:self.images.popitem(last=False)
    def on_report(self,m):
        d=json.loads(m.data);t=round(d['stamp'],6);self.reports[t]=d
        while len(self.reports)>40:self.reports.popitem(last=False)
    def sample(self):
        common=self.images.keys() & self.reports.keys()
        if not common or self.overview is None:return
        t=max(common);d=self.reports[t]
        if d.get('lap_id')!=self.a.lap:return
        if self.start is None:
            self.start=time.monotonic();self.run_id=d['run_id']
            self.encoder=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-y',
                '-f','rawvideo','-pix_fmt','rgb24','-s','2560x1440','-r','10','-i','pipe:0',
                '-an','-c:v','libx264','-preset','veryfast','-crf','18','-threads','2',
                '-pix_fmt','yuv420p','-movflags','+faststart',str(ROOT/self.a.output)],stdin=subprocess.PIPE)
            print('PATROL_VIDEO_STARTED',self.run_id,self.a.lap,flush=True)
        if (d['run_id'],d['lap_id'])!=(self.run_id,self.a.lap):
            self.error='run changed before recording completed';self.done=True;return
        now=time.monotonic();elapsed=now-self.start
        if d.get('status')!=self.last_status:
            self.last_status=d.get('status');self.events.append({'video_wall_s':elapsed,'sim_s':d['stamp'],'status':self.last_status})
        if d.get('status','').startswith('COMPLETE'):
            if self.finish is None:self.finish=now
            if now-self.finish>=3.:self.done=True;return
        expected=int(elapsed*10)+1
        if self.frames>=expected:return
        panel=PILImage.new('RGB',(2560,1440),(14,22,36));draw=ImageDraw.Draw(panel)
        draw.text((25,12),'AIC 智慧社区 · 全自主巡航实录 | 队伍：待填写',font=self.title,fill='white')
        draw.text((25,55),f'仿真时刻 {d["stamp"]:.2f}s  |  视频墙钟 {elapsed:.1f}s  |  Run {self.run_id} / Lap {self.a.lap}',font=self.small,fill='#b9cbe0')
        panel.paste(PILImage.fromarray(self.overview).resize((1200,1200)),(20,110))
        panel.paste(PILImage.fromarray(self.images[t]).resize((1280,960)),(1260,110))
        draw.text((25,85),'Gazebo 实际俯视相机（观察用途，不参与导航）',font=self.small,fill='#8ce2cb')
        draw.text((1265,85),'机器人实际相机 · 检测框 / 名称 / 置信度',font=self.small,fill='#8ce2cb')
        status=d.get('status','');draw.text((1265,1085),status[:105],font=self.font,fill='#ffdf83')
        c=d.get('counts',{});draw.text((1265,1120),f'社区人员 {c.get("residents",0)}   非社区人员 {c.get("visitors",0)}   去重总人数 {c.get("total",0)}   灯态 {d.get("traffic_light","unknown")}',font=self.font,fill='white')
        draw.text((1800,1160),'终端事件同步摘录（原始 JSON 另存验收日志）',font=self.small,fill='#8ce2cb')
        for i,e in enumerate(d.get('events',[])[-4:]):
            name=e.get('name',e.get('content',e.get('text','')))
            # Preserve source event values; formatting only.
            line=f'#{e.get("sequence", "?")}  {e.get("kind",e.get("category",e.get("type","")))}  {name}  conf={e.get("confidence", "?")}'
            draw.text((1800,1190+i*30),line[:60],font=self.small,fill='#e4edf6')
        detail=d.get('detail')
        if detail:
            box=detail['bbox'];x1,y1,x2,y2=box;source=self.images[t]
            crop=source[max(0,y1-8):min(source.shape[0],y2+8),max(0,x1-8):min(source.shape[1],x2+8)]
            if crop.size:
                enlarged=PILImage.fromarray(crop).resize((min(480,crop.shape[1]*4),min(180,crop.shape[0]*4)),PILImage.Resampling.NEAREST)
                panel.paste(enlarged,(1265,1180));draw.text((1265,1160),'当前车牌局部放大（同帧，仅显示）',font=self.small,fill='#8ce2cb')
        draw.text((25,1388),'完整一圈，正常速度；巡航过程连续记录，无人工驾驶、无动画替代运动。',font=self.small,fill='#b9cbe0')
        raw=panel.tobytes()
        age=abs(self.overview_stamp-d['stamp']);self.source_ages.append(age)
        n=expected-self.frames
        self.repeated+=max(0,n-1)+(1 if d['frame_id']==self.prev_frame_id else 0)
        self.prev_frame_id=d['frame_id'];self.paired+=1
        for _ in range(n):self.encoder.stdin.write(raw);self.frames+=1
        self.frame_log.append({'video_wall_s':elapsed,'camera_stamp':t,'overview_stamp':self.overview_stamp,'frame_id':d['frame_id'],'encoded_frames':n,'status':status})
    def close_video(self):
        if self.encoder is not None:
            self.encoder.stdin.close();self.encoder.wait(timeout=60)
        passed=self.finish is not None and self.error is None and self.encoder is not None and self.encoder.returncode==0
        report={'passed':passed,'run_id':self.run_id,'lap_id':self.a.lap,'fps':10,'frames':self.frames,
            'duration_s':self.frames/10,'resolution':'2560x1440','source':'Actual Gazebo overhead image and exact stamp-paired ROS annotated JPEG/result; chronological wall-time recording',
            'audio':'Chinese captions, no narration','events':self.events,'frame_samples':self.frame_log,
            'repeated_frames':self.repeated,'overview_frames_seen':self.overview_seen,
            'max_overview_camera_delta_s':max(self.source_ages,default=None),'error':self.error,'file':self.a.output}
        (ROOT/self.a.output).with_suffix('.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ('events','frame_samples')},ensure_ascii=False),flush=True)
        return passed

def main():
    p=argparse.ArgumentParser();p.add_argument('--lap',type=int,default=1);p.add_argument('--output',required=True)
    p.add_argument('--ready',required=True);p.add_argument('--timeout',type=float,default=600);a=p.parse_args()
    rclpy.init();n=Recorder(a);deadline=time.monotonic()+a.timeout
    try:
        while not n.done and time.monotonic()<deadline:
            rclpy.spin_once(n,timeout_sec=.015);n.sample()
        if not n.done:n.error='recording timeout'
    finally:
        passed=n.close_video();n.destroy_node();rclpy.try_shutdown()
    return 0 if passed else 1
if __name__=='__main__':raise SystemExit(main())
