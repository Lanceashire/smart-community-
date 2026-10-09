"""Readable detection labels with explicit OCR scores and no overlapping text."""
from PIL import ImageDraw

def target_display_name(category,name):
    if category in ('resident','visitor'):
        return ('社区人员' if category=='resident' else '非社区人员')+' '+name.rsplit('_',1)[-1]
    if category=='traffic_light':return {'red':'红灯','green':'绿灯','yellow':'黄灯'}.get(name,name)
    if name in ('plate','blue plate'):return '车牌候选'
    return name

def overlap(a,b):
    return max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))

def draw_detection_labels(canvas,detections,font,top_margin=70):
    draw=ImageDraw.Draw(canvas);occupied=[];placements=[]
    proposals=[d for d in detections if 'ocr_confidence' in d]
    for d in detections:
        box=d['bbox']
        if d['name']=='plate' and any(overlap(box,p['bbox'])/max(1,(box[2]-box[0])*(box[3]-box[1]))>.5 for p in proposals):continue
        score=d.get('ocr_confidence');name=target_display_name(d.get('category'),d['name']);status=''
        if d.get('category') in ('resident','visitor'):
            status=' 已确认' if d.get('confirmed') else ' 待确认'
        geometry=d.get('confidence_type','').startswith('geometry')
        label=(f"{name} OCR {score:.2f}" if score is not None else
               f"{name} {'几何评分' if geometry else '检测'} {d['confidence']:.2f}")+status
        bounds=draw.textbbox((0,0),label,font=font,stroke_width=1);width=bounds[2]-bounds[0]+8;height=bounds[3]-bounds[1]+8
        x=max(0,min(box[0],canvas.width-width));preferred=max(top_margin,box[1]-height-3)
        candidates=[preferred]
        for step in range(1,25):candidates.extend([preferred-step*(height+3),preferred+step*(height+3)])
        y=next((y for y in candidates if top_margin<=y<=canvas.height-height and not any(overlap((x,y,x+width,y+height),r)>0 for r in occupied)),None)
        if y is None:
            # Dense scenes may require a different horizontal column.
            for cx in range(0,max(1,canvas.width-width+1),max(1,width+3)):
                for cy in range(top_margin,canvas.height-height+1,height+3):
                    if not any(overlap((cx,cy,cx+width,cy+height),r)>0 for r in occupied):x,y=cx,cy;break
                if y is not None:break
        if y is None:continue
        rect=(x,y,x+width,y+height);occupied.append(rect)
        if abs(y-preferred)>3:draw.line((box[0],box[1],x+4,y+height),fill=(255,230,80),width=1)
        draw.rectangle(rect,fill=(15,23,42),outline=(160,145,60),width=1)
        draw.text((x+4-bounds[0],y+4-bounds[1]),label,font=font,fill=(255,230,80),stroke_width=1,stroke_fill=(0,0,0))
        placements.append({'name':d['name'],'bbox':box,'label':label,'label_rect':list(rect)})
    return placements
