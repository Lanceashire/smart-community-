"""Blue-plate boundary proposals: pixel geometry, no text or layout lookup."""
import cv2
import math

def select_plate_target(proposals, detections, image_width, image_height):
    """Prefer a detected plate over blue bodywork; retain colour-only fallback.

    Colour-connected regions may join a blue plate to a blue vehicle. A valid
    neural plate box keeps that background out of the OCR crop. No scene
    positions, plate text or simulator identity are used here.
    """
    candidates=[]
    for detection in detections:
        try:
            x1,y1,x2,y2=map(float,detection['bbox'])
            confidence=float(detection['confidence'])
        except (KeyError,TypeError,ValueError):continue
        if not all(math.isfinite(v) for v in (x1,y1,x2,y2,confidence)):continue
        x1=max(0,math.floor(x1));y1=max(0,math.floor(y1))
        x2=min(image_width,math.ceil(x2));y2=min(image_height,math.ceil(y2))
        if x2-x1<42 or y2-y1<4 or not 0<=confidence<=1:continue
        candidates.append({'bbox':[x1,y1,x2,y2],'confidence':confidence,
                           'confidence_type':'YOLO probability','method':'YOLO plate box / pixel OCR'})
    if candidates:
        return min(candidates,key=lambda p:(abs((p['bbox'][0]+p['bbox'][2])/2-image_width/2),-p['confidence']))
    if proposals:
        target=min(proposals,key=lambda p:abs((p['bbox'][0]+p['bbox'][2])/2-image_width/2))
        return {**target,'confidence':target['geometry_score'],
                'confidence_type':'geometry score, not YOLO probability',
                'method':'blue-colour geometry / pixel OCR'}
    return None

def blue_plates(frame,min_width=12):
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,(110,65,35),(140,255,245))
    mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,cv2.getStructuringElement(cv2.MORPH_RECT,(3,2)))
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    boxes=[]
    for contour in contours:
        x,y,w,h=cv2.boundingRect(contour);area=cv2.contourArea(contour)
        if w<min_width or h<4 or not 2.1<w/h<5.3 or area<w*h*.45:continue
        boxes.append({'bbox':[x,y,x+w,y+h],'geometry_score':round(min(1.,area/(w*h)),3),'method':'blue-colour geometry; not a neural probability'})
    return boxes
