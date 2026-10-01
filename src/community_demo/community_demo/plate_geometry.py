"""Blue-plate boundary proposals: pixel geometry, no text or layout lookup."""
import cv2

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
