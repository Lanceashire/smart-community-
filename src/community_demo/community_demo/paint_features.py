"""Extract achromatic road paint under bounded exposure changes; reject blank scenes."""
import cv2
import numpy as np

def white_paint_mask(image):
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    value=hsv[:,:,2];achromatic=hsv[:,:,1]<32
    region=np.zeros(value.shape,bool)
    region[int(image.shape[0]*.59):int(image.shape[0]*.88)]=True
    values=value[region & achromatic]
    mask=np.zeros(value.shape,bool)
    if len(values)<100:return mask,None,0.
    dark,bright=np.percentile(values,[10,99.5]);contrast=float(bright-dark)
    # A uniform gray/white frame and small noise cannot create localization evidence.
    if bright<120 or contrast<45:return mask,None,contrast
    # Preserve the original extraction when it already has sufficient paint.
    strong=region & achromatic & (value>205)
    threshold=205. if np.count_nonzero(strong)>=100 else min(205.,max(120.,float(bright)*.85))
    mask=region & achromatic & (value>threshold)
    mask[:,::2]=False
    return mask,threshold,contrast
