"""
This code determines which object is in the pan.
"""
import cv2
import numpy as np
import time


def Area(x1,y1,x2,y2):
    x=x2-x1
    y=y2-y1
    return(x*y)

def range_overlap(x1_min,x1_max,x2_min,x2_max,y1_min,y1_max,y2_min,y2_max):
    overlapping=True
    if (x1_min>=x2_max) or (x1_max<=x2_min):
        overlapping=False
    if (y1_min>=y2_max) or (y1_max<=y2_min):
        overlapping=False
    return(overlapping)
    
def Overlap(rectangles):
    i=0
    rect=[]
    for x1,y1,x2,y2 in rectangles:
        d=len(rect)
        x=False
        if i==0:
            rect.append([x1,y1,x2,y2])
        else:
            r=0
            #print(d)
            for x3,y3,x4,y4 in rect:
                    if range_overlap(min(x1,x2),max(x1,x2),min(x3,x4),max(x3,x4),min(y1,y2),max(y1,y2),min(y3,y4),max(y3,y4))==True:
                                #print('yes')
                                x_min=min(x1,x2,x3,x4)
                                x_max=max(x1,x2,x3,x4)
                                y_min=min(y1,y2,y3,y4)
                                y_max=max(y1,y2,y3,y4)
                                rect[r]=[x_min,y_min,x_max,y_max]
                                x=True
                                
                    r+=1
                    if r==d and x==False:
                        rect.append([x1,y1,x2,y2])
                        break
                    if r==d:
                        r=0
        i+=1
    return(rect)
                
        
        
        
file="D:\\Personal_Misc\\Region 5 Robotics\\2019-2020\\Bot Code\\Opencv Stuff\\Objects.jpg"
image=cv2.imread(file,-1)

def MultipleRectangles():
    dilated=cv2.dilate(image,np.ones((7,7),np.uint8))
    bg_img=cv2.medianBlur(dilated,21)
    diff=255-cv2.absdiff(image,bg_img)
    hsv=cv2.cvtColor(diff,cv2.COLOR_BGR2HSV)
    lower=np.array([30,30,30])
    upper=np.array([240,240,240])
    mask=cv2.inRange(diff,lower,upper)
    res=cv2.bitwise_and(image,image,mask=mask)
    canny=cv2.Canny(res,150,200)
    """lines=cv2.HoughLinesP(
        canny,
        rho=10,
        theta=np.pi/180,
        threshold=5,
        lines=np.array([]),
        minLineLength=5,
        maxLineGap=10)        
    line_image=np.zeros(
        (
            canny.shape[0],
            canny.shape[1],
            3),
        dtype=np.uint8)"""
    contours,hierarchy=cv2.findContours(canny,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    contours_poly = [None]*len(contours)
    boundRect = [None]*len(contours)
    centers = [None]*len(contours)
    radius = [None]*len(contours)
    for i, c in enumerate(contours):
        contours_poly[i] = cv2.approxPolyDP(c, 3, True)
        boundRect[i] = cv2.boundingRect(contours_poly[i])
    rects=[]
    drawing = np.zeros((canny.shape[0], canny.shape[1], 3), dtype=np.uint8)
    for i in range(len(contours)):
        color = (0, 255, 0)
        if Area(int(boundRect[i][0]),int(boundRect[i][1]),int(boundRect[i][0]+boundRect[i][2]), int(boundRect[i][1]+boundRect[i][3])) >=400:
            cv2.rectangle(drawing, (int(boundRect[i][0]), int(boundRect[i][1])), \
          (int(boundRect[i][0]+boundRect[i][2]), int(boundRect[i][1]+boundRect[i][3])), (255,0,0), 2)
            rects.append(boundRect[i])
    return(canny,diff,drawing,rects)

canny,diff,drawing,rects=MultipleRectangles()
while True:
    k=cv2.waitKey(5) & 0xFF
    if k==27:
        break
    cv2.imshow('canny',canny)
    cv2.imshow('diff',diff)
    cv2.imshow('frame',image)
    cv2.imshow('drawing',drawing)

cv2.destroyAllWindows()

rect=Overlap(rects)
for i in rect:
    cv2.rectangle(image, (i[0], i[1]), (i[0]+i[2], i[1]+i[3]), (0,255,0), 2)
    print(Area(i[0],i[1],i[2],i[3]))
while True:
    k=cv2.waitKey(5) & 0xFF
    if k==27:
        break
    cv2.imshow('maybe',image)

cv2.destroyAllWindows()










    
