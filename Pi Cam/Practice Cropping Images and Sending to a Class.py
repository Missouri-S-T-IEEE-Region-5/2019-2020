import cv2
import numpy as np

class Grid:
    def __init__(self,w,h,d,item):
        self.width=w
        self.height=h
        self.divisor=d
        self.grid=[[False for x in range(h)] for y in range(w)]
        #print(len(self.grid))
        
    def data(self,i,value):
        x=i//self.divisor
        y=i%self.divisor
        #print(x,y)
        self.grid[y][x]=value

    def Check(self):
        for i in range(0,len(self.grid)):
            print (" ".join([str(j) for j in self.grid[i]]))

def Detection(test,i,g):
    dilated=cv2.dilate(test,np.ones((7,7),np.uint8))
    blur_image=cv2.medianBlur(dilated,21)
    diff=255-cv2.absdiff(test,blur_image)
    hsv=cv2.cvtColor(diff,cv2.COLOR_BGR2HSV)
    lower=np.array([30,30,30])
    upper=np.array([240,240,240])
    mask=cv2.inRange(diff,lower,upper)
    res=cv2.bitwise_and(test,test,mask=mask)
    canny=cv2.Canny(res,150,200)
    """while True:
        k=cv2.waitKey(5)&0xFF
        if k==27:
            break
        cv2.imshow("image",canny)"""
    #print(canny)
    if canny.any()!=0: #if any of the numbers in the matrix are not zero (black)
        #print(True)
        g.data(i,True)
    else: #if the matrix is nothing but zeros (black)
        #print(False)
        g.data(i,False)

file="D:\\Personal_Misc\\Region 5 Robotics\\2019-2020\\Bot Code\\Opencv Stuff\\Practice Image.jpg"
image=cv2.imread(file,-1)


width=image.shape[1]
height=image.shape[0]
divisor=40
w=width/divisor
h=height/divisor
i=0
x=0
g=Grid(divisor,divisor,divisor,False)
while(x!=width):
    y=0
    while(y!=height):
        #i+=1
        img=image[int(y):int(y+h), int(x):int(x+w)].copy()
        #print("width",x,"height",y, "y+h",int(y+h),"x+w",int(x+w))
        Detection(img,i,g)
        i+=1
        y+=h
        #print(i)
        if(y==(height)):
            x+=w
            
g.Check()
