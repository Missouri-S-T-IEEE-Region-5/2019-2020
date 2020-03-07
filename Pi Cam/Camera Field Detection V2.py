import math, cv2,pygame
import numpy as np

#CURRENT PROBLEM:
#ALL NUMBERS GOING INTO THE GRID ARE EXACTLY THE SAME.  FIND THE PROBLEM AND FIX IT



class base_points:
    def __init__(self):
        self.width=72
        self.height=72
        self.grid=[[False for x in range(self.height+1)] for y in range(self.width+1)]
        self.data=[]
        self.y=[]

    def Length_of_Data(self):
        print(len(self.data))

    def Proof(self):
        print(self.data)

    def Y(self):
        print(len(self.y))

    def Print_Y(self):
        print(self.y)

    def Check(self):
        print("Length of grid", len(self.grid))
        for i in range(0,len(self.grid)):
            print(" ".join([str(j) for j in self.grid[i]]))
            
    def Check_2(self):
        print("Exact Locations:")
        for i in range(0,len(self.grid[0])):
            for j in range(0,len(self.grid[1])):
                if self.grid[i][j]==True:
                    print("x, y:",i,j)


def lines(p1,p2):
    #print("P2[0]:",p2[0],"P1[0]:",p1[0])
    if (p2[0]-p1[0])==0:
        slope=0
    else:
        slope=(p2[1]-p1[1])/(p2[0]-p1[0])
    b=p2[1]-(slope*p2[0])
    return(slope,b)

def Intersection(slope1,b1,slope2,b2):
    final_slope=slope1-slope2
    final_b=b2-b1
    x=final_b/final_slope
    y=slope1*x+b1
    return(x,y)

#-------------------------------------------------
#Presteps
Pos=(36,36)
Angle=90

#cap=cv2.VideoCapture(0)
#_,frame=cap.read()
file="D:\\Personal_Misc\\Region 5 Robotics\\2019-2020\\Bot Code\\Opencv Stuff\\Objects.jpg"
#cv2.imwrite(file,frame)
#cap.release()

image=cv2.imread(file,-1)
image1=pygame.image.load(file)

display_width=image.shape[1]
display_height=image.shape[0]
#print("Width:",display_width)
#print("Height:",display_height)

pixel_length=343#larger the number, larger the grid squares

pixel_height_horizon=-78 #decrease to make red lines more vertical
p_middle=display_width/2
numbe=460 #increase to make blue lines closer together

pixel_horizon_right=p_middle+numbe
pixel_horizon_left=p_middle-numbe
pixel_horizon_width=pixel_horizon_right+abs(pixel_horizon_left)

base_line=640
Display=pygame.display.set_mode([display_width,display_height])

#colors
red=(255,0,0)
blue=(0,0,255)
green=(0,255,0)
purple=(255,0,255)
black=(0,0,0)
white=(255,255,255)

if Angle==0:
    x_left=72-Pos[1]
    x_right=Pos[1]
    y_forward=72-Pos[0]
elif Angle==90:
    x_left=Pos[0]
    x_right=72-Pos[0]
    y_forward=72-Pos[1]
elif Angle==180:
    x_left=Pos[1]
    x_right=72-Pos[1]
    y_forward=Pos[0]
elif Angle==270:
    x_left=72-Pos[0]
    x_right=Pos[0]
    y_forward=Pos[1]
else:
    print("Error in angle")

trial=base_points()
camera_length=pixel_length+(11*math.tan(math.radians(17.5))*pixel_length)
print("Camera Length:",camera_length)

#================================================

Display.fill(black)
Display.blit(image1,(0,0))

#base line
pygame.draw.line(Display,blue,(0,799+camera_length),(799,799+camera_length),1)
#horizon line
#pygame.draw.line(Display,white,(pixel_horizon_left,pixel_height_horizon),(pixel_horizon_right,pixel_height_horizon),1)

#getting the base line points
number_of_points=0
for i in reversed(range(0,x_left)):
    number_of_points+=1
    trial.data.append((-i*pixel_length+display_width/2,(display_width-1)+camera_length))
for i in range(0,x_right):
    number_of_points+=1
    trial.data.append((i*pixel_length+display_width/2,(display_width-1)+camera_length))

#cosmetic step, showing the lines on the pygame display
for i in range(0,number_of_points):
    #middle point
    pygame.draw.line(Display,red,trial.data[i],(display_width/2,pixel_height_horizon),1)
    """
    #right corner
    pygame.draw.line(Display,green,trial.data[i],(pixel_horizon_right,pixel_height_horizon),1)
    #left corner
    pygame.draw.line(Display,purple,trial.data[i],(pixel_horizon_left,pixel_height_horizon),1)
    """

#finding the y intercepts
for i in range(0,y_forward):
    #right line
    slope1,b1=lines((trial.data[i+1]),(pixel_horizon_left,pixel_height_horizon))
    #middle line
    slope2,b2=lines((trial.data[0]),(pixel_horizon_width/2,pixel_height_horizon))
    #Intersection of the two lines
    x,y=Intersection(slope1,b1,slope2,b2)
    #displaying the new line on the y axis
    pygame.draw.line(Display,blue,(0,y),(display_width,y),1)
    #adding y to the class variable "y"
    trial.y.append(y)
    #print("slope1:",slope1,"slope2:",slope2)

#messing with the image before cropping it

dilated=cv2.dilate(image,np.ones((7,7),np.uint8))
blur_image=cv2.medianBlur(dilated,21)
diff=255-cv2.absdiff(image,blur_image)
hsv=cv2.cvtColor(diff,cv2.COLOR_BGR2HSV)
lower=np.array([30,30,30])
upper=np.array([240,240,240])
mask=cv2.inRange(diff,lower,upper)
res=cv2.bitwise_and(image,image,mask=mask)
canny=cv2.Canny(res,150,200)
while True:
            k=cv2.waitKey(5) & 0xFF
            if k==27:
                break
            cv2.imshow('image',canny)
counter=0
#cropping the image and telling the class if something exists there or not
for i in range(0,y_forward-1):
    for j in range(0,number_of_points-1):
        counter+=1
        y0=trial.y[i]
        if i==y_forward:
            y1=trial.y[i]
        else:
            #print(i+1)
            y1=trial.y[i+1]
        x01=trial.data[j][0] #x point 01 from base line
        if j==number_of_points:
            x02=trial.data[j][0] #x point 02 from base line
        else:
            #print(j+1)
            x02=trial.data[j+1][0] #x point 02 from base line
        slope,b=lines((trial.data[j]),(pixel_horizon_width/2,pixel_height_horizon))
        if slope==0:
            x1=x3=x01
        else:
            x1=(y0-b)/slope
            x3=(y1-b)/slope
        slope1,b1=lines((trial.data[j+1]),(pixel_horizon_width/2,pixel_height_horizon))
        #print("slope:",slope,"slope1:",slope1)
        if slope1==0:
            x4=x2=x02
        else:
            x2=(y0-b1)/slope1
            x4=(y1-b1)/slope1
        img=canny.copy()
        pts=np.array([[(x1,y0),(x3,y1),(x4,y1),(x2,y0)]],dtype=np.int32)
        mask=np.zeros(img.shape,dtype=np.uint8)
        channel_count=3
        ignore_mask_color=(255,)*channel_count
        cv2.fillPoly(mask,pts,ignore_mask_color)
        masked_image=cv2.bitwise_and(img,mask)
        """
        while True:
            k=cv2.waitKey(5) & 0xFF
            if k==27:
                break
            cv2.imshow('image',masked_image)
            """
        if masked_image.any()!=0:
            detection=True
            #print("slope:",slope,"slope1:",slope1)
            #print((x1,y0),(x3,y1),(x4,y1),(x2,y0))
            while True:
                k=cv2.waitKey(5) & 0xFF
                if k==27:
                    break
                cv2.imshow('image',masked_image)
        else:
            detection=False
            #detection=True
        if Angle==0:
                number=(trial.data[j][0]-display_width/2)//pixel_length #x pos from the middle of the screen
                x_data=int(Pos[1]-number) #number should be negative on left and positive on right
                y_data=int(Pos[0]+i)

                if x_data<=72 and y_data<=72:
                    trial.grid[y_data][x_data]=detection
                else:
                    print("ERROR:", "x data:",x_data,"y data:",y_data)
                    
        elif Angle==180:
                number=(trial.data[j][0]-display_width/2)//pixel_length #x pos from the middle of the screen
                x_data=int(Pos[1]+number) #number should be negative on left and positive on right
                y_data=int(Pos[0]-i)

                if x_data<=72 and y_data<=72:
                    trial.grid[y_data][x_data]=detection
                else:
                    print("ERROR:", "x data:",x_data,"y data:",y_data)
                    
        elif Angle==90:
                number=(trial.data[j][0]-display_width/2)//pixel_length #x pos from the middle of the screen
                x_data=int(Pos[0]+number) #number should be negative on left and positive on right
                y_data=int(Pos[1]+i)

                if x_data<=72 and y_data<=72:
                    trial.grid[x_data][y_data]=detection
                else:
                    print("ERROR:", "x data:",x_data,"y data:",y_data)
                    
        else:
                number=(trial.data[j][0]-display_width/2)//pixel_length #x pos from the middle of the screen
                x_data=int(Pos[0]-number) #number should be negative on left and positive on right
                y_data=int(Pos[1]-i)

                if x_data<=72 and y_data<=72:
                    trial.grid[x_data][y_data]=detection
                else:
                    print("ERROR:", "x data:",x_data,"y data:",y_data)   
        #print(detection)
"""while True:
    k=cv2.waitKey(5) &0xFF
    if k==27:
        break
    cv2.imshow('image',image)
    cv2.imshow('canny',canny)
"""
#print("y_forward",y_forward,"number_of_points",number_of_points,"counter",counter)
pygame.display.update()
#trial.Print_Y()
trial.Check_2()
#trial.Proof()
