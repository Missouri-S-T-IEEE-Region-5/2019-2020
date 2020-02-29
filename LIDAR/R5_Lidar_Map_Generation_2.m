% IEEE Region 5 Robotics Competition 2020 - Lidar Map Generation Test Data 
%% Test Sample XY Data
testxy = [144 144 144 144 144 144 144 133.5 122.25 110.25 102 91.5 81 69.75 56.25 41.25 27.75 14.25 0 0 0 0 0 0 0 0 0 0 52.875 56.25 58.5 60 58.875 43.5 57.75 69 81 90.75 102 112.5 121.5 132.75 144 144 144 144 144 144; 42 60 71.25 89.25 103.875 123.75 144 144 144 144 144 144 144 144 144 144 144 144 144 103.5 85.5 74.25 63 53.25 42 34.5 28.5 21.75 32.25 31.5 30.375 28.5 24.75 0 0 0 0 0 0 0 0 0 0 7.5 15 23.25 30 36];
testx = testxy(1, :);
testy = testxy(2, :);
plot(testx,testy, '.');
%% Gather Important Data to Send
clear all, close all;
sampledata = [63 65.51 69.46 78.75 88.3 103.21 119.89 114.72 110.03 106.11 104.14 102.54 102 102.62 104.96 109.47 115.07 121.9 130.33 101.7 91.94 87.18 83.68 81.78 81 81.35 82.12 83.49 29.77 26.89 25.32 24.96 27.06 56.3 48.01 43.68 42 43.12 46.96 52.5 58.35 66.65 75.7 71.83 68.54 65.73 64.13 63.29; 0 15.9 24.9 36.87 44.48 52.38 58.3 62.76 67.98 74 78.37 84.12 90 96.29 103.64 111.29 117.57 123.2 128.5 142.79 151.76 158.29 165.47 172.09 180 185.29 189.46 194.04 199.12 202.99 207.3 212.74 217.94 228.24 241.03 254.05 270 283.07 296.57 306.87 313.96 320.94 326.3 331.29 336.8 343.43 349.22 354.56];
countbot = 1
countm = 2;
countc = 1;
senddata = zeros(2,10);
xyresult = zeros(2,10);
length = size(sampledata,2);
for i=1:length  % Look at all data and determine Bot side right and left
   if i>1 && i<length && ((sampledata(1,i)-(sampledata(1,i+1))))>28.75 % if next instance is greater then by said amount, then its bot wall 1. Said amount is lowest distance between bot wall 1 and greatest distance between scans.
        senddata(1,countbot)=sampledata(1,i+1);
        senddata(2,countbot)=sampledata(2,i+1);
        countbot = countbot+1;
        bot1 = i+1;
    end
    if i>1 && i<length && ((sampledata(1,i)-(sampledata(1,i-1))))>28.75 % if previous instance is less then by said amount, then its bot wall 2.
        senddata(1,countbot)=sampledata(1,i-1);
        senddata(2,countbot)=sampledata(2,i-1);
        countbot = countbot+1;
        bot2 = i-1;
    end 
end
for i=1:length
    if i==1 && (sampledata(1,i+1)-(sampledata(1,i)))<3  % Initial value is midpoint 1 if bot resets to nearest 90degs before scan.
        senddata(1,3)=sampledata(1,1);
        senddata(2,3)=sampledata(2,1);
    end
    if i>1 && i<length && sampledata(1,i)<(sampledata(1,i+1)) && sampledata(1,i)<(sampledata(1,i-1)) && (sampledata(1,i)-(sampledata(1,i+1)))<28.75 && ((sampledata(1,i)-(sampledata(1,i-1))))<28.75 && not(i>=bot1 && i<=bot2) % If next and previous intervals are greater, then its a midpoint, exclude jumps greater then said amount and intervals in bot range.
        if sampledata(2,i)<180 && sampledata(2,i)>0 % Select storage locations
            countm = 4
        end 
        if sampledata(2,i)<270 && sampledata(2,i)>90
            countm = 5
        end 
        if sampledata(2,i)<360 && sampledata(2,i)>180
            countm = 6
        end 
        senddata(1,countm)=sampledata(1,i);
        senddata(2,countm)=sampledata(2,i);
    end
    if i>1 && i<length && sampledata(1,i)>(sampledata(1,i+1)) && sampledata(1,i)>(sampledata(1,i-1)) && ((sampledata(1,i)-(sampledata(1,i+1))))<28.75 && ((sampledata(1,i)-(sampledata(1,i-1))))<28.75 && not(i>=bot1 && i<=bot2) % If next and previous intervals are lesser, then its a corner, exclude jumps greater then said amount and intervals in bot range.
        if sampledata(2,i)<90 && sampledata(2,i)>0 % Select Storage Locations
            countc = 7
        end 
        if sampledata(2,i)<180 && sampledata(2,i)>90
            countc = 8
        end 
        if sampledata(2,i)<270 && sampledata(2,i)>180
            countc = 9
        end 
        if sampledata(2,i)<360 && sampledata(2,i)>270
            countc = 10
        end 
        senddata(1,countc)=sampledata(1,i);
        senddata(2,countc)=sampledata(2,i);
    end
end


if i==3 && senddata(1,3)==0 % if data is missing, fill in using measurements from nearby cardinal or corners.
    senddata(1,3)=senddata(1,7)*cosd(senddata(2,7));
    senddata(2,3)=0;
end
if i==4 && senddata(1,4)==0
    senddata(1,4)=senddata(1,8)*cosd(senddata(2,8)-90);
    senddata(2,4)=90;
end
if senddata(1,5)==0
    senddata(1,5)=senddata(1,9)*cosd(senddata(2,9));
    senddata(2,5)=180;
end
if senddata(1,6)==0
    senddata(1,6)=senddata(1,10)*cosd(senddata(2,10));
    senddata(2,6)=270;
end
if senddata(1,7)==0
    if not(senddata(1,4)==0)
        oppo=senddata(1,4);
    end
    if not(senddata(1,3)==0)
        adj=senddata(1,3);
    end
    senddata(2,7)=atand(oppo/adj);
    senddata(1,7)=oppo/sind(senddata(2,7));
end
if senddata(1,8)==0
    if not(senddata(1,5)==0)
        oppo=senddata(1,5);
    end
    if not(senddata(1,4)==0)
        adj=senddata(1,4);
    end
    senddata(2,8)=atand(oppo/adj)+90;
    senddata(1,8)=oppo/sind(senddata(2,8)-90);
end
if senddata(1,9)==0
    if not(senddata(1,6)==0)
        oppo=senddata(1,6);
    end
    if not(senddata(1,5)==0)
        adj=senddata(1,5);
    end
    senddata(2,9)=atand(oppo/adj)+180;
    senddata(1,9)=oppo/sind(senddata(2,9)-180);
end
if senddata(1,10)==0
    if not(senddata(1,3)==0)
        oppo=senddata(1,3);
    end
    if not(senddata(1,6)==0)
        adj=senddata(1,6);
    end
    senddata(2,10)=atand(oppo/adj)+270;
    senddata(1,10)=oppo/sind(senddata(2,10)-270);
end

lengthsd = size(senddata,2);
for i=1:lengthsd % Go through stored points based on what section it is in
    if senddata(2,7)>=senddata(2,i) && senddata(2,3)<=senddata(2,i) % Evaluate section 1
        if i==3 && not(senddata(1,i)==0) % Convert midpoint 1 if not missing
           xyresult(1,3)=senddata(1,3)+senddata(1,5);
           xyresult(2,3)=senddata(1,6);
        end
        if i==7 && not(senddata(1,i)==0) % Convert corner 1 if not missing
           xyresult(1,7)=senddata(1,5)+(senddata(1,7)*cosd(senddata(2,7)));
           xyresult(2,7)=senddata(1,6)+(senddata(1,7)*sind(senddata(2,7)));
        end
        if not(i==3) && not(i==7) && not(senddata(1,i)==0) % If bot wall present convert it if not missing
           xyresult(1,i)=senddata(1,5)+(senddata(1,i)*cosd(senddata(2,i)));
           xyresult(2,i)=senddata(1,6)+(senddata(1,i)*sind(senddata(2,i)));
        end
    end
   if senddata(2,4)>=senddata(2,i) && senddata(2,7)<=senddata(2,i) % Evaluate section 2
        if i==4 && not(senddata(1,i)==0) % Convert midpoint 2 if not missing
           xyresult(1,4)=senddata(1,5);
           xyresult(2,4)=senddata(1,4)+senddata(1,6);
        end
        if not(i==4) && not(i==7) && not(senddata(1,i)==0) % If bot wall present convert it if not missing
           xyresult(1,i)=senddata(1,5)+(senddata(1,i)*sind(90-senddata(2,i)));
           xyresult(2,i)=senddata(1,6)+(senddata(1,i)*cosd(90-senddata(2,i)));
        end
   end
   if senddata(2,8)>=senddata(2,i) && senddata(2,4)<=senddata(2,i) % Evaluate section 3
        if i==8 && not(senddata(1,i)==0) % Convert corner 2 if not missing
           xyresult(1,8)=senddata(1,5)-(senddata(1,8)*sind(senddata(2,8)-90));
           xyresult(2,8)=senddata(1,6)+(senddata(1,8)*cosd(senddata(2,8)-90));
        end
        if not(i==8) && not(i==4) && not(senddata(1,i)==0) % If bot wall present convert it if not missing
           xyresult(1,i)=senddata(1,5)-(senddata(1,i)*sind(senddata(2,i)-90));
           xyresult(2,i)=senddata(1,6)+(senddata(1,i)*cosd(senddata(2,i)-90));
        end
   end
   if senddata(2,5)>=senddata(2,i) && senddata(2,8)<=senddata(2,i) % Evaluate section 4
        if i==5 && not(senddata(1,i)==0) % Convert midpoint 3 if not missing
           xyresult(1,5)=0
           xyresult(2,5)=senddata(1,6)
        end
        if not(i==5) && not(i==8) && not(senddata(1,i)==0) % If bot wall present convert it if not missing
           xyresult(1,i)=senddata(1,5)-(senddata(1,i)*cosd(180-senddata(2,i)));
           xyresult(2,i)=senddata(1,6)+(senddata(1,i)*sind(180-senddata(2,i)));
        end
   end
   if senddata(2,9)>=senddata(2,i) && senddata(2,5)<=senddata(2,i) % Evaluate section 5
        if i==9 && not(senddata(1,i)==0) % Convert corner 3 if not missing
           xyresult(1,9)=senddata(1,5)-(senddata(1,9)*cosd(senddata(2,9)-180));
           xyresult(2,9)=senddata(1,6)-(senddata(1,9)*sind(senddata(2,9)-180));
        end
        if not(i==9) && not(i==5) && not(senddata(1,i)==0) % If bot wall present convert it if not missing
           xyresult(1,i)=senddata(1,5)-(senddata(1,i)*cosd(senddata(2,i)-180));
           xyresult(2,i)=senddata(1,6)-(senddata(1,i)*sind(senddata(2,i)-180));
        end
   end
   if senddata(2,6)>=senddata(2,i) && senddata(2,9)<=senddata(2,i) % Evaluate section 6
        if not(i==6) && not(i==9) && not(senddata(1,i)==0) % If bot wall present convert it if not missing
           xyresult(1,i)=senddata(1,5)-(senddata(1,i)*sind(270-senddata(2,i)));
           xyresult(2,i)=senddata(1,6)-(senddata(1,i)*cosd(270-senddata(2,i)));
        end
   end
   if senddata(2,10)>=senddata(2,i) && senddata(2,6)<=senddata(2,i) % Evaluate section 7
        if i==6 && not(senddata(1,i)==0) % Convert midpoint 4 if not missing
           xyresult(1,6)=senddata(1,5)
           xyresult(2,6)=0
        end
        if not(i==10) && not(i==6) && not(senddata(1,i)==0) % If bot wall present convert it if not missing
           xyresult(1,i)=senddata(1,5)+(senddata(1,i)*sind(senddata(2,i)-270));
           xyresult(2,i)=senddata(1,6)-(senddata(1,i)*cosd(senddata(2,i)-270));
        end
   end
   if 360>=senddata(2,i) && senddata(2,10)<=senddata(2,i) % Evaluate section 8
        if i==10 && not(senddata(1,i)==0) % Convert corner 4 if not missing
           xyresult(1,10)=senddata(1,5)+(senddata(1,10)*sind(senddata(2,10)-270));
           xyresult(2,10)=senddata(1,6)-(senddata(1,10)*cosd(senddata(2,10)-270));
        end
        if not(i==10) && not(senddata(1,i)==0) % If bot wall present convert it if not missing
           xyresult(1,i)=senddata(1,5)+(senddata(1,i)*cosd(360-senddata(2,i)));
           xyresult(2,i)=senddata(1,6)-(senddata(1,i)*sind(360-senddata(2,i)));
        end
   end
end

plot(xyresult(1,:),xyresult(2,:), '.');