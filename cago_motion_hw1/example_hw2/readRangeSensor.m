function readings = readRangeSensor(lines,currLoc)
% 360 discrete readings to be returned
readings = zeros(1,360);
% assume square shaped maps
% MAXREADING defines the longest ray from currLoc
MAXREADING = 30;
% currLoc = [x,y]

% apply ray casting to find discrete 360 rays for distance
% assume no error in sensor readings
% max sensor reading would be 200
% infinite would be 600

% radian arrays
 cosAngs = cos(deg2rad([0:1:359]'));
 sinAngs = sin(deg2rad([0:1:359]'));
 
 % find 360 degree rays emanating from currLoc
 % rowPs & colPs = 360xMAXREADING vectors
 % those include MAXREADING number of points in each ray 
 % angle: from 1:360
 
 xPs = currLoc(1) + cosAngs*MAXREADING;
 yPs = currLoc(2) + sinAngs*MAXREADING;
 
 xCurr = repmat(currLoc(1),360,1);
 yCurr = repmat(currLoc(2),360,1);
 
 % 360 degree of laser lines of MAXREADING length
 laserLines = [xCurr yCurr xPs yPs];
 lineSize = size(lines,1);

 
 for i =1:360
     % find here the obstacles that are found on the line
     % created by MAXREADING of point for each angle i    
      minDist = 450;
      
     for j = 1:lineSize
         ln1 = [lines(j,1),lines(j,2);lines(j,3),lines(j,4)];
         ln2 = [laserLines(i,1),laserLines(i,2);laserLines(i,3),laserLines(i,4)];
        [xInt,yInt] = curveIntersect(ln1(:,1),ln1(:,2),ln2(:,1),ln2(:,2)); 
        
        if (isempty(xInt) && isempty(yInt))
            dist = 450;
        else
            dist = sqrt((currLoc(1) - xInt)^2 + (currLoc(2) - yInt)^2);
        end
        
        if dist<minDist
            minDist = dist;
        end
                                         
     end
     
     readings(1,i ) = minDist;
     
 end
 
end