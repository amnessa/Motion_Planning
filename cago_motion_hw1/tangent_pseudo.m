% Initialize parameters
MAX_SENSOR_READING = 450;
move_step = 2;  % Movement step size for the robot
minDFollowed = inf;
firstBound = 1;
step = 1;
failed = 0;

% Continuously check robot position relative to obstacles and goal
while true
    % 1. Calculate the angle from the robot to the goal
    angle_to_goal = atan2(qgoal(2) - position(2), qgoal(1) - position(1));
    
    % 2. Read sensor data to check for obstacles
    reading = readRangeSensor(lines, position);
    
    % 3. Identify closest obstacles based on sensor data
    obstacle_info = findObstacles(reading, position, qgoal);
    
    % 4. Determine if there is a clear path or if boundary-following is needed
    if closest_obstacle == MAX_SENSOR_READING
        % Clear path detected, move towards the goal
        position = [position(1) + move_step * cos(angle_to_goal), ...
                    position(2) + move_step * sin(angle_to_goal)];
        break; % Switch back to motion-to-goal behavior
    else
        % Obstacles detected, switch to boundary-following mode
        if firstBound
            firstBound = 0;
            % Initial boundary-following step, move tangentially
            new_position = [position(1) + move_step * cos(angle_to_goal), ...
                            position(2) + move_step * sin(angle_to_goal)];
        else
            % Detect and follow the boundary tangentially
            closest_point = findClosestBoundaryPoint(reading, position, obstacle_info);
            angle_to_obstacle = atan2(closest_point(2) - position(2), ...
                                      closest_point(1) - position(1));
            
            % Move tangentially along the obstacle
            new_position = [position(1) + move_step * cos(angle_to_obstacle), ...
                            position(2) + move_step * sin(angle_to_obstacle)];
            
            % Update position and distance tracking
            position = new_position;
            
            % Exit if the robot loops back or the goal is visible
            if step > 50 && distanceToStart < 5
                failed = 1; % Exit with failure if stuck in a loop
                break;
            end
        end
    end
    
    % Increment step counter
    step = step + 1;
end

% Output status or move to next step in your main algorithm
if failed
    disp('Boundary-following failed, unable to reach goal.');
else
    disp('Successfully navigated obstacle, continuing towards goal.');
end

function ret = findObstacles(readings, currPose, goalPose)

    % MINTHRES is the min dist. the robot can get close
    % DISTTHRES is the thres to detect discontinuities
    
    MINTHRES = 5;
    DISTTHRES = 20;
    
    % quickly find discountinuities in the readings
    endArr = [readings(end) readings];
    begArr = [readings readings(1)];
    difArr = abs(begArr-endArr);
    
    % indices in the readings pointing discont. angle locs
    discLocs = find(difArr>=DISTTHRES);
    
    x = [1 discLocs 361];
    x = x - 1;
    sizeX = size(x,2);
    ret = zeros(1,(sizeX-1)*2);
    for i = 1:(sizeX-1)
        ret(i*2-1:i*2) = [x(i),x(i+1)-1];
    end
    
    end

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

function angRev = findTangentAngle(angle,dir)

    % angleX = rad2deg(angle);
    
    % if angleX < 0
    %     angleX = angleX +360;
    % end
    % 
    % if 0<= angleX && angleX <90
    %     angRev = angleX + 90;
    % elseif 90<= angleX && angleX <180
    %     angRev = angleX - 90;
    % elseif 180<= angleX && angleX <270
    %     angRev = angleX - 90;   
    % else
    %     angRev = angleX - 270;
    % end
          if dir == 1          
            angleX = angle - pi/2;
          else
            angleX = angle + pi/2;
          end
          
          if(angleX>2*pi)
                angleX = angleX - 2*pi;
          elseif(angleX<0)
                angleX = angleX + 2*pi;
          end
    
    angRev = angleX;
end