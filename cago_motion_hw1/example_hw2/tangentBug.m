function tangentBug
% map will be a bounded rectangle 300 x 300
map = [ 0,0; 300,0; 300,300; 0,300; 0,0 ];
% robot initial pose: x = 150, y = 250, robot face north
robot = [ 150, 20];
% goal pose
goal = [150 250];
% obstacles
%case 1
% obs1 = [100,90; 200,100; 200,200; 100,200; 100,90];
%case 2
% obs1 = [100,90;140,120;200,100;190,180;100,150;100,90];
% %case 3
% obs1 = [100,70;140,100;200,80;190,130;130,130;100,70];
% obs2 = [80,190;160,160;210,200;100,250;80,190];
% case 4 - fail
obs1 = [80,190;160,160;210,200;100,250;80,190];
goal = [150 200];
% find all lines in the map
linesCell = {map, obs1};
linesArr = createLines(linesCell);

% MOVE is the movement factor for the robot to move
MOVE = 2;
% MAX is the infinite read of sensor
MAX = 450;
% to be updated robot location
xNew = 0;
yNew = 0;
% flag to go straigth to goal, default 1
goStr = 1;
% motion to goal behaviour = 1, 0 = boundary following
mot2goal = 1;
% distance left
distLeft = findSqDistance(robot,goal);
currDist = distLeft;
prevDist = currDist;
% selected Oi
prevPt = 0;
% angle to follow during boundary-following
ang2follow = 0;
% the decision to go tangent
goTang = 0;
% min follow distance
MIN = 10;
% obstacle follow started
obsFol = 0;
% dreach and dfollowed
dreach = 0;
dfollowed = 0;
minDFollowed = dfollowed;

firstBound = 1;
step = 1;
% pose of the robot when boundry following started
% for fail checking
boundStPos = [];
failed = 0;

while 1
    % plot map
    hnd = figure(1);
    plot(map(:,1),map(:,2));
    hold on;
    patch(obs1(:,1),obs1(:,2),'r');
     hold on;
         plot(goal(1),goal(2),'bo');
    hold on;
%     patch(obs2(:,1),obs2(:,2),'r');
%     hold on;
    
    
    if mot2goal 
        %% motion to goal
        
        % update prevDist
        prevDist = currDist;
        
        % find distance&angle from robot to goal
        % invoke virtual pose sensors
        gl = getGoalPose(goal);
        rb = getRobotPose(robot);
        
        figure(1);
        plot(rb(1),rb(2),'g*');
        hold on;
        title('motion-to-goal');
        hold on;
        
        angRob2Goal = atan2(gl(2) - rb(2), gl(1) - rb(1));
        degAng = rad2deg(angRob2Goal);
        % read sensor
        reading = readRangeSensor(linesArr,rb);
        % check if there's obstacle on the direct line
        ang2Read = mod(round(degAng)+1,360);
        
       % check for obstacles on the line to goal
        if  reading(ang2Read) == MAX % there's no obstacle
           goStr = 1; 
        else % there's obstacle
           goStr = 0;
           % we shall consider rob2Oi + Oi2goal as the distance function
           % instead of the direct distance from robot to goal
           if obsFol == 0
               prevDist = MAX;
               obsFol = 1;
           end
           
           % check if robot has too close to obstacle
           if  reading(ang2Read) < MIN
               goTang = 1; % invoke tangent motion
           else
               goTang = 0;
           end
        end

        % if robot is following the path then motion-to-goal is activated on
        % the straight line
        if goStr
            %% follow straigth line to goal
            xNew = rb(1) + MOVE*cos(angRob2Goal);
            yNew = rb(2) + MOVE*sin(angRob2Goal);
            robot(1) = xNew;
            robot(2) = yNew;
            currDist = findSqDistance(rb,gl);
        else % there's obstacle, find Oi's on the obstacle 
            %% approach Oi's on the followed obstacle
            % find obstacles
            obs = findObstacles(reading,rb,gl);
            % find the discontinuity points
            ind = find(obs>=round(degAng),1);
            if mod(ind,2) == 1
                Oi = [obs(ind) , obs(ind+1)];
            else
                Oi = [obs(ind-1) , obs(ind)];
            end
           
            if Oi(1) - Oi(2) == 0 % if the sensor sensed only a point
                
                xNew = rb(1) + MOVE*cos(angRob2Goal);
                yNew = rb(2) + MOVE*sin(angRob2Goal);
                robot(1) = xNew;
                robot(2) = yNew;
                currDist = findSqDistance(rb,gl);
                obsFol = 0;
            else % sensor has detected a segment intersecting the 
                 % line to the goal
                 
                % find the points of these disconts.
%                 sensInd = mod(round(degAng)+1,360); 
                P1 = [rb(1) + reading(Oi(1)+1)*cos(deg2rad(Oi(1))), ...
                      rb(2) + reading(Oi(1)+1)*sin(deg2rad(Oi(1)))];

                P2 = [rb(1) + reading(Oi(2)+1)*cos(deg2rad(Oi(2))), ...
                      rb(2) + reading(Oi(2)+1)*sin(deg2rad(Oi(2)))];
                
                figure(1);
                plot(P1(1),P1(2),'ko');
                hold on;
                plot(P2(1),P2(2),'ko');
                hold on;
                  
                % find the point that minimizes the rob2Oi + Oi2goal
                rob2O1 = findSqDistance(rb,P1); 
                O12goal = findSqDistance(P1,gl);  
                distO1 = rob2O1 + O12goal;

                rob2O2 = findSqDistance(rb,P2); 
                O22goal = findSqDistance(P2,gl); 
                distO2 = rob2O2 + O22goal;

                minOi = min(distO1,distO2);
                
                if prevPt >0
                   if prevPt == 1
                       minOi = distO1;
                   else
                       minOi = distO2;
                   end
                end
                
                % select the point that minimizes the distances
                if minOi == distO1 % select P1    
                    angRob2Oi = atan2(P1(2)-rb(2), P1(1) - rb(1));
                    prevPt = 1;
                    currDist = distO1;
                else
                    angRob2Oi = atan2(P2(2)-rb(2), P2(1) - rb(1));
                    prevPt = 2;
                    currDist = distO2;
                end
                
                % if distance started to increase initiate boundary
                % following procedure
                if prevDist < currDist                                   
                    mot2goal = 0;
                    ang2follow = angRob2Oi; 
                    dfollowed = minDist;
                    firstBound = 1;
                else
                    mot2goal = 1; 
                    
                    % update min dist ever sensed
                    if minOi == distO1
                        minDist = O12goal;
                    else
                        minDist = O22goal;
                    end
                    
                    
                    if goTang
                        degOi = rad2deg(findTangentAngle(angRob2Oi,prevPt)); 
                        xNew = rb(1) + MOVE*cos(findTangentAngle(angRob2Oi,prevPt));
                        yNew = rb(2) + MOVE*sin(findTangentAngle(angRob2Oi,prevPt));
                        robot(1) = xNew;
                        robot(2) = yNew;  
                    else
                        degOi = rad2deg(angRob2Oi);
                        xNew = rb(1) + MOVE*cos(angRob2Oi);
                        yNew = rb(2) + MOVE*sin(angRob2Oi);
                        robot(1) = xNew;
                        robot(2) = yNew;  
                    end
                end    


           
            end
                                 
        end

    else
        %% boundary following
        % invoke virtual pose sensors
        gl = getGoalPose(goal);
        rb = getRobotPose(robot);

        % angle to goal
        angRob2Goal = atan2(gl(2)-rb(2), gl(1) - rb(1));
        degAng = rad2deg(angRob2Goal);

        figure(1);
        plot(rb(1),rb(2),'g*');
        hold on;
        title('boundary-following');
        hold on;
        
        if firstBound 
           boundStPos = rb;
        end
        
        % read sensor
        reading = readRangeSensor(linesArr,rb);
        % check if there's obstacle on the direct line
        ang2Read = mod(round(degAng)+1,360); 
        
        obs = findObstacles(reading,rb,gl);
        % find the discontinuity points
        ind = find(obs>=round(degAng),1);
        if mod(ind,2) == 1
            Oi = [obs(ind) , obs(ind+1)];
        else
            Oi = [obs(ind-1) , obs(ind)];
        end
        % find the closest point to the robot on the boundary
        [val,ind] =min(reading(Oi(1)+1 : Oi(2)+1));
        
        if val == 450 % saw the goal ahead! or sonar max range
            % initiate motion to goal
            mot2goal = 1;
            firstBound = 1;
            currDist = dreach;
            obsFol = 0; 
            step = 0;
        else
            Prx = rb(1) + val*cos(deg2rad(Oi(1)+ind-1));
            Pry = rb(2) + val*sin(deg2rad(Oi(1)+ind-1));
            Pr = [Prx Pry];

            plot(Pr(1),Pr(2),'b*');
            hold on;

            % distance between the goal and the closest point on the boundary
            % to the robot
            % angle between the closest pt and robot
            angRob2Pr = atan2(Pr(2)-rb(2), Pr(1) - rb(1));
            angRob2Pr = findTangentAngle(angRob2Pr,prevPt);

            [p,dreach] = find2ReachDist(reading,rb,gl);

            if firstBound == 1
                firstBound = 0;
                step = step + 1;
                minDFollowed = dfollowed;
                xNew = rb(1) + MOVE*cos(ang2follow);
                yNew = rb(2) + MOVE*sin(ang2follow);
                robot(1) = xNew;
                robot(2) = yNew;
            else
                
                if step > 50
                    if findSqDistance(boundStPos,rb) < 5
                        failed = 1;                        
                    end                    
                end
                                                            
                [pt,dfollowed] = find2GoalDist(reading,Oi,rb,gl);
                if dfollowed < minDFollowed
                    minDFollowed = dfollowed;
                end
                % check for the end of boundary following
                if dreach < minDFollowed
                    mot2goal = 1;
                    firstBound = 1;
                    currDist = dreach;
                    obsFol = 0;
                    step = 0;
                else
                    xNew = rb(1) + MOVE*cos(angRob2Pr);
                    yNew = rb(2) + MOVE*sin(angRob2Pr);
                    robot(1) = xNew;
                    robot(2) = yNew;
                end
           
            end
            
        end
       
    end 

    
  %% PLOTTING
    % plot the robot and covered path
%     figure(1)
% %     plot(xNew,yNew,'r*');
% %     hold on;
%     plot(robot(1),robot(2),'g.');
%     hold on;
%     if mot2goal
%         title('motion-to-goal');
%     else
%         title('boundary-following');
%     end
%     hold on;
    % plot the readings
%     reading = readRangeSensor(linesArr,robot);
%     figure(2);
%     plot(reading);
%     axis([0,360,0,500]);
    if failed 
        title('FAILED');
        break;
    end


    if findSqDistance(robot,goal) < 5
        title('SUCCESS');
        break;
    else
%         clf(hnd);
    end
    
end

end