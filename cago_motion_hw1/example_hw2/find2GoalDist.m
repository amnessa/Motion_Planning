function [pt,dist] = find2GoalDist(reading,Oi,rb,gl)

% map = [ 0,0; 300,0; 300,300; 0,300; 0,0 ];
% obs1 = [100,90;140,120;200,100;190,180;100,150;100,90];
% hnd = figure(3);
% clf(hnd);
% plot(map(:,1),map(:,2));
% hold on;
% patch(obs1(:,1),obs1(:,2),'r');
% hold on;

angles = [Oi(1):Oi(2)];
sizeAng = size(angles,2);

minDist = 450;
pt = [0,0];
dist = minDist;

for i=1:sizeAng
    
    P1 = [rb(1) + reading(angles(i)+1)*cos(deg2rad(angles(i))), ...
          rb(2) + reading(angles(i)+1)*sin(deg2rad(angles(i)))];
%     figure(3);
%     plot(P1(1),P1(2),'ko');
%     hold on;
    d2goal = sqrt((P1(1)-gl(1))^2 + (P1(2) - gl(2))^2 );
    if d2goal < minDist
        minDist = d2goal;
        dist = minDist;
        pt = P1;       
    end
end

end