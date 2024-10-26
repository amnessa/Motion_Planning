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