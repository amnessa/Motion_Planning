function [pt,dist] = find2ReachDist(reading,rb,gl)

sens = reading;
sens(sens==450)= 30;

minDist = 450;
pt = [0,0];
dist = minDist;

for i = 1:360
       
    P1 = [rb(1) + sens(i)*cos(deg2rad(i-1)), ...
          rb(2) + sens(i)*sin(deg2rad(i-1))]; 
      
    d2goal = sqrt((P1(1)-gl(1))^2 + (P1(2) - gl(2))^2 );
    
    if d2goal < minDist
        minDist = d2goal;
        dist = minDist;
        pt = P1;       
    end
    
end


end