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