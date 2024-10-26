%%%obstacles will be a cell that contains polygos vx(first line),and
%vy(second line) s by using size we will use it  thus #of obstacles will
%not be important
%%%range will be the range of the sensor
%%%resolution of the sensor will be range/sensor_res
%%%pos=[pos_x pos_y] of the bug
function [theta distance dist_pos]=get_sensor(obstacle,range,pos,res)
% #of obstacles
N_obs=size(obstacle);N_obs=N_obs(2);
res_sample=360;
sensor_res=res;%sensor_res=2000;
theta=linspace(0,2*pi*(1-1/res_sample),res_sample);
distance=ones(1,length(theta))*range;
dist_pos=zeros(2,length(theta));
for i=1:res_sample;
    distance_list(1,:)=linspace(pos(1),pos(1)+range*cos(theta(i)),sensor_res);
    distance_list(2,:)=linspace(pos(2),pos(2)+range*sin(theta(i)),sensor_res);
    in=0;
    curr_distance=sensor_res;
    for j=1:N_obs
        obs_x=obstacle{1,j};
        obs_y=obstacle{2,j};
        in=inpolygon(distance_list(1,:),distance_list(2,:),obs_x,obs_y);
        dummy=find(in);
        if(~isempty(dummy))
            if(dummy(1)<curr_distance)
                curr_distance=dummy(1);
            end
        end
    end
    distance(i)=curr_distance*range/sensor_res;
    dist_pos(:,i)=[pos(1)+distance(i)*cos(theta(i));pos(2)+distance(i)*sin(theta(i))];
end
theta=theta*180/pi;