%                                                           (__)
%                                                           / /
%                                                          / /
%BILKENT UNIVERSITY                          ___          / /
%DEPT. OF ELECTRICAL&ELECTRONICS            / _ \        / /
%ENGINEERING                               / / \ \      / /
%ANKARA                                   / /   \ \    / /
%ALÝ NAÝL ÝNAL - 20501174                / /     \ \  / /
%Research Assistant                     / /_______\ \/ /
%CS 548  Homework # 1                  / /---------\  /
%DUE: 22.01.2010                      / /           \/
%                                    / /
%                                   / /
%                                  / /
%                                 / /
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
J=complex(0,1);
pos_bug=[3,6];
pos_goal=[9 1];%2,2.5 zor ama hedeflenen case
map_size=10;
range=1.5;
sensor_res=100;
step_size=0.01*map_size;
%obs=get_obs({'test2', 'rect','circle';[1 2 6], [1.5 4 2 1] ,[3 1 0.5]});
obstacles={ 'rect','circle','bound';[4 9 2 3] ,[5 3 2],[10]};
% obstacles={ 'rect','bound';[4 9 2 6] ,[10]};
% obstacles={ 'circle','bound';[5 3 2],[10]};
obs=get_obs(obstacles);
%%%%%BUG ALGORITHM%%%%%%%%%%%%%%
d_f=inf;%minimum total distance from bug to end point to goal point
d_f_n=d_f;
k=1;
while (true)
    diff=pos_goal-pos_bug;
    d_reach=sqrt(diff(1)^2+diff(2)^2);
    goal_angle=make_limited(angle(diff(1)+J*diff(2))*180/pi);
    if(d_reach<=step_size)
        break
    end
    [t d dp]=get_sensor(obs,range,pos_bug,sensor_res);
    print_map(obs,pos_bug,pos_goal,dp);
    frames_mov(k)=getframe;
    k=k+1;
    %motion to goal
    if (d(floor(goal_angle)+1)>=min(range,d_reach) & d(ceil(goal_angle)+1)>=min(range,d_reach))
        pos_bug=pos_bug+[cos(goal_angle*pi/180) sin(goal_angle*pi/180)]*step_size;
    else
        endp=endpoints(d,range,map_size);
        if (~isempty(endp))
            for i=1:length(endp)
                plot(dp(1,endp(i)),dp(2,endp(i)),'o','color','red')
                dummy_f=dp(:,endp(i))-pos_goal';
                dummy_f=sqrt(sum(dummy_f.^2));
                dummy_d=dp(:,endp(i))-pos_bug';
                dummy_d=sqrt(sum(dummy_d.^2));
                if(d_f_n>dummy_f+dummy_d)
                    d_f_n=dummy_f+dummy_d;
                    go_end_p=dp(:,endp(i))';
                end
            end
            if(d_f_n<=d_f)
                d_f=d_f_n;
                go_end=go_end_p;
                dummy_d2=go_end-pos_bug;
                dummy_angle=make_limited(angle(dummy_d2(1)+J*dummy_d2(2))*180/pi);
                dummy_d2=sqrt(sum(dummy_d2.^2));
                limitlow=min(make_limited(round(dummy_angle-90)),make_limited(round(dummy_angle+90)));
                limitmax=max(make_limited(round(dummy_angle-90)),make_limited(round(dummy_angle+90)));
                [dumm_min index_min]=min(d(limitlow:limitmax));
                if(dumm_min>3*step_size)
                    pos_bug=pos_bug+[cos(dummy_angle*pi/180) sin(dummy_angle*pi/180)]*step_size;
                else
                    %nearest point to end point vector that we also use in
                    %a safe distance
                    go_vector=go_end-dp(:,index_min)';
                    pos_bug=pos_bug+go_vector/norm(go_vector)*step_size;
                end
                %%%%%%%%%%%%follow boundry
            else
                
                [dumm_min index_min]=min(d);
                d_re=d_f+1;
                while(d_re>d_f)
                    %Ýs there enough space to go one step size???????????
                    if(((t(index_min)-90)-dummy_angle)<91)
                        %0 360 arasýna taþý
                        dummy_angle=make_limited(t(index_min)-90);
                        pos_bug=pos_bug+[cos(dummy_angle*pi/180) sin(dummy_angle*pi/180)]*step_size;
                    else
                        dummy_angle=make_limited(t(index_min)+90);
                        pos_bug=pos_bug+[cos(dummy_angle*pi/180) sin(dummy_angle*pi/180)]*step_size;
                    end
                    
                    diff=pos_goal-pos_bug;
                    d_reach=sqrt(diff(1)^2+diff(2)^2);
                    goal_angle=make_limited(angle(diff(1)+J*diff(2))*180/pi);
                    
                    if(d_reach<step_size)
                        break
                    end
                    [t d dp]=get_sensor(obs,range,pos_bug,sensor_res);
                    [dumm_min index_min]=min(d);
                    dum_goal=sqrt((dp(1,index_min)-pos_goal(1))^2+(dp(2,index_min)-pos_goal(2))^2);
                    d_re=d(index_min)+dum_goal;
                    print_map(obs,pos_bug,pos_goal,dp);
                    frames_mov(k)=getframe;
                    k=k+1;
                    
                end
            end
        else
            disp('No solution exist');
            break
        end
    end
end









