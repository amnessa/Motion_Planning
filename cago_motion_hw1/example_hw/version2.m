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
pos_bug=[4 5];
pos_goal=[8.5 6 ];%2,2.5 zor ama hedeflenen case
map_size=10;
range=1.5;
sensor_res=100;
safety_factor=9;
step_size=0.005*map_size;
%obs=get_obs({'test2', 'rect','circle';[1 2 6], [1.5 4 2 1] ,[3 1 0.5]});
% obstacles={ 'rect','circle','bound' 'test2';[1.5 4 2 1] ,[3 1 0.5],[map_size],[1 2 6]};
% obstacles={ 'rect','bound' 'test2';[1.5 4 2 1] ,[map_size],[1 2 6]};
% obstacles={ 'rect','bound' ,'testbon','circle','circle','circle';[8 8 1 1] ,[map_size],[5 5 3],[5.5 5 1],[2 8 0.5],[2 2 0.5]};
obstacles={ 'rect','bound' ,'testbon','circle';[8 8 1 1] ,[map_size],[5 5 3],[8 5 0.6]};
% obstacles={ 'rect','bound';[4 9 2 6] ,[map_size]};
% obstacles={ 'circle','bound';[5 3 2],[map_size]};
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
    [dumm_min index_min]=min(d);
    a_min=find(d==dumm_min);
    a_min=sum(a_min)/length(a_min);
    index_min=round(a_min);
    %motion to goal
    if (d(floor(goal_angle)+1)>=min(range,d_reach) && d(ceil(goal_angle)+1)>=min(range,d_reach) && dumm_min>safety_factor*step_size)
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
            if(d_f_n<d_f)
                d_f=d_f_n;
                go_end=go_end_p;
                dummy_d2=go_end-pos_bug;
                dummy_angle=make_limited(angle(dummy_d2(1)+J*dummy_d2(2))*180/pi);
                dummy_d2=sqrt(sum(dummy_d2.^2));
                [dumm_min index_min]=min(d);
                if(dumm_min>safety_factor*step_size)
                    pos_bug=pos_bug+[cos(dummy_angle*pi/180) sin(dummy_angle*pi/180)]*step_size;
                else
                    %nearest point to end point vector that we also use in
                    %a safe distance
                    go_vector=go_end-dp(:,index_min)';
                    escape=dp(:,index_min)'-pos_bug;
                    pos_bug=pos_bug-escape/norm(escape)*(safety_factor*step_size-d(index_min));
                    pos_bug=pos_bug+go_vector/norm(go_vector)*step_size;
                end
                %%%%%%%%%%%%follow boundry
            else
                
                d_re=d_f;
                if(((t(index_min))-dummy_angle)<0 && dumm_min>safety_factor*step_size)
                    %0 360 arasýna taþý
                    dummy_angle=make_limited(t(index_min)-90);
                    pos_bug=pos_bug+[cos(dummy_angle*pi/180) sin(dummy_angle*pi/180)]*step_size;
                elseif((t(index_min))-dummy_angle>=0 && dumm_min>safety_factor*step_size)
                    dummy_angle=make_limited(t(index_min)+90);
                    pos_bug=pos_bug+[cos(dummy_angle*pi/180) sin(dummy_angle*pi/180)]*step_size;
                else
                    escape=dp(:,index_min)'-pos_bug;
                    pos_bug=pos_bug-escape/norm(escape)*(safety_factor*step_size-d(index_min));
                end
                d_angle=dummy_angle;
                while(true)
                    disp('Follow Boundry');
                    if(dumm_min>5*step_size)
                        d_angle=make_limited(t(index_min)-90);
                        pos_bug=pos_bug+[cos(d_angle*pi/180) sin(d_angle*pi/180)]*step_size;
                    else
                        escape=dp(:,index_min)'-pos_bug;
                        pos_bug=pos_bug-escape/norm(escape)*(5*step_size-d(index_min));
                    end
%                     if(((t(index_min))-d_angle)<0 && dumm_min>5*step_size)
%                         d_angle=make_limited(t(index_min)-90);
%                         pos_bug=pos_bug+[cos(d_angle*pi/180) sin(d_angle*pi/180)]*step_size;
%                    elseif((t(index_min))-d_angle>=0 && dumm_min>5*step_size)
%                         d_angle=make_limited(t(index_min)+90);
%                         pos_bug=pos_bug+[cos(d_angle*pi/180) sin(d_angle*pi/180)]*step_size;
%                     else
%                         escape=dp(:,index_min)'-pos_bug;
%                         pos_bug=pos_bug-escape/norm(escape)*(5*step_size-d(index_min));
%                     end
                    
                    diff=pos_goal-pos_bug;
                    d_reach=sqrt(diff(1)^2+diff(2)^2);
                    goal_angle=make_limited(angle(diff(1)+J*diff(2))*180/pi);
                    
                    if(d_reach<step_size | d_re<d_f)
                        break
                    end
                    sensor_res=100;
                    [t d dp]=get_sensor(obs,range,pos_bug,sensor_res);
                    [dumm_min index_min]=min(d);
                    a_min=find(d==dumm_min);
                    a_min=sum(a_min)/length(a_min);
                    index_min=round(a_min);
                    dum_goal=sqrt((dp(1,index_min)-pos_goal(1))^2+(dp(2,index_min)-pos_goal(2))^2);
                    d_re=d(index_min)+dum_goal;
                    print_map(obs,pos_bug,pos_goal,dp);
                    frames_mov(k)=getframe;
                    k=k+1;
                    
                end
                disp('Follow Boundry FÝNÝSHED');
                sensor_res=100;
            end
        else
            disp('No solution exist');
            break
        end
    end
end
movie(frames_mov)








