%%%%%%%%%%%%%%%%%%%TEST CASE ini uygun ÞEKLE SOK!!!!!!!!!!!!!!!!!!!!

function obstacles=get_obs(names)
nums_obs=size(names);
obstacles=cell(size(names));
for i=1:nums_obs(2)
    obs_name=names{1,i};
    obs_pos=names{2,i};
    switch obs_name;
        case 'circle'
            %for a circle x&y position of center and radius should be given
            res=40;
            ang=linspace(0,2*pi,res);
            rad=ones(1,res)*obs_pos(3);
            [X,Y] = pol2cart(ang,rad);
            %vertices of circle is obtained
            X=X(1,:)+obs_pos(1);
            Y=Y(1,:)+obs_pos(2);
        case 'rect'
            %for a square or rectangle x&y position of left upper corner
            %and length(parallel to x axis) and width(parallel to y axis)
            %should be given
            %vertices of rectangle is obtained
            X=[obs_pos(1);obs_pos(1)+obs_pos(3);obs_pos(1)+obs_pos(3);obs_pos(1)];
            Y=[obs_pos(2);obs_pos(2);obs_pos(2)-obs_pos(4);obs_pos(2)-obs_pos(4)];
        case 'Parallel'
            %for a square or rectangle x&y position of left upper corner
            %and length(parallel to x axis) and width(parallel to y axis)
            %and CCW angle to the x axis should be given
            %vertices of rectangle is obtained
            angle=obs_pos(5)*pi/180;
            X=[obs_pos(1);obs_pos(1)+obs_pos(3)*cos(angle);obs_pos(1)+obs_pos(3)*cos(angle);obs_pos(1)];
            Y=[obs_pos(2);obs_pos(2)+obs_pos(3)*sin(angle);obs_pos(2)+obs_pos(3)*sin(angle)-obs_pos(4);obs_pos(2)-obs_pos(4)];
        case 'test'
            %test object with rigth lower corner x-y coordinates
            %vertices of object is obtained
            X=[0.1;0.2;2;1;1.5;0]+obs_pos(1);
            Y=[0.1;0.1;1.54;4;2;0.5]+obs_pos(2);
        case 'test2'
            %test object with rigth lower corner x-y coordinates
            %vertices of object is obtained
            d=[0 0;0.5 0;0.5 0.5;0.45 0.5;0.45 0.05;0.05 0.05;0.05 0.95;...
                0.95 0.95;0.95 0.05;0.85 0.05;0.85 0.75;0.2 0.75;0.2 0.7;...
                0.8 0.7;0.8 0;0.95 0;1 0;1 1;0 1];
            X=obs_pos(3)*d(:,1)+obs_pos(1);
            Y=obs_pos(3)*d(:,2)+obs_pos(2);
        case 'bound'
            %boundry object with mapsize
            %vertices of object is obtained
            X=[0; 1; 1; 0; -0.05; -0.05; 1.05; 1.05 ;-0.05;-0.05; 0]*obs_pos(1);
            Y=[0; 0; 1; 1;     1;  1.05; 1.05;-0.05 ;-0.05;    1; 1]*obs_pos(1);
        case 'testbon'
            %test object with rigth lower corner x-y coordinates
            %vertices of object is obtained
            X=obs_pos(3)*[0;-1;1;-1]+obs_pos(1);
            Y=obs_pos(3)*[0; 1;0;-1]+obs_pos(2);
        otherwise
            X=[1,0,-1,0];
            Y=[0,1,0,-1];
    end
    obstacles{1,i}=X;
    obstacles{2,i}=Y;
end