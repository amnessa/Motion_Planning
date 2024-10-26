%takes first derivative to find discontinuities of the data given
function index=endpoints(dist,range,map_size)
%discontinuity indice
a=min(2,range);
a=min(a,map_size);
ind=0.1*a;
for i=1:length(dist)
    if(dist(i)>=range)
        dist(i)=0;
    end
end
dist(end+1)=dist(1);
dist2=[dist(2:end) 0];
dumm=abs(dist2-dist);
dumm=dumm(1:end-1);
index=find(dumm>ind);
% dumm=dist2-dist;
% diff=dumm(1:end-1);
% rise_edge=(diff>ind);
% fall_edge=(diff<-ind);
% fall_edge=circshift(fall_edge,[0 1]);
% index_dumm=fall_edge+rise_edge;
% index=find(index_dumm>0);
