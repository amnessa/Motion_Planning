function a=print_map(obs,pos,pos_goal,dp)
    hold off
    plot(dp(1,:),dp(2,:),'.','color','blue') 
    hold on
    plot(pos(1),pos(2),'*','color','green')
    plot(pos_goal(1),pos_goal(2),'O','color','red')
    [a b]=size(obs);
    for i=1:b
    fill(obs{1,i},obs{2,i},'k')
    grid on
    axis([ 0,10 0 10])
    end
