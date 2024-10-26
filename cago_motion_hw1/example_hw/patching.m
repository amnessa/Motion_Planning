X=[0.1;0.2;2;1;1.5;0];
Y=[0.1;0.1;1.54;4;2;0.5];
Z=linspace(1,6,6);
C=ones(6,1)*0.1;
a=patch(X,Y,C);
%surface(a)
% Xi=[1;2;2;3];
% Yi=[1;1.6;3];
%xv=[X;X(1)];yv=[Y;Y(1)];
xv=X;yv=Y;
Xi=rand(10,1);
Yi=rand(10,1);
in=inpolygon(Xi,Yi,xv,yv)
handle=fill(X,Y,'k')
