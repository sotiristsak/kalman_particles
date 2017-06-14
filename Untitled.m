clc;clear
xy = rand(100,2);
plot(xy(:,1),xy(:,2),'b.')
hold on
t = linspace(0,2*pi);
plot(.5+.25*cos(t),.5+.25*sin(t),'r-')
axis square
r = hypot(xy(:,1)-.5,xy(:,2)-.5);
plot(xy(r<=.35,1),xy(r<=.35,2),'go')
