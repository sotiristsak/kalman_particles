clc
clear
M = 100;% total number of steps
X = zeros(4,1);
X([1 2]) = [10 20];     	%Initial position
X([3 4]) = [10 20];      	%Initial velocity
T=0.1;
P = eye(4)*10;

% proeraitika
% Set Q, see [1]
% sigma=1;         %state transition variance
Qxy =  [60 0;0 60];
Q = blkdiag(Qxy,Qxy);
%%
A=importdata('control.dat');
x=A(:,1);
y=A(:,2);
angle=importdata('radar1.dat');
N=0.1^2;

plot(x,y)
hold on
plot(angle)
for i = 1:M
    % Set g
    g = @(Xp,yo,xo) PseudorangeEquation(X, yo,xo);                 
    yo=y(i);xo=x(i);ang=angle(i);
    [X,P,gXp,Pp] = Extended_KF(T,X,P,Q, yo,xo,N,ang,g);
    Pos_KF(:,i) = X([1 2]);  
    V(:,i) = X([3 4]);
    angle_pred(i,1)=gXp;    

end

k=zeros(100,1);l=zeros(100,1);
k=Pos_KF(1,:); l=Pos_KF(2,:);
figure(2)
plot(k,l)
hold on
plot(x,y)


figure(3)
plot(angle)
hold on
plot(angle_pred)

figure(4)
plot(1:100,V(1,:))
hold on
plot(1:100,V(2,:))
