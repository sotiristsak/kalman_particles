clear all;
close all;
%%Load data
load('dataset/control.dat');
radar = load('dataset/radar1.dat');

n=4;      %number of states
dt=0.1;
q=0.1;    %std of process 
r=0.1;    %std of measurement

Q = q/1000 * eye(n); 
% Q = [q 0 0 0 ; 0 q 0 0 ; 0 0 (q*10^-4) 0; 0 0 0 (q*10^-4)];
R=r^2;        % covariance of measurement

% A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
f=@(x)[x(1) + x(3)*dt;x(2) + x(4)*dt;x(3);x(4)];% nonlinear state equations

ux = -0.27;
uy = 0.12;

% s=[3.1;2.5;ux;uy];        % initial state
% s=[4.1;3.3;ux;uy];        % initial state                        
% s=[5.1;4;ux;uy];          % initial state
% s=[5.5;4.5;ux;uy];        % initial state
s=[6.1;4.8;ux;uy];        % initial state  
% s=[1.45;-0.3;ux;uy];      % initial state
% s=[7.1;5.6;ux;uy];        % initial state
% s=[7.1;7.6;ux;uy];        % initial state
% s=[8.1;6.3;ux;uy];        % initial state
% s=[55;38;ux;uy];          % initial state
x = s; 


P = [q*50 0 0 0; 0 q*50 0 0; 0 0 q/100 0; 0 0 0 q/100];     % initial state covariance


N=100;                                    % total dynamic steps
xV = zeros(n,N);          %estmate        % allocate memory
sV = zeros(n,N);          %actual 
zV = zeros(1,N);          %measurement
estimateV = zeros(n,N);
ligooV = zeros(n,N);
eV = zeros(1,N);            %error

figure(2)
hold on
xlabel('X axis in meters');
ylabel('Y axis in meters');
axis equal
plot (control(:,1),control(:,2),'DisplayName','Vehicle Trajectory')
xprev = x;

eeeee = zeros(2,N);
eeeee(1,1) = s(1);
eeeee(2,1) = s(2);
for i=2:N
    eeeee(1,i) = eeeee(1,i-1) + ux*0.1;
    eeeee(2,i) = eeeee(2,i-1) + uy*0.1;
end
plot(eeeee(1,:),eeeee(2,:),'g','DisplayName','Obstacle movement without noise');
legend('show');
line_eq

% p = polyfit(1:100, radar', 7);
% x7 = linspace(1,100);
% radar_poly = polyval(p,x7);
% for i = 1 : 100 
%     l = tan(radar_poly(i) + degtorad(90));
%     if tan(radar_poly(i) + degtorad(90)) < 0
%         radar_poly(i) = -(radar_poly(i) + degtorad(90));
%         l = tan(radar_poly(i));
%         b = control(i,2) - l*control(i,1);
%         xl = control(i,1) : control(i,1) + 8;
%         yl = l * xl + b;
%         plot(-xl + 2*control(i,1),yl)
%         hold on
%     else
%         b = control(i,2) - l*control(i,1);
%         xl = control(i,1) : control(i,1) + 8;
%         yl = l * xl + b;
%         plot(xl,yl)
%         hold on
%     end
% end

for k=1:N
  ctrlX = control(k,1);
  ctrlY = control(k,2);

  h=@(x)(-atan((( ctrlX - x(1) ) / ( ctrlY - x(2) ) ) )); % measurement equation

  z = radar(k);                             % measurments
  sV(:,k)= s;                             % save actual state
  zV(k)  = z;                             % save measurment
  
  %nonlinear update and linearization at current state
  Q1 = Q;
  Q1(1,1) = Q(1,1)*randn(1);
  Q1(2,2) = Q(2,2)*randn(1);
  Q1(3,3) = Q(3,3)*randn(1);
  Q1(4,4) = Q(4,4)*randn(1);
  
  [x1,A]=jaccsd(f,x); 
  P=A*P*A'+Q1;                 %partial update

  % update
  [z1,H]=jaccsd(h,x1);    %nonlinear measurement and linearization
  P12=P*H';                   %cross covariance
  K=P12*inv(H*P12+R);       %Kalman filter gain
  x=x1+K*(z - z1);            %state estimate
  P=P-K*P12';

  figure(2)
  hold on
  error_ellipse(P(1:2, 1:2),x(1:2))
  
  ligoo = K*(z - z1);
  e = z - z1;
 

  eV(:,k) = e;                            % save error
  xV(:,k) = x;                            % save estimate

  
  % plot(sV(1,:), sV(2,:),  'b')
  plot([xprev(1) xV(1,k)],[xprev(2) xV(2,k)],  'r')
  xprev = x;
  
  %   estimateV(:,k) = est;
  ligooV(:,k) = ligoo;
  x = f(x);  
  pause(0.1);
end

hold off
for k=1:n                                 % plot results
  figure(1)
  subplot(n,1,k)
  plot(1:N, sV(k,:), 'b', 1:N, xV(k,:), 'r')
end







