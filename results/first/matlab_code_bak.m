clear;
clf;
%%Load data
load('dataset/control.dat');
radar = load('dataset/radar1.dat');

global ctrlX
global ctrlY

n=4;      %number of state
dt = 0.1;
qP=00.02;    %std of process - location 
qU=0;    %std of process - velocity

r=0.1;    %std of measurement
% Q=q^2*eye(n); % covariance of process
Q = [qP 0 0 0 ; 0 qP 0 0 ; 0 0 qU 0; 0 0 0 qU];
% Q = [qP qP 0 0 ; qP qP 0 0 ; 0 0 qU qU; 0 0 qU qU];
P = Q;                               % initial state covraiance
R=r^2;        % covariance of measurement
%ctrlX = control(1,1); %initial control X - INSIDE THE LOOP
%ctrlY = control(1,2); %initial control Y - INSIDE THE LOOP
% A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
f=@(x)[x(1) + x(3)*dt;x(2) + x(4)*dt;x(3);x(4)];% nonlinear state equations
ux = -0.7;
uy = 0;
% h=@(x)(-atan(( ctrlX - x(1) ) / ( ctrlY - x(2) )));% measurement equation - INSIDE THE LOOP
% s=[3.1;2.5;ux;uy]; 
% s=[4.1;3.3;ux;uy];                                % initial state
% s=[5.1;4;ux;uy];                                  % initial state
% s=[6.1;4.8;ux;uy]; 
% s=[1.45;-0.3;ux;uy]; % initial state
% s=[7.1;5.6;ux;uy];                                % initial state
% s=[8.1;6.3;ux;uy];                                % initial state
% s=[55;38;ux;uy];                                % initial state

s=[7.1;7.6;ux;uy];              % best initial from test

% x=s+q*randn(n,1); %initial state          % initial state with noise
x = s; 
% P = 0.1*eye(n);                               % initial state covraiance

N=96;                                    % total dynamic steps
xV = zeros(n,N);          %estmate        % allocate memory
sV = zeros(n,N);          %actual
pV = zeros(n,N*4);          %save P - for error ellipse
zV = zeros(1,N);
estimateV = zeros(n,N);
ligooV = zeros(n,N);
eV = zeros(1,N);            %error
for k=1:N
  ctrlX = control(k,1);
  ctrlY = control(k,2);
%   h=@(x)(-atan2( ctrlX - x(1) , ctrlY - x(2)  ) );% measurement equation
  h=@(x)(-atan((( ctrlX - x(1) ) / ( ctrlY - x(2) ) ) ));
%   z = h(s) + r*randn;                     % measurments
  z = radar(k);                             % measurments
  sV(:,k)= s;                             % save actual state
  zV(k)  = z;                             % save measurment
  [est, x, P, e,ligoo] = ekf(f,x,P,h,z,Q,R);            % ekf 
%   [x, P] = ekfA(f,x,P,h,z,Q,R,A);          % ekf with matrix A
  eV(:,k) = e;                            % save error
  xV(:,k) = x;                            % save estimate
  pV(:,(k-1)*4 + 1 : (k-1)*4 + 4) = P;                                 %save P - for error ellipse
  estimateV(:,k) = est;
  ligooV(:,k) = ligoo;
%   s = f(s) + q*randn(n,1);                % update process 
  s = f(s);                % update process 
end
for k=1:n                                 % plot results
  figure(1)
  title('Metavoli katastaseon')
  subplot(n,1,k)
  plot(1:N, sV(k,:), 'b', 1:N, xV(k,:), 'r')
end


% plot control circle
axis equal
figure(2)
title('Actual epipedo')
axis equal
plot(sV(1,:), sV(2,:),  'b')
hold on
plot(xV(1,:), xV(2,:),  'r')
hold on
plot (control(:,1),control(:,2))
hold on
plot(estimateV(1,:), estimateV(2,:),  'g')
hold on

% %error ellipse
% for i =1:N
%    
%     error_ellipse(pV(1:2,(i-1)*4 + 1:(i-1)*4 + 2),xV(1:2,i))
%     
% end

figure(3)
title('Error')
plot(abs(eV(1,1:end)))
RMSE = sqrt(mean(eV.^2));
legend(['RMSE = ' num2str(RMSE)]);


% line_eq

figure(4)
title('K*(z - z1)')
% legend(ligooV)
plot(ligooV')
% 
% figure(5)
% title('ligo - error')
% plot(ligooV')

hold off



