clear;
close all %close all figures
%%Load data
load('dataset/control.dat');
radar = load('dataset/radar1.dat');
rng(180)

global ctrlX
global ctrlY

%%%%%%%%%%%%%%%%Epilogi arxikis katastasis%%%%%%%%%%%%%%%%%%
s=[5.1;4.1996;-0.5;0.3];        % best initial from test  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

n=4;      %number of states

dt = 0.1; %dt - 0.1 sec, f= 10Hz
q=1;
P = [2.25 0 0 0 ; 0 2.25 0 0 ; 0 0 q 0; 0 0 0 q];
Q = q/1000^2*eye(n);
r=0.1;    %std of measurement
R=r^2;        % covariance of measurement
f=@(x)[x(1) + x(3)*dt;x(2) + x(4)*dt;x(3);x(4)]; % state equations

% %dokimes gia figures
% q=1;
% P = [0.1 0 0 0 ; 0 0.1 0 0 ; 0 0 q 0; 0 0 0 q];
% Q = q/1000^2*eye(n);

N=100;                                    % total dynamic steps
xV = zeros(n,N);          %estmate       
sV = zeros(n,N);          %actual
pV = zeros(n,N*4);          %save P - for error ellipse
zV = zeros(1,N);
estimateV = zeros(n,N);
corrV = zeros(n,N);
eV = zeros(1,N);            %error

x = s; 

for k=1:N
  ctrlX = control(k,1);
  ctrlY = control(k,2);
  h=@(x)(-atan((( ctrlX - x(1) ) / ( ctrlY - x(2) ) ) ));
  z = radar(k);                             % measurments
  sV(:,k)= s;                             % save actual state
  zV(k)  = z;                             % save measurment
  [est, x, P, e,corr] = ekf(f,x,P,h,z,Q,R);            % ekf 
%   [x, P] = ekfA(f,x,P,h,z,Q,R,A);          % ekf with matrix A
  eV(:,k) = e;                            % save error
  xV(:,k) = x;                            % save estimate
  pV(:,(k-1)*4 + 1 : (k-1)*4 + 4) = P;                                 %save P - for error ellipse
%   estimateV(:,k) = est;
  corrV(:,k) = corr;
%   s = f(s) + q*randn(n,1);                % update process 
  s = f(s);                % update process 
end




%%%% PLOTS

% plot katastaseon
figure(1)
subplot(4,1,1)
plot(1:N, sV(1,:), 'b', 1:N, xV(1,:), 'r')
title('Variation of x')
subplot(4,1,2)
plot(1:N, sV(2,:), 'b', 1:N, xV(2,:), 'r')
title('Variation of y')
subplot(4,1,3)
plot(1:N, sV(3,:), 'b', 1:N, xV(3,:), 'r')
title('Variation of Ux')
subplot(4,1,4)
plot(1:N, sV(4,:), 'b', 1:N, xV(4,:), 'r')
title('Variation of Uy')


% plot control circle
figure(2)

axis equal
plot(sV(1,:), sV(2,:))
hold on
plot(xV(1,:), xV(2,:))
hold on
plot (control(:,1),control(:,2))


% plot(estimateV(1,:), estimateV(2,:),  'g')
title('Problem topology')
xlabel('X (length units)');
ylabel('Y (length units)');
legend('Actual','Predicted','Vehicle');
hold on

%plot error ellipse
for i =1:N
    figure(2)
    error_ellipse(pV(1:2,(i-1)*4 + 1:(i-1)*4 + 2),xV(1:2,i))
    
    
end
hold off

%plot angle error
figure(3)
plot(abs(eV(1,1:end)))
RMSE = sqrt(mean(eV.^2));
legend(['RMSE = ' num2str(RMSE)]);
title('Angle Error')


line_eq

%plot correction
figure(4)


plot(corrV')

RMSEx = sqrt(mean(corrV(1,1:end).^2));
RMSEy = sqrt(mean(corrV(2,1:end).^2));
RMSEux = sqrt(mean(corrV(3,1:end).^2));
RMSEuy = sqrt(mean(corrV(4,1:end).^2));

legend_string=cell(1,1);

legend_string{1} = strcat('RMS corr X = ', num2str(RMSEx));
legend_string{2} = strcat('RMS corr Y = ', num2str(RMSEy));
legend_string{3} = strcat('RMS corr Ux = ', num2str(RMSEux));
legend_string{4} = strcat('RMS corr Uy = ', num2str(RMSEuy));


legend(legend_string);
xlabel('Time (time steps)');
title('Correction [K*(z - z1)]')


% 
% figure(5)
% title('ligo - error')
% plot(corrV')

hold off



