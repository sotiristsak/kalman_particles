% clear everything
clear all
close all
clc
rng(1541)
%% Load data
load('dataset/control.dat');
radar = load('dataset/radar1.dat');

%% initialize the variables
% set(0,'DefaultFigureWindowStyle','docked') %dock the figures..just a personal preference you don't need this.
ux = -0.7;
uy = 0;

% git test2

% x=[3.1;2.5;ux;uy]';     % initial state
% x=[4.1;3.3;ux;uy]';     % initial state                             
% x=[5.1;4;ux;uy]';       % initial state                            
% x=[6.1;4.8;ux;uy]';       % initial state  
% x=[1.45;-0.3;ux;uy]';   % initial state
% x=[7.1;5.6;ux;uy]';     % initial state
% x=[8.1;6.3;ux;uy]';     % initial state
% x=[11;8;ux;uy]';     % initial state
% x=[55;38;ux;uy]';       % initial state  
% x=[31;25;ux;uy]';       % initial state  
% x=[-5.099;14.199;ux;uy]';
x=[7.1;7.6;ux;uy]';              % best initial from test

dt=0.1; %time interval

q=0.01;    %std of process 
r=0.1;    %std of measurement 

% Q = [q 0 0 0 ; 0 q 0 0 ; 0 0 q 0; 0 0 0 q];   
Q = [q;q;0;0]'; % Noise covariance in the system
R = r^2;  % Noise covariance in the measurement 

T = 100; % duration the chase (i.e. number of iterations).
N = 10000; % The number of particles the system generates. The larger this is, the better your approximation, but the more computation you need.

A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1]; % state transition model
%initilize our initial, prior particle distribution as a gaussian around
%the true initial value

V = 0.2; % define the variance of the initial esimate
x_P = zeros(N,4); % define the vector of particles

% make the randomly generated particles from the initial prior gaussian distribution
for i = 1:N
    a = [sqrt(V) * randn;sqrt(V) * randn;sqrt(V) * randn;sqrt(V) * randn];
    x_P(i,:) = x + a';
end

% %show the distribution the particles around this initial value of x.
% figure(1)
% clf
% subplot(121)
% plot(1,x_P,'.k','markersize',5)
% xlabel('time step')
% ylabel('flight position')
% subplot(122)
% hist(x_P,100)
% xlabel('flight position')
% ylabel('count')
% pause

figure(1)
clf
autodelete = plot(x_P(:,1),x_P(:,2),'.k','markersize',5);
hold on

plot(x(1),x(2),'or','markersize',5) % Initial actual state
hold on

% plot control circle
axis equal
plot (control(:,1),control(:,2))
axis equal
hold on
plot(control(:,1), control(:,2))
hold on 


%plot smooth lines
p = polyfit(1:100, radar', 7);

x7 = linspace(1,100);
radar_poly = polyval(p,x7);



% for i = 1 : 100       
%     
%     l = tan(radar_poly(i)+degtorad(90));
%     if tan(radar_poly(i)+degtorad(90)) < 0
%         radar_poly(i) = -(radar_poly(i) + degtorad(90));
%         l = tan(radar_poly(i));
%     end
%     b = control(i,2) - l*control(i,1);
%     xl = control(i,1) : control(i,1) + 10;
%     yl = l * xl + b;
%     plot(xl,yl)
%     hold on
% end

% First measurement angle and plotting the first line

l = tan(radar(1) + degtorad(90));
b = control(1,2) - l*control(1,1);


xc = control(1,1) : control(1,1) + 20;
yc = l * xc + b;
plot(xc,yc)
hold on

%estimate location
estimate = [];
best_traj = [];

%% update
resample_time = zeros(T+1,1);
counter =0;
for t = 1:T
    %from the previou time step, update the flight position, and observed
    %position (i.e. update the Quails position with the non linear function
    %and update from this position what the chasing ninja's see confounded
    %by the Quails illusions.
    
    x = (A*x')' + Q.*randn(1,4);
    ctrlX = control(t,1);
    ctrlY = control(t,2);
    z = radar(t);

    %Here, we do the particle filter
    for i = 1:N
        %given the prior set of particle (i.e. randomly generated locations
        %the quail might be), run each of these particles through the state
        %update model to make a new set of transitioned particles.
        x_P_update(i,:) = (A*x_P(i,:)')' + Q.*randn(1,4);
        %with these new updated particle locations, update the observations
        %for each of these particles.
        z_update(i) = -atan((( ctrlX - x_P(i,1) ) / ( ctrlY - x_P(i,2) ) ) );
        %Generate the weights for each of these particles.
        %The weights are based upon the probability of the given
        %observation for a particle, GIVEN the actual observation.
        %That is, if we observe a location z, and we know our observation error is
        %guassian with variance x_R, then the probability of seeing a given
        %z centered at that actual measurement is (from the equation of a
        %gaussian)
        P_w(i) = ((1/sqrt(2*pi*R)) * exp(-(z - z_update(i))^2/(2*R)))';
%         prox(i,:) = [P_w(i) i];
    end
    
    % Normalize to form a probability distribution (i.e. sum to 1).
    P_w = P_w./sum(P_w);
    
     
    delete(autodelete);
    autodelete = plot(x_P_update(:,1),x_P_update(:,2),'.b','markersize',5);
    
    
    
    %{
    figure(1)
    clf
    subplot(121)
    plot(P_w,z_update,'.k','markersize',5)
    hold on
    plot(0,z,'.r','markersize',50)
    xlabel('weight magnitude')
    ylabel('observed values (z update)')    
    subplot(122)
    plot(P_w,x_P_update,'.k','markersize',5)
    hold on
    plot(0,x,'.r','markersize',50)
    xlabel('weight magnitude')
    ylabel('updated particle positions (x P update)')
    pause
    
    
    %plot the before and after
    figure(1)
    clf
    subplot(131)
    plot(0,x_P_update,'.k','markersize',5)
    title('raw estimates')
    xlabel('fixed time point')
    ylabel('estimated particles for flight position')
    subplot(132)
    plot(P_w,x_P_update,'.k','markersize',5)
    hold on
    plot(0,x,'.r','markersize',40)
    xlabel('weight magnitude')
    title('weighted estimates')
    %}
    %% Resampling: From this new distribution, now we randomly sample from it to generate our new estimate particles
    
    %what this code specifically does is randomly, uniformally, sample from
    %the cummulative distribution of the probability distribution
    %generated by the weighted vector P_w.  If you sample randomly over
    %this distribution, you will select values based upon there statistical
    %probability, and thus, on average, pick values with the higher weights
    %(i.e. high probability of being correct given the observation z).
    %store this new value to the new estimate which will go back into the
    %next iteration
    P_W_index = [1:N; P_w]'; 
    Prox = sortrows(P_W_index, 2);
  
    if (sum(Prox(1:round((N/3)),2)) < 0.003)
        counter = counter + 1;
        resample_time(counter,1)= t;
        indexes_backup = Prox(1:(N/10), 1);
        best_indexes(1:(N/10),1) = Prox((N-(N/10)+1):N, 1);
%         best_indexes(101:200,1) = Prox(901:1000, 1);
%         best_indexes(201:300,1) = Prox(901:1000, 1);
%         plot(x_P_update(best_indexes(size(best_indexes,1)),1),x_P_update(best_indexes(size(best_indexes,1)),2) ,'--c','markersize',5) % plot movement of best particle
        x_P_update(indexes_backup,:) = x_P_update(best_indexes,:);
    end
    
    x_P = x_P_update;
    
    %The final estimate is some metric of these final resampling, such as
    %the mean value or variance
    x_est = mean(x_P(:,1));
    y_est = mean(x_P(:,2));
    estimate = [estimate ; x_est y_est];
    
    
    %{
    %the after
    subplot(133)
    plot(0,x_P_update,'.k','markersize',5)
    hold on
    plot(0,x_P,'.r','markersize',5)
    plot(0,x_est,'.g','markersize',40)
    xlabel('fixed time point')
    title('weight based resampling')
    pause
    %}
   
%     plot(x_est,y_est,'*g','markersize',5)
    plot(x_P_update(Prox(size(Prox,1)),1),x_P_update(Prox(size(Prox,1)),2) ,'m*-','markersize',5)
    hold on
    best_traj = [best_traj; x_P_update(Prox(size(Prox,1)),1) x_P_update(Prox(size(Prox,1)),2) ];
    pause(0.1)
end

xxx = sgolayfilt(best_traj(:,1), 3, 7);
yyy = sgolayfilt(best_traj(:,2), 3, 7);
plot(best_traj(:,1),best_traj(:,2))
plot(xxx,yyy,'g')
% 
% t = 0:T;
% figure(1);
% clf
% plot(t, x_out, '.-b', t, x_est_out, '-.r','linewidth',3);
% set(gca,'FontSize',12); set(gcf,'Color','White');
% xlabel('time step'); ylabel('Quail flight position');
% legend('True flight position', 'Particle filter estimate');