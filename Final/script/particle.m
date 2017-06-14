% clear everything
clear all
close all
clc
rng(180)
%% Load data
load('dataset/control.dat');
radar = load('dataset/radar1.dat');

%% initialize the variables


%%%%%%%%%%%%%%%%Epilogi arxikis katastasis%%%%%%%%%%%%%%%%%%
x=[5.1;4.1996;-0.5;0.3]';        % best initial from test  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% x=[8.1;7.1996;-0.5;0.3]';
% x=[5.1;4.1996;0.5;-0.3]';

dt=0.1; %time interval
q=0.1;
Q = q*eye(4);
r=0.1;    %std of measurement
R=r^2;        % covariance of measurement

T = 100; % total duration 

N = 10000; % The number of particles the system generates

%resampling params
elite_percentage = 0.0005;   % % ton kaliteron particles
resampling_rate = 5;   %kathe pote tha elegxei gia resampling
radius_percentage = 0.15;


A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1]; % state transition model

V = 0.02; % =variance of the initial esimate
x_P = zeros(N,4,100); % vector of particles

% mrandomly generated particles from the initial prior gaussian distribution
for i = 1:N
    a = [sqrt(V) * randn;sqrt(V) * randn;sqrt(V) * randn;sqrt(V) * randn];
    x_P(i,:,1) = x + a';
end

figure(1)
clf
axis equal
autodelete = plot(x_P(:,1,1),x_P(:,2,1),'.k','markersize',5);
axis equal
hold on

plot_init = plot(x(1),x(2),'ok','markersize',5,'DisplayName','Initial Actual state'); % Initial actual state
axis equal
hold on

% plot control circle

plot_veh = plot(control(:,1),control(:,2),'DisplayName','Vehicle');
hold on

% First measurement angle and plotting the first line

l = tan(radar(1) + degtorad(90));
b = control(1,2) - l*control(1,1);


xc = control(1,1) : control(1,1) + 10;
yc = l * xc + b;
plot_first = plot(xc,yc,'DisplayName','First measurement');
hold on

% Last measurement angle and plotting the last line
l = tan(radar(100) + degtorad(90));
b = control(100,2) - l*control(100,1);


xc = control(100,1)-2 : control(100,1);
yc = l * xc + b;
plot_last = plot(xc,yc,'DisplayName','Last measurement');
hold on

%radar polyfit
p = polyfit(1:100, radar', 7);
x7 = linspace(1,100);
radar_poly = polyval(p,x7);

%% update
resampling_counter =0;
for t = 1:T
     
    ctrlX = control(t,1);
    ctrlY = control(t,2);
%     z = radar_poly(t);
    z = radar(t);

    %particle filter
    for i = 1:N
        x_P_update(i,:) = (A*x_P(i,:,t)')' + q*randn(1,4);

        z_update(i) = -atan((( ctrlX - x_P(i,1) ) / ( ctrlY - x_P(i,2) ) ) );

        if t > 1
%             P_w(i,t) = (((1/sqrt(2*pi*R)) * exp(-(z - z_update(i))^2/(2*R)))')+P_w(i,t-1);
            P_w(i,t) = (((1/sqrt(2*pi*R)) * exp(-(z - z_update(i))^2/(2*R)))')*P_w(i,t-1);
%             P_w(i,t) = (((1/sqrt(2*pi*R)) * exp(-(z - z_update(i))^2/(2*R)))');
        else
            P_w(i,t) = ((1/sqrt(2*pi*R)) * exp(-(z - z_update(i))^2/(2*R)))';
        end

    end %telos olon ton particles
    
    
    % Normalize to form a probability distribution
    P_w(:,t) = P_w(:,t)./sum(P_w(:,t));
    
    %avoiding weight elimintaion
    P_w(:,t) = P_w(:,t) + 7.250390539616982e-239;
    
     
    delete(autodelete);
    autodelete = plot(x_P_update(:,1),x_P_update(:,2),'.b','markersize',5,'DisplayName','Particles');


    x_P(:,:,t) = x_P_update;
    
    %Resampling
    
    if mod(t,resampling_rate) == 0 %kathe resampling rate
       
        resampling_counter = resampling_counter + 1;
        resampling_period = [(resampling_counter-1)*resampling_rate+1:resampling_counter*resampling_rate];
        
%         %sum of weights for that period of resampling
%         for i = 1:N
%             period_Sum(i,1) = sum(P_w(i,resampling_period));
%         end
%         
%         [sortedSum,sortingIndices] = sort(period_Sum,'descend');
%         % sortedX will be in descending order. Therefore, the first N elements will be the N maximum values.
%         maxValues = sortedSum(1:N*elite_percentage);
%         maxValueIndices = sortingIndices(1:N*elite_percentage);

        
        %weight only in t iteration
        [sortedSum,sortingIndices] = sort(P_w(:,t),'descend');
        % sortedX will be in descending order. Therefore, the first N elements will be the N maximum values.
        maxValues = sortedSum;
        maxValueIndices = sortingIndices(1:N*elite_percentage);
        
        %calculating center nad radius
        max_dis = max_distance(x_P(maxValueIndices,1:2,t));
        centerX = (max_dis(1) + max_dis(3))/2;
        centerY = (max_dis(2) + max_dis(4))/2;
        radius = max_dis(5)/2;
        r = hypot(x_P(:,1,t)-centerX,x_P(:,2,t)-centerY);
        
        c = linspace(0,2*pi);

        plot_circles = plot(centerX+radius*cos(c),centerY+radius*sin(c),'r-','DisplayName','Resampling circles');
        
        title(strcat('Problem topology, resampling counter: ', num2str(resampling_counter)));

        %change particels out of circle
        
%         %only position
%         x_P(r>radius,1,t) = normrnd(centerX,radius*radius_percentage/3,size(x_P(r>radius,1,t),1),1);
%         x_P(r>radius,2,t) = normrnd(centerY,radius*radius_percentage/3,size(x_P(r>radius,1,t),1),1);
        
        
        %position and vel
        x_P(r>radius,1,t) = normrnd(centerX,radius*radius_percentage/3,size(x_P(r>radius,1,t),1),1);
        x_P(r>radius,2,t) = normrnd(centerY,radius*radius_percentage/3,size(x_P(r>radius,1,t),1),1);
        x_P(r>radius,3,t) = normrnd(x(3),1/9,size(x_P(r>radius,1,t),1),1);
        x_P(r>radius,4,t) = normrnd(x(4),1/9,size(x_P(r>radius,1,t),1),1);
        
    end %end of resampling
    
    %next gen
    x_P(:,:,t+1) = x_P(:,:,t);
        
    pause(0.1)
end %telos genias

%for unsuprevised particles - BEST PARTICLE from start to end
for i = 1:N

    P_w_Sum(i) = sum(P_w(i,:))';
       
end


[bestMax,bestIndex] = max(P_w_Sum(:));

bestParticle(:,:) = [x_P(bestIndex,1,:);x_P(bestIndex,2,:)];
plot_best = plot(bestParticle(1,:),bestParticle(2,:),'m.--','markersize',5,'DisplayName','Best Particle');
% title('Problem topology')
xlabel('X (length units)');
ylabel('Y (length units)');
legend([plot_init plot_veh plot_first plot_last plot_circles plot_best ],'Initial Position','Vehicle','First Measurement','Last Measurement','Resamplin circles','Best Particle')
