function [bestMax,bestIndex] = particle_fun_withoutPlot(seed,x)


    global  control
    global  radar

    rng(seed)

    dt=0.1; %time interval
    q=1;
    Q = q/1000^2*eye(4);
    r=0.1;    %std of measurement
    R=r^2;        % covariance of measurement
    T = 100; % duration the chase (i.e. number of iterations).
    N = 10000; % The number of particles the system generates. The larger this is, the better your approximation, but the more computation you need.
    A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1]; % state transition model
    V = 0.02; % define the variance of the initial esimate
    x_P = zeros(N,4,100); % define the vector of particles
 
    % make the randomly generated particles from the initial prior gaussian distribution
    for i = 1:N
        a = [sqrt(V) * randn;sqrt(V) * randn;sqrt(V) * randn;sqrt(V) * randn];
        x_P(i,:,1) = x + a';
    end

    %estimate location
    estimate = [];
    best_traj = [];

    %update
    resample_time = zeros(T+1,1);
    counter =0;
    
    for t = 1:T

        x = (A*x')' + Q.*randn(1,4);
        ctrlX = control(t,1);
        ctrlY = control(t,2);
        z = radar(t);

        %Here, we do the particle filter
        for i = 1:N
    %         x_P_update(i,:) = (A*x_P(i,:)')' + Q.*randn(1,4);
            x_P_update(i,:) = (A*x_P(i,:,t)')';    %XORIS THORYBO STO UPDATE
            z_update(i) = -atan((( ctrlX - x_P(i,1) ) / ( ctrlY - x_P(i,2) ) ) );
            P_w(i,t) = ((1/sqrt(2*pi*R)) * exp(-(z - z_update(i))^2/(2*R)))';

        end
        P_w(:,t) = P_w(:,t)./sum(P_w(:,t));
        x_P(:,:,t+1) = x_P_update;
    end


    for i = 1:N

        P_w_Sum(i) = sum(P_w(i,:))';

    end


    [bestMax,bestIndex] = max(P_w_Sum(:));

%     bestParticle(:,:) = [x_P(bestIndex,1,:);x_P(bestIndex,2,:)]
%     plot(bestParticle(1,:),bestParticle(2,:),'m.--','markersize',5)


end







