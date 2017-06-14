function [rmse]=best_fit_plus_plot(start_vector)

    %%Load data
    load('dataset/control.dat');
    radar = load('dataset/radar1.dat');

    N=100;

    s = start_vector;

    dt = 0.1;
    A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];



    for i = 1 : N
        x(:,i) = s;
        s = A*s;
    end

    for i = 1 : N

       slope = -(x(1,i) - control(i,1)) / (x(2,i) - control(i,2)) ;

       a = atan(slope);

       radar_new(i) = a;

    end

    eV = zeros(N);

    for i = 1 : 100

       eV(i) = abs(radar_new(i) - radar(i));

    end
    
    rmse = sum(eV(1:end));
    
    figure(5)
    axis equal
    plot(x(1,:),x(2,:))
    hold on
    plot(control(:,1), control(:,2))
    legend('Obstacle','Vehicle');
    xlabel('X (length units)');
    ylabel('Y (length units)'); 
    title('Problem topology')
    hold on




    figure(6)
    plot(radar_new')
    hold on
    plot(radar)

    legend('Best Fit','Measurements');
    xlabel('Time (time steps)');
    ylabel('Angle (rads)'); 
    title('Initial measurements and Best Fit')
    hold on
    
    

end