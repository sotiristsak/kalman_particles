%ypologismos neou radar dataset

% %best values
% ux = -0.7;
% uy = 0;
% 
% dt = 0.1;
% A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
% 
% s=[7.1;7.6;ux;uy]; 

clear;


%%Load data
load('dataset/control.dat');
radar = load('dataset/radar1.dat');


% p = polyfit(1:100, radar', 7);
% x7 = linspace(1,100);
% radar_poly = polyval(p,x7);
N=100;



% ux = -0.7;
% uy = 0;
% 
% dt = 0.1;
% A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
% 
% s=[7.1;7.6;ux;uy]; 

s = [4.099;3.402;-0.33;0.4];

% proti eutheia
l = tan(radar(1)+degtorad(90));
b = control(1,2) - l*control(1,1);

x = 4:0.1:7;
ux = -1:0.1:2;
uy = -1:0.1:2;
% y = l * x + b;

results = zeros(5,N);
counter = 1;

for x_count = x
    for ux_count = ux
        for uy_count = uy
                
            y = l * x_count + b;
            s = [x_count;y;ux_count;uy_count];
            rmse = best_fit(s);
            
            results(:,counter) = [s;rmse];
            
            counter = counter + 1;
        end
    end
end

[m,i] = min(results(5,:))

x = results(1:4,i);
best_fit_plus_plot(x)


% % l = tan(radar(1)+degtorad(90));
% l = -1
% b = 6.1 - l*4,8;
% 
% 
% x = 6.1 : control(1,1) + 10;
% y = l * x + b;
% L1 = [x;y];
% 
% figure(1)
% plot(x,y)
% hold on
% 
% 
% 
% x = control(1,1) : control(1,1) + 10;
% y = l * x + b;
% plot(x,y)
% L2 = [x;y];
% 
% 
% l = tan(radar_poly(10)+degtorad(90));
% b = control(10,2) - l*control(10,1);
% 
% 
% 
% p = InterX(L1,L2)
% 
