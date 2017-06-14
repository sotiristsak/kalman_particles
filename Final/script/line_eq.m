clf;
figure(2)
axis equal
hold on
plot(control(:,1), control(:,2), 'm*','markersize',3)
% l = zeros(10);

p = polyfit(1:100, radar', 7);

x7 = linspace(1,100);
radar_poly = polyval(p,x7);
% plot(x1,y1)


for i = 1 : 97
    l = tan(radar_poly(i)+degtorad(90));
%     l = tan(degtorad(90) - radar(i));
    b = control(i,2) - l*control(i,1);


    x = control(i,1) : control(i,1) + 10;
    y = l * x + b;
    plot(x,y)
    legend_string=cell(1,1);
    legend_string{1} = strcat('Vehicle');
    legend(legend_string);
    xlabel('X (length units)');
    ylabel('Y (length units)'); 
    title('Problem topology')
    hold on
end

for i = 98 : 100
    l = tan(radar(i) + degtorad(90));
    b = control(i,2) - l*control(i,1);


    x = control(i,1)-10 : control(i,1);
    y = l * x + b;
    plot(x,y)
end


% for i = 1 :5: 100
%     l = tan(radar(i)+degtorad(90));
%     b = control(i,2) - l*control(i,1);
% 
% 
%     x = control(i,1) : control(i,1) + 4;
%     y = l * x + b;
%     plot(x,y)
% end
hold off