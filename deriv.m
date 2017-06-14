s=[7.1;7.6;ux;uy];              % best initial from test

% x=s+q*randn(n,1); %initial state          % initial state with noise
x = s; 

H(1) = 1 /((ctrlY - x(2)) * ( (ctrlX-x(1))^2/(ctrlY-x(2))^2+1));
H(2) = -(((ctrlX - x(1))^2/(ctrlY - x(2))^2)-1)/(((ctrlX - x(1))^2/((ctrlY-x(2))^2)+1)^2 * ((ctrlY-x(2))^2));
H(3) = 0;
H(4) = 0;

H