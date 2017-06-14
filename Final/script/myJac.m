function [z1,H] = myJac(hfun,x)
global ctrlX
global ctrlY

z1 = hfun(x);


H(1) = 1 /((ctrlY - x(2)) * ( (ctrlX-x(1))^2/(ctrlY-x(2))^2+1));
H(2) = -(((ctrlX - x(1))^2/(ctrlY - x(2))^2)-1)/(((ctrlX - x(1))^2/((ctrlY-x(2))^2)+1)^2 * ((ctrlY-x(2))^2));
H(3) = 0;
H(4) = 0;


end