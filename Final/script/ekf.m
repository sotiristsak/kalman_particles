function [x1,x,P,e,corr]=ekf(fstate,x,P,hmeas,z,Q,R)

[x1,A]=jaccsd(fstate,x);    %nonlinear update and linearization at current state
P=A*P*A'+Q;                 %partial update
[z1,H]=jaccsd(hmeas,x1);    %nonlinear measurement and linearization
% [z1,H]=myJac(hmeas,x1);    %nonlinear measurement and linearization
P12=P*H';                   %cross covariance
K=P12*inv(H*P12+R);       %Kalman filter gain
x=x1+K*(z - z1);            %state estimate
corr = K*(z - z1);      %correction
e = z - z1;             %measurement error
P=P-K*P12';               %state covariance matrix
hold on


%ipologismos iakovianis
function [z,A]=jaccsd(fun,x)
% JACCSD Jacobian through complex step differentiation
% [z J] = jaccsd(f,x)
% z = f(x)
% J = f'(x)
%
z=fun(x);
n=numel(x);
m=numel(z);
A=zeros(m,n);
h=n*eps;
for k=1:n
    x1=x;
    x1(k)=x1(k)+h*i;
    A(:,k)=imag(fun(x1))/h;
end