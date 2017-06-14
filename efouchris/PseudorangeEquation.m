% Compute Val = || Xs - X || + b and its Jacobian.
function [Val, Jacob2] = PseudorangeEquation(X,yo,xo)

Val=-atan((xo-X(1))/(yo-X(2)));
syms k l m n;
Jacob2 = jacobian(-atan((xo-k)/(yo-l)),[k,l,m,n]);
Jacob2 = [Jacob2(1),  Jacob2(2),0,0];

end
