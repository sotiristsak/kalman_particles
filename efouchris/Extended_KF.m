function [Xo,Po,gXp,P] = Extended_KF(T,X,P,Q, yo,xo,N,ang,g)
N_state = size(X, 1);    
%% predict
Xp = zeros(size(X));
Xp(1) = X(1) +T * X(3);
Xp(2) = X(2) + T* X(4);
Xp(3) = X(3);
Xp(4) = X(4);

Ft = [1 0 T 0; 0 1 0 T; 0 0 1 0; 0 0 0 1 ];

[gXp, H] = g(Xp,yo,xo);%3 ektimisi gwnias kai H
% H=  [ -1/(((k - 2199/2000)^2/(l - 101/100)^2 + 1)*(l - 101/100)), (k - 2199/2000)/(((k - 2199/2000)^2/(l - 101/100)^2 + 1)*(l - 101/100)^2), 0, 0]
% H.'=[-1/(((k - 2199/2000)^2/(l - 101/100)^2 + 1)*(l - 101/100)); (k - 2199/2000)/(((k - 2199/2000)^2/(l - 101/100)^2 + 1)*(l - 101/100)^2);0;0];
% H'=[-1/((conj(l) - 101/100)*((conj(k) - 2199/2000)^2/(conj(l) - 101/100)^2 + 1)); (conj(k) - 2199/2000)/((conj(l) - 101/100)^2*((conj(k) - 2199/2000)^2/(conj(l) - 101/100)^2 + 1));0;0];
 
Pt = Ft * P * Ft.' + Q;%4 Pt-->pinakas autosusxetisewn
Pt=double(Pt);
%%

H1 = subs(H,Xp(1));
H2 = subs(H1,Xp(2));
H2=double(H2);
%% update
Yt=ang-gXp; %5 diafora metroumenis k aktimomenis gwnias

% S=H * Pp * H.' + n;
S=H * Pt * H.' + N;
S1 = subs(S,Xp(1));
S2 = subs(S1,Xp(2));
S2=double(S2);

% K = Pp * H' / (S)
K = Pt * H.' / S2;%6 Kalman Gain
K1 = subs(K,Xp(1));
K2 = subs(K1,Xp(2));
K2=double(K2);
Xo = Xp + K2 * Yt;%7 Xt

I = eye(N_state, N_state);
Po = (I - K2 * H2) * Pt; %8