plot_stage = figure(3);
  hold on;
  title('Trajectories');
  xlabel('x position');
  ylabel('y position');
  axis([-5 70 -5 70]);
  set(gca,'fontsize',18);
  set(plot_stage, 'position', [0 0 1300 1300]);
for i = 1:N
  position_x = data_position(i, 1);
  position_y = data_position(i, 2);
  % Predict next state of the quail with the last state and predicted motion.
  h = @(x)(-atan(((position_x - x(1)) / (position_y - x(2)))) + R);
  %z = h(x) + r*randn; % F * s;
  z = data_theta_rad(i);
  % START solve extended kalman filter
  %z = data_theta(i);
  x1 = A * x;
  P = A * P * A' + Q;             %partial update
  [z1, H] = jaccsd(h, x1);         %nonlinear measurement and linearization
  P12 = P * H';                       %cross covariance
  K = P12 * inv(H * P12 + R);           %Kalman filter gain
  x = x1 + K*(z-z1);                   %state estimate
  P = P - K * P12';
  error_ellipse(P(1:2, 1:2), x(1:2));
  e = z - z1;
  %state covariance matrix
  % END solve extended kalman filter
  s = f(s); %+ q*randn(n,1);        % update process
end
hold off;
