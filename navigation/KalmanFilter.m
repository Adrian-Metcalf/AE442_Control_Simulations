
% x = best estimate
% P = covariance matrix
% F = Prediciton step
% Q = covariance of noise
% B = control matrix
% u = control vector

xhat = F*xhat + B*u
P = F*P*F.' + Q
l
% K = Kalman Gain
% H = sensor model
% R = covariance of uncertainty
% z = distribution mean

xhat_dot = xhat + Kdot*(z-H*xhat)
P_dot = P-K'*H*P
K_dot = P*H.'*(H*P*H.'+R)^-1
