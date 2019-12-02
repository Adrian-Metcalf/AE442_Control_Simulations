% Destiny Fawley
% 11/6/2019
function ctrl = stateSpace(motor, rocket, ctrl, nav)

% x = [phi;      phidot;    psi;    psidot;   z;   zdot]
% x = [pitch(Y); pitchrate; yaw(X); yawrate;  alt; vel];

% phi = ctrl.obsv.phi;
% theta = ctrl.obsv.theta;

% ctrl.state = [phi; nav.imu9250.omega(2); theta; nav.imu9250.omega(1); nav.MPL.alt; nav.zdot];
% ctrl.equilibrium = [0; 0; 0; 0; 0; 0];


ctrl.alpha = 0*pi/180; % rad, gimbal angle around y axis 
ctrl.beta = 0; % rad, gimbal angle around x axis



end