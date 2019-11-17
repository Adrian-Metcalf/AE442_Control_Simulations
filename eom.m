% Destiny Fawley
% 11/6/2019

function ydot = eom(tCurr, yi, rocket, motor, ctrl, models)

trajCalcs = getTrajCalcs(tCurr, yi, rocket, motor, ctrl, models);

%% Translational Acceleration
aTrans = (trajCalcs.FGravI + trajCalcs.FThrustI + trajCalcs.FLiftBodyI + trajCalcs.FDragBodyI + sum(trajCalcs.FDragFinI,2))/trajCalcs.massTot...
    + sum(trajCalcs.FLiftFinI,2);

%% Angular Acceleration
alpha = trajCalcs.angAccel;

%% Angular Rates 
% Need to convert angular velocity to 321 angular rates
phi = yi(9); % yaw(X)
theta = yi(8); % pitch(Y)
psi = yi(7); % roll(Z)

R = [0 sin(phi)/cos(theta) cos(phi)/cos(theta);
    0 cos(phi) -sin(phi);
    1 tan(theta)*sin(phi) cos(phi)*tan(theta)];
rateDots = R*yi(10:12);


ydot(1:3,1) = yi(4:6); % [x, y, z]
ydot(4:6,1) = aTrans; % [xdot, ydot, zdot]
ydot(7:9,1) = rateDots; % [roll(Z), pitch(Y), yaw(X)]
ydot(10:12,1) = alpha;
ydot(13) = trajCalcs.dmdt;


end


