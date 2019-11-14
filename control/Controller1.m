% Destiny Fawley
% 11/6/2019
function func = Controller
% INTERFACE
%
%   sensors
%       .e_lateral      (error in lateral position relative to road)
%       .e_heading      (error in heading relative to road)
%       .v              (forward speed)
%       .w              (turning rate)
%       .r_road         (signed radius of curvature of road - to find the
%                        corresponding turning rate for a given forward
%                        speed: w_road = v_road/sensors.r_road)
%
%   references
%       (none)
%
%   parameters
%       .tStep      (time step)
%       .tauMax     (maximum wheel torque)
%       .roadwidth  (width of road - half on each side of centerline)
%       .symEOM     (nonlinear EOMs in symbolic form)
%       .numEOM     (nonlinear EOMs in numeric form)
%
%   data
%       .whatever   (yours to define - put whatever you want into "data")
%
%   actuators
%       .tauR       (right wheel torque)
%       .tauL       (left wheel torque)

% Do not modify this function.
func.init = @initControlSystem;
func.run = @runControlSystem;

end

%
% STEP #1: Modify, but do NOT rename, this function. It is called once,
% before the simulation loop starts.
%

function [actuators,data] = initControlSystem(sensors,references,parameters,data)

load('lin_model.mat');

% x = [phi; phidot; w; v; elateral; eheading];
% u = [tauR; tauL];

phieq = 0;
phidoteq = 0;
weq = 0;
veq = 5.5;
vroadeq = veq;
wroadeq = 0;
eheadingeq = 0;
elateraleq = 0;
tauReq = 0;
tauLeq = 0;

data.A = A(eheadingeq, elateraleq, phieq, phidoteq, tauLeq, tauReq, veq, vroadeq,  weq, wroadeq);
data.B = B(phieq);
data.C = C();

% Qo = eye(size(data.A))*10;
Qo = [1 0 0 0 0 0;
      0 1 0 0 0 0;
      0 0 1 0 0 0;
      0 0 0 1 0 0;
      0 0 0 0 1 0;
      0 0 0 0 0 1];
Ro = [1000 0 0 0;
      0 1000 0 0;
      0 0 1000 0;
      0 0 0 1000];

Qc = [8000 0 0 0 0 0;
      0 10 0 0 0 0;
      0 0 10 0 0 0;
      0 0 0 1 0 0;
      0 0 0 0 500 0;
      0 0 0 0 0 500];
Qc = Qc*1;
Rc = eye(size(data.B,2));

data.L = lqr(data.A',data.C',inv(Qo), inv(Ro))';
data.K = lqr(data.A,data.B,Qc,Rc);

data.xeq = [phieq; phidoteq; weq; veq; elateraleq; eheadingeq;];
data.ueq = [tauReq; tauLeq];

% xhat0 = [phi; phidot; w; v; elateral; eheading];
data.xhat = [0; 0; 0; -veq; 0; 0];


[actuators,data] = runControlSystem(sensors,references,parameters,data);
end

%
% STEP #2: Modify, but do NOT rename, this function. It is called every
% time through the simulation loop.
%

function [actuators,data] = runControlSystem(sensors,references,parameters,data)

% x = [phi; phidot; w; v; elateral; eheading];
% u = [tauR; tauL];
A = data.A;
B = data.B;
C = data.C;
K = data.K;
L = data.L;

yeq = [data.xeq(4);
    data.xeq(3);
    data.xeq(5);
    data.xeq(6)];

y = [ sensors.v;
    sensors.w;
    sensors.e_lateral;
    sensors.e_heading] - yeq;

u = -K*(data.xhat);
actuators.tauR = u(1) + data.ueq(1);
actuators.tauL = u(2) + data.ueq(2);

xhatdot = (A-B*K-L*C)*(data.xhat) + L*y;
data.xhat = data.xhat + xhatdot*parameters.tStep;

end