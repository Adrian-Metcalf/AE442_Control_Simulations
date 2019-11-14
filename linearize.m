% Destiny Fawley
% 11/6/2019


%% Linearize system
clear all

load('DesignProblem04_EOMs.mat');
syms phi phidot w v tauR tauL wroad vroad eheading elateral real
var = [phi; phidot; w; v; tauR; tauL; wroad; vroad; eheading; elateral];

data.EOM = numEOM.f;
f = symEOM.f; % phiddot vdot wdot 
f(4) = phidot; % phidot
f(5) = -v*sin(eheading);% elateraldot
f(6) = w - wroad*(v*cos(eheading)/(vroad+wroad*elateral)); % eheadingdot

EOM(1) = f(4); % phidot
EOM(2) = f(1); % phiddot
EOM(3) = f(3); % wdot
EOM(4) = f(2); % vdot
EOM(5) = f(5); % elatdot
EOM(6) = f(6); % eheaddot

y = [v, w, elateral, eheading];

x = [phi; phidot; w; v; elateral; eheading];
u = [tauR; tauL];
v = [phi; phidot; w; v; elateral; eheading; tauR; tauL; wroad; vroad];

% f_numeric = matlabFunction(f,'vars', {v});


A = jacobian(EOM,x);
B = jacobian(EOM,u);
C = jacobian(y,x);

A = matlabFunction(A);
B = matlabFunction(B);
C = matlabFunction(C);
EOM = matlabFunction(EOM,'vars',{var});

save('lin_model','A','B','C','EOM');

%% Determine observability and controllability
phieq = 0;
phidoteq = 0;
weq = 0;
veq = 5;
vroadeq = veq;
wroadeq = 0;
eheadingeq = 0;
elateraleq = 0;
tauReq = 0;
tauLeq = 0;

eq_guess = [phieq; phidoteq; weq; veq; tauReq; tauLeq; wroadeq; vroadeq; eheadingeq; elateraleq];
eq = fsolve(EOM,eq_guess);

data.A = A(eheadingeq, elateraleq, phieq, phidoteq, tauLeq, tauReq, veq, vroadeq,  weq, wroadeq);
data.B = B(phieq);
data.C = C();

O = obsv(data.A,data.C);
CTRL = ctrb(data.A,data.B);
eigA = eig(data.A)


rank(O)
rank(CTRL)




