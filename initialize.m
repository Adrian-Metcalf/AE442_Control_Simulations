% Destiny Fawley
% 11/6/2019

function result = initialize(rocket, ctrl, nav, trajCalcs, models)

% Trajectory
result.traj.tCurr = NaN(1,length(models.maxIter));
result.traj.posI = NaN(3,length(models.maxIter));
result.traj.velI = NaN(3,length(models.maxIter));
result.traj.EulerAngles = NaN(3,length(models.maxIter));
result.traj.omega = NaN(3,length(models.maxIter));
result.traj.propMass = NaN(1,length(models.maxIter));
result.traj.thrustI = NaN(3,length(models.maxIter));
result.traj.gravityI = NaN(3,length(models.maxIter));
result.traj.dragBodyI = NaN(3,length(models.maxIter));

result.traj.dragFin1I = NaN(3,length(models.maxIter));
result.traj.dragFin2I = NaN(3,length(models.maxIter));
result.traj.dragFin3I = NaN(3,length(models.maxIter));
result.traj.dragFin4I = NaN(3,length(models.maxIter));

result.traj.liftFin1I = NaN(3,length(models.maxIter));
result.traj.liftFin2I = NaN(3,length(models.maxIter));
result.traj.liftFin3I = NaN(3,length(models.maxIter));
result.traj.liftFin4I = NaN(3,length(models.maxIter));

result.traj.momentFin1I = NaN(3,length(models.maxIter));
result.traj.momentFin2I = NaN(3,length(models.maxIter));
result.traj.momentFin3I = NaN(3,length(models.maxIter));
result.traj.momentFin4I = NaN(3,length(models.maxIter));

result.traj.rho = NaN(1,length(models.maxIter));
result.traj.qi2b = NaN(4,length(models.maxIter));
result.traj.clFin = NaN(4,length(models.maxIter));
result.traj.cdFin = NaN(4,length(models.maxIter));
result.traj.aoaFin = NaN(4,length(models.maxIter));

% Navigation
result.nav.posI = NaN(3,length(models.maxIter));
result.nav.velI = NaN(3,length(models.maxIter));
result.nav.EularAngles = NaN(3,length(models.maxIter));
result.nav.omega = NaN(3,length(models.maxIter));

% Control
result.ctrl.igniteCotor = NaN(1,length(models.maxIter));
result.ctrl.tIgnite = NaN(1,length(models.maxIter));

result.term.endCondition = NaN(1,length(models.maxIter));




end