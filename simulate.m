% Destiny Fawley
% 11/6/2019

function result = simulate(motor, rocket, controller, models)

y = rocket.y0;
tCurr = 0;
dt = 1/models.integrationRate; 

ctrl = controlInit(controller, models);
trajCalcs = getTrajCalcs(tCurr, y, rocket, motor, ctrl, models);
nav = navInit(models, y);
result = initialize(rocket, ctrl, nav, trajCalcs, models);

terminate = 0;
counter = 2;
datacounter = 2;

result = storeData(trajCalcs, tCurr, y, nav, ctrl, terminate, result, 1 ); % log initial state

while ~terminate
    
    % Integrate with RK4(tCurr, yi, rocket, motor, ctrl, models)
    h1 = eom(tCurr, y, rocket, motor, ctrl, models);
    h2 = eom(tCurr+.5*dt, y+.5*dt*h1, rocket, motor, ctrl, models);
    h3 = eom(tCurr+.5*dt, y+.5*dt*h2, rocket, motor, ctrl, models);
    h4 = eom(tCurr+dt, y+dt*h3, rocket, motor, ctrl, models);
    
    y = y + dt/6*(h1 + 2*h2 + 2*h3 + h4);
    tCurr = tCurr + dt;
    
    trajCalcs = getTrajCalcs(tCurr, y, rocket, motor, ctrl, models);
    
    nav = navigation(y,tCurr, trajCalcs, models, nav, counter);
    ctrl = control(motor, rocket, ctrl, nav, tCurr, models, counter);
    
    % check termination conditions
    if y(3) <= models.minAlt
        terminate = 1;
    elseif counter > models.maxIter
        terminate = 2;
        
    end
    
    if mod(counter-1,round(models.integrationRate/models.dataRate)) == 0 % rate limit calls to data logging
        result = storeData(trajCalcs, tCurr, y, nav, ctrl, terminate, result, datacounter );
        datacounter = datacounter+1;
    end
    
    counter = counter + 1;
end

    

end

function result = storeData(trajCalcs, tCurr, y, nav, ctrl, terminate, result, counter )

% Trajectory
result.traj.time(:,counter) = tCurr;
result.traj.posI(:,counter) = y(1:3);
result.traj.velI(:,counter) = y(4:6);
result.traj.EulerAngles(:,counter) = y(7:9);
result.traj.omega(:,counter) = y(10:12);
result.traj.propMass(:,counter) = y(13);
result.traj.thrustI(:,counter) = trajCalcs.FThrustI;
result.traj.gravityI(:,counter) = trajCalcs.FGravI;
result.traj.dragBodyI(:,counter) = trajCalcs.FDragBodyI;

result.traj.dragFin1I(:,counter) = trajCalcs.FDragFinI(:,1);
result.traj.dragFin2I(:,counter) = trajCalcs.FDragFinI(:,2);
result.traj.dragFin3I(:,counter) = trajCalcs.FDragFinI(:,3);
result.traj.dragFin4I(:,counter) = trajCalcs.FDragFinI(:,4);

result.traj.liftFin1I(:,counter) = trajCalcs.FLiftFinI(:,1);
result.traj.liftFin2I(:,counter) = trajCalcs.FLiftFinI(:,2);
result.traj.liftFin3I(:,counter) = trajCalcs.FLiftFinI(:,3);
result.traj.liftFin4I(:,counter) = trajCalcs.FLiftFinI(:,4);

result.traj.momentFin1I(:,counter) = trajCalcs.MFinsI(:,1);
result.traj.momentFin2I(:,counter) = trajCalcs.MFinsI(:,2);
result.traj.momentFin3I(:,counter) = trajCalcs.MFinsI(:,3);
result.traj.momentFin4I(:,counter) = trajCalcs.MFinsI(:,4);

result.traj.rho(:,counter) = trajCalcs.rho;
result.traj.qi2b(:,counter) = trajCalcs.qi2b;
result.traj.MOI(:,counter) = diag(trajCalcs.MOI);
result.traj.clFin(:,counter) = trajCalcs.clFin';
result.traj.cdFin(:,counter) = trajCalcs.cdFin';
result.traj.aoaFin(:,counter) = trajCalcs.alpha';
result.traj.accel(:,counter) = trajCalcs.accel';



% Navigation
result.nav.posI(:,counter) = nav.posI;
result.nav.velI(:,counter) = nav.velI;
result.nav.EularAngles(:,counter) = nav.EulerAngles;
result.nav.omega(:,counter) = nav.omega;

% Control
result.ctrl.igniteMotor(counter) = ctrl.igniteMotor;
result.ctrl.tIgnite(counter) = ctrl.tIgnite;

result.term.endCondition(counter) = terminate;


end