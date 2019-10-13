function trajCalcs = getTrajCalcs(tCurr, yi, rocket, motor, ctrl, models)

pos = yi(1:3); % m, position vector, (x,y,z)
vel = yi(4:6); % m/s, velocity vector
angPos = yi(7:9); % rad, euler angles, (theta, phi, psi) = (roll, pitch, yaw)
propMass = yi(13); % kg, mass of propellant left

cg = getCg(rocket, motor, propMass);
massTot = propMass + motor.dryMass + rocket.structure.dryMass;
rho = getAtmDensity(pos(3), models);
qi2b = quatFromEulerAngles(angPos);
MOI = getMOI(rocket, motor, cg, yi);

%% Fin drag
% roll direction offset of each fin
finAngles = linspace(0,2*pi,rocket.fin.numFins+1);
finAngles(end) = [];

% calculate fin normal in body frame
finNormalsB = zeros(3,length(finAngles));
finRadialVecB = zeros(3,length(finAngles));
rCgFinsB = zeros(3,length(finAngles));
for i = 1:length(finAngles)
    finRadialVecB(:,i) = [sin(finAngles(i)); cos(finAngles(i)); 0]; % radial unit vector along each fin
    finNormalsB(:,i) = cross(finRadialVecB(:,i), [0; 0; 1]); % CW unit vector normal to fin looking from above
    rCgFinsB(:,i) = finRadialVecB(:,i)*(rocket.fin.width/2 + rocket.structure.tubeDiameter/2) ...
        + [0; 0; 1]*(rocket.structure.length - cg);
end

finNormalsI = quatVectorRotation(quatConjugate(qi2b),finNormalsB);
finRadialVecI = quatVectorRotation(quatConjugate(qi2b),finRadialVecB); 
rCgFinsI = quatVectorRotation(quatConjugate(qi2b), rCgFinsB);

% Decompose velocity into component parallel to fin and perp to fin
velRadial = repmat(sum(vel.*finRadialVecI),[3,1]).*finRadialVecI;
velPerp = vel - velRadial;
velPerpMag = normVectorColumn(velPerp);

% Calculate drag on fin area perp to velocity
alpha = pi/2 - acos(dot(finNormalsI,unit(velPerp))); % rad, angle of attack of fin
alpha(normVectorColumn(velPerp) == 0) = 0;
if dot(finNormalsI,velPerp) < 0
    alpha = -alpha;
end
clFin = rocket.aero.finClSlope*alpha; % lift coefficient, update with better model later
cdFin = rocket.aero.finCd; % assume a cd for now, update this later

dragVec = -unit(velPerp); % nd, direction of drag vec
liftVec = cross(dragVec, finRadialVecI);
liftVec = unit(liftVec); % nd, direction of lift vector

FDragFinI = .5*rho*velPerpMag.^2*rocket.fin.area.*cdFin.*dragVec;
FLiftFinI = .5*rho*velPerpMag.^2*rocket.fin.area.*clFin.*liftVec;

%% Body Tube and Nosecone Drag
% Assume constant drag and no lift
cdBody = rocket.aero.bodyCd;

% Model drag on circular cylinder with axial flow
% https://aip.scitation.org/doi/pdf/10.1063/1.864706  
epsilon = 1/log(2*rocket.structure.length/rocket.structure.tubeDiameter); % length/diameter of rocket
FDragBodyI = (epsilon + .80685*epsilon^2 + 0.82854*epsilon^3)*rocket.structure.length; % N

%% Thrust Force
if ctrl.igniteMotor
    if tCurr - ctrl.tIgnite < motor.thrust(1,1)
        motor.thrust = [0 0; motor.thrust];
    end
    
    if tCurr - ctrl.tIgnite > motor.thrust(end,1)
        thrust = 0;
    else
        thrust = interp1(motor.thrust(:,1), motor.thrust(:,2), tCurr - ctrl.tIgnite);
    end
    FThrustB = [0; 0; 1]*thrust;
    
    % Find dm/dt from thrust
    switch models.deltaMassMode
        case 1 % constant mass flow
            dmdt = (motor.wetMass - motor.dryMass)/motor.thrust(end,1);
            
        case 2 % fit thrust curve profile
            
    end
    
else
    FThrustB = [0; 0; 0]; % N, thrust force
    dmdt = 0; % kg/s, mass flow rate
end

FThrustI = quatVectorRotation(quatConjugate(qi2b),FThrustB);


%% Gravity
FGravI = [0; 0; -9.81*massTot];

%% Moments
% Get moments on fins and body tube
% Assume force acts in center of fins
FFinsI = FDragFinI + FLiftFinI;
MFinsI = cross(rCgFinsI, FFinsI);


accel = (sum(FFinsI')' + FThrustI + FGravI)/massTot;
%% Store Data
trajCalcs.cg = cg;
trajCalcs.massTot = massTot;
trajCalcs.rho = rho;
trajCalcs.qi2b = qi2b;
trajCalcs.finNormalsB = finNormalsB;
trajCalcs.finRadialVecB = finRadialVecB;
trajCalcs.rCgFinsB = rCgFinsB;
trajCalcs.finNormalsI = finNormalsI;
trajCalcs.finRadialVecI = finRadialVecI;
trajCalcs.velRadial = velRadial;
trajCalcs.velPerp = velPerp;
trajCalcs.alpha = alpha;
trajCalcs.clFin = clFin;
trajCalcs.cdFin = cdFin;
trajCalcs.dragVec = dragVec;
trajCalcs.liftVec = liftVec;
trajCalcs.FDragFinI = FDragFinI;
trajCalcs.FLiftFinI = FLiftFinI;
trajCalcs.FDragBodyI = FDragBodyI;
trajCalcs.FThrustI = FThrustI;
trajCalcs.FGravI = FGravI;
trajCalcs.MFinsI = MFinsI;
trajCalcs.dmdt = dmdt;
trajCalcs.MOI = MOI;
trajCalcs.accel = accel;


end