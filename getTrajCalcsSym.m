% Destiny Fawley
% 11/6/2019

function trajCalcs = getTrajCalcs(tCurr, yi, rocket, motor, ctrl, models)

syms PHI PSI THETA ZDOT Z real

pos = [0 0 Z];%yi(1:3); % m, position vector, (x,y,z)
vel = [0;0;ZDOT];%yi(4:6); % m/s, velocity vector
angPos = [0; PHI; PSI]; % rad, 321 euler angles, (theta, phi, psi) = (roll(Z), pitch(Y), yaw(X))
angVel = yi(10:12); % rad/s, inertial angular velocity (NOT angular rates)
propMass = yi(13); % kg, mass of propellant left

cg = getCg(rocket, motor, propMass); % cg from bottom of rocket
massTot = propMass + motor.dryMass + rocket.structure.dryMass;
rho = 1.16;%getAtmDensity(pos(3), models);
qi2b = quatFromEulerAngles(angPos);
MOI = getMOI(rocket, motor, cg, yi);

%% Fin drag
% roll direction offset of each fin from +y axis
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

finNormalsI = (quatVectorRotation(quatConjugate(qi2b),finNormalsB));
finRadialVecI = (quatVectorRotation(quatConjugate(qi2b),finRadialVecB)); 
rCgFinsI = quatVectorRotation(quatConjugate(qi2b), rCgFinsB);

% Decompose velocity into component parallel to fin and perp to fin
velRadial = repmat(sum(vel.*finRadialVecI),[3,1]).*finRadialVecI;
velPerp = vel - velRadial;
velPerpMag = normVectorColumn(velPerp);

% Calculate drag on fin area perp to velocity
alpha = -(pi/2 - acos(dot(finNormalsI,unit(velPerp)))); % rad, angle of attack of fin
% alpha(normVectorColumn(velPerp) == 0) = 0;
% for i = 1:length(alpha)
%     if dot(finNormalsI(:,i),velPerp(:,i)) < 0
%         alpha(i) = -alpha(i);
%     end
% end
clFin = rocket.aero.finClSlope*alpha; % lift coefficient, update with better model later
cdFin = rocket.aero.finCd; % assume a cd for now, update this later

dragVec = -unit(velPerp); % nd, direction of drag vec
liftVec = cross(dragVec, finRadialVecI);
liftVec = unit(liftVec); % nd, direction of lift vector

FDragFinI = .5*rho*velPerpMag.^2*rocket.fin.area.*cdFin.*dragVec;
FLiftFinI = .5*rho*velPerpMag.^2*rocket.fin.area.*clFin.*liftVec;

%% Body Tube
% Assume constant drag and no lift
cdBody = rocket.aero.bodyCd;

% Model drag on circular flat plate
FDragBodyI = .5*rho*norm(vel)^2*cdBody*pi*(rocket.structure.tubeDiameter/2)^2*unit(-vel);

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
    FThrustB = -[sin(ctrl.alpha)*cos(ctrl.beta);
                sin(ctrl.beta); 
                -cos(ctrl.alpha)*cos(ctrl.beta)]*thrust;
    
    % Find dm/dt from thrust
    switch models.deltaMassMode
        case 1 % constant mass flow
            dmdt = -(motor.wetMass - motor.dryMass)/motor.thrust(end,1);
            
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
MFinsI(:,1) = cross(rCgFinsI(:,1), FFinsI(:,1));
MFinsI(:,2) = cross(rCgFinsI(:,2), FFinsI(:,2));
MFinsI(:,3) = cross(rCgFinsI(:,3), FFinsI(:,3));
MFinsI(:,4) = cross(rCgFinsI(:,4), FFinsI(:,4));

rCgMotorI = quatVectorRotation(quatConjugate(qi2b),[0;0;motor.height/2-cg]);
MThrustI = cross(rCgMotorI, FThrustI);

sumMoments = sum(MFinsI,2) + MThrustI;

angAccel = (MOI\eye(3))*sumMoments; % sum M = I*alpha
accel = (sum(FFinsI,2) + FDragBodyI + FThrustI + FGravI)/massTot;
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
trajCalcs.MThrustI = MThrustI;

trajCalcs.dmdt = dmdt;
trajCalcs.MOI = MOI;
trajCalcs.accel = accel;
trajCalcs.angAccel = angAccel;


end