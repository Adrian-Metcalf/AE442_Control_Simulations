% Destiny Fawley
% 11/6/2019

function rocket = getRocketCDR(motor, mass, alt0, length, diameter)
% Returns dimensions, cg, and initial state of rocket
% body data
rocket.structure.length = length; % 12 in rocket
rocket.structure.tubeDiameter = diameter; % m, diameter of rocket

% fin data
rocket.fin.numFins = 4; % number of fins
rocket.fin.height = 0.1016; % m, axial height of rectangular fins
rocket.fin.width = 0.0508; % m, radial width of fins
rocket.fin.thickness = 0.003175; % m, thickness of fins
rocket.fin.area = rocket.fin.height*rocket.fin.width; % m^2, area of one fin
rocket.fin.AR = rocket.fin.width/rocket.fin.height; % nd, aspect ratio

% avionics bay data
rocket.avBay.width = .07112; % m, X width of avionics sled
rocket.avBay.thickness = 0.003175; % m, Y thickness of avionics sled
rocket.avBay.height = 0.1016; % m, Z height of avionics sled

% landing legs information
rocket.legs.radius = 0.003175; % m, radius of landing legs
rocket.legs.upperStageLength = .206; % m, length of rods on upper stage
rocket.legs.upperStageAngle = 20*pi/180; % rad, angle of upper stage rods from -Z axis
rocket.legs.upperZRadius = 0.05; % m, radial distance from Z-axis of each upper stage leg
rocket.legs.lowerStageLength = 0.155; % m, length of rods on lower stage
rocket.legs.lowerStageAngle = 45*pi/180; % rad, angle of lower stage rods from -Z axis
rocket.legs.lowerZRadius = 0.05; % m, radial distance from Z-axis of each lower stage leg

% mass and MOI data
rocket.structure.finMass = .002625; % kg, mass of each fin
rocket.structure.tubeMass = .1329; % kg, mass of body tube
rocket.structure.electronicsMass = 0.1275; % kg, mass of electronics
rocket.structure.ballastMass = 0.16588; % kg, mass of all other components
rocket.structure.epoxyMass = .04; % kg, mass of epoxy/other
rocket.structure.legsMass = .1535; % kg, total mass of landing legs and attachments
rocket.structure.gimbalMass = .17172 - motor.wetMass;

rocket.structure.dryMass = mass;

rocket.structure.ballastMass = mass - (rocket.fin.numFins*rocket.structure.finMass + ...
    rocket.structure.tubeMass + rocket.structure.electronicsMass + ...
    + rocket.structure.epoxyMass + ...
    rocket.structure.legsMass + rocket.structure.gimbalMass); % kg, mass of all other components

rocket.structure.finCg = rocket.structure.length-rocket.fin.height/2; % m, cg distance from bottom of rocket
rocket.structure.tubeCg = rocket.structure.length/2; % m, cg distance from bottom of rocket
rocket.structure.electronicsCg = .1818; % m, cg distance from bottom of rocket
rocket.structure.ballastCg = 0.119; % m, cg distance from bottom of rocket
rocket.structure.epoxyCg = rocket.structure.length/2; % m, cg distance from bottom of rocket
rocket.structure.legsCg = .04326; % m, cg distance from bottom of rocket
rocket.structure.gimbalCg = motor.height/2;

rocket.aero.finClSlope = 2*pi; % nd, lift slope of fins with aoa
rocket.aero.finCd = .03; % nd, drag coefficient of fins
rocket.aero.bodyCd = 1.12;  % nd, drag coefficient of body

% iniial state 
% [posX posY posZ velX velY velZ roll(Z) pitch(Y) yaw(X) omegaX omegaY omegaZ mass]
% Note that omegas are NOT derivatives of the euler angles, they are ang
% vel in inertial frame
rocket.y0 = [0 0 alt0, ...
            0 0 0, ...
            0 0*pi/180 0, ...
            0 0 0, ...
            motor.wetMass]';

end