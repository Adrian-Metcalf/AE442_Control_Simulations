% Destiny Fawley
% 11/6/2019
function rocket = getRocket(motor)
% Returns dimensions, cp, cg, and initial state of rocket

rocket.structure.length = .5; % 12 in rocket
rocket.structure.tubeDiameter = 0.076; % m, diameter of rocket

% fin data
rocket.fin.numFins = 4; % number of fins
rocket.fin.height = 0.0381; % m, axial height of rectangular fins
rocket.fin.width = 0.0127; % m, radial width of fins
rocket.fin.thickness = 0.003175; % m, thickness of fins
rocket.fin.area = rocket.fin.height*rocket.fin.width; % m^2, area of one fin
rocket.fin.AR = rocket.fin.width/rocket.fin.height; % nd, aspect ratio

% mass and MOI data
rocket.structure.finMass = 0.01; % kg, mass of each fin
rocket.structure.tubeMass = 0.03; % kg, mass of body tube
rocket.structure.electronicsMass = 0.05; % kg, mass of electronics
rocket.structure.ballastMass = 0.05; % kg, mass of all other components
rocket.structure.dryMass = 0.707185;%rocket.fin.numFins*rocket.structure.finMass + ...
%     rocket.structure.tubeMass + rocket.structure.electronicsMass + ...
%     rocket.structure.ballastMass + motor.dryMass;

rocket.structure.electronicsCg = 7/12*rocket.structure.length; % m, cg distance from bottom of rocket
rocket.structure.ballastCg = 2/3*rocket.structure.length; % m, cg distance from bottom of rocket

rocket.aero.finClSlope = 2*pi; % nd, lift slope of fins with aoa
rocket.aero.finCd = .03; % nd, drag coefficient of fins
rocket.aero.bodyCd = 1.12;  % nd, drag coefficient of body

% iniial state 
% [posX posY posZ velX velY velZ roll pitch yaw rollDot pitchDot yawDot mass]
rocket.y0 = [0 0 20 0 0 0 0 0 0 0 0 0 motor.wetMass]';

end