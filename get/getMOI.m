% Destiny Fawley
% 11/6/2019
function MOI = getMOI(rocket, motor, cg, yi)

% motor
motorMass = motor.dryMass + yi(13);
motorHeight = motor.height;
motorDiameter = motor.diameter;

gimbalMass = rocket.structure.gimbalMass;
gimbalInnerRadius = motor.diameter/2;
gimbalOuterRadius = rocket.structure.tubeDiameter/2;
gimbalHeight = motor.height;

% fins
numFins = rocket.fin.numFins;
finWidth = rocket.fin.width;
finHeight = rocket.fin.height;
finThickness = rocket.fin.thickness;
finMass = rocket.structure.finMass;
finAngles = linspace(0,2*pi,rocket.fin.numFins+1);
finAngles(end) = [];

% body tube
tubeMass = rocket.structure.tubeMass;
tubeDiameter = rocket.structure.tubeDiameter;
tubeLength = rocket.structure.length;

% avionics bay
avBayWidth = rocket.avBay.width; % X
avBayHeight = rocket.avBay.height; % Z
avBayThickness = rocket.avBay.thickness; % Y
avBayMass = rocket.structure.electronicsMass;

% epoxy
epoxyMass = rocket.structure.epoxyMass;
epoxyCg = rocket.structure.epoxyCg;

% landing legs
legMass = rocket.structure.legsMass/8; % kg, mass of each leg
legRadius = rocket.legs.radius; % m, radius of landing legs
upperLength = rocket.legs.upperStageLength; % m, length of rods on upper stage
lowerLength = rocket.legs.lowerStageLength; % m, length of rods on lower stage
upperAngle = rocket.legs.upperStageAngle; % rad, angle of upper stage rods from -Z axis
lowerAngle = rocket.legs.lowerStageAngle; % rad, angle of lower stage rods from -Z axis
upperZRadius = rocket.legs.upperZRadius; % m,  radial distance from Z-axis of each upper stage leg
lowerZRadius = rocket.legs.lowerZRadius; % m,  radial distance from Z-axis of each lower stage leg
legCg = rocket.structure.legsCg; % m, c.g. location of landing legs

% Get ZZ component
IzzFins = numFins*(1/12*finMass*(finWidth^2 + finThickness^2) + ...
    finMass*(tubeDiameter/2 + finWidth/2)^2);

IzzTube = tubeMass*(tubeDiameter/2)^2;

IzzMotor = .5*motorMass*(motorDiameter/2)^2;

% av bay is .07112 x 0.003175 x 0.1016 m (X,Y,Z) modeled as solid cuboid
IzzElectronics = 1/12*avBayMass*(avBayWidth^2 + avBayThickness^2);

IzzEpoxy = 0;

IzzGimbal = 0.5*gimbalMass*(3*(gimbalInnerRadius^2 + gimbalOuterRadius^2));

% Get YY component
finYAxisDist = sqrt((tubeLength - cg - finHeight/2)^2 + ...
    ((tubeDiameter/2 + finWidth/2)*sin(finAngles)).^2);

IyyFins = sum(1/12*finMass*(finWidth^2*sin(finAngles).^2 + finThickness^2*cos(finAngles).^2 + finHeight^2) + ...
    finMass*(finYAxisDist).^2);

IyyTube = 1/12*tubeMass*(3*((tubeDiameter/2)^2 + ((tubeDiameter - .02)/2)^2) ...
    + tubeLength^2) + tubeMass*(cg - tubeLength/2)^2;

IyyMotor = 1/12*motorMass*(3*(motorDiameter/2)^2 + (motorHeight)^2) + ...
    motorMass*(cg - motorHeight/2)^2;

IyyElectronics = 1/12*avBayMass*(avBayWidth^2 + avBayHeight^2);

IyyEpoxy = epoxyMass*(epoxyCg - cg)^2;

IyyGimbal = 1/12*gimbalMass*(3*(gimbalInnerRadius^2 + gimbalOuterRadius^2) + gimbalHeight^2);

% Get XX component
finXAxisDist = sqrt((tubeLength - cg - finHeight/2)^2 + ...
    ((tubeDiameter/2 + finWidth/2)*cos(finAngles)).^2);

IxxFins = sum(1/12*finMass*(finWidth^2*cos(finAngles).^2 + finThickness^2*sin(finAngles).^2 + finHeight^2) + ...
    finMass*(finXAxisDist).^2);
IxxTube = 1/12*tubeMass*(3*((tubeDiameter/2)^2 + ((tubeDiameter - .02)/2)^2) ...
    + tubeLength^2) + tubeMass*(cg - tubeLength/2)^2;
IxxMotor = 1/12*motorMass*(3*(motorDiameter/2)^2 + (motorHeight)^2) + ...
    motorMass*(cg - motorHeight/2)^2;

IxxElectronics = 1/12*avBayMass*(avBayThickness^2 + avBayHeight^2);

IxxEpoxy = epoxyMass*(epoxyCg - cg)^2;

IxxGimbal = 1/12*gimbalMass*(3*(gimbalInnerRadius^2 + gimbalOuterRadius^2) + gimbalHeight^2);

% Landing legs separately from asymmetry
Iu = [1/12*legMass*(3*legRadius^2+upperLength^2) 0 0;
    0 1/12*legMass*(3*legRadius^2+upperLength^2) 0;
    0 0 .5*legMass*legRadius^2];

Il =  [1/12*legMass*(3*legRadius^2+lowerLength^2) 0 0;
    0 1/12*legMass*(3*legRadius^2+lowerLength^2) 0;
    0 0 .5*legMass*legRadius^2];

Iu(1,1) = Iu(1,1) + legMass*(legCg-cg)^2;
Iu(2,2) = Iu(2,2) + legMass*(legCg-cg)^2;
Iu(3,3) = Iu(3,3) + legMass*(upperZRadius)^2;
Il(1,1) = Il(1,1) + legMass*(legCg-cg)^2;
Il(2,2) = Il(2,2) + legMass*(legCg-cg)^2;
Il(3,3) = Il(3,3) + legMass*(lowerZRadius)^2;

Rupper1 = [cos(upperAngle) 0 -sin(upperAngle); % rotation of legs on x-axis
    0 1 0;
    sin(upperAngle) 0 cos(upperAngle)];
Rupper2 = [1 0 0 % rotation of legs on y-axis
   0 cos(upperAngle) -sin(upperAngle);
     0 sin(upperAngle) cos(upperAngle)];

Rlower1 = [cos(lowerAngle) 0 -sin(lowerAngle); % rotation of legs on x-axis
    0 1 0;
    sin(lowerAngle) 0 cos(lowerAngle)];
Rlower2 = [1 0 0 % rotation of legs on y-axis
   0 cos(lowerAngle) -sin(lowerAngle);
     0 sin(lowerAngle) cos(lowerAngle)];
 
Il = 2*Rlower1*Il*Rlower1' + 2*Rlower2*Il*Rlower2';
Iu = 2*Rupper1*Iu*Rupper1' + 2*Rupper2*Iu*Rupper2';
ILegs = Il + Iu;

Ixx = IxxFins+IxxTube+IxxMotor+IxxElectronics+IxxEpoxy+IxxGimbal;
Iyy = IyyFins+IyyTube+IyyMotor+IyyElectronics+IyyEpoxy+IyyGimbal;
Izz = IzzFins+IzzTube+IzzMotor+IzzElectronics+IzzEpoxy+IzzGimbal;

MOI = [Ixx 0 0;
       0 Iyy 0;
       0 0 Izz] + ILegs;
   
end