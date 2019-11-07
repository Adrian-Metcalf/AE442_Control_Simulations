% Destiny Fawley
% 11/6/2019
function [pointsRocket, facesRocket, cdataRocket, ...
    pointsThruster, facesThruster, cdataThruster] = getRocketModel(rocket)


% - vertices, faces, and colors for flaps in B
n = rocket.fin.numFins;
finAngles = linspace(0,2*pi,n+1);
finAngles(end) = [];

ang = linspace(0,2*pi,n+1);
facesRocket = [];
pointsRocket = [];
for j = 1:n
    % top of fin
    pointsRocket = [pointsRocket, [(rocket.structure.tubeDiameter*sin(ang(j))/2 + rocket.fin.thickness*cos(ang(j))/2);
        (rocket.structure.tubeDiameter/2)*cos(ang(j)) - (rocket.fin.thickness/2)*sin(ang(j));
        rocket.structure.length]];
    pointsRocket = [pointsRocket, [(rocket.structure.tubeDiameter*sin(ang(j))/2 - rocket.fin.thickness*cos(ang(j))/2);
        (rocket.structure.tubeDiameter/2)*cos(ang(j)) + (rocket.fin.thickness/2)*sin(ang(j));
        rocket.structure.length]];
    pointsRocket = [pointsRocket, [(rocket.structure.tubeDiameter/2 + rocket.fin.width)*sin(ang(j)) + rocket.fin.thickness*cos(ang(j))/2;
        (rocket.structure.tubeDiameter/2 + rocket.fin.width)*cos(ang(j)) - (rocket.fin.thickness/2)*sin(ang(j));
        rocket.structure.length]];
    pointsRocket = [pointsRocket, [(rocket.structure.tubeDiameter/2 + rocket.fin.width)*sin(ang(j)) - rocket.fin.thickness*cos(ang(j))/2;
        (rocket.structure.tubeDiameter/2 + rocket.fin.width)*cos(ang(j)) + (rocket.fin.thickness/2)*sin(ang(j));
        rocket.structure.length]];
    % right side looking down
    pointsRocket = [pointsRocket, [(rocket.structure.tubeDiameter*sin(ang(j))/2 + rocket.fin.thickness*cos(ang(j))/2);
        (rocket.structure.tubeDiameter/2)*cos(ang(j)) - (rocket.fin.thickness/2)*sin(ang(j));
        rocket.structure.length - rocket.fin.height]];
    pointsRocket = [pointsRocket, [(rocket.structure.tubeDiameter/2 + rocket.fin.width)*sin(ang(j)) + rocket.fin.thickness*cos(ang(j))/2;
        (rocket.structure.tubeDiameter/2 + rocket.fin.width)*cos(ang(j)) - (rocket.fin.thickness/2)*sin(ang(j));
        rocket.structure.length - rocket.fin.height]];
    % left side looking down
    pointsRocket = [pointsRocket, [(rocket.structure.tubeDiameter*sin(ang(j))/2 - rocket.fin.thickness*cos(ang(j))/2);
        (rocket.structure.tubeDiameter/2)*cos(ang(j)) + (rocket.fin.thickness/2)*sin(ang(j));
        rocket.structure.length - rocket.fin.height]];
    pointsRocket = [pointsRocket, [(rocket.structure.tubeDiameter/2 + rocket.fin.width)*sin(ang(j)) - rocket.fin.thickness*cos(ang(j))/2;
        (rocket.structure.tubeDiameter/2 + rocket.fin.width)*cos(ang(j)) + (rocket.fin.thickness/2)*sin(ang(j));
        rocket.structure.length - rocket.fin.height]];
  
    facesRocket = [facesRocket; 8*(j-1)+[1 2 3]; 8*(j-1)+[2 3 4]; 8*(j-1)+[1 3 5]; ...
        8*(j-1)+[3 5 6]; 8*(j-1)+[2 7 8]; 8*(j-1)+[2 4 8]; 8*(j-1)+[3 4 6]; ...
        8*(j-1)+[6 8 4]; 8*(j-1)+[1 2 7]; 8*(j-1)+[1 7 5]; 8*(j-1)+[5 6 7]; ...
        8*(j-1)+[7 8 6]];
    
end

% rocket body
startIdx = length(pointsRocket(1,:))+3;
faceEndIdx = size(facesRocket,1);
n = 180;
angles = linspace(0,2*pi,n+1);
angles(end) = [];

pointsThruster = [0; 0; -rocket.structure.length/3];
pointsThruster = [pointsThruster, [0; ...
    rocket.structure.tubeDiameter/2; ...
    0]];
facesThruster = [];

pointsRocket = [pointsRocket, [0; ...
    0; ...
    0]];
pointsRocket = [pointsRocket, [0; ...
    0; ...
    rocket.structure.length]];
pointsRocket = [pointsRocket, [0; ...
    rocket.structure.tubeDiameter/2; ...
    rocket.structure.length]];
pointsRocket = [pointsRocket, [0; ...
    rocket.structure.tubeDiameter/2; ...
    0]];

for j = 2:n
    
    pointsRocket = [pointsRocket, [sin(angles(j))*rocket.structure.tubeDiameter/2; ...
        cos(angles(j))*rocket.structure.tubeDiameter/2; ...
        rocket.structure.length]];
    pointsRocket = [pointsRocket, [sin(angles(j))*rocket.structure.tubeDiameter/2; ...
        cos(angles(j))*rocket.structure.tubeDiameter/2; ...
        0]];
    
    facesRocket = [facesRocket; [2*(j-2)+startIdx, 2*(j-2)+startIdx+1, 2*(j-2)+startIdx+2];
        [2*(j-2)+startIdx+1, 2*(j-2)+startIdx+2, 2*(j-2)+startIdx+3];];
    facesRocket = [facesRocket; [2*(j-2)+startIdx, 2*(j-2)+startIdx+2, startIdx-1];
        [2*(j-2)+startIdx+1, 2*(j-2)+startIdx+3, startIdx-2];];
    
    % Thruster 
    pointsThruster = [pointsThruster,[sin(angles(j))*rocket.structure.tubeDiameter/2; ...
        cos(angles(j))*rocket.structure.tubeDiameter/2; ...
        0]];
    facesThruster = [facesThruster; [1, j-1, j]];
    
end
% connect endpoints to start points
facesRocket = [facesRocket; [startIdx-1 startIdx size(pointsRocket,2)-1]; [startIdx-2 startIdx+1 size(pointsRocket,2)];
    [startIdx, startIdx+1, size(pointsRocket,2)]; [startIdx, size(pointsRocket,2)-1, size(pointsRocket,2)]];

facesThruster = [facesThruster; [1, 2, size(pointsThruster,2)]];

% - colors
flapColor = [1,0.6,0];
rocketColor =[0.4745,0.6471,0.9098];
cdataRocket = [repmat(flapColor,[faceEndIdx,1]);
        repmat(rocketColor,[size(facesRocket,1)-faceEndIdx,1])];
    
thrustColor = [1, 0, 0];
cdataThruster = repmat(thrustColor,[size(facesThruster,1),1]);

end