% Destiny Fawley
% 11/6/2019

function nav = navInit(models, y)


nav.rateRatio = round(models.integrationRate/models.navigationRate); % sim/nav rate ratio, nd

% initialize states
nav.posI = y(1:3);
nav.velI = y(4:6);
nav.EulerAngles = y(7:9);
nav.omega = y(10:12);

end