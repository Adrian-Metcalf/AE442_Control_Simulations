% Destiny Fawley
% 11/6/2019

function nav = navInit(models, y)


nav.rateRatio = round(models.integrationRate/models.navigationRate); % sim/nav rate ratio, nd

% initialize states
nav.imu.accel = [0;0;-9.81];
nav.imu.omega = y(10:12);
nav.imu.mag = y(1:3);

nav.baro.alt = y(3);

nav.EulerAngles = y(7:9);
nav.omega = y(10:12);
nav.velI = y(4:6);
nav.posI = y(1:3);
end