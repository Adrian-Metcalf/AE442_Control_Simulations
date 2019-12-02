% Destiny Fawley
% 11/6/2019

function nav = navInit(models, y, trajCalcs)


nav.rateRatio = round(models.integrationRate/models.navigationRate); % sim/nav rate ratio, nd
nav.IMURateRatio = round(models.integrationRate/models.IMURate); % sim/IMU rate ratio, nd
nav.MPLRateRatio = round(models.integrationRate/models.MPLRate); % sim/MPL rate ratio, nd

% initialize states
% noise on IMU acceleration reading
accelNoise = normrnd(models.IMUAccelNoiseBias,...
    models.IMUAccelNoiseStd,[3,1]);
nav.imu9250.accel = trajCalcs.accel + accelNoise;

% noise on IMU gyro reading
gyroNoise = normrnd(models.IMUGyroNoiseBias,...
    models.IMUGyroNoiseStd,[3,1]);
nav.imu9250.omega = y(10:12) + gyroNoise;

% noise on IMU magnetometer reading
magNoise = normrnd(models.IMUMagNoiseBias,...
    models.IMUMagNoiseStd,[3,1]);
nav.imu9250.mag = y(1:3) + magNoise;

% noise on barometer
baroNoise = normrnd(models.MPLAltNoiseBias,...
    models.MPLAltNoiseStd,1);
alt = y(3) + baroNoise;
% limit resolution on altitude reading
if mod(alt,models.MPLAltResolution) < models.MPLAltResolution/2
    nav.baro.alt = alt - mod(alt,models.MPLAltResolution);
else
    nav.baro.alt = alt - mod(alt,models.MPLAltResolution)+models.MPLAltResolution;
end


end