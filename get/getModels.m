% Destiny Fawley
% 11/6/2019
function models = getModels()

models.atmMode = 2; % table atm, no winds
models.navMode = 1; % perfect nav
models.ctrlMode = 1; % state space controller
models.deltaMassMode = 1; % constant change in mass  
models.minAlt = 0; % m, elevation of Champaign
models.seed = 0; % nd, seed for random number generator

models.integrationRate = 100; % Hz, rate of each integration step
models.dataRate = 100; % Hz, rate data will be logged for sim
models.controlRate = 50; % Hz, rate controller is run
models.navigationRate = 50; % Hz, rate navigation is run
models.storageRate = 100; % Hz, rate data will be logged in EEPROM 
models.maxIter = models.integrationRate*20; % nd, max number of loops

models.IMURate = 50; % rate IMU provides new sensor data
models.IMUAccelNoiseStd = 0.008*9.81; % m/s^2
models.IMUAccelNoiseBias = 0; % m/s^2
models.IMUGyroNoiseStd =0.1*pi/180; %rad/s
models.IMUGyroNoiseBias = 0; % rad/s
models.IMUMagNoiseStd = 0; % T
models.IMUMagNoiseBias = 0; % T
models.IMUMagResolution = 0.6e-6; % T

models.MPLRate = 1; % Hz, rate barometer provides new sensor data
models.MPLAltNoiseBias = 0; % m
models.MPLAltNoiseStd = 0.12; % m
models.MPLAltResolution = 0.3; % m

end