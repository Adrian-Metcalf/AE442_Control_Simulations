% Destiny Fawley
% 11/6/2019
function ctrl = controlInit(controller, models)

ctrl.igniteMotor = 0;
ctrl.rateRatio = round(models.integrationRate/models.controlRate); % sim/control rate ratio, nd
ctrl.tIgnite = 1.1;
ctrl.alpha = 0; % rad, gimbal angle around y axis 
ctrl.beta = 0; % rad, gimbal angle around x axis

end





