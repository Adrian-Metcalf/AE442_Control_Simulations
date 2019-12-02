% Destiny Fawley
% 11/20/2019
%%
function onboard = codeLoop(y, tCurr, trajCalcs, models, onboard, i)

nav = navigation(y,tCurr, trajCalcs, models, onboard.nav, i); % get sensor data
ctrl = control(motor, rocket, onboard, nav, tCurr, models, counter); % run controller
storage = storeOnboardData(onboard, nav, ctrl, tCurr, models);

onboard.nav = nav;
onboard.ctrl = ctrl;
onboard.storage = storage;

end