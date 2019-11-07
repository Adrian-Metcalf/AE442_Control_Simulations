% Destiny Fawley
% 11/6/2019
function models = getModels

models.atmMode = 2; % table atm, no winds
models.navMode = 1; % perfect nav
models.deltaMassMode = 1; % constant change in mass  
models.minAlt = 0; % m, elevation of Champaign
models.integrationRate = 100; % Hz
models.maxIter = models.integrationRate*20; % nd, max number of loops

end