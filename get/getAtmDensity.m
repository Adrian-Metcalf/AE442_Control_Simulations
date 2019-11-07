% Destiny Fawley
% 11/6/2019
function rho = getAtmDensity(pos, models)

switch models.atmMode
    case 1 % exponential
        
    case 2 % table atm, no winds
        atm = load('atm_earth');
        
        alt = atm.table(:,1); % m
        density = atm.table(:,2); % kg/m^3
        
        rho = interp1(alt,density,pos);
        
    case 3 % table atm with winds
        
        
end



end