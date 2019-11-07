% Destiny Fawley
% 11/6/2019
function Cg = getCg(rocket, motor, propMass)
massTot = rocket.structure.dryMass + propMass - rocket.structure.ballastMass;
Cg = (rocket.structure.finMass*(rocket.structure.finCg)*rocket.fin.numFins + ...
    rocket.structure.tubeMass*rocket.structure.tubeCg + ...
    rocket.structure.electronicsMass*rocket.structure.electronicsCg + ...
    rocket.structure.epoxyMass*rocket.structure.epoxyCg + ...
    rocket.structure.legsMass*rocket.structure.legsCg + ...
    rocket.structure.gimbalMass*rocket.structure.gimbalCg + ...
    propMass*motor.height/2)/...
    (massTot);

end