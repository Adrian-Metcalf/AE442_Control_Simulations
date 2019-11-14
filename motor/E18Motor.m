% Destiny Fawley
% 11/6/2019

function motordata = E18Motor()
% http://www.thrustcurve.org/simfilesearch.jsp?id=638
motordata.thrust = [
   0 0
   0.016    6.586
   0.042   18.004
   0.073   27.138
   0.098   29.815
   0.134   30.357
   0.170   30.347
   0.195   31.080
   0.236   30.347
   0.287   30.878
   0.338   30.337
   0.368   30.878
   0.404   29.795
   0.424   30.688
   0.465   29.976
   0.526   29.785
   0.592   29.063
   0.669   28.341
   0.786   26.908
   0.908   23.850
   1.025   21.163
   1.157   17.905
   1.284   14.857
   1.462   11.338
   1.660    7.106
   1.838    3.470
   2.006    1.309
   2.083    0.588
   2.140    0.000]; % [s, N]

motordata.dryMass = 0.0383; % kg
motordata.wetMass = .059; % kg
motordata.height = .07; % m
motordata.diameter = .024; % m


