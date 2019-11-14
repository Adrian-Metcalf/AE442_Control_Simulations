% Destiny Fawley
% 11/6/2019

function motordata = C3Motor()
% http://nar.org/SandT/pdf/Estes/C6.pdf
motordata.thrust = [
0 0 
0.023 3.188
0.028 5.669
0.093 9.080
0.235 8.208
0.427 6.881
0.513 6.188
0.600 5.438
0.666 4.803
0.762 3.649
0.838 2.668
0.970 2.149
1.228 1.918
1.522 1.918
1.800 1.860
2.013 1.745
2.068 2.034
2.134 1.803
2.326 1.803
2.509 1.745
2.645 1.687
2.721 1.457
2.807 0.879
2.860 0.000]; % [s, N]

motordata.dryMass = 0.0187; % kg
motordata.wetMass = .0239; % kg
motordata.height = .072; % m
motordata.diameter = .018; % m
