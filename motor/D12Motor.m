% Destiny Fawley
% 11/6/2019

function motordata = D12Motor()
% http://nar.org/SandT/pdf/Estes/C6.pdf
motordata.thrust = [
0 0
0.049 2.569
0.116 9.369
0.184 17.275
0.237 24.258
0.282 29.730
0.297 27.010
0.311 22.589
0.322 17.990
0.348 14.126
0.386 12.099
0.442 10.808
0.546 9.876
0.718 9.306
0.879 9.105
1.066 8.901
1.257 8.698
1.436 8.310
1.590 8.294
1.612 4.613
1.650 0.000]; % [s, N]

motordata.dryMass = 0.016; % kg
motordata.wetMass = .0371; % kg
motordata.height = .070; % m
motordata.diameter = .024; % m

