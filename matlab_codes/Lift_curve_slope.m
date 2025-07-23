clc
clear
CL1=0.4676;
alfa1=4;
CL2=0.8667;
alfa2=8;
dCL_alfa = (CL2-CL1)/(alfa2-alfa1);
Lift_Curve_Slope = dCL_alfa
Lift_Curve_Slope_rad=Lift_Curve_Slope/0.01745329

%clc
max_thick_to_chord=0.12; % max thickness / chord
LCS_per_rad = 1.8*3.142*(1+(0.8*max_thick_to_chord))
LCS_per_deg = LCS_per_rad*0.01745329