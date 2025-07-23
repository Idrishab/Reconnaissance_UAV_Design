clc
clear
n_places = 0; % no of places
ind_weight = 0*9.8; % individyual weight (N)
carry_on = 0*9.8; % passenger carry on (N)
cargo_weight = 10*9.8; % (N)
k = 3.5; % k is 3.5 for reciprocating engine
rho = 1.225; % air density at sea level (kg/m^3)
Vstall = 16.667; % stall speed (m/s)
CLmax = 1.6; 
Vmax = 33.33; % maximum speed (m/s)
hc = 4572; % cruise altitude (m)
sigmac = 0.6292;% air relative density at cruising altitude (kg/m^3)
AR = 10; % wing aspect ratio
CDp=0.025; % Profile drag
e=0.825; % Oswald span efficiency factor
np = 0.8; % propeller efficiency
pi = 3.142;
ROC = 0.5; % Rate of climb (m/s)
CDo = 0.035; %CD0=zero lift CD

a = -1.21e-5; b = 0.95; C_f = 0.51; % C_f is correction factor
% C_f is 0.9 for composite construction, 1.03 for Utility GA and 1.06 for
% acrobatic type
%A = 1; c = 1; Kus = 1;
DR = 170*2; %Design range (in km)
Endur = 10; % Endurance in hours
SFC = 0.6; % Specific fuel consumption in lb/hp.h

Wp = (n_places*(ind_weight+carry_on))+cargo_weight; % payload weight (N)
Wpl = Wp/9.8; % Payload weight in (kg)
G_weight = k * Wp; % (N)
Wg = G_weight; % gross weight (N)
WL = 0.5*rho*(Vstall^2)*CLmax % (N/m^2)
Vland = 1.38*Vstall % landing speed (m/s)
Vcr = Vmax/1.3 % Cruise speed (m/s)
Se = Wg/WL % effective wing area (m^2)
% THIS SECTION IS FOR CRUISE ALTITUDE DENSITY INTERPOLATION
%x=hc;
%x0=0; y0=1;
%x1=305; y1=0.9711;
%x2=610; y2=0.9428;
%L0=((x-x1)*(x-x2))/((x0-x1)*(x0-x2));
%L1=((x-x0)*(x-x2))/((x1-x0)*(x1-x2));
%L2=((x-x0)*(x-x1))/((x2-x0)*(x2-x1));
%y=(L0*y0)+(L1*y1)+(L2*y2);
%sigmac=y;
%rhoc=y*1.225; % density at cruise (Kg/m^3)
rhoc=sigmac*1.225; % density at cruise (Kg/m^3)
DP = 0.5*rhoc*sigmac*Vcr^2; % dynamic pressure (N/m^2)
CLc = WL/DP; % coefficient of lift at cruise
F = 1/CLc;
CD = CDp+((CLc^2)/(pi*e*AR));% Total drag
LD = CLc/CD; % Lift to drag ratio
D = Wg/LD; % Total drag (N)
THP = D*Vcr/745.7; % thrust horsepower required at cruise (hp)
BHPcr = THP/(np*0.7); % Brake horsepower at cruise (hp)
BHPsl = BHPcr*(rho/rhoc)^0.96; % BHP at sea level (hp)
CLc_opt = sqrt(pi*e*AR*CDp); % optimum cruise CL
Vcr_opt = sqrt(2*Wg/(rhoc*sigmac*CLc_opt*Se*F))
K = 1/(pi*e*AR);
PL_roc=1/((ROC/np)+(sqrt(2*WL/(rho*sqrt(3*CDo/K)))*(1.155/(LD*np))));
P = Wg/PL_roc; % Power required for ROC (W)

% SECOND PASS DESIGN
WTO = (Wg/9.8)*2.204623; % Takeoff weight in (lb)
We = ((a*WTO) + b)* C_f*WTO; % Empty weight (from Muhammad Sadraey). This works
%in pounds
Wem = We * 0.4536 % Empty weight in Kg
% We = A*(WTO^c)*Kus*WTO; % Empty weight (from Source 2)
LR = (Endur*3600)*Vcr_opt; % Loiter range in (m)
Woil = ((0.01/603504)*(BHPcr*745.7)*((DR*1000)+LR)/Vcr_opt)/9.8 % Oil weight (Kg)
Wfuel = ((SFC/603504)*(BHPcr*745.7)*((DR*1000)+LR)/Vcr_opt)/9.8 % Fuel weight (Kg)
WG = Wem+Woil+Wfuel+Wpl % New gross weight (Kg)
CF = WG/Wg; % Correction factor
Se2 = Se*CF % New wing area (m^2)
D2 = D*CF % New drag force (N)
T2 = D2 % New thrust (N)
BHPsl2 = BHPsl*CF % New maxiumum Brake Horsepower