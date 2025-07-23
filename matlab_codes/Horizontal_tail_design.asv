clc
clear
VH = 0.7; % horizontal tail volume coefficient
Kc = 1.1; % Fuselage aft shape correction factor
MAC = 0.3553; % wing MAC (m)
S = 1.2623; % Wing area (m^2)
Df = 0.35; % Fuselage diameter (m)
lL = 0.45; % Ratio of horizontal tail arm to aircraft legnth
Cm_af = -0.1328; % wing airfoil pitching moment
AR = 10; % Wing aspect ratio
swp = 0; % Wing sweep angle (deg)
alfa_twist = 0; % Wing twist angle (deg)
Wavg = 316.35; % Aircraft average weight (N)
Vc = 27.78; % Cruise speed (m/s)
rho_c = 0.7708; % Air density at cruise altitude (Kg/m^3)
nh = 0.92; % Horizontal tail (HT) efficiency
h = 0.2; % Center of gravity limit (m/m)
ho = 0.25;% Wing/fuselage aerodynamic center (m/m)
swp_h = 0; % HT sweep angle (deg)
dihed = 0; % HT dihedral angle (deg)
lambdah = 0.7;% HT taper ratio 
ARh = 2*AR/3 % HT aspect ratio
Cl_alfa = 6.108; % HT airfoil lift curve slope (1/rad)
CL_alfa_w = 4.827578; % Wing lift curve slope (1/rad)
iw = 2.8; % Wing setting angle (deg)
alfa_sw = 13; % Wing airfoil stall angle (deg)
alfa_fus = 0; % Fuselage AOA at cruise (rad)
alfa_sh = 10; % HT airfoil stall AOA (deg)
ac_wf = 0.25; ach = 0.25; % Wing and HT aerodynamic center

l_opt = Kc *sqrt(4*MAC*S*VH/(3.142*Df))% optimum tail arm (m)
l = l_opt;
L = l/lL
Sh = VH*MAC*S/l % Horizontal tail area (m^2)
Cmo_wf= (Cm_af*AR*(cosd(swp)^2)/(AR+(2*cosd(swp)^2)))+(0.01*alfa_twist)  % Wing/fuselage aerodynamic pitching moment coefficient
CLc=2*Wavg/(rho_c*S*Vc^2) % Aircraft cruise lift coefficient
CLh = (Cmo_wf+(CLc*(h-ho)))/(nh*VH) % Horizontal tail required CL at cruise
CL_alfa_h = Cl_alfa/(1+(Cl_alfa/(3.142*ARh))) % HT lift curve slope (1/rad)
set_h = CLh/CL_alfa_h % Initial HT setting angle (rad)
eo = 2*CLc/(3.142*AR) % Downwash angle at zero angle of attack (rad)
e_slope = (2*CL_alfa_w)/(3.142*AR) % Downwash slope (rad/rad)
E = eo+(e_slope*(iw/57.3)) % Downwash angle at tail (rad)
bh = sqrt(ARh*Sh) % HT span
MACh = bh/ARh % HT MAC
Ch_root =3*MACh/(2*((1+lambdah+(lambdah^2))/(1+lambdah))) % HT root chord
Ch_tip = lambdah * Ch_root % HT tip chord
ih = set_h-alfa_fus+E % HT setting angle (rad)

% Calculation of HT CL with lifting line theory
N = 9; % (number of segments-1) 
alpha_twist = 0.00001; % Twist angle (deg) 
a_h = -5.12; % tail angle of attack (deg) 
a_2d = Cl_alfa; % tail airfoil lift curve slope (1/rad) 
alpha_0 = 0; % Airfoil zero-lift angle of attack (deg)
theta = pi/(2*N):pi/(2*N):pi/2;
alpha=a_h+alpha_twist:-alpha_twist/(N-1):a_h;% segment’s angle of attack
z = (bh/2)*cos(theta);
c = Ch_root * (1 - (1-lambdah)*cos(theta)); % Mean Aerodynamics chord at each segment
mu = c * a_2d / (4 * bh);
LHS = mu .* (alpha-alpha_0)/57.3; % Left Hand Side
% Solving N equations to find coefficients A(i):
for i=1:N
for j=1:N
B(i,j)=sin((2*j-1) * theta(i)) * (1+(mu(i) *(2*j-1))/sin(theta(i)));
end
end
A=B\transpose(LHS);
for i = 1:N
sum1(i) = 0;
sum2(i) = 0;
for j=1:N
sum1(i) = sum1(i) + (2*j-1) * A(j)*sin((2*j-1)*theta(i));
sum2(i) = sum2(i) + A(j)*sin((2*j-1)*theta(i));
end
end
CL_htail = pi * ARh * A(1)

set_h = a_h; % HT setting angle (deg) from lifting line theory
ih = set_h-alfa_fus+(E*57.3) % HT setting angle (deg)
HT_stall_AOA = alfa_sh-ih % Fuselage AOA for HT to stall (deg)
hnp = (1/4)+((1+(2/AR))/(1+(2/ARh))*(1-(4/(AR+2)))*VH); % Neutral point to chord ratio
Xcg = h*MAC; % CG distance (m) from Wing LE towards TE
SM = hnp-h % Static Margin
HT_Wing_area = Sh/S; % HT to wing area ratio
Cm_alfa = (CL_alfa_w *(h-ho))-(CL_alfa_h*nh*(Sh/S)*((l/MAC)-h)*(1-e_slope))% Longitudinal static stability derivative (1/rad)
ht_above = l*tand(alfa_sw-iw+3); % HT location above wing (m) for HT with higher vertical location relative to the wing ac
ht_below = l*tand(alfa_sw-iw-3) % HT location below wing (m) for HT with lower vertical location relative to the wing ac
LE_to_LE = (ac_wf*MAC)+l-((ach*MACh)+(Ch_root-MACh)) % Horizontal distance from wing root LE to tail root LE
LE_to_TE = LE_to_LE + Ch_root % Horizontal distance from wing root LE to tail root TE

% Horizontal tail trim analysis
Mowf = Cmo_wf*0.5*rho_c*S*Vc^2; % Wing/Fuselage pitching moment
Lh = CL_htail * 0.5*rho_c*Sh*nh*Vc^2; % HT Lift
Lwf = Wavg; % Cruise lift
Summation_Mcg = Mowf+(Lh*l)+(Lwf*(h-ho)*MAC); % Trim analysis. This should be apprxt. equal to 0