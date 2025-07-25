clc
clear
N = 9; % (number of segments-1)
Vs = 16.667; % Stall Speed (m/s)
Wto=343.35; % Takeoff weight (N)
S = 1.2623; % m^2
AR = 10; % Aspect ratio
lambda = 1; % Taper ratio
alpha_twist = -1e-10; % Twist angle (deg)
i_w = 6 % wing setting angle (deg)
a_2d = 6.1985; % Airfoil lift curve slope (1/rad)
a_0 = -5.9709; % flap up zero-lift angle of attack (deg)
cf_c=0.2; %flap to wing chord ratio
bf_b=0.5; %flap-to-wing span ratio
df=15; %flap deflection(deg)
rho = 1.225; % Air density at takeoff altitude (Kg/m^3)

a_0_fd = 2*(-1.15*cf_c*df); % flap down zero-lift angle of attack (deg)
b = sqrt(AR*S); % wing span
MAC = S/b; % Mean Aerodynamic Chord
Vto = Vs * 1.2; % Takeoff speed (m/s)
CL_to = 0.85*2*Wto/(rho*(Vto^2)*S) % Wing required takeoff CL
Croot = (1.5*(1+lambda)*MAC)/(1+lambda+lambda^2); % root chord
theta = pi/(2*N):pi/(2*N):pi/2;
alpha=i_w+alpha_twist:-alpha_twist/(N-1):i_w;
% segment�s angle of attack
for i=1:N
if (i/N)>(1-bf_b)
    alpha_0(i)=a_0_fd; %flap down zero lift AOA
else
alpha_0(i)=a_0; %flap up zero lift AOA
end
end
z = (b/2)*cos(theta);
c = Croot * (1 - (1-lambda)*cos(theta)); % MAC at each segment
mu = c * a_2d / (4 * b);
LHS = mu .* (alpha-alpha_0)/57.3; % Left Hand Side
% Solving N equations to find coefficients A(i):
for i=1:N
for j=1:N
B(i,j) = sin((2*j-1) * theta(i)) * (1 + (mu(i) * (2*j-1)) / sin(theta(i)));
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
cf = cf_c * MAC % flap chord
bf = bf_b * b  % flap span for both sides
CL_TO = pi * AR * A(1)