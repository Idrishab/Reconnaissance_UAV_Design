clc 
clear 
N = 9; % (number of segments-1) 
Sh = 0.0346; % m ˆ 2 
ARh = 4.667; % Aspect ratio 
lambdah = 0.7; % Taper ratio 
alpha_twist = 1e-8; % Twist angle (deg) 
a_h = -2.56; % tail angle of attack (deg) 
a_2d = 6.108; % tail airfoil lift curve slope (1/rad) 
alpha_0 = 0; % Airfoil zero-lift angle of attack (deg)
b = sqrt(ARh*Sh); % tail span
Chroot = 0.1004; % root chord
MAC = (2/3)*Chroot*(1+lambdah+lambdah^2)/(1+lambdah); % Mean Aerodynamic Chord
theta = pi/(2*N):pi/(2*N):pi/2;
alpha=a_h+alpha_twist:-alpha_twist/(N-1):a_h;% segment’s angle of attack
z = (b/2)*cos(theta);
c = Chroot * (1 - (1-lambdah)*cos(theta)); % Mean Aerodynamics chord at each segment
mu = c * a_2d / (4 * b);
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