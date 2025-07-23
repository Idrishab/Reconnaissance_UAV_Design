Wpl = 1.5; % Payload mass in Kg
Wc = 3; % Crew mass in kg
W21 = 0.98;
W32 = 0.97;
W76 = 0.99;
W87 = 0.997;

R = 165; % Range in km
Vs = 80; % Stall speed in Km/h
VEmax = 1.3*Vs*0.911344; % Endurance speed in ft/s
E = 10; % Endurance in hours
LD_max = 12; % Maximum lift to drag ratio
np = 0.8; % Propeller efficiency
SHC_cr = 0.66; % Specific Fuel Consumption during cruise in lb/(hp.h)
SHC_loi = 0.7; % Specific Fuel Consumption during loiter in lb/(hp.h)
C_cr = SHC_cr / (3600*550) % SHF in 1/ft
C_loi = SHC_loi / (3600*550); % SHF in 1/ft

Wcr1 = exp(-((R*3280.84)*C_cr)/(np*LD_max));
Wloi = exp (-((E*3600)*C_loi*VEmax)/(0.866*np*LD_max));
Wcr2 = exp(-((R*3280.84)*C_cr)/(np*LD_max));

W43 = Wcr1
W54 = Wloi
W65 = Wcr2
W81 = W21*W32*W43*W54*W65*W76*W87
Wf = 1.05*(1-W81)

Wto=1;Wtoo=0; 
W_payload_and_crew=(Wpl+Wc)*2.204623 
a=-1.21e-5; b=0.95;
iterations=0;
while abs((Wto-Wtoo)/Wto)>0.001;
    iterations=iterations+1;
    Wtoo=Wto;
    empty_ratio= (a*Wto)+b;
    Wto=(W_payload_and_crew)/(1-Wf-(empty_ratio));
end
iterations
Wto
empty_ratio
empty_weight_kg = empty_ratio * Wto / 2.204623
Wto_kg = Wto / 2.204623 
