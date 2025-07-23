function preliminary=prel_values(WL,PL)
% All dimensions are in S.I. unit
% Correct all values that has been altered in Prel_design
% such as mto, CLmax, AR,rho_climb, etc.
% If the value(s) obtained for Vmax or Sto seem to be wrong switch the
% equation to be used within the loop
%Vmax = 10; S = 0.1818; Pmax = 38;

WL = 272; PL = 0.08;
mto=35; Wto=mto*9.81; rho=1.225; CLmax=1.6;
CD0 = 0.035;
%STALL SPEED
Vs=sqrt(WL*2/(rho*CLmax))
%MAXIMUM SPEED
pi=3.142;  %e=oswald span efficiency factor
AR=10; np=0.6; %AR=aspect ratio, np=propeller efficiency
e= (1.78*(1-(0.0045*(AR^0.68))))-0.68;
%Vmax_altitude=20;
%x=Vmax_altitude;
%x0=0; y0=1;
%x1=305; y1=0.9711;
%x2=610; y2=0.9428;
%L0=((x-x1)*(x-x2))/((x0-x1)*(x0-x2));
%L1=((x-x0)*(x-x2))/((x1-x0)*(x1-x2));
%L2=((x-x0)*(x-x1))/((x2-x0)*(x2-x1));
%y=(L0*y0)+(L1*y1)+(L2*y2);
%sigma_max=y;
%rho_max=y*1.225;
sigma_max = 0.6292;
rho_max=sigma_max*1.225;
k=1/(pi*e*AR);
%CD0=(((2*Pmax*np)/Vmax)-((4*k*Wto^2)/(rho_max*sigma_max*S*Vmax^2)))/(rho*(Vmax^2)*S); %CD0=zero lift CD
V_max=1; Vmax=0;
while abs((V_max-Vmax)/V_max)>0.000001
    Vmax=V_max;
    V_max=(((np/PL)-(2*k*WL/(rho_max*sigma_max*Vmax)))*WL*2/(rho*CD0))^(1/3);
    %V_max=(2*k*WL/(rho_max*sigma_max))/((np/PL)-0.5*rho*(Vmax^3)*CD0/WL);
end
Vmax=V_max
Vc=Vmax/1.3
%TAKEOFF RUN
Vto=1.3*Vs %takeoff velocity
g=9.81;miu=0.04; %miu=runway friction coefficient
CLc=0.3; delta_CL_flap_to=0.55;%flap lift coefficient at takeoff
CD0_LG=0.009; CD0_HLD_to=0.0055; % zero lift CD for landing gear and high lift device at takeoff
CD0_to=CD0+CD0_LG+CD0_HLD_to;
CL_to=CLc+delta_CL_flap_to;
CD_to=CD0_to+(k*(CL_to^2));
CDg=CD_to-(miu*CL_to);
%CLr=2*Wto/(rho*S*Vto^2);
CLr = CLmax/1.2;
S_to=0; Sto=1;
while abs((S_to-Sto)/S_to)> 0.000001
    Sto=S_to;
    S_to=(log(1-(PL*Vto/np*(miu-((miu+(CDg/CLr))*(exp(0.6*rho*g*CDg*Sto/WL)))))))/(0.6*rho*g*CDg/WL);
    %S_to=(log((miu-((1-exp(0.6*rho*g*CDg*Sto/WL))/(PL*Vto/np)))/(miu+(CDg/CLr))))/(0.6*rho*g*CDg/WL);
end
Sto=S_to
%TURN RATE REQUIREMENT
%ht=20; %Turn altitude
Vt=Vs*1.2; CLt=CLmax*0.6;
%x=ht;
%x0=0; y0=1;
%x1=305; y1=0.9711;
%x2=610; y2=0.9428;
%L0=((x-x1)*(x-x2))/((x0-x1)*(x0-x2));
%L1=((x-x0)*(x-x2))/((x1-x0)*(x1-x2));
%L2=((x-x0)*(x-x1))/((x2-x0)*(x2-x1));
%y=(L0*y0)+(L1*y1)+(L2*y2);
y = 0.6292; % Relative density at turning altitude
rho_t=y*1.225;
q=0.5*rho_t*(Vt^2);
n=q*CLt/WL;
Turn_Rate=(g*sqrt((n^2)-1))/Vt*57.3
%RATE OF CLIMB
rho_climb=1.225;LDmax=1/(2*sqrt(k*CD0));
ROC_max=np*((1/PL)-(sqrt(2*WL/(rho_climb*sqrt(3*CD0/k)))*(1.155/(LDmax*np))))

Power_loading=PL, Wing_loading=WL
mto
Wto
wing_area = Wto/WL
Power_required = Wto/PL