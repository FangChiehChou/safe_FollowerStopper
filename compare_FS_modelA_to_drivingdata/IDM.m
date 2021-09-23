%IDM model returning acceleration of the IDM vehicle.
% This function is based on the model shwon in "Martin Treiber and Arne Kesting. 2013. Traffic Flow Dynamics. (2013)."

function AccelCmd = IDM(v,h,hdot,uMin,uMax,params)
% v : current vehicle speed
% h : bumper-to-bumber distance
% hdot : 

% delta = 4; 
% a = 1;
% b = 1.5;
% S0 = 2;
% T = 1;
% v0 = 30;

delta = params.delta; 
a = params.a;
b = params.b;
S0 = params.S0;
T = params.T;
v0 = params.v0;

% S_STAR = S0 +   max([0,v*T-(v.*hdot)/(2*sqrt(a*b))]);

temp_term = v*T-(v.*hdot)/(2*sqrt(a*b));
temp_term(temp_term<0) = 0;
S_STAR = S0 +  temp_term;

AccelCmd = a*(1-(v/v0).^delta-(S_STAR./h).^2);


AccelCmd(AccelCmd>=uMax) = uMax;
AccelCmd(AccelCmd<=uMin) = uMin;
