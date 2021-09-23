function ss_spacing =  IDM_equilibrium_spacing(v,delta,S0,T,v0)



if(v>v0 || v<0)
   error('IDM is not well defined for speed above %f or below zero',v0);
end

ss_spacing = sqrt((S0+v*T)^2/(1-(v/v0)^delta));

end