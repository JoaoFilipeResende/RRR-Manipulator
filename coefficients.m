function [ a0,a1,a2,a3 ] = coefficients( tetai,tetaf,t )
a0=tetai;
a1=0;
a2=(3/t.^2)*(tetaf-tetai);
a3=-(2/t.^3)*(tetaf-tetai);
end

