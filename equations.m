function [ posicao,velocidade,aceleracao ] = equations( a0,a1,a2,a3,t )
posicao=a0+a1*t+a2*t.^2+a3*t.^3;
velocidade=a1+2*a2*t+3*a3*t.^2;
aceleracao=2*a2+6*a3*t;
end

