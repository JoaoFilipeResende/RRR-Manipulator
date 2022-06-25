function [ x,y,z ] = DK( teta1,teta2,teta3,L1,L2,h )
x = (L1.*cos(teta2) + (L2.*cos((teta2) + (teta3)))) .* cos(teta1);
y = (L1.*cos(teta2) + (L2.*cos((teta2) + (teta3)))) .* sin(teta1);
z = h + ( L1.*sin(teta2) + L2.*sin((teta2) + (teta3)));
end

