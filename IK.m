function [ teta1,teta2,teta3 ] = IK( x,y,z,L1,L2,h )
D = ( x^2 + y^2 +(z-h)^2 - L1^2 - L2^2 ) / (2 * L1 * L2);

teta1 =  atan2(y,x);
teta3 =  atan2(-sqrt(1-(D^2)),D);
teta2 =  atan2((z-h),sqrt(x^2+y^2)) - atan2((L2*sin(teta3)),(L1+L2*cos(teta3)));
end

