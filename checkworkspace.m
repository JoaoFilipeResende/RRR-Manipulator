function [ e, handle ] = checkworkspace( L1,L2,h,x,y,z)
if ( z < 0 )
     msg=sprintf('The coordinate Z=%0.2f has no meaning',z);
     handle = msgbox(msg, 'Z<0', 'warn');
     e=1;
elseif ( (x^2+y^2+(z-h)^2) == (L1+L2)^2 )
    msg=sprintf('The point(%0.2f,%0.2f,%0.2f) is at the edge of the workspace',x,y,z);
    handle = msgbox(msg,'Singularidade', 'warn');
    e=0;
elseif ( (x==0) && (y==0) )
    msg=sprintf('The point(%0.2f,%0.2f,%0.2f) is on the Z axis. Infinite solutions',x,y,z);
    handle = msgbox(msg, 'Singularidade', 'warn');
    e=0;
elseif ( (x^2+y^2+(z-h)^2) > (L1+L2)^2 )
    msg=sprintf('The point(%0.2f,%0.2f,%0.2f) is outside of the workspace',x,y,z);
    handle = msgbox(msg,'Erro', 'error');
    e=1;
elseif ( (x^2+y^2+(z-h)^2) < (L1+L2)^2 )
    e=0;
    handle=[];
end

end

