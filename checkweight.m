function [ e, handle ] = checkweight( m2,m23,m3,m34 )
if ( m2<0 )
    msg=sprintf('Value of mass m2=%dkg is less than zero',m2);
    handle = msgbox(msg,'Erro', 'error');
    e=1;
elseif ( m23<0 )
    msg=sprintf('Value of mass m23=%dkg is less than zero',m23);
    handle = msgbox(msg,'Erro', 'error');
    e=1;
elseif ( m3<0 )
    msg=sprintf('Value of mass m3=%dkg is less than zero',m3);
    handle = msgbox(msg,'Erro', 'error');
    e=1;
elseif ( m34<0 )
    msg=sprintf('Value of mass m34=%dkg is less than zero',m34);
    handle = msgbox(msg,'Erro', 'error');
    e=1;
elseif ( m2>0 && m23>0 && m3>0 && m34>0 )
    e=0;
    handle=[];
end
end

