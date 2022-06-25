function [ e, handle ] = checktime( tf1,tf2 )
if ( tf1 < 0 ) 
     msg=sprintf('Time interval of the first trajectory meaningless!');
     handle = msgbox(msg, 'tf1<0', 'warn');
     e=1;
elseif ( tf2 < 0 ) 
     msg=sprintf('Time interval of the second trajectory meaningless!');
     handle = msgbox(msg, 'tf2<0', 'warn');
     e=1;
elseif ( tf1>0 && tf2>0 ) 
     e=0;
     handle=[];
end
end

