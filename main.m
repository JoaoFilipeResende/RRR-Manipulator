clear;
clc;
close all;

global h L1 L2 m2 m23 m3 m34 x1 y1 z1 x2 y2 z2 x3 y3 z3 tf1 tf2;
global hFig1;
global hFig2;
global hFig3;
global hFig4;

hFig1 = figure('name','Trajectory Planning','Position',[1 1 0.6 0.6].*get(0,'ScreenSize'));
hFig2 = figure('name','Velocities and Accelerations in Joint Space','visible','off');
hFig3 = figure('name','Positions, Velocities and Accelerations in the Operational Space','visible','off');
hFig4 = figure('name','Inverse Dynamics','visible','off');

[h,L1,L2,m2,m23,m3,m34,x1,y1,z1,x2,y2,z2,x3,y3,z3,tf1,tf2] = deal(1,1,1,5,5,5,5,0.5,-0.5,0,0.5,1,0.5,1,-1.5,1.5,1,1);

[e1, handle1] = checkweight(m2,m23,m3,m34);
[e2, handle2] = checktime(tf1,tf2);
[e3, handle3] = checkworkspace(L1,L2,h,x1,y1,z1);
[e4, handle4] = checkworkspace(L1,L2,h,x2,y2,z2);
[e5, handle5] = checkworkspace(L1,L2,h,x3,y3,z3);

if(e1==0 && e2==0 && e3==0 && e4==0 && e5==0 )
    run(h,L1,L2,m2,m23,m3,m34,x1,y1,z1,x2,y2,z2,x3,y3,z3,tf1,tf2);
end

function run(h,L1,L2,m2,m23,m3,m34,x1,y1,z1,x2,y2,z2,x3,y3,z3,tf1,tf2)
global hFig1;
global hFig2;
global hFig3;
global hFig4;

% Time vector
t1=(0:0.01:tf1);
t2=(0:0.01:tf2);

%%% Inverse Kinematics %%%        

[teta1_1,teta2_1,teta3_1]=IK(x1,y1,z1,L1,L2,h);
[teta1_2,teta2_2,teta3_2]=IK(x2,y2,z2,L1,L2,h);
[teta1_3,teta2_3,teta3_3]=IK(x3,y3,z3,L1,L2,h);

%%% Cubic Interpolation - Trajectory 1 %%%

[a0_t1_j1,a1_t1_j1,a2_t1_j1,a3_t1_j1]=coefficients( teta1_1,teta1_2,tf1 );
[a0_t1_j2,a1_t1_j2,a2_t1_j2,a3_t1_j2]=coefficients( teta2_1,teta2_2,tf1 );
[a0_t1_j3,a1_t1_j3,a2_t1_j3,a3_t1_j3]=coefficients( teta3_1,teta3_2,tf1 );

%%% Cubic Interpolation - Trajectory 2 %%%

[a0_t2_j1,a1_t2_j1,a2_t2_j1,a3_t2_j1]=coefficients( teta1_2,teta1_3,tf2 ); 
[a0_t2_j2,a1_t2_j2,a2_t2_j2,a3_t2_j2]=coefficients( teta2_2,teta2_3,tf2 );
[a0_t2_j3,a1_t2_j3,a2_t2_j3,a3_t2_j3]=coefficients( teta3_2,teta3_3,tf2 );

%%% Cubic Interpolation - Polynomials %%%

% Eq Joint 1,2 and 3 - trajectory 1
[ Posi_t1_j1,vel_t1_j1,ace_t1_j1 ] = equations( a0_t1_j1,a1_t1_j1,a2_t1_j1,a3_t1_j1,t1 );
[ Posi_t1_j2,vel_t1_j2,ace_t1_j2 ] = equations( a0_t1_j2,a1_t1_j2,a2_t1_j2,a3_t1_j2,t1 );
[ Posi_t1_j3,vel_t1_j3,ace_t1_j3 ] = equations( a0_t1_j3,a1_t1_j3,a2_t1_j3,a3_t1_j3,t1 );

% Eq Joint 1,2 and 3 - trajectory 2
[ Posi_t2_j1,vel_t2_j1,ace_t2_j1 ] = equations( a0_t2_j1,a1_t2_j1,a2_t2_j1,a3_t2_j1,t2 );
[ Posi_t2_j2,vel_t2_j2,ace_t2_j2 ] = equations( a0_t2_j2,a1_t2_j2,a2_t2_j2,a3_t2_j2,t2 );
[ Posi_t2_j3,vel_t2_j3,ace_t2_j3 ] = equations( a0_t2_j3,a1_t2_j3,a2_t2_j3,a3_t2_j3,t2 );

t1=(0:0.01:tf1);
t2=(tf1:0.01:(tf1+tf2));

% Speed
tv1=(0:0.01:tf1);
tv2=(tf1:0.01:((tf1+tf2)-0.01));

% Accelerations
ta1=(0:0.01:tf1);
ta2=(tf1:0.01:((tf1+tf2)-0.02));

% Workspace path planning
[ x_pos,y_pos,z_pos ] = DK( cat(2,Posi_t1_j1,Posi_t2_j1),cat(2,Posi_t1_j2,Posi_t2_j2),cat(2,Posi_t1_j3,Posi_t2_j3),L1,L2,h );

% Derivation of position to get velocity vector
x_vel=diff(x_pos)/0.01;
y_vel=diff(y_pos)/0.01;
z_vel=diff(z_pos)/0.01;

% Derivation of position to get acceleration vector
x_ace=diff(x_vel)/0.01;
y_ace=diff(y_vel)/0.01;
z_ace=diff(z_vel)/0.01;

%%% RRR robot Drawubg %%%

% First Arm
X1=[0,0];
Y1=[0,0];
Z1=[0,h];

movegui(hFig1,'center');  

% Add pushbutton to change settings
uicontrol('Parent',hFig1,'Style','pushbutton','String','Change Inputs','Units','normalized','Position',[0.01 0.01 0.12 0.05],'Visible','on','Callback', @myFunction);

set(0,'CurrentFigure',hFig1)
subplot(3,2,[1 3 5])

teta1=cat(2,Posi_t1_j1,Posi_t2_j1);
teta2=cat(2,Posi_t1_j2,Posi_t2_j2);
teta3=cat(2,Posi_t1_j3,Posi_t2_j3);
     
for i= 1:size(cat(2,tv1,tv2),2)
% Second Arm
X2=[0,(L1*cos(teta2(i)))*cos(teta1(i))];
Y2=[0,(L1*cos(teta2(i)))*sin(teta1(i))];
Z2=[h,(h+(L1*sin(teta2(i))))];

% Third Arm
X3=[(L1*cos(teta2(i)))*cos(teta1(i)),x_pos(i)];
Y3=[(L1*cos(teta2(i)))*sin(teta1(i)),y_pos(i)];
Z3=[(h+(L1*sin(teta2(i)))),z_pos(i)];

plot3(0,0,0,'k.','MarkerSize',20); %JOINT
a=plot3(X1,Y1,Z1,'r','LineWidth',4);%ELO
hold on;
b=plot3(X2,Y2,Z2,'b','LineWidth',4);%ELO
c=plot3(X3,Y3,Z3,'g','LineWidth',4);%ELO
d=plot3((L1.*cos(teta2(i))).*cos(teta1(i)),(L1.*cos(teta2(i))).*sin(teta1(i)),(h+(L1.*sin(teta2(i)))),'k.','MarkerSize',20); %JOINT
e=plot3(0,0,h,'k.','MarkerSize',20); %JOINT
plot3(x_pos(i),y_pos(i),z_pos(i),'r.','Color','g','MarkerSize',4)
axis([(-L1-L2) (L1+L2),(-L1-L2) (L1+L2),0 (h+(L1+L2))]);

xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
grid on;

%Plot of input points
if i==1
    plot3(x_pos(i),y_pos(i),z_pos(i),'k.','MarkerSize',15);
    msg=sprintf(' P_{1}(%0.2f,%0.2f,%0.2f)',x_pos(i),y_pos(i),z_pos(i));
    text(x_pos(i),y_pos(i),z_pos(i),[msg]);
end 

if i==size(tv1,2) 
     plot3(x_pos(i),y_pos(i),z_pos(i),'k.','MarkerSize',15);
     msg=sprintf(' P_{2}(%0.2f,%0.2f,%0.2f)',x_pos(i),y_pos(i),z_pos(i));
     text(x_pos(i),y_pos(i),z_pos(i),[msg]);
end

if i==size(cat(2,tv1,tv2),2) 
     plot3(x_pos(i),y_pos(i),z_pos(i),'k.','MarkerSize',15);
     msg=sprintf(' P_{3}(%0.2f,%0.2f,%0.2f)',x_pos(i),y_pos(i),z_pos(i));
     text(x_pos(i),y_pos(i),z_pos(i),[msg]);
end

M(i)=getframe;

if i<size(cat(2,tv1,tv2),2)
delete(a);delete(b);delete(c);delete(d);delete(e);

end
end

% Creation of plots with different perspectives
subplot(3,2,2)
i=size(cat(2,tv1,tv2),2);
plot3(0,0,0,'k.','MarkerSize',20); %JOINT
a=plot3(X1,Y1,Z1,'r','LineWidth',4);%ELO
hold on;
b=plot3(X2,Y2,Z2,'b','LineWidth',4);%ELO
c=plot3(X3,Y3,Z3,'g','LineWidth',4);%ELO
d=plot3((L1.*cos(teta2(i))).*cos(teta1(i)),(L1.*cos(teta2(i))).*sin(teta1(i)),(h+(L1.*sin(teta2(i)))),'k.','MarkerSize',20); %JOINT
e=plot3(0,0,h,'k.','MarkerSize',20); %JOINT
plot3(x_pos(i),y_pos(i),z_pos(i),'r.','Color','g','MarkerSize',4)
axis([(-L1-L2) (L1+L2),(-L1-L2) (L1+L2),0 (h+(L1+L2))]);
view(0,90);
xlabel('x(m)');
ylabel('y(m)');
grid on;

subplot(3,2,4)
i=size(cat(2,tv1,tv2),2);
plot3(0,0,0,'k.','MarkerSize',20); %JOINT
a=plot3(X1,Y1,Z1,'r','LineWidth',4);%ELO
hold on;
b=plot3(X2,Y2,Z2,'b','LineWidth',4);%ELO
c=plot3(X3,Y3,Z3,'g','LineWidth',4);%ELO
d=plot3((L1.*cos(teta2(i))).*cos(teta1(i)),(L1.*cos(teta2(i))).*sin(teta1(i)),(h+(L1.*sin(teta2(i)))),'k.','MarkerSize',20); %JOINT
e=plot3(0,0,h,'k.','MarkerSize',20); %JOINT
plot3(x_pos(i),y_pos(i),z_pos(i),'r.','Color','g','MarkerSize',4)
axis([(-L1-L2) (L1+L2),(-L1-L2) (L1+L2),0 (h+(L1+L2))]);
view(0,0);
xlabel('x(m)');
zlabel('z(m)');
grid on;

subplot(3,2,6)
i=size(cat(2,tv1,tv2),2);
plot3(0,0,0,'k.','MarkerSize',20); %JOINT
a=plot3(X1,Y1,Z1,'r','LineWidth',4);%ELO
hold on;
b=plot3(X2,Y2,Z2,'b','LineWidth',4);%ELO
c=plot3(X3,Y3,Z3,'g','LineWidth',4);%ELO
d=plot3((L1.*cos(teta2(i))).*cos(teta1(i)),(L1.*cos(teta2(i))).*sin(teta1(i)),(h+(L1.*sin(teta2(i)))),'k.','MarkerSize',20); %JOINT
e=plot3(0,0,h,'k.','MarkerSize',20); %JOINT
plot3(x_pos(i),y_pos(i),z_pos(i),'r.','Color','g','MarkerSize',4)
axis([(-L1-L2) (L1+L2),(-L1-L2) (L1+L2),0 (h+(L1+L2))]);
view(90,0);
ylabel('y(m)');
zlabel('z(m)');
grid on;

% Graphics of Velocities and Accelerations in Joint Space 
hFig2.Visible = 1;
set(0,'CurrentFigure',hFig2);
subplot(3,3,1);
plot(cat(2,t1,t2),cat(2,Posi_t1_j1,Posi_t2_j1),'r');
title('$\theta$(t)','interpreter','latex','FontSize',20);
ylabel('${\theta_1(rad)}$','interpreter','latex','FontSize',15);
xlabel('t(s)','interpreter','latex','FontSize',15);
grid on;

subplot(3,3,2);
plot(cat(2,t1,t2),cat(2,vel_t1_j1,vel_t2_j1),'b');
title('$\dot{\theta}$(t)','interpreter','latex','FontSize',20);
ylabel('${\dot{\theta_1}(rad/s)}$','interpreter','latex','FontSize',15);
xlabel('t(s)','interpreter','latex','FontSize',15);
grid on;

subplot(3,3,3);
plot(cat(2,t1,t2),cat(2,ace_t1_j1,ace_t2_j1),'g');
title('$\ddot{\theta}$(t)','interpreter','latex','FontSize',20);
ylabel('${\ddot{\theta_1}(rad/s^2)}$','interpreter','latex','FontSize',15);
xlabel('t(s)','interpreter','latex','FontSize',15);
grid on;

subplot(3,3,4);
plot(cat(2,t1,t2),cat(2,Posi_t1_j2,Posi_t2_j2),'r');
ylabel('$\theta_2(rad)$','interpreter','latex','FontSize',15);
xlabel('t(s)','interpreter','latex','FontSize',15);
grid on;

subplot(3,3,5);
plot(cat(2,t1,t2),cat(2,vel_t1_j2,vel_t2_j2),'b');
ylabel('${\dot{\theta_2}(rad/s)}$','interpreter','latex','FontSize',15);
xlabel('t(s)','interpreter','latex','FontSize',15);
grid on;

subplot(3,3,6);
plot(cat(2,t1,t2),cat(2,ace_t1_j2,ace_t2_j2),'g');
ylabel('${\ddot{\theta_2}(rad/s^2)}$','interpreter','latex','FontSize',15);
xlabel('t(s)','interpreter','latex','FontSize',15);
grid on;

subplot(3,3,7);
plot(cat(2,t1,t2),cat(2,Posi_t1_j3,Posi_t2_j3),'r');
ylabel('${\theta_3(rad)}$','interpreter','latex','FontSize',15);
xlabel('t(s)','interpreter','latex','FontSize',15);
grid on;

subplot(3,3,8);
plot(cat(2,t1,t2),cat(2,vel_t1_j3,vel_t2_j3),'b');
ylabel('${\dot{\theta_3}(rad/s)}$','interpreter','latex','FontSize',15);
xlabel('t(s)','interpreter','latex','FontSize',15);
grid on;

subplot(3,3,9);
plot(cat(2,t1,t2),cat(2,ace_t1_j3,ace_t2_j3),'g');
ylabel('${\ddot{\theta_3}(rad/s^2)}$','interpreter','latex','FontSize',15);
xlabel('t(s)','interpreter','latex','FontSize',15);
grid on;

%Graphics of Speeds and Accelerations in the Operating Space
hFig3.Visible = 1;
set(0,'CurrentFigure',hFig3);
subplot(3,3,1);
plot(cat(2,t1,t2),x_pos,'r');
title('$\theta$(t)','interpreter','latex','FontSize',20);
ylabel('x(m)','interpreter','latex','FontSize',15);
xlabel('t(s)','interpreter','latex','FontSize',15);
grid on;

subplot(3,3,2);
plot(cat(2,tv1,tv2),x_vel,'b');
title('$\dot{\theta}$(t)','interpreter','latex','FontSize',20);
ylabel('$\dot{x}(m/s)$','interpreter','latex','FontSize',15);
xlabel('t(s)','interpreter','latex','FontSize',15);
grid on;

subplot(3,3,3);
plot(cat(2,ta1,ta2),x_ace,'g');
title('$\ddot{\theta}$(t)','interpreter','latex','FontSize',20);
ylabel('$\ddot{x}(m/s^2)$','interpreter','latex','FontSize',15);
xlabel('t(s)','interpreter','latex','FontSize',15);
grid on;

subplot(3,3,4);
plot(cat(2,t1,t2),y_pos,'r');
ylabel('y(m)','interpreter','latex','FontSize',15);
xlabel('t(s)','interpreter','latex','FontSize',15);
grid on;

subplot(3,3,5);
plot(cat(2,tv1,tv2),y_vel,'b');
ylabel('$\dot{y}(m/s)$','interpreter','latex','FontSize',15);
xlabel('t(s)','interpreter','latex','FontSize',15);
grid on;

subplot(3,3,6);
plot(cat(2,ta1,ta2),y_ace,'g');
ylabel('$\ddot{y}(m/s^2)$','interpreter','latex','FontSize',15);
xlabel('t(s)','interpreter','latex','FontSize',15);
grid on;

subplot(3,3,7);
plot(cat(2,t1,t2),z_pos,'r');
ylabel('z(m)','interpreter','latex','FontSize',15);
xlabel('t(s)','interpreter','latex','FontSize',15);
grid on;

subplot(3,3,8);
plot(cat(2,tv1,tv2),z_vel,'b');
ylabel('$\dot{z}(m/s)$','interpreter','latex','FontSize',15);
xlabel('t(s)','interpreter','latex','FontSize',15);
grid on;

subplot(3,3,9);
plot(cat(2,ta1,ta2),z_ace,'g');
ylabel('$\ddot{z}(m/s^2)$','interpreter','latex','FontSize',15);
xlabel('t(s)','interpreter','latex','FontSize',15);
grid on;

teta1_vel=cat(2,vel_t1_j1,vel_t2_j1);
teta2_vel=cat(2,vel_t1_j2,vel_t2_j2);
teta3_vel=cat(2,vel_t1_j3,vel_t2_j3);

teta1_ace=cat(2,ace_t1_j1,ace_t2_j1);
teta2_ace=cat(2,ace_t1_j2,ace_t2_j2);
teta3_ace=cat(2,ace_t1_j3,ace_t2_j3);

% Masses and Inertia
Ma=0.5*m2+m23+m3+m34;
Mb=m3+2*m34;
Ia=(0.25*m2+m23+m3+m34)*(L1^2);
Ib=(m3+2*m34)*L1*L2;
Ic=(m3+4*m34)*(L2^2);
        
Torque1=(Ia.*(cos(teta2).^2)+Ib.*cos(teta2).*cos(teta2+teta3)+0.25.*Ic.*(cos(teta2+teta3)).^2).*teta1_ace ...
            - (Ia.*sin(2.*teta2)+Ib.*sin(2.*teta2+teta3)+0.25.*Ic.*sin(2.*teta2+2.*teta3)).*teta1_vel.*teta2_vel ...
            - (Ib.*cos(teta2).*sin(teta2+teta3)+0.25.*Ic.*sin(2.*teta2 + 2.*teta3)).*teta1_vel.*teta3_vel;

Torque2=(Ia+0.25.*Ic+Ib.*cos(teta3)).*teta2_ace+[0.25.*Ic+0.5.*Ib.*cos(teta3)].*teta3_ace ...
            +[0.5.*Ia.*sin(2*teta2)+0.125.*Ic.*sin(2.*teta2+2.*teta3)+0.5.*Ib.*sin(2.*teta2+teta3)].*teta1_vel.^2 ...
            -[0.5.*Ib.*sin(teta3)].*teta3_vel.^2-Ib.*sin(teta3).*teta2_vel.*teta3_vel ...
            +[Ma.*L1.*cos(teta2)+0.5.*Mb.*L2.*cos(teta2+teta3)].*9.8;

Torque3=(0.25.*Ic+0.5.*Ib.*cos(teta3)).*teta2_ace+(0.25.*Ic).*teta3_ace ...
            +((0.5.*Ib.*cos(teta2)+0.25.*Ic.*cos(teta2+teta3)).*sin(teta2+teta3)).*teta1_vel.^2 ...
            +(0.5.*Ib.*sin(teta3)).*teta2_vel.^2 ...
            +(0.5.*Mb.*L2.*cos(teta2+teta3)).*9.8;  
        
hFig4.Visible = 1;
set(0,'CurrentFigure',hFig4);
subplot(3,1,1);
plot(cat(2,t1,t2),Torque1,'r');
title('Torque 1','interpreter','latex','FontSize',20);
ylabel('T(N.m)','interpreter','latex');
xlabel('t(s)','interpreter','latex');
grid on;

subplot(3,1,2);
plot(cat(2,t1,t2),Torque2,'b');
title('Torque 2','interpreter','latex','FontSize',20);
ylabel('T(N.m)','interpreter','latex');
xlabel('t(s)','interpreter','latex');
grid on;

subplot(3,1,3);
plot(cat(2,t1,t2),Torque3,'g');
title('Torque 3','interpreter','latex','FontSize',20);
ylabel('T(N.m)','interpreter','latex');
xlabel('t(s)','interpreter','latex');
grid on;

end


function myFunction(src, event)
    global h L1 L2 m2 m23 m3 m34 x1 y1 z1 x2 y2 z2 x3 y3 z3 tf1 tf2;
    global hFig2;
    global hFig3;
    global hFig4;
    clf();

    if isvalid(hFig2)
        hFig2.Visible = 0;
    else
        hFig2 = figure('name','Velocidades e Acelerações no Espaço das juntas','visible','off');
    end
    
    if isvalid(hFig3)
        hFig3.Visible = 0;
    else
        hFig3 = figure('name','Posições, Velocidades e Acelerações no Espaço Operacional','visible','off');
    end
    
    if isvalid(hFig4)
        hFig4.Visible = 0;
    else
        hFig4 = figure('name','Dinâmica Inversa','visible','off');
    end
    
    while 1
        prompt = {'Length of h:',...
                  'Length of L1:',...
                  'Length of L2:'};
        dlgtitle = 'Input';
        dims = [1 40];
        definput = {'1','1','1'};

        length = str2double(inputdlg(prompt,dlgtitle,dims,definput));
    
        prompt = {'Mass of Elo1:',...
                  'Mass of joint actuator2:',...
                  'Mass of Elo2:',...
                  'Mass of joint actuator3:'};
        dlgtitle = 'Input';
        dims = [1 40];
        definput = {'5','5','5','5'};

        masses = str2double(inputdlg(prompt,dlgtitle,dims,definput));

        prompt = {'Initial coordinate value X1:',...
                  'Initial coordinate value Y1:',...
                  'Initial coordinate value Z1:',...
                  'Intermediate coordinate value X2:',...
                  'Intermediate coordinate value Y2:',...
                  'Intermediate coordinate value Z2:',...
                  'Final coordinate value X3:',...
                  'Final coordinate value Y3:',...
                  'Final coordinate value Z3:'};
        dlgtitle = 'Input';
        dims = [1 40];
        definput = {'0.5','-0.5','0','1','0.5','1','-1.5','1','1'};

        points = str2double(inputdlg(prompt,dlgtitle,dims,definput));

        [e1, handle1] = checkweight(masses(1),masses(2),masses(3),masses(4));
        [e2, handle2] = checktime(tf1,tf2);
        [e3, handle3] = checkworkspace(length(2),length(3),length(1),points(1),points(2),points(3));
        [e4, handle4] = checkworkspace(length(2),length(3),length(1),points(4),points(5),points(6));
        [e5, handle5] = checkworkspace(length(2),length(3),length(1),points(7),points(8),points(9));
        
        while any(ishandle(handle1)) || any(ishandle(handle2)) || any(ishandle(handle3)) || any(ishandle(handle4)) || any(ishandle(handle5))
            pause(0.1)
        end
               
        if~( e1==1 || e2==1 || e3==1 || e4==1 || e5==1 )
            break;
        end
    end
    
    run(h,L1,L2,m2,m23,m3,m34,x1,y1,z1,x2,y2,z2,x3,y3,z3,tf1,tf2);
end
