%% Creación modelo Phantom en Matlab empleando parámetros DHstd, modelo mirando hacia la caja de control
% Cálulo de las longitudes de eslabón
    l1 = 47;
    l2 = sqrt(100^2+32^2);
    l3 = 100;
    l4 = 100;
    l = [l1, l2, l3, l4]; 
% Cálculo de offsets
    off1 = pi;%Cambiar a pi si se desea el robot mirando hacia afuera, 0 hacia el controlador
    off2 = atand(100/32)*pi/(180);
    off3 = pi/2-off2;
    off4 = 0; 

%%Limites de los motores 
%%id    rango                       nuehome          Destino  rango
%%1     0       -   4095            2048 - 180       3073     0-360
%%2     1100    -   3300            2048 - 180       2477     
%%3     1000    -   3333            3073 - 270       1925
%%4     600     -   3333            2048 - 180       835
%%5     1550    -   3110 -  3600    3110             2180
                    
% Definicion del robot DH std
    L(1) = Link('revolute','alpha',pi/2, 'a',0,   'd',l(1),'offset',off1, 'qlim',[-pi pi]); %Rango útil -180° a +180°
    L(2) = Link('revolute','alpha',0,    'a',l(2),'d',0,   'offset',off2, 'qlim',[-1.454213787 1.920543946]); %Rango útil -83.320° a 110.039°
    L(3) = Link('revolute','alpha',0,    'a',l(3),'d',0,   'offset',off3, 'qlim',[-3.178408192 0.4003689856]); %Rango útil -182.109° a 22.939°
    L(4) = Link('revolute','alpha',0,    'a',l(4),'d',0,   'offset',off4, 'qlim',[-2.221204181 1.971165312]); %Rango útil -127.266° a 112.939°
    PhantomX = SerialLink(L,'name','Px');
    tool = [ 0  0  1  0; 
             1  0  0  0; 
             0  1  0  0;
             0  0  0  1];
    PhantomX.tool = tool; % Ajuste para emplear notación NOA 
    PhantomX.plot([0 0 0 0], 'notiles')
    hold on
    trplot(eye(4),'rgb','arrow','length',100,'frame','World')
    view(-35,20)
%     hold off
    %Revisar marcos coordenados
    q = zeros(1,4);
    M = eye(4);
    for i=1:PhantomX.n
        M = M * L(i).A(q(i));
        trplot(M,'rgb','arrow','frame',num2str(i),'length',75)
    end
    hold off
%% Cálculos cinemática inversa:

%rot = 10*pi/180; %rotación alrededor del eje Y base

qt = deg2rad([30, 30, 30, 30]);
Tt = PhantomX.fkine(qt);
% Desacople
T = Tt;

%T = rt2tr(roty(rot),[0;0;250]); %Pose erguida totalmente (home)

Ph = [0;0;-100;1]; %Muñeca respecto al marco de herramienta NOA
Pb = T*Ph;         %Muñeca respecto base

cos3 = (Pb(1)^2 + Pb(2)^2 +(Pb(3)-l1)^2-l2^2 - l3^2)/(2*l2*l3);
sin3 = sqrt(1-cos3^2); %Codo Abajo es la raíz positiva

k1 = l2+l3*cos3;
k2 = l3*sin3;

theta1 = atan2(Pb(2),Pb(1))-off1;
theta3 = atan2(sin3,cos3)-off3;
theta2 = (atan2(Pb(3)-l1,sqrt(Pb(1)^2+Pb(2)^2)) - atan2(k2,k1))-off2; %Codo abajo es resta
%theta4 = (rot-theta2-theta3)-off4;

Rp = (rotz(theta1))'*T(1:3,1:3);
pitch = atan2(Rp(3,1),Rp(1,1));

theta4 = pitch - theta2 - theta3;
%q4u = pitch - q2u - q3u;



PhantomX.teach([theta1 theta2 theta3 theta4]);
  hold on
    trplot(eye(4),'rgb','arrow','length',100,'frame','World');
    view(-30,25);
    hold off