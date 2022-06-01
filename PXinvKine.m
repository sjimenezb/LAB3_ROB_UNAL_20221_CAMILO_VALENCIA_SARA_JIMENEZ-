%% Creación modelo Phantom en Matlab

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
    off = [off1, off2, off3, off4];

% Limites de los motores 
% id    rango                       nuehome          Destino  rango
% 1     0       -   4095            2048 - 180       3073     0-360
% 2     1100    -   3300            2048 - 180       2477     
% 3     1000    -   3333            3073 - 270       1925
% 4     600     -   3333            2048 - 180       835
% 5     1550    -   3110 -  3600    3110             2180
%       cierra  - abre - casi cierra           
% Definicion del robot DH std
    L(1) = Link('revolute','alpha',pi/2, 'a',0,   'd',l(1),'offset',off(1), 'qlim',[-pi pi]); %Rango útil -180° a +180°
    L(2) = Link('revolute','alpha',0,    'a',l(2),'d',0,   'offset',off(2), 'qlim',[-1.454213787 1.920543946]); %Rango útil -83.320° a 110.039°
    L(3) = Link('revolute','alpha',0,    'a',l(3),'d',0,   'offset',off(3), 'qlim',[-3.178408192 0.4003689856]); %Rango útil -182.109° a 22.939°
    L(4) = Link('revolute','alpha',0,    'a',l(4),'d',0,   'offset',off(4), 'qlim',[-2.221204181 1.971165312]); %Rango útil -127.266° a 112.939°
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
    hold off

% Revisar marcos coordenados
%     q = zeros(1,4);
%     M = eye(4);
%     for i=1:PhantomX.n
%         M = M * L(i).A(q(i));
%         trplot(M,'rgb','arrow','frame',num2str(i),'length',75)
%     end
%     hold off
%% Cálculos cinemática inversa en pose home erguida:

    %qt = deg2rad([0 0 -90 -90]);
    %Tt = PhantomX.fkine(qt);

    X = -32;
    Y = 0;
    Z = 347;
    pitchY = 0;
    Tt = transl(X,Y,Z)*troty(pitchY)*trotz(atan2(Y,-X));

% Desacople
    T = Tt;
    Ph = [0;0;-100;1]; %Muñeca respecto al marco de herramienta NOA
    Pb = T*Ph;         %Muñeca respecto base

% Theta 3
    cos3 = (Pb(1)^2 + Pb(2)^2 +(Pb(3)-l(1))^2-l(2)^2 - l(3)^2)/(2*l(2)*l(3));
    sin3D = sqrt(1-cos3^2); %Codo Abajo es la raíz positiva
    sin3U = -sqrt(1-cos3^2); %Codo Arriba es la raíz negativa

% Theta 2
    k1 = l(2)+l(3)*cos3;
    k2 = l(3)*sin3D;

% Ángulos en ambas configuraciones
    theta1 = atan2(Pb(2),Pb(1))-off(1);
    theta3D = atan2(sin3D,cos3)-off(3);
    theta3U = atan2(sin3U,cos3)-off(3);
    theta2D = (atan2(Pb(3)-l(1),sqrt(Pb(1)^2+Pb(2)^2)) - atan2(k2,k1))-off(2); %Codo abajo es resta
    theta2U = (atan2(Pb(3)-l(1),sqrt(Pb(1)^2+Pb(2)^2)) + atan2(k2,k1))-off(2); %Codo arriba es suma

% Theta 4
    Rp = (rotz(theta1))'*T(1:3,1:3);
    pitch = atan2(Rp(3,1),Rp(1,1));
    
    theta4D = pitch - theta2D - theta3D;
    theta4U = pitch - theta2U - theta3U;

    %PhantomX.teach([theta1 theta2D theta3D theta4D]); %Codo Abajo
    PhantomX.teach([theta1 theta2U theta3U theta4U]); %Codo Arriba
    hold on
        trplot(eye(4),'rgb','arrow','length',100,'frame','World');
        view(-30,25);
        axis([-300 300 -300 300 -50 400])
    hold off

%% Planeación de trayectorias con rutas definidas empleando el sccript adjunto invKinPxC.m

% Checkpoints:
% Home:      X80  Y100    Z100    troty(170*pi/180)
% Izquierda: X0    Y-132 Z5:70   troty(pi)
% Centro:    X-132 Y0    Z5:70   troty(pi)
% Derecha:   X0    Y132  Z5:70   troty(pi)

% Definición de las rutas
    % Nuevo Home
    X = 80;
    Y = 100;
    Z = 100;
    pitchY = 170*pi/180;
    Thome = transl(X,Y,Z)*troty(pitchY)*trotz(atan2(Y,-X));

    % Centro Arriba
    X = -132;
    Y = 0;
    Z = 70;
    pitchY = pi;
    Tcu = transl(X,Y,Z)*troty(pitchY)*trotz(atan2(Y,-X));

    % Centro Abajo
    X = -132;
    Y = 0;
    Z = 5;
    pitchY = pi;
    Tcd = transl(X,Y,Z)*troty(pitchY)*trotz(atan2(Y,-X));

    % Izquierda Arriba
    X = 0;
    Y = -132;
    Z = 70;
    pitchY = pi;
    Tiu = transl(X,Y,Z)*troty(pitchY)*trotz(atan2(Y,-X));

    % Izquierda Abajo
    X = 0;
    Y = -132;
    Z = 5;
    pitchY = pi;
    Tid = transl(X,Y,Z)*troty(pitchY)*trotz(atan2(Y,-X));

    % Derecha Arriba
    X = 0;
    Y = 132;
    Z = 70;
    pitchY = pi;
    Tdu = transl(X,Y,Z)*troty(pitchY)*trotz(atan2(Y,-X));

    % Derecha Abajo
    X = 0;
    Y = 132;
    Z = 5;
    pitchY = pi;
    Tdd = transl(X,Y,Z)*troty(pitchY)*trotz(atan2(Y,-X));

% Definición de las trayectorias
    steps = 20;
% Desplazamiento de home al centro superior y al costado izquierdo.
    Thcu  = ctraj(Thome,Tcu,steps);
    Tcuiu  = ctraj(Tcu,Tiu,steps);
    Tiuid = ctraj(Tiu,Tid,steps);
    % Cierre gripper agarra pieza 1 a la izquierda
% Retorno del costado izquierdo inferior al centro
    Tidiu = ctraj(Tid,Tiu,steps);
    Tiucu = ctraj(Tiu,Tcu,steps);
    Tcucd = ctraj(Tcu,Tcd,steps); %%%%%%%%%%
    % Abre gripper suelta pieza 1 en el centro
% Desplazamiento del centro inferior al costado derecho
    Tcdcu = ctraj(Tcd,Tcu,steps); %%%%%%%%%%
    Tcudu = ctraj(Tcu,Tdu,steps);
    Tdudd = ctraj(Tdu,Tdd,steps);
    % Cierre gripper agarra pieza 2 a la derecha
% Retorno del costado derecho inferior al centro
    Tdddu = ctraj(Tdd,Tdu,steps);
    Tducu = ctraj(Tdu,Tcu,steps);
    Tcucd = ctraj(Tcu,Tcd,steps); %%%%%%%%%%
    % Abre gripper suelta pieza 2 en el centro
% Retorno a Home   
    Tcdcu = ctraj(Tcd,Tcu,steps); %%%%%%%%%%
	Tcuh  = ctraj(Tcu,Thome,steps);
    pause(1)
%% Ciclo para calcular y simular el robot
    
    solucion = 2; % 1-> Codo abajo; 2-> Codo arriba
    for i=1:steps
        qinv = invKinPxC(Thcu(:,:,i),l,off);
        PhantomX.plot(qinv(solucion,:),'notiles','noname')
        hold on
        view(-30,25);
        plot3(Thcu(1,4,i),Thcu(2,4,i),Thcu(3,4,i),'ro')
        q_inv(i,:) = qinv(solucion,:);
    end

    for i=1:steps
        qinv = invKinPxC(Tcuiu(:,:,i),l,off);
        PhantomX.plot(qinv(solucion,:),'notiles','noname')  
        plot3(Tcuiu(1,4,i),Tcuiu(2,4,i),Tcuiu(3,4,i),'ro')
        q_inv(i,:) = qinv(solucion,:);
    end

    for i=1:steps
        qinv = invKinPxC(Tiuid(:,:,i),l,off);
        PhantomX.plot(qinv(solucion,:),'notiles','noname')  
        plot3(Tiuid(1,4,i),Tiuid(2,4,i),Tiuid(3,4,i),'ro')
        q_inv(i,:) = qinv(solucion,:);         
    end

    for i=1:steps
        qinv = invKinPxC(Tidiu(:,:,i),l,off);
        PhantomX.plot(qinv(solucion,:),'notiles','noname')  
        plot3(Tidiu(1,4,i),Tidiu(2,4,i),Tidiu(3,4,i),'ro')
        q_inv(i,:) = qinv(solucion,:);         
    end

    for i=1:steps
        qinv = invKinPxC(Tiucu(:,:,i),l,off);
        PhantomX.plot(qinv(solucion,:),'notiles','noname')  
        plot3(Tiucu(1,4,i),Tiucu(2,4,i),Tiucu(3,4,i),'ro')
        q_inv(i,:) = qinv(solucion,:);         
    end  

    for i=1:steps
        qinv = invKinPxC(Tcucd(:,:,i),l,off);
        PhantomX.plot(qinv(solucion,:),'notiles','noname')  
        plot3(Tcucd(1,4,i),Tcucd(2,4,i),Tcucd(3,4,i),'ro')
        q_inv(i,:) = qinv(solucion,:);        
    end

    for i=1:steps
        qinv = invKinPxC(Tcdcu(:,:,i),l,off);
        PhantomX.plot(qinv(solucion,:),'notiles','noname')  
        plot3(Tcdcu(1,4,i),Tcdcu(2,4,i),Tcdcu(3,4,i),'ro')
        q_inv(i,:) = qinv(solucion,:);      
    end

    for i=1:steps
        qinv = invKinPxC(Tcudu(:,:,i),l,off);
        PhantomX.plot(qinv(solucion,:),'notiles','noname')  
        plot3(Tcudu(1,4,i),Tcudu(2,4,i),Tcudu(3,4,i),'ro')
        q_inv(i,:) = qinv(solucion,:);
         
    end

    for i=1:steps
        qinv = invKinPxC(Tdudd(:,:,i),l,off);
        PhantomX.plot(qinv(solucion,:),'notiles','noname')  
        plot3(Tdudd(1,4,i),Tdudd(2,4,i),Tdudd(3,4,i),'ro')
        q_inv(i,:) = qinv(solucion,:);         
    end

    for i=1:steps
        qinv = invKinPxC(Tdddu(:,:,i),l,off);
        PhantomX.plot(qinv(solucion,:),'notiles','noname')  
        plot3(Tdddu(1,4,i),Tdddu(2,4,i),Tdddu(3,4,i),'ro')
        q_inv(i,:) = qinv(solucion,:);        
    end

    for i=1:steps
        qinv = invKinPxC(Tducu(:,:,i),l,off);
        PhantomX.plot(qinv(solucion,:),'notiles','noname')  
        plot3(Tducu(1,4,i),Tducu(2,4,i),Tducu(3,4,i),'ro')
        q_inv(i,:) = qinv(solucion,:);        
    end

    for i=1:steps
        qinv = invKinPxC(Tcucd(:,:,i),l,off);
        PhantomX.plot(qinv(solucion,:),'notiles','noname')  
        plot3(Tcucd(1,4,i),Tcucd(2,4,i),Tcucd(3,4,i),'ro')
        q_inv(i,:) = qinv(solucion,:);         
    end

    for i=1:steps
        qinv = invKinPxC(Tcdcu(:,:,i),l,off);
        PhantomX.plot(qinv(solucion,:),'notiles','noname')  
        plot3(Tcdcu(1,4,i),Tcdcu(2,4,i),Tcdcu(3,4,i),'ro')
        q_inv(i,:) = qinv(solucion,:);
    end

    for i=1:steps
        qinv = invKinPxC(Tcuh(:,:,i),l,off);
        PhantomX.plot(qinv(solucion,:),'notiles','noname')  
        plot3(Tcuh(1,4,i),Tcuh(2,4,i),Tcuh(3,4,i),'ro')
        q_inv(i,:) = qinv(solucion,:);         
    end

    hold off
%% Comunicación Dynamixel ROS
rosinit;

% Definimos el cliente del servicio que vamos a emplear junto a su mensaje
client = rossvcclient('/dynamixel_workbench/dynamixel_command');
msg = rosmessage(client);
%%
% Realizamos las trayectorias
%Aseguramos pinza abierta
        qinv = invKinPxC(Thome,l,off);
        phantomCom(qinv(solucion,:),client,msg,"abre");

    for i=1:steps
        qinv = invKinPxC(Thcu(:,:,i),l,off);
        phantomCom(qinv(solucion,:),client,msg,"abre");
    end

    for i=1:steps
        qinv = invKinPxC(Tcuiu(:,:,i),l,off);
        phantomCom(qinv(solucion,:),client,msg,"abre");
    end

    for i=1:steps
        qinv = invKinPxC(Tiuid(:,:,i),l,off);
        phantomCom(qinv(solucion,:),client,msg,"abre");
    end

% Cierre gripper agarra pieza 1 a la izquierda
        qinv = invKinPxC(Tid,l,off);
        phantomCom(qinv(solucion,:),client,msg,"cierra");

    for i=1:steps
        qinv = invKinPxC(Tidiu(:,:,i),l,off);
        phantomCom(qinv(solucion,:),client,msg,"cierra");
    end

    for i=1:steps
        qinv = invKinPxC(Tiucu(:,:,i),l,off);
        phantomCom(qinv(solucion,:),client,msg,"cierra");
    end

    for i=1:steps
        qinv = invKinPxC(Tcucd(:,:,i),l,off);
        phantomCom(qinv(solucion,:),client,msg,"cierra");
    end

 % Abre gripper suelta pieza 1 en el centro
        qinv = invKinPxC(Tcd,l,off);
        phantomCom(qinv(solucion,:),client,msg,"abre");

    for i=1:steps
        qinv = invKinPxC(Tcdcu(:,:,i),l,off);
        phantomCom(qinv(solucion,:),client,msg,"abre");
    end

    for i=1:steps
        qinv = invKinPxC(Tcudu(:,:,i),l,off);
        phantomCom(qinv(solucion,:),client,msg,"abre");
    end

    for i=1:steps
        qinv = invKinPxC(Tdudd(:,:,i),l,off);
        phantomCom(qinv(solucion,:),client,msg,"abre");
    end

% Cierre gripper agarra pieza 2 a la derecha
        qinv = invKinPxC(Tdd,l,off);
        phantomCom(qinv(solucion,:),client,msg,"cierra");

    for i=1:steps
        qinv = invKinPxC(Tdddu(:,:,i),l,off);
        phantomCom(qinv(solucion,:),client,msg,"cierra");
    end

    for i=1:steps
        qinv = invKinPxC(Tducu(:,:,i),l,off);
        phantomCom(qinv(solucion,:),client,msg,"cierra");
    end

    for i=1:steps
        qinv = invKinPxC(Tcucd(:,:,i),l,off);
        phantomCom(qinv(solucion,:),client,msg,"cierra");
    end

% Abre gripper suelta pieza 2 en el centro
        qinv = invKinPxC(Tcd,l,off);
        phantomCom(qinv(solucion,:),client,msg,"abre");

    for i=1:steps
        qinv = invKinPxC(Tcdcu(:,:,i),l,off);
        phantomCom(qinv(solucion,:),client,msg,"abre");
    end

    for i=1:steps
        qinv = invKinPxC(Tcuh(:,:,i),l,off);
        phantomCom(qinv(solucion,:),client,msg,"abre");
    end