
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
    Robot = SerialLink(L,'name','Px');
 A=unifrnd(0,0,[1,30000]);% first joint variable limit
 B=unifrnd(-1.454213787,1.920543946,[1,30000]);% second joint variable limit
 C=unifrnd(-3.178408192,0.4003689856,[1,30000]);% third joint variable limit
 D=unifrnd(-2.221204181,1.971165312,[1,30000]);% fourth joint variable limit
 G= cell(30000, 3);% builds a cell array
for n = 1:30000
    G{n} =[A(n) B(n) C(n) D(n)];
end % produces 3000 sets of random points
 H1=cell2mat(G); % converts the array of cells into a matrix
 T=double(Robot.fkine(H1)); % mechanical arm positive solution
 figure(1)
 scatter3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)))% random dot plot
 Robot.plot([0 pi/4 0 0],'tilesize', 50)% mechanical arm diagram
 hold on
 trplot(eye(4),'rgb','arrow','length',200,'frame','0')