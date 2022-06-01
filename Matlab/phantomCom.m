%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Comunicación Matlab+dynamixel Phantom X nuevo
Por: Camilo Valencia
Prerequisitos: 
- Tener el nodo rosinit corriendo
- Tener el servicio y cliente corriendo
Entradas:
qrad: Ángulos a enviar en radianes
gripper: "abre" o "cierra"
motorSvcClient: rossvcclient('/dynamixel_workbench/dynamixel_command');
motorCommandMsg: rosmessage(motorSvcClient);
Salidas:
comm: Estado del llamado
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}
function comm = phantomCom(varargin)
    switch nargin
        case 4
         qrad = varargin{1};
         motorSvcClient = varargin{2};
         motorCommandMsg = varargin{3};
         gripper = varargin{4};
    otherwise
          disp('Wrong number of inputs')
    end

    q = rad2deg(qrad);
    if gripper == "abre"
        q(5) = 0;
    elseif gripper == "cierra"
        q(5) = -130; % Más negativo cierra más, positivo abre, rango de -130 a 0.
    end
    angulos=q+[180 180 270 180 273];
    posicion=(4096/360)*angulos; 

    motorCommandMsg.AddrName = "Goal_Position";
    motorCommandMsg.Id = 1;
    motorCommandMsg.Value = posicion(1);%%rango 0-4096
    call(motorSvcClient, motorCommandMsg);
    pause(0.1);
    
    motorCommandMsg.AddrName = "Goal_Position";
    motorCommandMsg.Id = 2;
    motorCommandMsg.Value = posicion(2);%%rango 0-4096
    call(motorSvcClient, motorCommandMsg);
    pause(0.1);
    
    motorCommandMsg.AddrName = "Goal_Position";
    motorCommandMsg.Id = 3;
    motorCommandMsg.Value = posicion(3);%%rango 0-4096
    call(motorSvcClient, motorCommandMsg);
    pause(0.1);
    
    motorCommandMsg.AddrName = "Goal_Position";
    motorCommandMsg.Id = 4;
    motorCommandMsg.Value = posicion(4);%%rango 0-4096
    call(motorSvcClient, motorCommandMsg);
    pause(0.1);
    
    motorCommandMsg.AddrName = "Goal_Position";
    motorCommandMsg.Id = 5;
    motorCommandMsg.Value = posicion(5);%%rango 0-4096
    call(motorSvcClient, motorCommandMsg);
    pause(0.1);

end