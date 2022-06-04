%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Cinematica inversa para robot Phantom X nuevo
Por: Felipe González, modificado por Camilo Valencia y Sara Jiménez
Entradas:
T: Pose del EF
l: Longitud de eslabones
Salida:
q_inv: Variables del espacio articular
4 soluciones, 2 configuraciones
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}

function q_inv = invKinPxC(varargin)
switch nargin
   case 3
      T = varargin{1};
      l = varargin{2};
      off = varargin{3};
   case 4
      T = varargin{1};
      l = varargin{2};
      off = varargin{3};
      qlim = varargin{4};
   otherwise
      disp('Wrong number of inputs')
end


% Desacople
    Ph = [0;0;-100;1]; %Muñeca respecto al marco de herramienta NOA
    Pb = T*Ph;         %Muñeca respecto base

% Theta 1
    theta1 = atan2(Pb(2),-Pb(1));%-off(1);

% Theta 3
    cos3  = (Pb(1)^2 + Pb(2)^2 +(Pb(3)-l(1))^2-l(2)^2 - l(3)^2)/(2*l(2)*l(3));
    sin3D = sqrt(1-cos3^2); %Codo Abajo es la raíz positiva
    sin3U = -sqrt(1-cos3^2); %Codo Arriba es la raíz negativa


if isreal(sin3D)
    % Codo Abajo
        theta3D = atan2(sin3D,cos3)-off(3);
    % Codo Arriba
        theta3U = atan2(sin3U,cos3)-off(3);

    % Theta 2
        k1 = l(2)+l(3)*cos3;
        k2 = l(3)*sin3D;
    % Codo Abajo
        theta2D = (atan2(Pb(3)-l(1),sqrt(Pb(1)^2+Pb(2)^2)) - atan2(k2,k1))-off(2); %Codo abajo es resta
    % Codo Arriba
        theta2U = (atan2(Pb(3)-l(1),sqrt(Pb(1)^2+Pb(2)^2)) + atan2(k2,k1))-off(2); %Codo arriba es suma

    % Theta 4
        Rp = (rotz(-theta1))'*T(1:3,1:3);
        pitch = atan2(Rp(3,1),Rp(1,1));
        theta4D = (pitch - theta2D - theta3D);
        theta4U = (pitch - theta2U - theta3U);
else
   theta2D = NaN;
   theta3D = NaN;
   theta4D = NaN;
   theta2U = NaN;
   theta3U = NaN;
   theta4U = NaN;
end

q_inv(1,1:4) = [theta1 theta2D theta3D theta4D];
q_inv(2,1:4) = [theta1 theta2U theta3U theta4U];
 
for i=1:size(q_inv,1)
   if any(isnan(q_inv(i,:)))
      q_inv(i,:) = [NaN NaN NaN NaN];
   end
end