function [RL] = qToR(qd)
% Conversión de cuaternio a Matriz de rotación
% v_i = RL v_b


RL = [1-2*(qd(3)^2)-2*(qd(4)^2), 2*qd(2)*qd(3)-2*qd(1)*qd(4),2*qd(2)*qd(4)+2*qd(1)*qd(3);
   2*qd(2)*qd(3)+2*qd(1)*qd(4),  1-2*(qd(2)^2)-2*(qd(4)^2) ,2*qd(3)*qd(4)-2*qd(1)*qd(2);
   2*qd(2)*qd(4)-2*qd(1)*qd(3), 2*qd(3)*qd(4)+2*qd(1)*qd(2),1-2*(qd(2)^2)-2*(qd(3)^2)];

end