function A_k = attStateJacobianFcn(x)
%  Jacobian of state transition function
%  states x = [vx, vy, psi];
A_k = [1, 0, 0;
       0, 1, 0;
       -x(2)/(x(1)^2+x(2)^2), x(1)/(x(1)^2+x(2)^2), 0];
end