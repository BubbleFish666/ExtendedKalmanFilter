function H_k = attMeasurementJacobianFcn(x)
%  Jacobian of measurement function
%  states x = [vx, vy, psi];
H_k = [1, 0, 0;
       0, 1, 0;
       0, 0, 1];
end