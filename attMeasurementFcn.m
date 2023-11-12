function z = attMeasurementFcn(x)
% measurement function
% measurement z = [vx, vy, psi];
z = zeros([3,1]);
z(1) = x(1);
z(2) = x(2);
z(3) = x(3);
% z = reshape(z, [], 1);
end