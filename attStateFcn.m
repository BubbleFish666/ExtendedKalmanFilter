function x = attStateFcn(x)
% state transition function
% states x = [vx, vy, psi];
x(1) = x(1);
x(2) = x(2);
x(3) = atan2(x(2), x(1));
% x(3) = atan2d(x(2), x(1));
% x(3) = -x(3) + 90 + 360;
% x(3) = mod(x(3),360);
x = reshape(x, [], 1);
end