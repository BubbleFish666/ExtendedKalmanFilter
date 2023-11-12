function x = attStateFcn(x_1)
% state transition function
% states x = [vx, vy, psi];
x = zeros([3,1]);
x(1) = x_1(1);
x(2) = x_1(2);
x(3) = atan2(x_1(2), x_1(1));
% x(3) = atan2d(x(2), x(1));
% x(3) = -x(3) + 90 + 360;
% x(3) = mod(x(3),360);

end