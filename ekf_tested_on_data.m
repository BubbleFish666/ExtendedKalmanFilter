%% load data
clear
data = readtable("Frames.xlsx");
t = table2array(data(3:end,1));
t = datenum(t, 'HH:MM:SS,FFF');
t = (t - t(1)) * 86400;
% VeloN and VeloE could have been swappd
VeloN = table2array(data(3:end,19));
VeloE = table2array(data(3:end,18));
heading = table2array(data(3:end,25));

%% plot data
data_range = (108 <= t) & (t <= 122);

subplot(3,1,1);
plot(t(data_range), VeloE(data_range), '.')
title('VeloE')
% xticks(0:10:200)
grid on
hold on

subplot(3,1,2);
plot(t(data_range), VeloN(data_range), '.')
title('VeloN')
% xticks(0:10:200)
grid on
hold on

subplot(3,1,3);
plot(t(data_range), heading(data_range), 'r')
title('Yaw Angle')
% xticks(0:10:200)
grid on
hold on

% Orientation based on Velocity
OboV = atan2d(VeloN, VeloE);
% OboV = -OboV + 360 + 90;
% OboV = mod(OboV,360);
OboV = invTanDegToCompassDeg(OboV);
subplot(3,1,3);
plot(t(data_range), OboV(data_range), 'b')
% xticks(0:10:200)
xlabel('Time [s]')
ylabel('Yaw Angle [degree]')
legend('Heading','atan2d(VeloN, VeloE)')
grid on
hold on

%% Initialization
% how to decide initial states?
initial_states = [0.1;0.1;pi/4];

obj = extendedKalmanFilter(@attStateFcn, @attMeasurementFcn, initial_states);

% how to decide process noise cov-matrix?
Q = [0.5, 0, 0.01;
     0, 0.5, 0.01;
     0.01, 0.01, 0.5];
Q_eigen_values = eig(Q);

obj.ProcessNoise = Q;

% how to decide measurement noise cov-matrix?
R = [0.5, 0, 0.1;
     0, 0.5, 0.1;
     0.1, 0.1, 0.1];
R_eigen_values = eig(R);

obj.MeasurementNoise = R;

% how to tell whether the noise is additive?
obj.StateTransitionJacobianFcn = @attStateJacobianFcn;
obj.MeasurementJacobianFcn = @attMeasurementJacobianFcn;

%% predict and correct
x_est_k = initial_states;
range_start = find(data_range,1,'first');
range_end = find(data_range,1,'last');
k = range_start;
while k <= range_end
    x_est_k_1 = x_est_k;
    x_pre_k = predict(obj);
    z_k = [VeloE(k); VeloN(k); compassDegToInvTanDeg(heading(k))/180*pi];
    x_est_k = correct(obj, z_k);
    k = k+1;

    subplot(3,1,1);
    plot(t(k), x_est_k(1), 'x')
    legend('Measured VeloE', 'Filtered Vx')
    grid on
    hold on

    subplot(3,1,2);
    plot(t(k), x_est_k(2), 'x')
    legend('Measured VeloN', 'Filtered Vy')
    grid on
    hold on

    subplot(3,1,3);
    plot(t(k), invTanDegToCompassDeg(x_est_k(3)/pi*180), 'x')
    legend('Heading','atan2d(VeloN, VeloE)','Filtered Yaw Angle')
    grid on
    hold on
end
% hold off