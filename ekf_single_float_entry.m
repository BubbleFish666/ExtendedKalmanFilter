%% load data
clear; close all
data = readtable("Frames.xlsx");
t = table2array(data(3:end,1));
t = datenum(t, 'HH:MM:SS,FFF');
t = single((t - t(1)) * 86400);
% VeloN and VeloE could have been swappd
VeloN = single(table2array(data(3:end,19)));
VeloE = single(table2array(data(3:end,18)));
heading = single(table2array(data(3:end,25)));

%% set data range in time (seconds)
% data_range = (144 <= t) & (t <= 160);
% data_range = (22 <= t) & (t <= 35);
data_range = (109 <= t) & (t <= 120);

%% plot data
subplot(3,1,1);
plot(t(data_range), VeloE(data_range), '.')
title('VeloE')
xlabel('Time [s]')
ylabel('Velocity East (m/s)')
% xticks(0:10:200)
grid on
hold on

subplot(3,1,2);
plot(t(data_range), VeloN(data_range), '.')
title('VeloN')
xlabel('Time [s]')
ylabel('Velocity North (m/s)')
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

%% initialization
x0 = [0.1;0.1;pi/4];
x_k_1_c = x0;
P_k_1 = eye(3);
Q = [0.5, 0, 0.01;
     0, 0.5, 0.01;
     0.01, 0.01, 0.3];
R = [0.5, 0, 0.1;
     0, 0.5, 0.1;
     0.1, 0.1, 0.6];

%% Kalman Filter
range_start = find(data_range,1,'first');
range_end = find(data_range,1,'last');
k = range_start;
% pre-allocate vector x_k_c
x_k_c = zeros(3,range_end-range_start+1);
yaw_k_c = zeros(1,range_end-range_start+1);

while k <= range_end
    z_k = [VeloE(k); VeloN(k); compassDegToInvTanDeg(heading(k))/180*pi];
    [x_k_c(:,k-range_start+1), P_k, yaw_k_c(1,k-range_start+1)] = ekf_single_float(x_k_1_c, P_k_1, z_k);

%     % prediction
%     x_k_p = attStateFcn(x_k_1_c);
%     A_k = attStateJacobianFcn(x_k_1_c);
%     P_k_p = A_k*P_k_1*A_k.' +  Q;
% 
%     % correction
%     H_k = attMeasurementJacobianFcn(x_k_1_c);
%     K_k = P_k_p*H_k.'/(H_k*P_k_p*H_k.' + R);
% %     z_k = [1;1.2;0.90];
% %     z_k = [VeloE(k); VeloN(k); compassDegToInvTanDeg(heading(k))/180*pi];
%     z_k_p = attMeasurementFcn(x_k_p);
%     x_k_c(:,k-range_start+1) = x_k_p + K_k*(z_k - z_k_p);
%     P_k = (eye(3) - K_k*H_k)*P_k_p;

    % update
    x_k_1_c = x_k_c(:,k-range_start+1);
    P_k_1 = P_k;
    k = k+1;
end

%% plot Kalman Filter result
subplot(3,1,1);
plot(t(data_range), x_k_c(1,:), '.')
legend('Measured VeloE', 'Filtered Vx')
grid on
hold on

subplot(3,1,2);
plot(t(data_range), x_k_c(2,:), '.')
legend('Measured VeloN', 'Filtered Vy')
grid on
hold on

subplot(3,1,3);
plot(t(data_range), invTanDegToCompassDeg(yaw_k_c/pi*180), '.')
legend('Heading','atan2d(VeloN, VeloE)','Filtered Yaw Angle')
grid on
hold on

% hold off