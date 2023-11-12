%% load data
data = readtable("Frames.xlsx");
t = table2array(data(3:end,1));
t = datenum(t, 'HH:MM:SS,FFF');
t = (t - t(1)) * 86400;
VeloN = table2array(data(3:end,18));
VeloE = table2array(data(3:end,19));
heading = table2array(data(3:end,25));

%% plot data
subplot(3,1,1);
plot(t, VeloN, '.')
title('VeloN')
xticks(0:10:200)
hold on

subplot(3,1,2);
plot(t, VeloE, '.')
title('VeloE')
xticks(0:10:200)
hold on

subplot(3,1,3);
plot(t, heading, 'r')
title('Yaw Angle')
xticks(0:10:200)
hold on
    
% Orientation based on Velocity
OboV = atan2d(VeloN, VeloE);
subplot(3,1,3);
plot(t, OboV, 'b')
xticks(0:10:200)
xlabel('Time [s]')
ylabel('Yaw Angle [degree]')
hold on
legend('Heading','atan2d(VeloN, VeloE)')