load('C:\Users\mario\OneDrive\Desktop\REPOS\differential_robot\control_theory\system_identification\Results\Data\data.mat')


% Subfigure 1
subplot(1,2,1);
plot(time',input');
xlim([0,20]);
ylim([150,250]);
ylabel("PWM")
xlabel("Time [s]")

% Subfigure 2
subplot(1,2,2);
plot(time',velocity');
xlim([0,20]);
ylim([13,16]);
ylabel("Velocity [rpm]")
xlabel("Time [s]")