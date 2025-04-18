% STABILITY ANALYSIS

s = tf('s');
G_motor = tf(0.305,[1, 4.321]);

disp("Open loop poles: ")
disp(pole(G_motor));

Kp = 10.8;
Ki = 22.5;
Kd = 0.1;
G_pid = Kp + Ki/s + Kd*s;

G_closed_loop = feedback(G_motor*G_pid, 1);

disp("Closed loop poles: ")
disp(pole(G_closed_loop))

figure;
step(G_motor); hold on;
step(G_closed_loop); 

figure;
pzmap(G_closed_loop);