function plot_out(out)

theta_m = out.theta_m;
u_vec = out.iq_ref;
omega_m = out.omega_m;
ref_vec = out.theta_m_ref;
slack_vec = out.slack_variable;

figure;
figure.WindowState = 'maximized';
tiledlayout(2, 2);
nexttile;
plot(theta_m);
hold
plot(ref_vec,'--');
xlabel('Time [s]');
ylabel('Position [rad]');
title('Position');
grid on;

%figure;
nexttile;
plot(omega_m);
xlabel('Time [s]');
ylabel('Velocity [rad/s]');
title('Velocity');
grid on;

%figure;
nexttile;
plot(u_vec);
xlabel('Time [s]');
ylabel('Control effort [Nm]');
title('Torque');
grid on;

%figure;
nexttile;
plot(slack_vec);
xlabel('Time [s]');
ylabel('Slack variable');
title('Slack');
grid on;


disp('Finished');