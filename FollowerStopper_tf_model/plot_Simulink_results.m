%% Plot Simulink simulation output
simulink_time = simout.Time;

sim_v_cmd = simout.Data(:,1);
sim_v_rel = simout.Data(:,2);
sim_x_rel = simout.Data(:,3);
sim_v_av = simout.Data(:,4);

figure()
plot(simulink_time,sim_v_cmd)
hold on
plot(simulink_time,sim_v_av)
xlabel('Time[s]')
ylabel('Speed[m/s]')
legend('v_{cmd}','v_{av}')
grid on
set(gca,'FontSize',30)


figure()
plot(simulink_time,sim_x_rel)
xlabel('Time[s]')
ylabel('Relative distance[m]')
grid on
set(gca,'FontSize',30)


figure()
plot(simulink_time,sim_v_rel)
xlabel('Time[s]')
ylabel('Relative speed[m/s]')
grid on
set(gca,'FontSize',30)
