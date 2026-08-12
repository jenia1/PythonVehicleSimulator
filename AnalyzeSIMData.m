clc,close all,clear all

%% load data
sim = readtable("simdata.csv");
sim_ref = readtable("simdata_ref.csv");

%% dif

diff_x = sim.x - sim_ref.x;
diff_y = sim.y - sim_ref.y;
diff_z = sim.z - sim_ref.z;

diff_theta = sim.theta - sim_ref.theta;
diff_phi = sim.phi - sim_ref.phi;
diff_psi = sim.psi - sim_ref.psi;

%% plot
figure("Name","Position");
subplot(3,2,1); hold on; grid on; title('x'); ylabel('x [m]'); xlabel('time [s]');
plot(sim.time,sim.x,'LineWidth',1.5);
plot(sim_ref.time,sim_ref.x,'--','LineWidth',1.5); 
subplot(3,2,2); hold on; grid on; title('x diff'); ylabel('x diff [m]'); xlabel('time [s]');
plot(sim.time,diff_x,'LineWidth',1.5);
subplot(3,2,3); hold on; grid on; title('y'); ylabel('y [m]'); xlabel('time [s]');
plot(sim.time,sim.y,'LineWidth',1.5);
plot(sim_ref.time,sim_ref.y,'--','LineWidth',1.5); 
subplot(3,2,4); hold on; grid   on; title('y diff'); ylabel('y diff [m]'); xlabel('time [s]');
plot(sim.time,diff_y,'LineWidth',1.5);
subplot(3,2,5); hold on; grid on; title('z'); ylabel('z [m]'); xlabel('time [s]');
plot(sim.time,sim.z,'LineWidth',1.5);
plot(sim_ref.time,sim_ref.z,'--','LineWidth',1.5); 
subplot(3,2,6); hold on; grid on; title('z diff'); ylabel('z diff [m]'); xlabel('time [s]');
plot(sim.time,diff_z,'LineWidth',1.5);  

figure("Name","Orientation");
subplot(3,2,1); hold on; grid on; title('theta'); ylabel('theta [rad]'); xlabel('time [s]');
plot(sim.time,sim.theta,'LineWidth',1.5);
plot(sim_ref.time,sim_ref.theta,'--','LineWidth',1.5); 
subplot(3,2,2); hold on; grid on; title('theta diff'); ylabel('theta diff [rad]'); xlabel('time [s]');
plot(sim.time,diff_theta,'LineWidth',1.5);
subplot(3,2,3); hold on; grid on; title('phi'); ylabel('phi [rad]'); xlabel('time [s]');
plot(sim.time,sim.phi,'LineWidth',1.5);
plot(sim_ref.time,sim_ref.phi,'--','LineWidth',1.5);
subplot(3,2,4); hold on; grid on; title('phi diff'); ylabel('phi diff [rad]'); xlabel('time [s]');
plot(sim.time,diff_phi,'LineWidth',1.5);
subplot(3,2,5); hold on; grid on; title('psi'); ylabel('psi [rad]'); xlabel('time [s]');
plot(sim.time,sim.psi,'LineWidth',1.5);
plot(sim_ref.time,sim_ref.psi,'--','LineWidth',1.5);
subplot(3,2,6); hold on; grid on; title('psi diff'); ylabel('psi diff [rad]'); xlabel('time [s]');
plot(sim.time,diff_psi,'LineWidth',1.5);