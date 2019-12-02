% Destiny Fawley
% 11/6/2019

% clear all;
close all
run setup.m

mass = .669;
diameter = .0762;
leng = .508;
alt0 = 16;


motor = D12Motor();
controller = [];
models = getModels();

rocket = getRocketCDR(motor, mass, alt0, leng, diameter);
result = simulate(motor,rocket,controller, models);

%%
% figure()
% grid on
% hold on
% 
% plot((result.traj.velI(3,:)),result.traj.posI(3,:),'linewidth',3);
% % plot((result.traj.time),result.traj.posI(3,:),'linewidth',3);
% % igniteTime = find(result.ctrl.igniteMotor == 1,1,'first');
% % scatter(result.traj.time(igniteTime),result.traj.posI(3,igniteTime),'filled');
% 
% xlabel('Planet-relative Velocity, m/s');
% ylabel('Altitude, m');
% 
% set(gca, 'FontName', 'Times New Roman','fontsize',30)

% set(gcf,'paperorientation','portrait');
% set(gcf,'paperunits','normalized')
% set(gcf,'paperposition',[0 0 1 1]);
% s_dir = "\\ad.uillinois.edu\engr-ews\dfawley2\documents\Classes\AE 442\Trade Study 3";
% fig_name = strcat('mass=',num2str(mass),'length=',num2str(leng),...
%     'diameter=',num2str(diameter),'.png');
% print(gcf,'-dpng',fullfile(s_dir,fig_name));
figure;
plot(result.traj.time,result.traj.momentFin1I+result.traj.momentFin2I+result.traj.momentFin3I+result.traj.momentFin4I);
hold on
plot(result.traj.time,result.traj.EulerAngles(2,:)*180/pi);
hold on
% plot(result.traj.time,result.traj.momentThrustI(1,:),'b');
% plot(result.traj.time,result.traj.momentThrustI(2,:),'r');
% plot(result.traj.time,result.traj.momentThrustI(3,:),'g');
ylim([-10 10])

% figure;
% plot(result.traj.time,result.traj.EulerAngles*180/pi);
% ylim([-10 10])

%%
drawRocket(result, rocket)
