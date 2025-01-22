%% figure作成
set(0,'DefaultTextInterpreter','latex')
set(0,'DefaultLegendInterpreter','latex')
clf;
font=25;
legend_font=25;
linewidth = 3;
linewidth2 = 2;
timelim = [3.5,7.5];
% timelim = [0, 15.0];

% savefig(filename1)
fig1 = figure(1);
data = out.Scope_XYZ;
fig1.WindowState = 'maximized';
subplot(3,1,1);
plot(data.time,data.signals(1).values(:,1), 'k-','LineWidth',linewidth/2);
hold on;
plot(data.time,data.signals(1).values(:,2), 'r--','LineWidth',linewidth/2);
hold on;
plot(data.time,data.signals(1).values(:,3), 'g-','LineWidth',linewidth/2);
hold on;
plot(data.time,data.signals(1).values(:,4), 'b--','LineWidth',linewidth/2);
hold on;
% pbaspect([1 1 1]);
% 表示範囲指定
% 凡例
lgd = legend;
lgd.NumColumns = 1;
title = {'$\rm{PI_{ref}}$','$\rm{PI_{res}}$','$\rm{MPC_{ref}}$','$\rm{MPC_{res}}$'};
legend(title,'Location','southeast','FontSize',legend_font); %凡例
grid on;
set(gca, 'Xlim',timelim); 
%set(gca, 'YLim',zlimit3b);
% xlabel('time [s]','FontSize',font, 'color','k');
ylabel('X [m]','FontSize',font, 'color','k');
set(gca, 'FontName','Times New Roman','FontSize',font);
set(gca, 'XColor','k','YColor','k');
set(gca, 'GridAlpha',0.2);
set(gca, 'LineWidth', 2);
set(gca, 'TickLabelInterpreter', 'latex');
set(gca,'FontSize',font); %目盛りの文字サイズ指定

subplot(3,1,2);
plot(data.time,data.signals(2).values(:,1), 'k-','LineWidth',linewidth/2);
hold on;
plot(data.time,data.signals(2).values(:,2), 'r--','LineWidth',linewidth/2);
hold on;
plot(data.time,data.signals(2).values(:,3), 'g-','LineWidth',linewidth/2);
hold on;
plot(data.time,data.signals(2).values(:,4), 'b--','LineWidth',linewidth/2);
hold on;
% pbaspect([1 1 1]);
% 表示範囲指定
% 凡例
% lgd = legend;
% lgd.NumColumns = 1;
% legend(title,'Location','southeast','FontSize',legend_font); %凡例
grid on;
set(gca, 'Xlim',timelim); 
%set(gca, 'Xlim',ylimit3b); 
%set(gca, 'YLim',zlimit3b);
% xlabel('time [s]','FontSize',font, 'color','k');
ylabel('Y [m]','FontSize',font, 'color','k');
set(gca, 'FontName','Times New Roman','FontSize',font);
set(gca, 'XColor','k','YColor','k');
set(gca, 'GridAlpha',0.2);
set(gca, 'LineWidth', 2);
set(gca, 'TickLabelInterpreter', 'latex');
set(gca,'FontSize',font); %目盛りの文字サイズ指定

subplot(3,1,3);
plot(data.time,data.signals(3).values(:,1), 'k-','LineWidth',linewidth/2);
hold on;
plot(data.time,data.signals(3).values(:,2), 'r--','LineWidth',linewidth/2);
hold on;
plot(data.time,data.signals(3).values(:,3), 'g-','LineWidth',linewidth/2);
hold on;
plot(data.time,data.signals(3).values(:,4), 'b--','LineWidth',linewidth/2);
hold on;
% pbaspect([1 1 1]);
% 表示範囲指定
% 凡例
% lgd = legend;
% lgd.NumColumns = 1;
% legend(title,'Location','southeast','FontSize',legend_font); %凡例
grid on;
set(gca, 'Xlim',timelim); 
%set(gca, 'Xlim',ylimit3b); 
%set(gca, 'YLim',zlimit3b);
xlabel('time [s]','FontSize',font, 'color','k');
ylabel('Z [m]','FontSize',font, 'color','k');
set(gca, 'FontName','Times New Roman','FontSize',font);
set(gca, 'XColor','k','YColor','k');
set(gca, 'GridAlpha',0.2);
set(gca, 'LineWidth', 2);
set(gca, 'TickLabelInterpreter', 'latex');
set(gca,'FontSize',font); %目盛りの文字サイズ指定

%% fig2
% savefig(filename1)
fig2 = figure(2);
data = out.Scope_PPIres;
data1 = out.Scope_MPCres;
fig2.WindowState = 'maximized';
%% グラフ枚数の設定
subplot(3,2,1);
plot(data.time,data.signals(1).values(:,2), 'k-','LineWidth',linewidth/2);
hold on;
plot(data.time,data.signals(1).values(:,3), 'r-','LineWidth',linewidth/2);
hold on;
plot(data1.time,data1.signals(1).values(:,2), 'g-','LineWidth',linewidth/2);
hold on;
plot(data1.time,data1.signals(1).values(:,3), 'b-','LineWidth',linewidth/2);
hold on;
% 表示範囲指定
% 凡例
lgd = legend;
lgd.NumColumns = 1;
% title = {'PI_ref','PI_res','MPC_ref','MPC_res'};
legend(title,'Location','southeast','FontSize',legend_font); %凡例
% lgd = legend;
% lgd.NumColumns = 1;
% legend({'cmd'},'Location','southeast','FontSize',legend_font); %凡例
grid on;
set(gca, 'Xlim',timelim); 
% set(gca, 'Xlim',ylimit3b); 
% set(gca, 'YLim',zlimit3b);
% xlabel('Y [mm]','FontSize',font, 'color','k');
ylabel('qm2 [rad]','FontSize',font, 'color','k');
set(gca, 'FontName','Times New Roman','FontSize',font);
set(gca, 'XColor','k','YColor','k');
set(gca, 'GridAlpha',0.2);
set(gca, 'LineWidth', 2);
set(gca, 'TickLabelInterpreter', 'latex');
set(gca,'FontSize',font); %目盛りの文字サイズ指定

subplot(3,2,3);
plot(data.time,data.signals(2).values(:,1), 'k-','LineWidth',linewidth/2);
hold on;
plot(data.time,data.signals(2).values(:,2), 'r-','LineWidth',linewidth/2);
hold on;
plot(data1.time,data1.signals(2).values(:,1), 'g-','LineWidth',linewidth/2);
hold on;
plot(data1.time,data1.signals(2).values(:,2), 'b-','LineWidth',linewidth/2);
hold on;
% 表示範囲指定
% 凡例
% lgd = legend;
% lgd.NumColumns = 1;
% title = {'PI_ref','PI_res','MPC_ref','MPC_res'};
% legend(title,'Location','southeast','FontSize',legend_font); %凡例
% lgd = legend;
% lgd.NumColumns = 1;
% legend({'cmd'},'Location','southeast','FontSize',legend_font); %凡例
grid on;
set(gca, 'Xlim',timelim); 
% set(gca, 'Xlim',ylimit3b); 
% set(gca, 'YLim',zlimit3b);
% xlabel('Y [mm]','FontSize',font, 'color','k');
ylabel('wm2 [rad/s]','FontSize',font, 'color','k');
set(gca, 'FontName','Times New Roman','FontSize',font);
set(gca, 'XColor','k','YColor','k');
set(gca, 'GridAlpha',0.2);
set(gca, 'LineWidth', 2);
set(gca, 'TickLabelInterpreter', 'latex');
set(gca,'FontSize',font); %目盛りの文字サイズ指定

subplot(3,2,5);
plot(data.time,data.signals(3).values(:,1), 'k-','LineWidth',linewidth/2);
hold on;
plot(data1.time,data1.signals(3).values(:,1), 'g-','LineWidth',linewidth/2);
hold on;
% 表示範囲指定
% 凡例
% lgd = legend;
% lgd.NumColumns = 1;
% title = {'PI','MPC'};
% legend(title,'Location','southeast','FontSize',legend_font); %凡例
% lgd = legend;
% lgd.NumColumns = 1;
% legend({'cmd'},'Location','southeast','FontSize',legend_font); %凡例
grid on;
set(gca, 'Xlim',timelim); 
% set(gca, 'Xlim',ylimit3b); 
% set(gca, 'YLim',zlimit3b);
xlabel('time [s]','FontSize',font, 'color','k');
ylabel('Iq2 [rad/s]','FontSize',font, 'color','k');
set(gca, 'FontName','Times New Roman','FontSize',font);
set(gca, 'XColor','k','YColor','k');
set(gca, 'GridAlpha',0.2);
set(gca, 'LineWidth', 2);
set(gca, 'TickLabelInterpreter', 'latex');
set(gca,'FontSize',font); %目盛りの文字サイズ指定


subplot(3,2,2);
plot(data.time,data.signals(4).values(:,2), 'k-','LineWidth',linewidth/2);
hold on;
plot(data.time,data.signals(4).values(:,3), 'r-','LineWidth',linewidth/2);
hold on;
plot(data1.time,data1.signals(4).values(:,2), 'g-','LineWidth',linewidth/2);
hold on;
plot(data1.time,data1.signals(4).values(:,3), 'b-','LineWidth',linewidth/2);
hold on;
% 表示範囲指定
% 凡例
% lgd = legend;
% lgd.NumColumns = 1;
% title = {'PI_ref','PI_res','MPC_ref','MPC_res'};
% legend(title,'Location','southeast','FontSize',legend_font); %凡例
% lgd = legend;
% lgd.NumColumns = 1;
% legend({'cmd'},'Location','southeast','FontSize',legend_font); %凡例
grid on;
set(gca, 'Xlim',timelim); 
% set(gca, 'Xlim',ylimit3b); 
% set(gca, 'YLim',zlimit3b);
% xlabel('Y [mm]','FontSize',font, 'color','k');
ylabel('qm3 [rad]','FontSize',font, 'color','k');
set(gca, 'FontName','Times New Roman','FontSize',font);
set(gca, 'XColor','k','YColor','k');
set(gca, 'GridAlpha',0.2);
set(gca, 'LineWidth', 2);
set(gca, 'TickLabelInterpreter', 'latex');
set(gca,'FontSize',font); %目盛りの文字サイズ指定

subplot(3,2,4);
plot(data.time,data.signals(5).values(:,1), 'k-','LineWidth',linewidth/2);
hold on;
plot(data.time,data.signals(5).values(:,2), 'r-','LineWidth',linewidth/2);
hold on;
plot(data1.time,data1.signals(5).values(:,1), 'g-','LineWidth',linewidth/2);
hold on;
plot(data1.time,data1.signals(5).values(:,2), 'b-','LineWidth',linewidth/2);
hold on;
% 表示範囲指定
% 凡例
% lgd = legend;
% lgd.NumColumns = 1;
% title = {'PI_ref','PI_res','MPC_ref','MPC_res'};
% legend(title,'Location','southeast','FontSize',legend_font); %凡例
% lgd = legend;
% lgd.NumColumns = 1;
% legend({'cmd'},'Location','southeast','FontSize',legend_font); %凡例
grid on;
set(gca, 'Xlim',timelim); 
% set(gca, 'Xlim',ylimit3b); 
% set(gca, 'YLim',zlimit3b);
% xlabel('Y [mm]','FontSize',font, 'color','k');
ylabel('wm3 [rad/s]','FontSize',font, 'color','k');
set(gca, 'FontName','Times New Roman','FontSize',font);
set(gca, 'XColor','k','YColor','k');
set(gca, 'GridAlpha',0.2);
set(gca, 'LineWidth', 2);
set(gca, 'TickLabelInterpreter', 'latex');
set(gca,'FontSize',font); %目盛りの文字サイズ指定

subplot(3,2,6);
plot(data.time,data.signals(6).values(:,1), 'k-','LineWidth',linewidth/2);
hold on;
plot(data1.time,data1.signals(6).values(:,1), 'g-','LineWidth',linewidth/2);
hold on;
% 表示範囲指定
% 凡例
% lgd = legend;
% lgd.NumColumns = 1;
% title = {'PI','MPC'};
% legend(title,'Location','southeast','FontSize',legend_font); %凡例
grid on;
set(gca, 'Xlim',timelim); 
% set(gca, 'Xlim',ylimit3b); 
% set(gca, 'YLim',zlimit3b);
xlabel('time [s]','FontSize',font, 'color','k');
ylabel('Iq3 [rad/s]','FontSize',font, 'color','k');
set(gca, 'FontName','Times New Roman','FontSize',font);
set(gca, 'XColor','k','YColor','k');
set(gca, 'GridAlpha',0.2);
set(gca, 'LineWidth', 2);
set(gca, 'TickLabelInterpreter', 'latex');
set(gca,'FontSize',font); %目盛りの文字サイズ指定
%}