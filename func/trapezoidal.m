function pos_ref = trapezoidal(target_rad,Tall,Ts)
%Ts = 2e-3;
Tup = Tall/4;
Tdown = Tup;
delta = target_rad;

if(Tall < (2*Tup))
    flag_triangle = 1;
    vel = (2.0*delta)/Tall;
else
    flag_triangle = 0;
    vel = (delta)/(Tall-Tup);
end

if flag_triangle == 1
    T_line = round((Tall * (Tup/(Tup+Tdown))),2);
    a0 = 0;
    a2 = vel/(T_line);
    
    c0 = vel*(T_line/2.0);
    c1 = vel;
    c2 = -vel/(Tall-T_line);
else
    a0 = 0;
    a2 = vel/Tup;
    
    b0 = vel*(Tup/2.0);
    b1 = vel;
    b2 = 0;
    
    c0 = vel*(Tup/2.0)+vel*(Tall-(Tup+Tdown));
    c1 = vel;
    c2 = -vel/Tdown;
end

long = round((Tall)/Ts)+1;
t = zeros(long,1);
a = zeros(long,1);
w = zeros(long,1);
p = zeros(long,1);
wZ1 = 0;
pZ1 = 0;
for i = 1:long
    t(i) = (i-1)*Ts;
    if flag_triangle == 1
        nnn = Tup/(Tup+Tdown);
        line = round((Tall*nnn)/Ts);
        if i<=line-1
            a(i) = a2;
        elseif i > line
            a(i) = c2;
        end
    else
        if i < Tup/Ts+1
            a(i) = a2;
        elseif i <  (Tall-Tdown)/Ts+1
            a(i) = b2;
        elseif i >= (Tall-Tdown)/Ts+1
            a(i) = c2;
        end
    end
    w(i) = wZ1 + a(i)*Ts;
    wZ1 = w(i);

    p(i) = pZ1 + w(i)*Ts;
    pZ1 = p(i);
end

X = t;
A = p;
B = w;
C = a;

%{
set(0,'DefaultTextInterpreter','latex')
set(0,'DefaultLegendInterpreter','latex')
clf;
font=30;
legend_font=25;
linewidth = 3;
linewidth2 = 3;

filename = 'tau_dis_est_up';
fig1 = figure(1);
fig1.WindowState = 'maximized';
h0 = plot(X, A, 'r-');
set(h0, 'linewidth',linewidth);
hold on;
h1 = plot(X, B, 'b-');
set(h1, 'linewidth',linewidth);
hold on;
h2 = plot(X, C, 'g-');
set(h2, 'linewidth',linewidth);
%hold on;
%h3 = plot(X, A4, 'y-');
%set(h3, 'linewidth',linewidth);
grid on;

% 表示範囲指定
%軸の範囲
%set(gca, 'Xlim',[0, 20]); 
%set(gca, 'Xlim',[0 3]); 
% 凡例
lgd = legend;
lgd.NumColumns = 1;
legend({'$\theta_M^{\rm{cmd}}$'},'Location','southeast','FontSize',legend_font); %凡例
% 軸ラベル
%ylabel('Current [A]','FontSize',font, 'color','k');
ylabel('Motor-side Position [rad]','FontSize',font, 'color','k');
xlabel({'Time [s]'},'FontSize',font, 'color','k');
%
% 
%set(gca, 'Xlim',[0.0,20.0]); 
%set(gca, 'Ylim',[-80,80]); 
set(gca, 'FontName','Times New Roman','FontSize',font);
set(gca, 'XColor','k','YColor','k');
set(gca, 'GridAlpha',1);
set(gca, 'LineWidth', 2);
set(gca, 'TickLabelInterpreter', 'latex');
set(gca,'FontSize',font); %目盛りの文字サイズ指定
%}
pos_ref = p;