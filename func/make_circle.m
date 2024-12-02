function goal = make_circle(size,freq,Ts)
R = size;
pi = 3.141592653;
theta = linspace(0,(1.5)*2*pi,1.5*freq/Ts);
L = length(theta);
time = linspace(0,L*Ts,L);
x = zeros(1,L);
y = zeros(1,L);
z = zeros(1,L);

for i=1:length(theta)
    x(1,i) = R * sin(theta(i));
    y(1,i) = R * cos(theta(i));
    z(1,i) = 0;
end
goal.pos = [time; x; y; z];