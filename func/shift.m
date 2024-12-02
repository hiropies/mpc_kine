function traj = shift(target,slide,theta,axis)
time = target.pos(1,:);
L = length(time);
xs = slide(1);
ys = slide(2);
zs = slide(3);

Smat = [1 0 0 xs;
        0 1 0 ys;
        0 0 1 zs;
        0 0 0 1];

rad = theta*pi/180;
c = cos(rad);
s = sin(rad);
if axis == 'x'
    Rmat = [1 0  0 0;
            0 c -s 0;
            0 s  c 0;
            0 0  0 1];

elseif axis == 'y'
    Rmat = [ c 0 s 0;
             0 1 0 0;
            -s 0 c 0;
             0 0 0 1];

elseif axis == 'z'
    Rmat = [c -s 0 0;
            s  c 0 0;
            0  0 1 0;
            0  0 0 1];
end

mat = Smat*Rmat;

x = zeros(1,L);
y = zeros(1,L);
z = zeros(1,L);
trans = zeros(4,4,L);

for i = 1:L
    trans(:,:,i) = mat*[1 0 0 target.pos(2,i);
                        0 1 0 target.pos(3,i);
                        0 0 1 target.pos(4,i);
                        0 0 0 1];
    x(i) = trans(1,4,i);
    y(i) = trans(2,4,i);
    z(i) = trans(3,4,i);
end

traj.pos = [time; x; y; z];
traj.trans = trans;
traj.mat = mat;
traj.Smat = Smat;
traj.Rmat = Rmat;
traj.xs = xs;
traj.ys = ys;
traj.zs = zs;