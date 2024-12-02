function matrix = rot(r,deg)

rad = deg*pi/180;
if r == 'x' 
    matrix = [ 1,                0,          0;
               0,          cos(rad),  -sin(rad);
               0,         sin(rad),   cos(rad); ];

elseif r == 'y'          
    matrix = [ cos(rad),         0,   sin(rad);
               0,                1,          0;
              -sin(rad),         0,   cos(rad); ];

elseif r == 'z'         
    matrix = [ cos(rad), -sin(rad),          0;
               sin(rad),  cos(rad),          0;
               0,                0,          1; ];
% else
%     disp('üÍř1 ń]˛(x or y or z), üÍř2 ń]px(deg)')
end
end