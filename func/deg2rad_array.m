function rad_array = deg2rad_array(deg_array)
    L = length(deg_array);
    rad_array = zeros(L,1);
    for i = 1:L
        rad_array(i) = deg2rad(deg_array(i));
    end
end