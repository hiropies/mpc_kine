function deg_array = rad2deg_array(rad_array)
    L = length(rad_array);
    deg_array = zeros(L,1);
    for i = 1:L
        deg_array(i) = rad2deg(rad_array(i));
    end
end