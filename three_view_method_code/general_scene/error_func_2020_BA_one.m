function error_seq = error_func_2020_BA_one(lambda_c_value, correspondences_sonar, coordinates_3d_lambda,...
    R_i2j,t_i2j,R_k2j,t_k2j,inlier_index,weights_list,weights_flag)
    if weights_flag == 1
        weights_list_compute = weights_list;
        weights_list_compute(~inlier_index) = 0;
        weights_list_compute = weights_list_compute/sum(weights_list_compute);
    end
    error_seq = [];
    syms lambda_c
    for index = 1:length(inlier_index)
        if inlier_index(index)
            coord_x_sub = double(subs(coordinates_3d_lambda{index}{1},lambda_c,lambda_c_value));
            coord_y_sub = double(subs(coordinates_3d_lambda{index}{2},lambda_c,lambda_c_value));
            coord_z_sub = double(subs(coordinates_3d_lambda{index}{3},lambda_c,lambda_c_value));
            coords_sub_j = double([coord_x_sub;coord_y_sub;coord_z_sub]);
            coords_sub_i = R_i2j\(coords_sub_j-lambda_c_value*t_i2j);
            coords_sub_k = R_k2j\(coords_sub_j-lambda_c_value*t_k2j);
            if weights_flag == 0
                weight_value = 1;
            elseif weights_flag == 1
                weight_value = weights_list_compute(index);
            end                
            error_seq = [   error_seq,...
                            weight_value * (coords_sub_i(1)/norm(coords_sub_i(1:2))*norm(coords_sub_i) - correspondences_sonar{index}(1,1)),...
                            weight_value * (coords_sub_i(2)/norm(coords_sub_i(1:2))*norm(coords_sub_i) - correspondences_sonar{index}(2,1)),...
                            weight_value * (coords_sub_j(1)/norm(coords_sub_j(1:2))*norm(coords_sub_j) - correspondences_sonar{index}(1,2)),...
                            weight_value * (coords_sub_j(2)/norm(coords_sub_j(1:2))*norm(coords_sub_j) - correspondences_sonar{index}(2,2)),...
                            weight_value * (coords_sub_k(1)/norm(coords_sub_k(1:2))*norm(coords_sub_k) - correspondences_sonar{index}(1,3)),...
                            weight_value * (coords_sub_k(2)/norm(coords_sub_k(1:2))*norm(coords_sub_k) - correspondences_sonar{index}(2,3))];
        end
    end
    
end



