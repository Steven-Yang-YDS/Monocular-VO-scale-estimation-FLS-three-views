function error_seq = error_func_2020_inlier_weights_one(lambda_c_value,lambda_c_equation_list_separate,valid_constraints,inlier_index,vectors_projection_ijk_range,weighted_flag,seperate_valid_flag)
%-----------------------------input data----------------------------------------
    error_seq = [];
    syms lambda_c
    if weighted_flag
        weights_list = vectors_projection_ijk_range;
        weights_list(~inlier_index)=0;
        weights_list = weights_list/sum(weights_list);
    else
        weights_list = ones(size(vectors_projection_ijk_range));
    end

    for index_img_point = 1:length(inlier_index)
        if inlier_index(index_img_point) 
            if seperate_valid_flag
                for ind = valid_constraints{index_img_point}
                    error_seq = [error_seq,weights_list(index_img_point) * double(subs(lambda_c_equation_list_separate{index_img_point}{ind},lambda_c,lambda_c_value))];
                end
            else
                for ind = 1:3
                    error_seq = [error_seq,weights_list(index_img_point) * double(subs(lambda_c_equation_list_separate{index_img_point}{ind},lambda_c,lambda_c_value))];
                end
            end
        end
    end
    
end



