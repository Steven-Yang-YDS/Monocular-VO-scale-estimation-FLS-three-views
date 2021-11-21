clc;
close all;
clear;
warning off;
format long;
%-----------------------------------------------------------------------------------------------------------------
%addpath([pwd,'\auxiliary_subfunc'],[pwd,'\PointClasses_subfunc'],[pwd,'\PoseClasses_subfunc'],[pwd,'\plotAxesAndPoints_subfunc']);
addpath([pwd,'\PointClasses'],[pwd,'\PoseClasses'],[pwd,'\plotAxesAndPoints']);
%-----------------------------------------------------------------------------------------------------------------
noise_test = 1;                % whether to add sonar image noise 
noise_number = 15;%5;%10;%13;%2;              % sonar noise level
%--------------------------------------------------------
approximate_plane = 1;
%--------------------------------------------------------
camera_noise = 1;              % whether to add camera image noise, noise level is set to be sigma = 1 pixel
camera_noise_fixed = 1;        % whether use one and the same set of noise for 15 sets camera image data
fix_noise_set_num = 6;
%--------------------------------------------------------
use_pre_compute_pose = 1;
preComputeNum = 6;
%--------------------------------------------------------
optimization_flag = 1;
compute_add_equ_flag = 1;
show_noise = 0;
show_correspondences = 0;
disp_result_and_error = 1;
%---------------------------------------------------------
if approximate_plane == 0
    filename = '50posesPlane';
elseif approximate_plane == 1
    filename = '50posesPlane_approximate';
end
if use_pre_compute_pose ==1
    if noise_test == 0
        if approximate_plane == 0
            load([pwd,'\estimated_results_plane\withoutNoise\estimated_cameraPoses_planeNormals.mat']);
        elseif approximate_plane == 1
            load([pwd,'\estimated_results_plane\withoutNoise\estimated_cameraPoses_planeNormals_approximate.mat']);
        end
    elseif noise_test == 1
        if approximate_plane == 0
        load([pwd,'\estimated_results_plane\',num2str(preComputeNum),'\estimated_cameraPoses_planeNormals.mat']);
        elseif approximate_plane == 1
        load([pwd,'\estimated_results_plane\',num2str(preComputeNum),'\estimated_cameraPoses_planeNormals_approximate.mat']);
        end
    end
    estimated_pose_j_k_pre = estimated_pose;
    valid_pose_pre = valid_poses;
    ratio_list_pre = ratio_list;
end
%----------------------------------------------------extrinsic_parameters---------------------------------------------------------
R_c2s = eularAngle([0,0,-90]/180*pi,'zyx');
t_c2s = [0;0;-0.05];  
extrinsic_para.R = R_c2s;
extrinsic_para.t = t_c2s;
%---------------------------------------------------------------------------------------------------------------------------------
%---------------------------------------------------------------------------------------------------------------------------------
% focalLength = 0.01*5;                              % focal length in world units                      
% focalxy = focalLength*[ 80 , 80 ];                 % a two-element vector, [fx, fy].the number of pixels per world unit in the x and y direction respectively.
% principalPoint = [ 960 , 540 ];                    % a two-element vector, [cx,cy], in pixels.
% imageSize = [ 1920,1080 ];                         % Image size produced by the camera, specified as a two-element vector, [mrows,ncols]
% bound_x = [-imageSize(1)*focalLength/focalxy(1)/2,imageSize(1)*focalLength/focalxy(1)/2];
% bound_y = [-imageSize(2)*focalLength/focalxy(2)/2,imageSize(2)*focalLength/focalxy(2)/2];
%load([pwd,'\ground_truth_data\ground_truth_for_test.mat']);
%load([pwd,'\input_data\sensor_measurements_for_test.mat']);
%---------------------------------------------------------------------------------------------------------------------------------
focalLength = 0.01*5;                              % focal length in world units                      
focalxy = focalLength*[ 20 , 20 ]*1000;            % a two-element vector, [fx, fy].the number of pixels per world unit in the x and y direction respectively.
principalPoint = [ 960 , 540 ];                    % a two-element vector, [cx,cy], in pixels.
imageSize = [ 1920,1080 ];                         % Image size produced by the camera, specified as a two-element vector, [mrows,ncols]
bound_x = [-imageSize(1)/focalxy(1)/2,imageSize(1)/focalxy(1)/2];
bound_y = [-imageSize(2)/focalxy(2)/2,imageSize(2)/focalxy(2)/2];

load([pwd,'\ground_truth_data_plane\ground_truth_50posesPlane.mat']);

if noise_test == 1      
    if noise_number < 10
        str_noise_num = ['0',num2str(noise_number)];
    elseif mod(noise_number,10) == 0
        str_noise_num = num2str(noise_number/10);
    else
        str_noise_num = num2str(noise_number);
    end        
    load([pwd,'\input_data_plane\sensor_measurements_50posesPlane.mat']);
    if logical(show_noise)
        camera_image_list_truth = camera_image_list;
        sonar_image_list_truth = sonar_image_list;
    end
    load([pwd,'\input_data_plane\sensor_measurements_noise_(mean_0_sigma_0.0',str_noise_num,')_',filename,'.mat']);    
    sonar_image_list = sonar_image_list_noise;                                         % import noisy sonar inputdata
    if camera_noise == 1 
        if camera_noise_fixed == 1
            if fix_noise_set_num < 10
                str_fix_noise_num = ['0',num2str(fix_noise_set_num)];
            elseif mod(fix_noise_set_num,10) == 0
                str_fix_noise_num = num2str(fix_noise_set_num/10);
            else
                str_fix_noise_num = num2str(fix_noise_set_num);
            end
            load([pwd,'\input_data_plane\sensor_measurements_noise_(mean_0_sigma_0.0',str_fix_noise_num,')_',filename,'.mat'],...
                'camera_image_list_noise','camera_image_list_pixel_noise');
        end
        camera_image_list = camera_image_list_noise;
        camera_image_list_pixel = camera_image_list_pixel_noise;        
    end
else
    load([pwd,'\input_data_plane\sensor_measurements_',filename,'.mat']);
end
%=============================================================================
start_frame = 1;
end_frame = 51;                                % the last number of frame in sonar sequence
valid_poses = ones(1,end_frame-1);               % the n-th valid flag indicates that the validation of pose from n to n+1
% meaning: 0 -- no correspondences among three sonar views
%          1 -- the three-view method works
%          2 -- one of the two relative pose is unsolvable for no correspondence between color images
%          3 -- the correspondences in three sonar views contain heavily noises and are not proper data.
%=============================================================================
estimated_pose_j_k = cell(1,end_frame-1);
%-----------------------------------------------------------------
scale_estimated_weights = zeros(1,end_frame-1-1);
scale_estimated_weights_ransac = zeros(1,end_frame-1-1);
%--------------------------------------------------------
result_addition_w_list = zeros(1,end_frame-1-1);
result_addition_ransac_inlier_w_list = zeros(1,end_frame-1-1);
%--------------------------------------------------------
scale_estimated_refined_weights_ransac = zeros(1,end_frame-1-1);
%--------------------------------------------------------
scale_estimated_refined_all_BA_RW = zeros(1,end_frame-1-1);
scale_estimated_refined_valid_BA_RW = zeros(1,end_frame-1-1);
scale_estimated_refined_ransac_BA_RW = zeros(1,end_frame-1-1);
scale_estimated_refined_weights_BA_RW = zeros(1,end_frame-1-1);
scale_estimated_refined_weights_ransac_BA = zeros(1,end_frame-1-1);
%----------------------------------------------------------
scale_estimated_refined_all_BA_W = zeros(1,end_frame-1-1);
scale_estimated_refined_valid_BA_W = zeros(1,end_frame-1-1);
scale_estimated_refined_ransac_BA_W = zeros(1,end_frame-1-1);
scale_estimated_refined_weights_BA_W = zeros(1,end_frame-1-1);
scale_estimated_refined_weights_ransac_BA_W = zeros(1,end_frame-1-1);
scale_estimated_refined_weights_object_W = zeros(1,end_frame-1-1);

%----------------------------------------------------------------
pointNum_list_camera = zeros(1,end_frame-1);
total_num_list = zeros(1,end_frame-1-1);        
number_of_valid_list = zeros(1,end_frame-1-1);
ransac_valid_num_list = zeros(1,end_frame-1-1);
residual_thres_ransac1_list = zeros(1,end_frame-1-1);

flag_weights_list = ones(1,end_frame-1-1);
flag_ransac_weights_list = ones(1,end_frame-1-1);

ratio_list = zeros(1,end_frame-1-1);        

offset_x1 = 2 * bound_x(2);
offset_x2 = 4 * bound_x(2);
time_point_list = cell(1,end_frame - 1 - 1);
time_pose = zeros(1,end_frame - 1 - 1);


time_optimization_all = zeros(1,end_frame - 1 - 1);
time_optimization_v = zeros(1,end_frame - 1 - 1);
time_optimization_r = zeros(1,end_frame - 1 - 1);
time_optimization_w = zeros(1,end_frame - 1 - 1);
time_optimization_r_w = zeros(1,end_frame - 1 - 1);
time_optimization = zeros(1,end_frame - 1 - 1);

time_optimization_all_W = zeros(1,end_frame - 1 - 1);
time_optimization_v_W = zeros(1,end_frame - 1 - 1);
time_optimization_r_W = zeros(1,end_frame - 1 - 1);
time_optimization_w_W = zeros(1,end_frame - 1 - 1);
time_optimization_r_w_W = zeros(1,end_frame - 1 - 1);
time_optimization_object_W = zeros(1,end_frame - 1 - 1);



for image_j = start_frame:end_frame - 1  % j must not greater than (max number of frames -1)
    %----------------------display the noisy data and ground truth camera projections---------------------------------
    if noise_test == 1 && logical(show_noise)                                                                
        figure;   
        set(gca,'YDir','reverse');
        axis equal;
        set(gca,'XLim',[bound_x(1),bound_x(2)]);
        set(gca,'YLim',bound_y);
        xlabel('x'),ylabel('y')
        hold on;  
        for index_correspond = 1:length(camera_image_list{image_j})
            point_vec = camera_image_list{image_j}{index_correspond}.coordinate;
            plot(point_vec(1),point_vec(2),'b.');   
            point_vec_gt = camera_image_list_truth{image_j}{index_correspond}.coordinate;
            plot(point_vec_gt(1),point_vec_gt(2),'r.');
        end
        hold off;
    end
    %-----------------------------------------------------------------------------------------------------------------
    if image_j > start_frame && valid_poses(image_j-1)~=2  % if can not obtain the previous pose 
        image_i = image_j - 1;
        image_k = image_j + 1;
        if use_pre_compute_pose ~= 1
            %-------------------------------------plot_correspondences_start----------------------------------------------
            list1 = zeros(1,length(camera_image_list{image_i}));
            list2 = zeros(1,length(camera_image_list{image_j}));
            list3 = zeros(1,length(camera_image_list{image_k}));
            if show_correspondences
                figure;
                set(gca,'YDir','reverse');
                axis equal;
                set(gca,'XLim',[bound_x(1),5*bound_x(2)]);
                set(gca,'YLim',bound_y);
                xlabel('x');
                ylabel('y');
                hold on;
            end

            %----------------------------plot points in 3 frames----------------------------
            for index_correspond = 1:length(camera_image_list{image_i})
                if show_correspondences
                    point_vec = camera_image_list{image_i}{index_correspond}.coordinate;
                    plot(point_vec(1),point_vec(2),'c.');   
                end
                list1(index_correspond) = camera_image_list{image_i}{index_correspond}.feature_index;
            end
            for index_correspond = 1:length(camera_image_list{image_j})
                if show_correspondences
                    point_vec = camera_image_list{image_j}{index_correspond}.coordinate;
                    plot((point_vec(1)+ offset_x1),point_vec(2),'g.');
                end
                list2(index_correspond) = camera_image_list{image_j}{index_correspond}.feature_index;
            end
            for index_correspond = 1:length(camera_image_list{image_k})
                if show_correspondences
                    point_vec = camera_image_list{image_k}{index_correspond}.coordinate;
                    plot((point_vec(1)+ offset_x2),point_vec(2),'m.');
                end
                list3(index_correspond) = camera_image_list{image_k}{index_correspond}.feature_index;
            end
            %--------------------------------------------------------------------------------
            %--------------------------------plot boundaries---------------------------------
            if show_correspondences
                plot([bound_x(1),bound_x(1)],bound_y,'k-');
                plot([bound_x(2),bound_x(2)],bound_y,'k-');
                plot([3*bound_x(2),3*bound_x(2)],bound_y,'k-');
                plot([5*bound_x(2),5*bound_x(2)],bound_y,'k-');
                plot([bound_x(1),5*bound_x(2)],[bound_y(1),bound_y(1)],'k-');
                plot([bound_x(1),5*bound_x(2)],[bound_y(2),bound_y(2)],'k-');
            end
            %--------------------------------------------------------------------------------
            %-------------------------------find correspondences-----------------------------
            [common_ij, index_a1, index_b1] = intersect(list1, list2);
            [common_jk, index_a2, index_b2] = intersect(list2, list3);

            corres_ij_epipo = zeros(length(common_ij),2);
            corres_ji_epipo = zeros(length(common_ij),2);
            corres_jk_epipo = zeros(length(common_jk),2);
            corres_kj_epipo = zeros(length(common_jk),2);
            %--------------------------------------------------------------------------------
            %-------------------------------------plot lines---------------------------------
            for ind_match_line = 1:length(common_ij)
                start_x = camera_image_list{image_i}{index_a1(ind_match_line)}.coordinate(1);
                start_y = camera_image_list{image_i}{index_a1(ind_match_line)}.coordinate(2);
                end_x = camera_image_list{image_j}{index_b1(ind_match_line)}.coordinate(1);
                end_y = camera_image_list{image_j}{index_b1(ind_match_line)}.coordinate(2);
                if show_correspondences
                    plot([start_x,end_x + offset_x1],[start_y,end_y],'r-');
                end
                corres_ij_epipo(ind_match_line,1) = start_x;
                corres_ij_epipo(ind_match_line,2) = start_y;
                corres_ji_epipo(ind_match_line,1) = end_x;
                corres_ji_epipo(ind_match_line,2) = end_y;
            end
            for ind_match_line = 1:length(common_jk)
                start_x = camera_image_list{image_j}{index_a2(ind_match_line)}.coordinate(1);
                start_y = camera_image_list{image_j}{index_a2(ind_match_line)}.coordinate(2);
                end_x = camera_image_list{image_k}{index_b2(ind_match_line)}.coordinate(1);
                end_y = camera_image_list{image_k}{index_b2(ind_match_line)}.coordinate(2);
                if show_correspondences
                    plot([start_x + offset_x1,end_x + offset_x2],[start_y,end_y],'b-');
                end
                corres_jk_epipo(ind_match_line,1) = start_x;
                corres_jk_epipo(ind_match_line,2) = start_y;
                corres_kj_epipo(ind_match_line,1) = end_x;
                corres_kj_epipo(ind_match_line,2) = end_y;
            end
            if show_correspondences
                hold off;
            end
        end
    else
        image_k = image_j + 1;
        if use_pre_compute_pose ~= 1
            list2 = zeros(1,length(camera_image_list{image_j}));
            list3 = zeros(1,length(camera_image_list{image_k}));
            for index_correspond = 1:length(camera_image_list{image_j})
                %point_vec = camera_image_list{image_j}{index_correspond}.coordinate;
                list2(index_correspond) = camera_image_list{image_j}{index_correspond}.feature_index;
            end
            for index_correspond = 1:length(camera_image_list{image_k})
                %point_vec = camera_image_list{image_k}{index_correspond}.coordinate;
                list3(index_correspond) = camera_image_list{image_k}{index_correspond}.feature_index;
            end
            [common_jk, index_a2, index_b2] = intersect(list2, list3);
            corres_jk_epipo = zeros(length(common_jk),2);
            corres_kj_epipo = zeros(length(common_jk),2);
            for ind_match_line = 1:length(common_jk)
                start_x = camera_image_list{image_j}{index_a2(ind_match_line)}.coordinate(1);
                start_y = camera_image_list{image_j}{index_a2(ind_match_line)}.coordinate(2);
                end_x = camera_image_list{image_k}{index_b2(ind_match_line)}.coordinate(1);
                end_y = camera_image_list{image_k}{index_b2(ind_match_line)}.coordinate(2);

                corres_jk_epipo(ind_match_line,1) = start_x;
                corres_jk_epipo(ind_match_line,2) = start_y;
                corres_kj_epipo(ind_match_line,1) = end_x;
                corres_kj_epipo(ind_match_line,2) = end_y;
            end
        end
 %------------------------------plot_correspondences_end------------------------------
    end
 %---------------------------------------------------------------------------------------
    if use_pre_compute_pose == 0
        pointNum = length(common_jk);
        pointNum_list_camera(image_j) = pointNum;
    end
    
    if use_pre_compute_pose == 1
        estimated_pose_j_k{image_j} = estimated_pose_j_k_pre{image_j}; 
        valid_poses(image_j) = valid_pose_pre(image_j);  
    end    
    %point_dist_list(image_j) = position_3D_e_k;
    if image_j > start_frame && valid_poses(image_j-1)~=2 && valid_poses(image_j) ~= 2  %valid_poses(image_i-1)~=2 ensure the previous essential matrix is obtained.
        disp(['----------------------Pose_',num2str(image_j-1),'-----------------------------']);
        ratio_list(image_i) = ratio_list_pre(image_i);               
    end
%============================================ find sonar correspondences ===============================================
    %---------------weights + ransac--------------
    lambda_c_equation_weights = [];
    lambda_c_equation_weights_ransac =[];
    %==================================================================
    %---------------feature number-------------
    result_addition_w = 0;
    result_addition_ransac_inlier_w = 0;
    %====================================================================
    if image_j > start_frame && valid_poses(image_j-1)~=2  && valid_poses(image_j) ~= 2   %&& sonar_compute_flag       
        t_start = clock;
        %--------------------- find sonar correspondence ----------------------------------
        list1_s = zeros(1,length(sonar_image_list{image_i}));
        list2_s = zeros(1,length(sonar_image_list{image_j}));
        list3_s = zeros(1,length(sonar_image_list{image_k}));        
        for index_correspond = 1:length(sonar_image_list{image_i})
            list1_s(index_correspond) = sonar_image_list{image_i}{index_correspond}.feature_index;
        end
        for index_correspond = 1:length(sonar_image_list{image_j})
            list2_s(index_correspond) = sonar_image_list{image_j}{index_correspond}.feature_index;
        end
        for index_correspond = 1:length(sonar_image_list{image_k})
            list3_s(index_correspond) = sonar_image_list{image_k}{index_correspond}.feature_index;
        end
        [common_s1, index_a1_s, index_b1_s] = intersect(list1_s, list2_s);
        [common_s2, index_a2_s, index_b2_s] = intersect(list2_s, list3_s);
        [common_s, ind_s1, ind_s2] = intersect(common_s1, common_s2);
        index_common1 = index_a1_s(ind_s1);
        index_common2 = index_b1_s(ind_s1);
        index_common3 = index_b2_s(ind_s2);
        %-----------------------------------------------------------------------------------
        if ~isempty(index_common1)&&~isempty(index_common2)&&~isempty(index_common3)   % correspondences exist
            vectors_projection_ijk_range = zeros(1,length(index_common1));
            sol_i_get = zeros(1,length(index_common1));
            sol_k_get = zeros(1,length(index_common1));
            valid_constraints = cell(1,length(index_common1));
            
            valid_list = ones(1,length(index_common1));
            error_list = zeros(1,length(index_common1));
            angles_inclination = zeros(1,length(index_common1));
            
            lambda_c_equation_list_separate = cell(1,length(index_common1));
            lambda_c_equation_list = cell(1,length(index_common1));
            
            time_point = zeros(1,length(index_common1));
            
            
            coordinates_3d_lambda_list = cell(1,length(index_common1));
            correspondences_sonar = cell(1,length(index_common1));
            %-----------------------------------------------------------------------
            R_i2j = estimated_pose_j_k{image_i}(1:3,1:3);
            t_i2j = estimated_pose_j_k{image_i}(1:3,4);
            R_k2j = estimated_pose_j_k{image_j}(1:3,1:3).';
            t_k2j = ratio_list(image_i)*(-R_k2j)*estimated_pose_j_k{image_j}(1:3,4);

            v_j_x = [1;0;0];
            v_j_y = [0;1;0];
            v_j_z = [1;0;1];
            v_i_x = R_c2s * R_i2j * R_c2s.' * v_j_x;
            v_i_y = R_c2s * R_i2j * R_c2s.' * v_j_y;
            v_i_z = R_c2s * R_i2j * R_c2s.' * v_j_z;
            v_k_x = R_c2s * R_k2j * R_c2s.' * v_j_x;
            v_k_y = R_c2s * R_k2j * R_c2s.' * v_j_y;
            v_k_z = R_c2s * R_k2j * R_c2s.' * v_j_z;
            origin_j = [0;0;0];
            %-------------------------------------------------------------------------
            for sonar_feature_index = 1:length(index_common1)
                tic;
                coord_i = sonar_image_list{image_i}{index_common1(sonar_feature_index)}.coordinate;
                coord_j = sonar_image_list{image_j}{index_common2(sonar_feature_index)}.coordinate;
                coord_k = sonar_image_list{image_k}{index_common3(sonar_feature_index)}.coordinate;
                %----------------------------------------------------------------------
                correspondences_sonar{sonar_feature_index} = [coord_i,coord_j,coord_k];
                %----------------------------------------------------------------------
                range_R_i = norm(coord_i);
                sin_theta_i = coord_i(1)/range_R_i;
                cos_theta_i = coord_i(2)/range_R_i;
                range_R_j = norm(coord_j);
                sin_theta_j = coord_j(1)/range_R_j;
                cos_theta_j = coord_j(2)/range_R_j;
                range_R_k = norm(coord_k);
                sin_theta_k = coord_k(1)/range_R_k;
                cos_theta_k = coord_k(2)/range_R_k;
                %-------------------------------expression of the origin and axes vectors of ith and k-th frame in j-th frame------------------------
                syms xs_3d ys_3d zs_3d lambda_c
                origin_i = (eye(3) - R_c2s * R_i2j * R_c2s.') * t_c2s  + lambda_c * R_c2s * t_i2j;
                origin_k = (eye(3) - R_c2s * R_k2j * R_c2s.') * t_c2s  + lambda_c * R_c2s * t_k2j;
                    
                n_i = cos_theta_i * v_i_x - sin_theta_i * v_i_y;
                n_j = cos_theta_j * v_j_x - sin_theta_j * v_j_y;
                n_k = cos_theta_k * v_k_x - sin_theta_k * v_k_y;
                %---------------------------------------error analysis---------------------------------
                line_vector_ij = cross(n_i,n_j);
                vectors_projection_ij = abs(line_vector_ij.'* n_k);
                line_vector_jk = cross(n_j,n_k);
                vectors_projection_jk = abs(line_vector_jk.'* n_i);
                line_vector_ik = cross(n_i,n_k);
                vectors_projection_ik = abs(line_vector_ik.'* n_j);
                vectors_projection_ijk_range(sonar_feature_index) = vectors_projection_ij/range_R_k + vectors_projection_jk/range_R_i + vectors_projection_ik/range_R_j;     % error contribution creterion
                equ_plane_i = ([xs_3d;ys_3d;zs_3d]-origin_i).'*n_i ;
                equ_plane_j = ([xs_3d;ys_3d;zs_3d]-origin_j).'*n_j ;
                equ_plane_k = ([xs_3d;ys_3d;zs_3d]-origin_k).'*n_k ;
                [xs_3d_lambda, ys_3d_lambda, zs_3d_lambda] = solve([equ_plane_i==0,equ_plane_j==0,equ_plane_k==0],xs_3d,ys_3d,zs_3d);
                %-------------------------------------------------------------------------------
                coordinates_3d_lambda_list{sonar_feature_index}{1} = xs_3d_lambda;
                coordinates_3d_lambda_list{sonar_feature_index}{2} = ys_3d_lambda;
                coordinates_3d_lambda_list{sonar_feature_index}{3} = zs_3d_lambda;
                %-------------------------------------------------------------------------------
                %---------------------------------------------------------------------------------------------------------------------
                sol_i = double(solve((xs_3d_lambda-origin_i(1))^2 + (ys_3d_lambda-origin_i(2))^2 + (zs_3d_lambda - origin_i(3))^2 - range_R_i^2 == 0,lambda_c));
                sol_j = double(solve(xs_3d_lambda^2 + ys_3d_lambda^2 + zs_3d_lambda^2 - range_R_j^2 == 0,lambda_c));  %%
                sol_k = double(solve((xs_3d_lambda-origin_k(1))^2 + (ys_3d_lambda-origin_k(2))^2 + (zs_3d_lambda - origin_k(3))^2 - range_R_k^2 == 0,lambda_c));
                sol_i = sol_i.*(imag(sol_i)==0);
                sol_j = sol_j.*(imag(sol_j)==0);
                sol_k = sol_k.*(imag(sol_k)==0);
                if sum(sol_i>0 + sol_j>0 + sol_k>0)>=1                                                                %creterion 1
                    sol_candidates = [sol_i(sol_i>0),sol_j(sol_j>0),sol_k(sol_k>0)];
                    %---------------------------------------------------------------
                    if sum(sol_i>0)==2 ||sum(sol_j>0)==2 ||sum(sol_k>0)==2
                        disp('TWO POSITIVE SOLUTIONS!!!')
                    elseif sum(sol_i>0)==0 ||sum(sol_j>0)==0 ||sum(sol_k>0)==0
                        disp('TWO NEGATIVE SOLUTIONS!!!')
                    end
                    %---------------------------------------------------------------
                    sol_can_valid_one = zeros(1,length(sol_candidates));
                    range_test = zeros(1,length(sol_candidates));
                    for ind_p = 1:length(sol_candidates)                            
                        if ind_p <= sum(sol_i>0)
                            vec_j_test = double(subs([xs_3d_lambda,ys_3d_lambda,zs_3d_lambda],lambda_c,sol_candidates(ind_p)));
                            vec_k_test = double(subs([xs_3d_lambda-origin_k(1),ys_3d_lambda-origin_k(2),zs_3d_lambda-origin_k(3)],lambda_c,sol_candidates(ind_p)));
                            vec_k_self = (R_c2s * R_k2j * R_c2s.')\vec_k_test.';
                            if vec_k_self(1)* coord_k(1)>0 && vec_j_test(1)* coord_j(1)>0                             %creterion 2
                                range_test(ind_p) = abs(norm(vec_j_test)-range_R_j) + abs(norm(vec_k_test)-range_R_k);
                                sol_can_valid_one(ind_p) = 1;
                            else
                                sol_can_valid_one(ind_p) = 0;
                                range_test(ind_p) = 0;
                            end
                        elseif ind_p > sum(sol_i>0) && ind_p <= sum(sol_i>0 + sol_j>0)
                            vec_i_test = double(subs([xs_3d_lambda-origin_i(1),ys_3d_lambda-origin_i(2),zs_3d_lambda-origin_i(3)],lambda_c,sol_candidates(ind_p)));
                            vec_k_test = double(subs([xs_3d_lambda-origin_k(1),ys_3d_lambda-origin_k(2),zs_3d_lambda-origin_k(3)],lambda_c,sol_candidates(ind_p)));
                            vec_i_self = (R_c2s * R_i2j * R_c2s.')\vec_i_test.';
                            vec_k_self = (R_c2s * R_k2j * R_c2s.')\vec_k_test.';
                            if vec_i_self(1)* coord_i(1)>0 && vec_k_self(1)* coord_k(1)>0                  
                                range_test(ind_p) = abs(norm(vec_i_test)-range_R_i) + abs(norm(vec_k_test)-range_R_k);
                                sol_can_valid_one(ind_p) = 2;
                            else
                                sol_can_valid_one(ind_p) = 0;
                                range_test(ind_p) = 0;
                            end
                        else    
                            vec_i_test = double(subs([xs_3d_lambda-origin_i(1),ys_3d_lambda-origin_i(2),zs_3d_lambda-origin_i(3)],lambda_c,sol_candidates(ind_p)));
                            vec_j_test = double(subs([xs_3d_lambda,ys_3d_lambda,zs_3d_lambda],lambda_c,sol_candidates(ind_p)));    
                            vec_i_self = (R_c2s * R_i2j * R_c2s.')\vec_i_test.';
                            if vec_i_self(1)* coord_i(1)>0 && vec_j_test(1)* coord_j(1)>0
                                range_test(ind_p) = abs(norm(vec_i_test)-range_R_i) + abs(norm(vec_j_test)-range_R_j);
                                sol_can_valid_one(ind_p) = 3;
                            else
                                sol_can_valid_one(ind_p) = 0;
                                range_test(ind_p) = 0;
                            end
                        end                                
                    end               
                    if sum(logical(sol_can_valid_one))>=1
                        range_test_valid = range_test(logical(sol_can_valid_one));
                        sol_candidates_valid = sol_candidates(logical(sol_can_valid_one)); 
                        valid_constraints{sonar_feature_index} = unique(sol_can_valid_one(logical(sol_can_valid_one)));
                        [~,ind_result] = min(range_test_valid);                                                      %creterion 3
                        scale_estimated_test = sol_candidates_valid(ind_result);                             
                    else
                        disp('No proper solution. One unkown.');
                        valid_list(sonar_feature_index) = 0;
                        scale_estimated_test = 0;
                        valid_constraints{sonar_feature_index} = 0;
                    end
                else
                    disp('No proper solution. One unkown.');
                    valid_list(sonar_feature_index) = 0;
                    scale_estimated_test = 0;
                    valid_constraints{sonar_feature_index} = 0;
                end

                sol_i_get(sonar_feature_index) = scale_estimated_test;
                error_list(sonar_feature_index) = abs(scale_estimated_test - scale_list(image_i));
                
                %-----------------------------------------------------------------------------------
                tmpTime = toc;
                if valid_list(sonar_feature_index) ~= 0
                    time_point(sonar_feature_index) = tmpTime;
                else 
                    time_point(sonar_feature_index) = -1;
                end
                
                if  valid_list(sonar_feature_index)
                    vec_i_get = double(subs([xs_3d_lambda-origin_i(1),ys_3d_lambda-origin_i(2),zs_3d_lambda-origin_i(3)],lambda_c,sol_i_get(sonar_feature_index)));
                    vec_j_get = double(subs([xs_3d_lambda,ys_3d_lambda,zs_3d_lambda],lambda_c,sol_i_get(sonar_feature_index)));
                    vec_k_get = double(subs([xs_3d_lambda-origin_k(1),ys_3d_lambda-origin_k(2),zs_3d_lambda-origin_k(3)],lambda_c,sol_i_get(sonar_feature_index)));
                    vec_t_i = double(subs(origin_i,lambda_c,sol_i_get(sonar_feature_index)));      
                    vec_t_k = double(subs(origin_k,lambda_c,sol_i_get(sonar_feature_index)));    
                    angles_inclination(sonar_feature_index) = sqrt(1-(vec_i_get*vec_t_i/norm(vec_i_get)/norm(vec_t_i))^2) + sqrt(1-(vec_k_get*vec_t_k/norm(vec_k_get)/norm(vec_t_k))^2) +...
                        sqrt(1-(vec_j_get*vec_t_i/norm(vec_j_get)/norm(vec_t_i))^2) + sqrt(1-(vec_j_get*vec_t_k/norm(vec_j_get)/norm(vec_t_k))^2);
                end  
                %-------------------------------------------------------------------------------
                lambda_c_equation_list_separate{sonar_feature_index}{1} = (xs_3d_lambda-origin_i(1))^2 + (ys_3d_lambda-origin_i(2))^2 + (zs_3d_lambda - origin_i(3))^2 - range_R_i^2;
                lambda_c_equation_list_separate{sonar_feature_index}{2} = (xs_3d_lambda)^2 + (ys_3d_lambda)^2 + (zs_3d_lambda)^2 - range_R_j^2;
                lambda_c_equation_list_separate{sonar_feature_index}{3} = (xs_3d_lambda-origin_k(1))^2 + (ys_3d_lambda-origin_k(2))^2 + (zs_3d_lambda - origin_k(3))^2 - range_R_k^2;
                lambda_c_equation_list{sonar_feature_index} = (xs_3d_lambda)^2 + (ys_3d_lambda)^2 + (zs_3d_lambda)^2 - range_R_j^2 ...
                    + (xs_3d_lambda-origin_i(1))^2 + (ys_3d_lambda-origin_i(2))^2 + (zs_3d_lambda - origin_i(3))^2 - range_R_i^2 ...
                    + (xs_3d_lambda-origin_k(1))^2 + (ys_3d_lambda-origin_k(2))^2 + (zs_3d_lambda - origin_k(3))^2 - range_R_k^2;
                %-----------------------------------------------------------------------------------
            end
            time_point_list{image_i} = time_point;
            %disp('--------------- total time: compute scale with each sonar point ----------------');
            weights_list_sol_inlier = vectors_projection_ijk_range;
            weights_list_sol_inlier(~(valid_list)) = 0;
            weights_list_sol_inlier = weights_list_sol_inlier/sum(weights_list_sol_inlier);
            %===================================== RANSAC ========================================================
            disp('----total num----');
            length(index_common1)
            total_num_list(image_i) = length(index_common1);            
            disp('----valid num----');
            sum(valid_list)
            number_of_valid_list(image_i) = sum(valid_list);
            %------------------------------------------------
            weights_valid = vectors_projection_ijk_range;
            weights_valid(~(valid_list)) = 0;                      % set the invalid values to 0 
            weights_valid = weights_valid/sum(weights_valid);
            %------------------------------------------------
            if sum(valid_list)~=0
                tic;
                residual_array = zeros(length(valid_list),length(valid_list));
                %residual_array2 = zeros(length(valid_list),length(valid_list));
                for ind_a = 1:length(valid_list)
                    if valid_list(ind_a)
                        for ind_b = 1:length(valid_list)
                            if valid_list(ind_b) && ind_b~=ind_a
                                coord_i = sonar_image_list{image_i}{index_common1(ind_b)}.coordinate;
                                coord_j = sonar_image_list{image_j}{index_common2(ind_b)}.coordinate;
                                coord_k = sonar_image_list{image_k}{index_common3(ind_b)}.coordinate;
                                range_measurements_2 = [sum((coord_i).^2),sum((coord_j).^2),sum((coord_k).^2)]; 
                                residual_array(ind_a,ind_b) = (abs(double(subs(lambda_c_equation_list_separate{ind_b}{1},lambda_c,sol_i_get(ind_a))))/range_measurements_2(1)/2 +...
                                                            abs(double(subs(lambda_c_equation_list_separate{ind_b}{2},lambda_c,sol_i_get(ind_a))))/range_measurements_2(2)/2 +...
                                                            abs(double(subs(lambda_c_equation_list_separate{ind_b}{3},lambda_c,sol_i_get(ind_a))))/range_measurements_2(3)/2)/3;                                
%                                 %--------------------------------------------------------
%                                 for ind_c = valid_constraints{ind_b}
%                                     if ~compute_two_scale_flag                                
%                                         residual_array2(ind_a,ind_b) = residual_array2(ind_a,ind_b) + abs(double(subs(lambda_c_equation_list_separate{ind_b}{ind_c},lambda_c,sol_i_get(ind_a))))/range_measurements_2(ind_c)/2;
%                                     elseif compute_two_scale_flag == 1
%                                         residual_array2(ind_a,ind_b) = residual_array2(ind_a,ind_b) + abs(double(subs(lambda_c_equation_list_separate{ind_b}{ind_c},[lambda_c1,lambda_c2],[sol_i_get(ind_a),sol_k_get(ind_a)])))/range_measurements_2(ind_c)/2;
%                                     end
%                                 end
%                                 residual_array2(ind_a,ind_b) = residual_array2(ind_a,ind_b)/length(valid_constraints{ind_b});
%                                 %---------------------------------------------------------
                            end
                        end
                    end
                end
                residual_thres = 0.001;
                residual_array_valid = residual_array(logical(valid_list),logical(valid_list));
                inlier_number_array = residual_array_valid < residual_thres;
                while max(sum(inlier_number_array,2)) <  0.6 * number_of_valid_list(image_i)    %0.6 * total_num_list(image_i)
                    residual_thres = residual_thres + 0.001;
                    inlier_number_array = residual_array_valid < residual_thres;
                end
                [~,ind_row] = max(sum(inlier_number_array,2));
                residual_inlier_flag = residual_array(logical(valid_list),:);
                residual_inlier_flag = residual_inlier_flag(ind_row,:);
                %--------------------------ransac index----------------------------
                ransac_inlier_idx = (residual_inlier_flag < residual_thres).* valid_list;
                %------------------------------------------------------------------
                residual_thres
                disp('--------------- total time: 1-point ransac ----------------');
                toc;
                residual_thres_ransac1_list(image_i) = residual_thres;
                disp('----ransac valid num----');
                sum(ransac_inlier_idx)
                ransac_valid_num_list(image_i) = sum(ransac_inlier_idx);
                %--------------------------------------------------------------                
                weights_list_sol_ransac_inlier = vectors_projection_ijk_range;
                weights_list_sol_ransac_inlier(~(ransac_inlier_idx)) = 0;
                weights_list_sol_ransac_inlier = weights_list_sol_ransac_inlier/sum(weights_list_sol_ransac_inlier);
% % %                 %====================================================================================================                                  
% % %                 residual_thres = 0.01;
% % %                 residual_array_valid = residual_array2(logical(valid_list),logical(valid_list));
% % %                 inlier_number_array = residual_array_valid < residual_thres;
% % %                 while max(sum(inlier_number_array,2)) < 0.6 * number_of_valid_list(image_i)     %total_num_list(image_i))
% % %                     residual_thres = residual_thres + 0.01;
% % %                     inlier_number_array = residual_array_valid < residual_thres;
% % %                 end
% % %                 [~,ind_row] = max(sum(inlier_number_array,2));
% % %                 residual_inlier_flag = residual_array2(logical(valid_list),:);
% % %                 residual_inlier_flag = residual_inlier_flag(ind_row,:);
% % %                 %--------------------------ransac2 index----------------------------
% % %                 ransac_inlier_idx2 = (residual_inlier_flag < residual_thres).* valid_list;
% % %                 %-------------------------------------------------------------------
% % %                 residual_thres
% % %                 residual_thres_ransac2_list(image_i) = residual_thres;
% % %                 disp('----ransac2 valid num----');
% % %                 sum(ransac_inlier_idx2)
% % %                 ransac2_valid_num_list(image_i) = sum(ransac_inlier_idx2);
% % %                 %------------------------------------------------------------------------------------           
% % %                 weights_list_sol_ransac_inlier2 = vectors_projection_ijk_range;
% % %                 weights_list_sol_ransac_inlier2(~(ransac_inlier_idx2)) = 0;
% % %                 weights_list_sol_ransac_inlier2 = weights_list_sol_ransac_inlier2/sum(weights_list_sol_ransac_inlier2);
                %====================================================================================
                for ind_inlier = 1:length(index_common1)
                    if compute_add_equ_flag
                        if weights_list_sol_inlier(ind_inlier)~=0                              % inlier_index_range_ransac 
                            if ~isempty(lambda_c_equation_weights)
                                lambda_c_equation_weights = lambda_c_equation_weights + weights_list_sol_inlier(ind_inlier)*lambda_c_equation_list{ind_inlier};
                            else
                                lambda_c_equation_weights = weights_list_sol_inlier(ind_inlier)*lambda_c_equation_list{ind_inlier};                            
                            end                                     
                        end
                    end
                    %----------------------------------------------------------------------------------------------------------------------------------------
                    if weights_list_sol_inlier(ind_inlier)~=0
                        result_addition_w = result_addition_w +  weights_list_sol_inlier(ind_inlier)*sol_i_get(ind_inlier);
                    end
                    
                    %-------------------------------------------------------------------------------------------------------
                    if compute_add_equ_flag
                        if weights_list_sol_ransac_inlier(ind_inlier)~=0                              % inlier_index_range_ransac 
                            if ~isempty(lambda_c_equation_weights_ransac)
                                lambda_c_equation_weights_ransac = lambda_c_equation_weights_ransac + weights_list_sol_ransac_inlier(ind_inlier)*lambda_c_equation_list{ind_inlier};
                            else
                                lambda_c_equation_weights_ransac = weights_list_sol_ransac_inlier(ind_inlier)*lambda_c_equation_list{ind_inlier};                            
                            end                                     
                        end
                    end
                    %----------------------------------------------------------------------------------------------------------------------------------------
                    if weights_list_sol_ransac_inlier(ind_inlier)~=0
                        result_addition_ransac_inlier_w = result_addition_ransac_inlier_w +  weights_list_sol_ransac_inlier(ind_inlier)*sol_i_get(ind_inlier);
                    end
                end    
                result_addition_w_list(image_i) = result_addition_w;
                result_addition_ransac_inlier_w_list(image_i) = result_addition_ransac_inlier_w;
                %-------------------------------------------------------------------------------------------------------
                if disp_result_and_error
                    disp('-----------------------result weight --------------------------');
                    disp(result_addition_w_list(image_i));
                    disp('-----------------------error--------------------------');
                    disp(abs(scale_list(image_i+1) - result_addition_w_list(image_i)));
                %-----------------------------------------------
                    disp('-----------------------result weight ransac threshold inlier--------------------------');
                    disp(result_addition_ransac_inlier_w_list(image_i));
                    disp('-----------------------error--------------------------');
                    disp(abs(scale_list(image_i+1) - result_addition_ransac_inlier_w_list(image_i)));
                end
% %                 if noise_test
% %                     %--------------plot--------------------
% %                     h1 = figure;
% %                     hold on;
% %                     for ind_plot = 1:length(error_list)
% %                         if valid_list(ind_plot)
% %                             plot(vectors_projection_ijk_range(ind_plot),error_list(ind_plot),'r.');
% %                         end
% %                     end
% %                     title(['Pose ',num2str(image_i)]);
% %                     ylim = get(gca,'Ylim');
% %                     hold off;
% %                     if approximate_plane == 0
% %                         savefig(h1,[pwd,'\figure_save_plane\',num2str(noise_number),'\valid\Pose_',num2str(image_i),'.fig']);
% %                     elseif approximate_plane == 1
% %                         savefig(h1,[pwd,'\figure_save_plane\',num2str(noise_number),'\valid\Pose_',num2str(image_i),'_approximate.fig'])
% %                     end
% %                     close(h1);
% %                     %----------------------------------------------------------------------------
% %                     h2 = figure;
% %                     hold on;
% %                     for ind_plot = 1:length(error_list)
% %                         if ransac_inlier_idx(ind_plot)
% %                             plot(vectors_projection_ijk_range(ind_plot),error_list(ind_plot),'r.');
% %                         end
% %                     end
% %                     title(['Pose ',num2str(image_i),' ransac inlier']);
% %                     hold off;
% %                     if approximate_plane == 0
% %                         savefig(h2,[pwd,'\figure_save_plane\',num2str(noise_number),'\ransac\Pose_',num2str(image_i),'.fig'])
% %                     elseif approximate_plane == 1
% %                         savefig(h2,[pwd,'\figure_save_plane\',num2str(noise_number),'\ransac\Pose_',num2str(image_i),'_approximate.fig'])
% %                     end
% %                     close(h2);
% % % % %                     %-----------------------------------------------------------------------------
% % % % %                     h3 = figure;
% % % % %                     hold on;
% % % % %                     for ind_plot = 1:length(error_list)
% % % % %                         if ransac_inlier_idx2(ind_plot)
% % % % %                             plot(vectors_projection_ijk_range(ind_plot),error_list(ind_plot),'r.');
% % % % %                         end
% % % % %                     end
% % % % %                     title(['Pose ',num2str(image_i),' ransac seperate inlier']);
% % % % %                     hold off;
% % % % %                     savefig(h3,[pwd,'\figure_save\',subfolder,num2str(noise_number),'\ransac2\Pose_',num2str(image_i),'.fig'])
% % % % %                     close(h3);
% % % % %                     %----------------------------------------------------------------------------  
% %                 end
                %----------------------------------------------------------------------------------          
                if  ~isempty(lambda_c_equation_weights_ransac)&&compute_add_equ_flag
                    exist_flag = 1;
                    %----------------------------------------------------------------------------------
                    lambda_c_sol_range_weights_ransac = double(solve(lambda_c_equation_weights_ransac == 0,lambda_c));
                    plus_ind = (lambda_c_sol_range_weights_ransac>0).*(imag(lambda_c_sol_range_weights_ransac)==0);
                    if sum(plus_ind)==1
                        scale_estimated_weights_ransac(image_i) = lambda_c_sol_range_weights_ransac(logical(plus_ind));
                    elseif sum(plus_ind)==2
                        [~,ind_min] = min(abs(lambda_c_sol_range_weights_ransac - [1,1]*result_addition_ransac_inlier_w_list(image_i)));
                        scale_estimated_weights_ransac(image_i) = lambda_c_sol_range_weights_ransac(ind_min);
                    else
                        exist_flag = 0;
                        disp('No proper solution! weights + ransac.');
                        flag_ransac_weights_list(image_i) = 0;
                        scale_estimated_weights_ransac(image_i) = 0;
                    end
                    if disp_result_and_error && exist_flag
                        disp('---------------------------initial output--------------------------');
                        disp('-----weights + ransac-----')
                        disp(scale_estimated_weights_ransac(image_i));
                        disp('--------------------------------error------------------------------');
                        disp(abs(scale_list(image_i+1)-scale_estimated_weights_ransac(image_i)));
                    end
                    disp('=========================weight + ransac ini obtained===============================');
                end
                %----------------------------------------------------------------------------------
                if  ~isempty(lambda_c_equation_weights)&&compute_add_equ_flag
                    exist_flag = 1;
                    %----------------------------------------------------------------------------------
                    lambda_c_sol_range_weights = double(solve(lambda_c_equation_weights == 0,lambda_c));
                    plus_ind = (lambda_c_sol_range_weights>0).*(imag(lambda_c_sol_range_weights)==0);
                    if sum(plus_ind)==1
                        scale_estimated_weights(image_i) = lambda_c_sol_range_weights(logical(plus_ind));
                    elseif sum(plus_ind)==2
                        [~,ind_min] = min(abs(lambda_c_sol_range_weights - [1,1]*result_addition_w_list(image_i)));
                        scale_estimated_weights(image_i) = lambda_c_sol_range_weights(ind_min);
                    else
                        exist_flag = 0;
                        disp('No proper solution! weights + ransac.');
                        flag_weights_list(image_i) = 0;
                        scale_estimated_weights(image_i) = 0;
                    end
                    if disp_result_and_error && exist_flag
                        disp('---------------------------initial output--------------------------');
                        disp('-----weights-----')
                        disp(scale_estimated_weights(image_i));
                        disp('--------------------------------error------------------------------');
                        disp(abs(scale_list(image_i+1)-scale_estimated_weights(image_i)));
                    end
                    disp('------------------------- weights ini obtained -------------------------');
                end
                %------------------------------------------------------------------------------------------------------------------------------           
                if logical(optimization_flag)
                    disp('========================= optimization(one unknowns)=========================');
                    options = optimoptions('lsqnonlin','Display','iter','Algorithm','levenberg-marquardt');
                    %--------------------------------------------------------------------------------------------------------------------------
                    %==========================================================================================================================
                    disp('------------------ optimization --------------------');
                    if flag_ransac_weights_list(image_i)
                        ini_r_w = scale_estimated_weights_ransac(image_i);
                    else
                        ini_r_w = result_addition_ransac_inlier_w_list(image_i);
                    end                    
                    %=====================================same initial different object functions =============================================
                    %--------------------------------------------------------------------------------------------------------------------------
                    tic;
                    [output_refined_all,~,~,~,~] = lsqnonlin(@(variable)error_func_2020_BA_one(variable,correspondences_sonar,coordinates_3d_lambda_list,...
                        R_i2j,t_i2j,R_k2j,t_k2j,ones(1,length(index_common1)),vectors_projection_ijk_range,0),...
                        ini_r_w,[],[],options);
                    disp('time: all points + BA + RW_ini');
                    time_optimization_all(image_i) = toc;
                    scale_estimated_refined_all_BA_RW(image_i) = output_refined_all;                   
                    %---------------------------------------------------------------------
                    tic;
                    [output_refined_valid,~,~,~,~] = lsqnonlin(@(variable)error_func_2020_BA_one(variable,correspondences_sonar,coordinates_3d_lambda_list,...
                        R_i2j,t_i2j,R_k2j,t_k2j,valid_list,vectors_projection_ijk_range,0),...
                        ini_r_w,[],[],options);
                    disp('time: valid + no weights + BA + RW_ini');
                    time_optimization_v(image_i) = toc;
                    scale_estimated_refined_valid_BA_RW(image_i) = output_refined_valid;
                    %---------------------------------------------------------------------                                  
                    tic;
                    [output_refined_ransac,~,~,~,~] = lsqnonlin(@(variable)error_func_2020_BA_one(variable,correspondences_sonar,coordinates_3d_lambda_list,...
                        R_i2j,t_i2j,R_k2j,t_k2j,ransac_inlier_idx,vectors_projection_ijk_range,0),...
                        ini_r_w,[],[],options);
                    disp('time: ransac + no weights + BA + RW_ini');
                    time_optimization_r(image_i) = toc;
                    scale_estimated_refined_ransac_BA_RW(image_i) = output_refined_ransac;
                    %---------------------------------------------------------------------
                    tic;
                    [output_refined_weights,~,~,~,~] = lsqnonlin(@(variable)error_func_2020_BA_one(variable,correspondences_sonar,coordinates_3d_lambda_list,...
                        R_i2j,t_i2j,R_k2j,t_k2j,valid_list,vectors_projection_ijk_range,1),...
                        ini_r_w,[],[],options);
                    disp('time: valid + weights + BA + RW_ini');
                    time_optimization_w(image_i) = toc;
                    scale_estimated_refined_weights_BA_RW(image_i) = output_refined_weights;
                    %-------------------------------------------------------------------------------
                    tic;
                    [output_refined_weights_ransac,~,~,~,~] = lsqnonlin(@(variable)error_func_2020_BA_one(variable,correspondences_sonar,coordinates_3d_lambda_list,...
                        R_i2j,t_i2j,R_k2j,t_k2j,ransac_inlier_idx,vectors_projection_ijk_range,1),...
                        ini_r_w,[],[],options);
                    disp('time: ransac + weights + BA + RW_ini');
                    time_optimization_r_w(image_i) = toc;
                    scale_estimated_refined_weights_ransac_BA(image_i) = output_refined_weights_ransac;
                    %----------------------------------------------------------------------------------------------------------------------
                    tic;
                    [output_refined_weights_ransac,~,~,~,~] = lsqnonlin(@(variable)error_func_2020_inlier_weights_one(variable,lambda_c_equation_list_separate,valid_constraints,ransac_inlier_idx,vectors_projection_ijk_range,1,0),...
                                                    ini_r_w,[],[],options);
                    disp('------- total time: ransac + weights optimization ---------');
                    time_optimization(image_i) = toc;
                    scale_estimated_refined_weights_ransac(image_i) = output_refined_weights_ransac;
                    %----------------------------------------------------------------------------------------------------------------------
                    disp('=============================output================================');
                    disp('-----------------------refine weights ransac output-----------------------');
                    disp(scale_estimated_refined_weights_ransac(image_i));
                    disp('--------------------------------error------------------------------');
                    disp(abs(scale_list(image_i+1)-scale_estimated_refined_weights_ransac(image_i)));
                    disp('===================================================================');
                    %======================================================================================================================
                    if flag_weights_list(image_i)
                        ini_w = scale_estimated_weights(image_i);
                    else
                        ini_w = result_addition_ransac_inlier_w_list(image_i);
                    end
                    tic;
                    [output_refined_all,~,~,~,~] = lsqnonlin(@(variable)error_func_2020_BA_one(variable,correspondences_sonar,coordinates_3d_lambda_list,...
                        R_i2j,t_i2j,R_k2j,t_k2j,ones(1,length(index_common1)),vectors_projection_ijk_range,0),...
                        ini_w,[],[],options);
                    disp('time: all points + BA + W_ini');
                    time_optimization_all_W(image_i) = toc;
                    scale_estimated_refined_all_BA_W(image_i) = output_refined_all;                   
                    %---------------------------------------------------------------------
                    tic;
                    [output_refined_valid,~,~,~,~] = lsqnonlin(@(variable)error_func_2020_BA_one(variable,correspondences_sonar,coordinates_3d_lambda_list,...
                        R_i2j,t_i2j,R_k2j,t_k2j,valid_list,vectors_projection_ijk_range,0),...
                        ini_w,[],[],options);
                    disp('time: valid + no weights + BA + W_ini');
                    time_optimization_v_W(image_i) = toc;
                    scale_estimated_refined_valid_BA_W(image_i) = output_refined_valid;
                    %---------------------------------------------------------------------                                  
                    tic;
                    [output_refined_ransac,~,~,~,~] = lsqnonlin(@(variable)error_func_2020_BA_one(variable,correspondences_sonar,coordinates_3d_lambda_list,...
                        R_i2j,t_i2j,R_k2j,t_k2j,ransac_inlier_idx,vectors_projection_ijk_range,0),...
                        ini_w,[],[],options);
                    disp('time: ransac + no weights + BA + W_ini');
                    time_optimization_r_W(image_i) = toc;
                    scale_estimated_refined_ransac_BA_W(image_i) = output_refined_ransac;
                    %---------------------------------------------------------------------
                    tic;
                    [output_refined_weights,~,~,~,~] = lsqnonlin(@(variable)error_func_2020_BA_one(variable,correspondences_sonar,coordinates_3d_lambda_list,...
                        R_i2j,t_i2j,R_k2j,t_k2j,valid_list,vectors_projection_ijk_range,1),...
                        ini_w,[],[],options);
                    disp('time: valid + weights + BA + W_ini');
                    time_optimization_w_W(image_i) = toc;
                    scale_estimated_refined_weights_BA_W(image_i) = output_refined_weights;
                    %-------------------------------------------------------------------------------
                    tic;
                    [output_refined_weights_ransac,~,~,~,~] = lsqnonlin(@(variable)error_func_2020_BA_one(variable,correspondences_sonar,coordinates_3d_lambda_list,...
                        R_i2j,t_i2j,R_k2j,t_k2j,ransac_inlier_idx,vectors_projection_ijk_range,1),...
                        ini_w,[],[],options);
                    disp('time: ransac + weights + BA + W_ini');
                    time_optimization_r_w_W(image_i) = toc;
                    scale_estimated_refined_weights_ransac_BA_W(image_i) = output_refined_weights_ransac;
                    %----------------------------------------------------------------------------------------------------------------------
                    tic;
                    [output_refined_weights_ransac,~,~,~,~] = lsqnonlin(@(variable)error_func_2020_inlier_weights_one(variable,lambda_c_equation_list_separate,valid_constraints,valid_list,vectors_projection_ijk_range,1,0),...
                                                    ini_w,[],[],options);
                    disp('------- total time: weights optimization ---------');
                    time_optimization_object_W(image_i) = toc;
                    scale_estimated_refined_weights_object_W(image_i) = output_refined_weights_ransac;
                    %----------------------------------------------------------------------------------------------------------------------
                end               
            else
                disp('No proper correspondence for scale estimation!');
                valid_poses(image_i) = 3;
            end
        else
            disp('No correspondence among three sonar views!');
            valid_poses(image_i) = 0;
        end
        disp(['------- total time Pose_',num2str(image_j-1), ' ---------']);
        t_end = clock;
        time_pose(image_j-1) = etime(t_end,t_start);
        disp([num2str(time_pose(image_j-1)),' s.']);
    elseif image_j > start_frame 
        disp(['----------------------Pose_',num2str(image_j-1),'-----------------------------']);
        % flag_essential_get == 0
        if valid_poses(image_j) == 2    % the pose between j and k can't be computed  
            disp('there is no enough match points for Homography matrix estimation.');
            disp('can not estimate the absolute scale, for there is not enough match points in color images.');
        end
        if valid_poses(image_j-1) == 2 % the pose between i and j can't be computed 
            disp('Can not estimate the absolute scale, for the previous pose (one of the two relative pose among three viewpoints) cannot be computed.');
        end
    end

end
disp('-----ground truth-----')
scale_list(start_frame+1:end_frame-1+1)
scale_ratio_gt = zeros(1,end_frame-1-1);
for ind_ratio = start_frame+1:end_frame-1-1+1     %pose number == end_frame - 1 ,ratio number == pose_number -1
    scale_ratio_gt(ind_ratio-1) = scale_list(ind_ratio + 1)/scale_list(ind_ratio);
end
if disp_result_and_error       
    disp('-----weights + ransac----')
    scale_estimated_weights_ransac(start_frame:end_frame-1-1)
    error_abs = abs(scale_estimated_weights_ransac(start_frame:end_frame-1-1)-scale_list(start_frame+1:end_frame-1-1+1));
    %----------------------
    error_ransac_weights = error_abs;
    %----------------------
    disp('--average--')
    sum(error_abs)/length(error_abs)
    disp('--max value--')
    max(error_abs)
    figure;
    %plot(scale_ratio_gt(start_frame:end_frame-1),error_abs,'r.');
    plot(start_frame:end_frame-1-1,error_abs,'b.');
    title('weights + ransac');
    %=============================================================================================================
    disp('-----addition results ransac weight-----')
    result_addition_ransac_inlier_w_list(start_frame:end_frame-1-1)
    disp('--average--')
    sum(abs(result_addition_ransac_inlier_w_list(start_frame:end_frame-1-1)-scale_list(start_frame+1:end_frame-1-1+1)))/(end_frame-1-1-start_frame+1)    
end
if logical(optimization_flag)    
    disp('-----refined weights + ransac -----')
    scale_estimated_refined_weights_ransac(start_frame:end_frame-1-1)
    error_abs = abs(scale_estimated_refined_weights_ransac(start_frame:end_frame-1-1)-scale_list(start_frame+1:end_frame-1-1+1));
    disp('--average--')
    sum(error_abs)/length(error_abs)
    disp('--error sort--')
    sort(error_abs,'descend')
end
total_num_list        
number_of_valid_list
ransac_valid_num_list
if noise_test
    if approximate_plane == 0
        save([pwd,'\estimated_results_plane\',num2str(noise_number),'\three_view_method_BA\estimated_result_three_views_start(',num2str(start_frame),')_end(',num2str(end_frame-1),')_laptop1.mat'],...
        'start_frame','end_frame','scale_estimated_weights','scale_estimated_weights_ransac','scale_estimated_refined_weights_ransac','scale_estimated_refined_all_BA_RW','scale_estimated_refined_valid_BA_RW',...
        'scale_estimated_refined_ransac_BA_RW','scale_estimated_refined_weights_BA_RW','scale_estimated_refined_weights_ransac_BA','result_addition_w_list','result_addition_ransac_inlier_w_list',...
        'error_ransac_weights','total_num_list','number_of_valid_list','ransac_valid_num_list','time_point_list','time_pose','time_optimization',...
        'time_optimization_all','time_optimization_v','time_optimization_r','time_optimization_w','time_optimization_r_w',...
        'scale_estimated_refined_all_BA_W','scale_estimated_refined_valid_BA_W','scale_estimated_refined_ransac_BA_W',...
    'scale_estimated_refined_weights_BA_W','scale_estimated_refined_weights_ransac_BA_W','scale_estimated_refined_weights_object_W',...
    'time_optimization_all_W','time_optimization_v_W','time_optimization_r_W','time_optimization_w_W','time_optimization_r_w_W','time_optimization_object_W');
    elseif approximate_plane == 1
        save([pwd,'\estimated_results_plane\',num2str(noise_number),'\three_view_method_BA\estimated_result_three_views_start(',num2str(start_frame),')_end(',num2str(end_frame-1),')_approximate_laptop1.mat'],...
        'start_frame','end_frame','scale_estimated_weights','scale_estimated_weights_ransac','scale_estimated_refined_weights_ransac','scale_estimated_refined_all_BA_RW','scale_estimated_refined_valid_BA_RW',...
        'scale_estimated_refined_ransac_BA_RW','scale_estimated_refined_weights_BA_RW','scale_estimated_refined_weights_ransac_BA','result_addition_w_list','result_addition_ransac_inlier_w_list',...
        'error_ransac_weights','total_num_list','number_of_valid_list','ransac_valid_num_list','time_point_list','time_pose','time_optimization',...
        'time_optimization_all','time_optimization_v','time_optimization_r','time_optimization_w','time_optimization_r_w',...
        'scale_estimated_refined_all_BA_W','scale_estimated_refined_valid_BA_W','scale_estimated_refined_ransac_BA_W',...
    'scale_estimated_refined_weights_BA_W','scale_estimated_refined_weights_ransac_BA_W','scale_estimated_refined_weights_object_W',...
    'time_optimization_all_W','time_optimization_v_W','time_optimization_r_W','time_optimization_w_W','time_optimization_r_w_W','time_optimization_object_W')
    end
end
disp('======================================== end ==================================================');





