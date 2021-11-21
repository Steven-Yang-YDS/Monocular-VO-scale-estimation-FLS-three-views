clc;
close all;
clear;
warning off;
format long;
%--------------------------------------------------------------------------------------------------------
%--------------------------------------------------------------------------------------------------------
addpath([pwd,'\auxiliary_subfunc'],[pwd,'\PointClasses_subfunc'],[pwd,'\PoseClasses_subfunc'],[pwd,'\plotAxesAndPoints_subfunc']);
%-----------------------------------------------------------------------------------------------------------------
noise_test = 1;                % whether to add sonar image noise 
noise_number = 1;              % noise level   1~15mm
%--------------------------------------------------------
camera_noise = 1;              % whether to add camera image noise, noise level is set to be sigma == 1 pixel
camera_noise_fixed = 1;        % whether use one and the same set of noise for 15 sets camera image data ?
fix_noise_set_num = 3;
%--------------------------------------------------------
use_pre_compute_pose = 1;
preComputeNum = 3;
%--------------------------------------------------------
optimization_flag = 1;
compute_add_equ_flag = 1;
show_noise = 0;
show_correspondences = 0;
disp_result_and_error = 1;
%--------------------------------------------------------------------------------------------------------------------------------
if use_pre_compute_pose ==1
    if camera_noise == 1
        load([pwd,'\camera_pose_estimation\noise_',num2str(preComputeNum),'_camera_pose_perspective_errors_start(1)_end(200).mat']);
        estimated_pose_j_k_pre = estimated_pose_j_k_opencv;
    elseif camera_noise == 0
        estimated_pose_j_k_pre = cell(1,201);
        load([pwd,'\ground_truth_data\ground_truth_200poses.mat']);
        for i = 1:201
            estimated_pose_j_k_pre{i} = [R_list{i+1},[0;0;-1];0,0,0,1];
        end   
    end
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

load([pwd,'\ground_truth_data\ground_truth_200poses.mat']);

if noise_test == 1      
    if noise_number<10
        str_noise_num = ['0',num2str(noise_number)];
    elseif mod(noise_number,10) == 0
        str_noise_num = num2str(noise_number/10);
    else
        str_noise_num = num2str(noise_number);
    end        
    load([pwd,'\input_data\sensor_measurements_200poses.mat']);
    if logical(show_noise)
        camera_image_list_truth = camera_image_list;
        sonar_image_list_truth = sonar_image_list;
    end
    load([pwd,'\input_data\sensor_measurements_noise_(mean_0_sigma_0.0',str_noise_num,')_200poses.mat']);    
    sonar_image_list = sonar_image_list_noise;
    if camera_noise == 1 
        if camera_noise_fixed == 1
            if fix_noise_set_num<10
                str_fix_noise_num = ['0',num2str(fix_noise_set_num)];
            elseif mod(fix_noise_set_num,10) == 0
                str_fix_noise_num = num2str(fix_noise_set_num/10);
            else
                str_fix_noise_num = num2str(fix_noise_set_num);
            end
            load([pwd,'\input_data\sensor_measurements_noise_(mean_0_sigma_0.0',str_fix_noise_num,')_200poses.mat'],...
                'camera_image_list_noise','camera_image_list_pixel_noise');
        end
        camera_image_list = camera_image_list_noise;
        camera_image_list_pixel = camera_image_list_pixel_noise;        
    end
else
    load([pwd,'\input_data\sensor_measurements_200poses.mat']);
end
%=============================================================================
start_frame = 1;
end_frame = 200;  %length(R_list)-2;           %length(R_list)-3;
valid_poses = ones(1,end_frame);               %the n-th valid flag indicates that the validation of pose from n to n+1
% meaning: 0 -- no correspondences among three sonar views
%          1 -- the three-view method works
%          2 -- one of the two relative pose is unsolvable for no correspondence between color images
%          3 -- the correspondences in three sonar views contain heavily noises and are not proper data.
%=============================================================================
estimated_pose_j_k = cell(1,end_frame);
%-----------------------------------------------------------------
scale_estimated_valid = zeros(1,end_frame);
scale_estimated_range_ransac = zeros(1,end_frame);
scale_estimated_valid_w = zeros(1,end_frame);
scale_estimated_weights_ransac = zeros(1,end_frame);
%--------------------------------------------------------
scale_estimated_valid_seperate = zeros(1,end_frame);
scale_estimated_range_ransac2 = zeros(1,end_frame);
scale_estimated_valid_seperate_w = zeros(1,end_frame);
scale_estimated_weights_ransac2 = zeros(1,end_frame);
%--------------------------------------------------------
%--------------------------------------------------------
result_addition_valid_list = zeros(1,end_frame);
result_addition_ransac_inlier_list = zeros(1,end_frame);
result_addition_ransac_inlier_list2 = zeros(1,end_frame);
result_addition_valid_w_list = zeros(1,end_frame);
result_addition_ransac_inlier_w_list = zeros(1,end_frame);
result_addition_ransac_inlier_w_list2 = zeros(1,end_frame);
% % %--------------------------------------------------------
scale_estimated_refined_valid = zeros(1,end_frame);
scale_estimated_refined_ransac = zeros(1,end_frame);
scale_estimated_refined_weights = zeros(1,end_frame);
scale_estimated_refined_weights_ransac = zeros(1,end_frame);

scale_estimated_refined_valid_addition = zeros(1,end_frame);
scale_estimated_refined_ransac_addition = zeros(1,end_frame);
scale_estimated_refined_weights_addition = zeros(1,end_frame);
scale_estimated_refined_weights_ransac_addition = zeros(1,end_frame);

scale_estimated_refined_valid_BA = zeros(1,end_frame);
scale_estimated_refined_ransac_BA = zeros(1,end_frame);
scale_estimated_refined_weights_BA = zeros(1,end_frame);
scale_estimated_refined_weights_ransac_BA = zeros(1,end_frame);
scale_estimated_refined_all_BA_RW = zeros(1,end_frame);
scale_estimated_refined_valid_BA_RW = zeros(1,end_frame);
scale_estimated_refined_ransac_BA_RW = zeros(1,end_frame);
scale_estimated_refined_weights_BA_RW = zeros(1,end_frame);
%-----------------------------------------------------------------
pointNum_list_camera = zeros(1,end_frame-1);
total_num_list = zeros(1,end_frame-1);        
number_of_valid_list = zeros(1,end_frame-1);
ransac_valid_num_list = zeros(1,end_frame-1);

residual_thres_ransac_list = zeros(1,end_frame-1);

flag_valid_list = ones(1,end_frame-1);
flag_ransac_list = ones(1,end_frame-1);
flag_valid_weights_list = ones(1,end_frame-1);
flag_ransac_weights_list = ones(1,end_frame-1);

ratio_list = zeros(1,end_frame-1);        
offset_x1 = 2 * bound_x(2);
offset_x2 = 4 * bound_x(2);
for image_j = start_frame:end_frame
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
    else
        image_k = image_j + 1;
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
 %------------------------------plot_correspondences_end------------------------------
    end
 %---------------------------------------------------------------------------------------
    pointNum = length(common_jk);
    pointNum_list_camera(image_j) = pointNum;
    if pointNum >=5       % if do not satisfy the minimum needs of point number 
        corres_j = zeros(3,length(common_jk));
        corres_k = zeros(3,length(common_jk));
        corres_j_pixel = zeros(length(common_jk),2);
        corres_k_pixel = zeros(length(common_jk),2);
        for ind_match_line = 1:length(common_jk)

            corres_j(1,ind_match_line) = corres_jk_epipo(ind_match_line,1);
            corres_j(2,ind_match_line) = corres_jk_epipo(ind_match_line,2);
            corres_j(3,ind_match_line) = 1;
            corres_k(1,ind_match_line) = corres_kj_epipo(ind_match_line,1);
            corres_k(2,ind_match_line) = corres_kj_epipo(ind_match_line,2);
            corres_k(3,ind_match_line) = 1;

            corres_j_pixel(ind_match_line,1) = camera_image_list_pixel{image_j}{index_a2(ind_match_line)}.coordinate(1);
            corres_j_pixel(ind_match_line,2) = camera_image_list_pixel{image_j}{index_a2(ind_match_line)}.coordinate(2);
            corres_k_pixel(ind_match_line,1) = camera_image_list_pixel{image_k}{index_b2(ind_match_line)}.coordinate(1);
            corres_k_pixel(ind_match_line,2) = camera_image_list_pixel{image_k}{index_b2(ind_match_line)}.coordinate(2);

        end
    %-----------------------removed codes------------------------
    %input_corres_list = cat(1,corres_j,corres_k);
    %input = input_corres_list(:,1:8);
    %E = fundamental_fit8(input);
    %------------------------------------------------------------    
        input_1 = corres_j_pixel(1:pointNum,:);
        input_2 = corres_k_pixel(1:pointNum,:);
        [E_mat, inliers_c] = estimateEssentialMatrix(input_1, input_2, camera_intrinsics);
        if sum(inliers_c) >= 5
            if use_pre_compute_pose == 0
                points1 = cell(1,pointNum);%zeros(length(index_a),2);
                points2 = cell(1,pointNum);%zeros(length(index_b),2);    

                for ind1 = 1:pointNum%length(index_a)
                    points1{ind1} = camera_image_list{image_j}{index_a2(ind1)}.coordinate.';
                end
                for ind2 = 1:pointNum%length(index_b)
                    points2{ind2} = camera_image_list{image_k}{index_b2(ind2)}.coordinate.';
                end        
                %----------------------------------------------------------------------------
                points1_inliers = points1(logical(inliers_c));
                points2_inliers = points2(logical(inliers_c));
                pointsMat1 = cell2mat(points1_inliers.');
                pointsMat2 = cell2mat(points2_inliers.');
                % ------------------------ using normalized Point coordinates -------------------
                IntrinsicMatrix = [1,0,0;0,1,0;0,0,1];
                radialDistortion = [0,0];
                cameraParamsEye = cameraParameters('IntrinsicMatrix',IntrinsicMatrix,'RadialDistortion',radialDistortion); 
                [orientation,location] = relativeCameraPose(E_mat,cameraParamsEye,pointsMat1, pointsMat2);
                % ------------------------------------------------------------------------
                % orientation and location describe the pose of frame2 relative to frame1.
                % In Matlab, points coordinates are expressed with row vector (P1 = P2R + t).
                % so it needs to transform to column vector and use the transform from P1 to P2.
                % -------------------------------- R, t transforming ----------------------------------------
                R_jk_inter_func = orientation;
                t_jk_inter_func = - orientation * location.';        
                %--------------------------------------------------------------------------------------------
        %        [E_mat, mask] = cv.findEssentialMat(points1, points2, 'Method','Ransac');
        %        [R_jk,t_jk, ~, ~] = cv.recoverPose(E_mat, points1, points2);        
        %         sum(sum(abs(R_jk-R_list{2})))
                %---------------------------------------------------------
                disp('-------- error of camera estimated R ----------' );
                disp(sum(sum(abs(R_jk_inter_func-R_list{image_j+1}))));
                disp('---------------- estimated t ------------------' );
                disp(t_jk_inter_func);
                disp('-----------------------------------------------' );
                %---------------------------------------------------------
                estimated_pose_j_k{image_j} = [R_jk_inter_func,t_jk_inter_func/norm(t_jk_inter_func);0,0,0,1];  % save the estimated pose
            elseif use_pre_compute_pose == 1
                estimated_pose_j_k{image_j} = estimated_pose_j_k_pre{image_j}; 
            end
        else
            valid_poses(image_j) = 2;  
        end
        %flag_essential_get = 1;
    else        
        %flag_essential_get = 0;
        valid_poses(image_j) = 2;  
    end    
    %point_dist_list(image_j) = position_3D_e_k;
    if image_j > start_frame && valid_poses(image_j-1)~=2 && valid_poses(image_j) ~= 2  %valid_poses(image_i-1)~=2 ensure the previous essential matrix is obtained.
        disp(['----------------------Pose_',num2str(image_j-1),'-----------------------------']);
        list1_test = zeros(1,length(camera_image_list{image_i}));
        list2_test = zeros(1,length(camera_image_list{image_j}));
        list3_test = zeros(1,length(camera_image_list{image_k}));
        for index_correspond = 1:length(camera_image_list{image_i})
            list1_test(index_correspond) = camera_image_list{image_i}{index_correspond}.feature_index;
        end
        for index_correspond = 1:length(camera_image_list{image_j})
            list2_test(index_correspond) = camera_image_list{image_j}{index_correspond}.feature_index;
        end
        for index_correspond = 1:length(camera_image_list{image_k})
            list3_test(index_correspond) = camera_image_list{image_k}{index_correspond}.feature_index;
        end
        [common_test1, index_a1_test, index_b1_test] = intersect(list1_test, list2_test);
        [common_test2, index_a2_test, index_b2_test] = intersect(list2_test, list3_test);
        [common_test, ind_test1, ind_test2] = intersect(common_test1, common_test2);
        index_common1_t = index_a1_test(ind_test1);
        index_common2_t = index_b1_test(ind_test1);
        index_common3_t = index_b2_test(ind_test2);
        %sonar_compute_flag = 0;

        if ~isempty(index_common1_t)&&~isempty(index_common2_t)&&~isempty(index_common3_t)
        %--------------------------------------------------------------------------------------------------------
            pointNum = length(index_common1_t);        
            point_3d_coord_sum_ji = [0;0;0];
            for index_common = 1:pointNum
                coord_c_i = camera_image_list{image_i}{index_common1_t(index_common)}.coordinate;
                coord_c_j = camera_image_list{image_j}{index_common2_t(index_common)}.coordinate;

                P_i = [coord_c_i(1);coord_c_i(2);1];
                P_j = [coord_c_j(1);coord_c_j(2);1];

                A_ij = cross_mat(P_i)*[eye(3),[0;0;0]];
                B_ij = cross_mat(P_j)*estimated_pose_j_k{image_i}(1:3,:);
                C_ij = [A_ij(1:2,:);...
                        B_ij(1:2,:)];        
                [~,~,V_ij] = svd(C_ij);
                position_3D_e_augment_ij = V_ij(:,end);   

                position_3D_e_ij = position_3D_e_augment_ij(1:3)/position_3D_e_augment_ij(4);
                if position_3D_e_ij(3) < 0
                    position_3D_e_ij = -position_3D_e_ij;
                end
                position_3D_e_ji = estimated_pose_j_k{image_i}(1:3,1:3)*position_3D_e_ij + estimated_pose_j_k{image_i}(1:3,4);
                %position_3D_e_ji_list{index_common} = position_3D_e_ji;
                point_3d_coord_sum_ji = point_3d_coord_sum_ji + position_3D_e_ji;
            end
            sum_avg1 = point_3d_coord_sum_ji/pointNum;            
            %----------------------------------------------------------------------------------------            
            point_3d_coord_sum_jk = [0;0;0];
            %disp('%------------------------------------------------------------------------------');
            for index_common = 1:pointNum
                coord_c_j = camera_image_list{image_j}{index_common2_t(index_common)}.coordinate;
                coord_c_k = camera_image_list{image_k}{index_common3_t(index_common)}.coordinate;

                P_j = [coord_c_j(1);coord_c_j(2);1];
                P_k = [coord_c_k(1);coord_c_k(2);1];

                A_jk = cross_mat(P_j)*[eye(3),[0;0;0]];
                B_jk = cross_mat(P_k)*estimated_pose_j_k{image_j}(1:3,:);
                C_jk = [A_jk(1:2,:);...
                        B_jk(1:2,:)];        
                [~,~,V_jk] = svd(C_jk);
                position_3D_e_augment_jk = V_jk(:,end);   
                position_3D_e_jk = position_3D_e_augment_jk(1:3)/position_3D_e_augment_jk(4);

                if position_3D_e_jk(3) < 0
                    position_3D_e_jk = -position_3D_e_jk;
                end
                %position_3D_e_jk_list{index_common} = position_3D_e_jk;
                point_3d_coord_sum_jk = point_3d_coord_sum_jk + position_3D_e_jk;
            end
            sum_avg2 = point_3d_coord_sum_jk/pointNum;
            ratio_list(image_i) = norm(sum_avg1)/norm(sum_avg2);            
            %ratio_list(image_i) = norm(position_3D_e_ji)/norm(position_3D_e_jk)
            %scale_list(1)/scale_list(2)
            %sonar_compute_flag = 1;
        else
            disp('No correspondence among three camera views!');
% % %             [common_test3, index_a3_test, index_b3_test] = intersect(list1_test, list3_test);
% % %             if ~isempty(common_test3)
% % %                 corres_i = zeros(3,length(common_test3));
% % %                 corres_k = zeros(3,length(common_test3)); 
% % %                 for ind_match_line = 1:length(common_test3)
% % %                     corres_i(1,ind_match_line) = camera_image_list{image_i}{index_a3_test(ind_match_line)}.coordinate(1);
% % %                     corres_i(2,ind_match_line) = camera_image_list{image_i}{index_a3_test(ind_match_line)}.coordinate(2);
% % %                     corres_i(3,ind_match_line) = 1;
% % %                     corres_k(1,ind_match_line) = camera_image_list{image_k}{index_b3_test(ind_match_line)}.coordinate(1);
% % %                     corres_k(2,ind_match_line) = camera_image_list{image_k}{index_b3_test(ind_match_line)}.coordinate(2);
% % %                     corres_k(3,ind_match_line) = 1;
% % %                 end            
% % %                 input_corres_list = cat(1,corres_i,corres_k);
% % %                 input = input_corres_list(:,1:8);
% % %                 E = fundamental_fit8(input);
% % %                 points1 = cell(1,8);%zeros(length(index_a),2);
% % %                 points2 = cell(1,8);%zeros(length(index_b),2);
% % %                 for ind1 = 1:8%length(index_a)
% % %                     points1{ind1} = camera_image_list{image_i}{index_a(ind1)}.coordinate;
% % %                 end
% % %                 for ind2 = 1:8%length(index_b)
% % %                     points2{ind2} = camera_image_list{image_k}{index_b(ind2)}.coordinate;
% % %                 end
% % %                 [R_ik,t_ik, ~, ~] = cv.recoverPose(E, points1, points2);
% % %             else
            disp('can not compute the ratio, it needs to solve the two unknown scales!');
% % %             end
        end    
    end
%============================================ find sonar correspondences ===============================================
    %==================================================================
    %---------------w/o weights or ransac-----------------------
    lambda_c_equation_valid = [];        
    %---------------w/o ransac--------------
    lambda_c_equation_valid_w = [];
    %---------------w/o weights--------------
    lambda_c_equation_range_ransac =[];
    %---------------weights + ransac--------------
    lambda_c_equation_weights_ransac =[];
    %==================================================================
    %---------------feature number-------------    
    result_addition_valid = 0; 
    result_addition_ransac_inlier = 0;
    result_addition_valid_w = 0;
    result_addition_ransac_inlier_w = 0;
    %-----------------------------------------------
    %====================================================================
    if image_j > start_frame && valid_poses(image_j-1)~=2  && valid_poses(image_j) ~= 2   %&& sonar_compute_flag       
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
        if ~isempty(index_common1)&&~isempty(index_common2)&&~isempty(index_common3)

            vectors_projection_ijk_range = zeros(1,length(index_common1));

            sol_i_get = zeros(1,length(index_common1));
            sol_k_get = zeros(1,length(index_common1));
            valid_constraints = cell(1,length(index_common1));
            
            valid_list = ones(1,length(index_common1));
            error_list = zeros(1,length(index_common1));
            angles_inclination = zeros(1,length(index_common1));   
            
            lambda_c_equation_list_separate = cell(1,length(index_common1));
            lambda_c_equation_list = cell(1,length(index_common1));
            
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
                vectors_projection_ijk_range(sonar_feature_index) = vectors_projection_ij/range_R_k + vectors_projection_jk/range_R_i + vectors_projection_ik/range_R_j; 
                equ_plane_i = ([xs_3d;ys_3d;zs_3d]-origin_i).'*n_i ;
                equ_plane_j = ([xs_3d;ys_3d;zs_3d]-origin_j).'*n_j ;
                equ_plane_k = ([xs_3d;ys_3d;zs_3d]-origin_k).'*n_k ;
                [xs_3d_lambda, ys_3d_lambda, zs_3d_lambda] = solve([equ_plane_i==0,equ_plane_j==0,equ_plane_k==0],xs_3d,ys_3d,zs_3d);
                %-------------------------------------------------------------------------------
                coordinates_3d_lambda_list{sonar_feature_index}{1} = xs_3d_lambda;
                coordinates_3d_lambda_list{sonar_feature_index}{2} = ys_3d_lambda;
                coordinates_3d_lambda_list{sonar_feature_index}{3} = zs_3d_lambda;
                %-------------------------------------------------------------------------------
                    sol_i = double(solve((xs_3d_lambda-origin_i(1))^2 + (ys_3d_lambda-origin_i(2))^2 + (zs_3d_lambda - origin_i(3))^2 - range_R_i^2 == 0,lambda_c));
                    sol_j = double(solve(xs_3d_lambda^2 + ys_3d_lambda^2 + zs_3d_lambda^2 - range_R_j^2 == 0,lambda_c));  %%
                    sol_k = double(solve((xs_3d_lambda-origin_k(1))^2 + (ys_3d_lambda-origin_k(2))^2 + (zs_3d_lambda - origin_k(3))^2 - range_R_k^2 == 0,lambda_c));
                    sol_i = sol_i.*(imag(sol_i)==0);
                    sol_j = sol_j.*(imag(sol_j)==0);
                    sol_k = sol_k.*(imag(sol_k)==0);
                    if sum(sol_i>0 + sol_j>0 + sol_k>0)>=1                                                                %creterion 1
                        sol_candidates = [sol_i(sol_i>0),sol_j(sol_j>0),sol_k(sol_k>0)];
                        %---------------------------------------------------
                        if sum(sol_i>0)==2 ||sum(sol_j>0)==2 ||sum(sol_k>0)==2
                            disp('TWO POSITIVE SOLUTIONS!!!')
                        elseif sum(sol_i>0)==0 ||sum(sol_j>0)==0 ||sum(sol_k>0)==0
                            disp('TWO NEGATIVE SOLUTIONS!!!')
                        end
                        %-----------------------------------------------
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
                    
                    if  valid_list(sonar_feature_index)
                        vec_i_get = double(subs([xs_3d_lambda-origin_i(1),ys_3d_lambda-origin_i(2),zs_3d_lambda-origin_i(3)],lambda_c,sol_i_get(sonar_feature_index)));
                        vec_j_get = double(subs([xs_3d_lambda,ys_3d_lambda,zs_3d_lambda],lambda_c,sol_i_get(sonar_feature_index)));
                        vec_k_get = double(subs([xs_3d_lambda-origin_k(1),ys_3d_lambda-origin_k(2),zs_3d_lambda-origin_k(3)],lambda_c,sol_i_get(sonar_feature_index)));
                        vec_t_i = double(subs(origin_i,lambda_c,sol_i_get(sonar_feature_index)));      
                        vec_t_k = double(subs(origin_k,lambda_c,sol_i_get(sonar_feature_index)));    
                        angles_inclination(sonar_feature_index) = sqrt(1-(vec_i_get*vec_t_i/norm(vec_i_get)/norm(vec_t_i))^2) + sqrt(1-(vec_k_get*vec_t_k/norm(vec_k_get)/norm(vec_t_k))^2) +...
                            sqrt(1-(vec_j_get*vec_t_i/norm(vec_j_get)/norm(vec_t_i))^2) + sqrt(1-(vec_j_get*vec_t_k/norm(vec_j_get)/norm(vec_t_k))^2);
                    end
                     %------------------------------------------------------------------------------------------------------------------------                
                %-------------------------------------------------------------------------------                
                        lambda_c_equation_list_separate{sonar_feature_index}{1} = (xs_3d_lambda-origin_i(1))^2 + (ys_3d_lambda-origin_i(2))^2 + (zs_3d_lambda - origin_i(3))^2 - range_R_i^2;
                        lambda_c_equation_list_separate{sonar_feature_index}{2} = (xs_3d_lambda)^2 + (ys_3d_lambda)^2 + (zs_3d_lambda)^2 - range_R_j^2;
                        lambda_c_equation_list_separate{sonar_feature_index}{3} = (xs_3d_lambda-origin_k(1))^2 + (ys_3d_lambda-origin_k(2))^2 + (zs_3d_lambda - origin_k(3))^2 - range_R_k^2;
                        lambda_c_equation_list{sonar_feature_index} = (xs_3d_lambda)^2 + (ys_3d_lambda)^2 + (zs_3d_lambda)^2 - range_R_j^2 ...
                            + (xs_3d_lambda-origin_i(1))^2 + (ys_3d_lambda-origin_i(2))^2 + (zs_3d_lambda - origin_i(3))^2 - range_R_i^2 ...
                            + (xs_3d_lambda-origin_k(1))^2 + (ys_3d_lambda-origin_k(2))^2 + (zs_3d_lambda - origin_k(3))^2 - range_R_k^2;
               
                %-----------------------------------------------------------------------------------
                    %------------------------------------------------------------------------------------------------------------------------
             end
            %===================================== RANSAC ========================================================
            disp('----total num----');
            length(index_common1)
            total_num_list(image_i) = length(index_common1);            
            disp('----valid num----');
            sum(valid_list)
            number_of_valid_list(image_i) = sum(valid_list);
            %--------
            weights_valid = vectors_projection_ijk_range;
            weights_valid(~(valid_list)) = 0;                      % set the invalid values to 0 
            weights_valid = weights_valid/sum(weights_valid);
            %------------------------------------
            if sum(valid_list)~=0
                residual_array = zeros(length(valid_list),length(valid_list));
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
                                %-------------------------------------------------------
                                %---------------------------------------------------------
                            end
                        end
                    end
                end
                min_residual = min(min(residual_array));
                residual_thres = min_residual + 0.01;
                residual_array_valid = residual_array(logical(valid_list),logical(valid_list));
                inlier_number_array = residual_array_valid < residual_thres;
                while max(sum(inlier_number_array,2)) <  0.6 * number_of_valid_list(image_i)    %0.6 * total_num_list(image_i)
                    residual_thres = residual_thres + 0.01;
                    inlier_number_array = residual_array_valid < residual_thres;
                end
                [~,ind_row] = max(sum(inlier_number_array,2));
                residual_inlier_flag = residual_array(logical(valid_list),:);
                residual_inlier_flag = residual_inlier_flag(ind_row,:);
                %--------------------------ransac index----------------------------
                ransac_inlier_idx = (residual_inlier_flag < residual_thres).* valid_list;
                %------------------------------------------------------------------
                residual_thres
                residual_thres_ransac_list(image_i) = residual_thres;
                disp('----ransac valid num----');
                sum(ransac_inlier_idx)
                ransac_valid_num_list(image_i) = sum(ransac_inlier_idx);
                %--------------------------------------------------------------                
                weights_list_sol_ransac_inlier = vectors_projection_ijk_range;
                weights_list_sol_ransac_inlier(~(ransac_inlier_idx)) = 0;
                weights_list_sol_ransac_inlier = weights_list_sol_ransac_inlier/sum(weights_list_sol_ransac_inlier);
                %====================================================================================================                                  
                %====================================================================================================
                for ind_inlier = 1:length(index_common1)
                    if compute_add_equ_flag 
                        if valid_list(ind_inlier)
                            if ~isempty(lambda_c_equation_valid)
                                lambda_c_equation_valid = lambda_c_equation_valid +  lambda_c_equation_list{ind_inlier};
                                lambda_c_equation_valid_w = lambda_c_equation_valid_w +  weights_valid(ind_inlier)*lambda_c_equation_list{ind_inlier};
                            else
                                lambda_c_equation_valid = lambda_c_equation_list{ind_inlier};
                                lambda_c_equation_valid_w = weights_valid(ind_inlier)*lambda_c_equation_list{ind_inlier};
                            end
                        end
                        if weights_list_sol_ransac_inlier(ind_inlier)~=0                              % inlier_index_range_ransac 
                            if ~isempty(lambda_c_equation_range_ransac)                            
                                lambda_c_equation_range_ransac = lambda_c_equation_range_ransac + lambda_c_equation_list{ind_inlier};
                            else
                                lambda_c_equation_range_ransac = lambda_c_equation_list{ind_inlier};                            
                            end

                            if ~isempty(lambda_c_equation_weights_ransac)
                                lambda_c_equation_weights_ransac = lambda_c_equation_weights_ransac + weights_list_sol_ransac_inlier(ind_inlier)*lambda_c_equation_list{ind_inlier};
                            else
                                lambda_c_equation_weights_ransac = weights_list_sol_ransac_inlier(ind_inlier)*lambda_c_equation_list{ind_inlier};                            
                            end                                     
                        end
                        %----------------------------------------------------------
                    end
                    %----------------------------------------------------------------------------------------------------------------------------------------
                    if weights_valid(ind_inlier)~=0                                   %inlier_index_range(ind_inlier) && valid_list(ind_inlier)
                        result_addition_valid = result_addition_valid +  sol_i_get(ind_inlier);
                        result_addition_valid_w = result_addition_valid_w +  weights_valid(ind_inlier)*sol_i_get(ind_inlier);
                    end
                    if weights_list_sol_ransac_inlier(ind_inlier)~=0
                        result_addition_ransac_inlier = result_addition_ransac_inlier + sol_i_get(ind_inlier);
                        result_addition_ransac_inlier_w = result_addition_ransac_inlier_w +  weights_list_sol_ransac_inlier(ind_inlier)*sol_i_get(ind_inlier);
                    end                    
                end
                result_addition_valid_list(image_i) = result_addition_valid/number_of_valid_list(image_i);     
                result_addition_valid_w_list(image_i) = result_addition_valid_w;
                result_addition_ransac_inlier_list(image_i) = result_addition_ransac_inlier/ransac_valid_num_list(image_i);
                result_addition_ransac_inlier_w_list(image_i) = result_addition_ransac_inlier_w;
                %-----------------------------------------------------------------------------------
                if disp_result_and_error
                    disp('-----------------------result valid--------------------------');
                    result_addition_valid_list(image_i)
                    disp('-----------------------error--------------------------');                   
                    disp(abs(scale_list(image_i) - result_addition_valid_list(image_i))); 
                    disp('-----------------------result valid weight--------------------------');
                    result_addition_valid_w_list(image_i)
                    disp('-----------------------error--------------------------');                   
                    disp(abs(scale_list(image_i) - result_addition_valid_w_list(image_i))); 
                    
                    disp('-----------------------result ransac threshold inlier--------------------------');
                    result_addition_ransac_inlier_list(image_i)
                    disp('-----------------------error--------------------------');
                    disp(abs(scale_list(image_i) - result_addition_ransac_inlier_list(image_i)));
                    disp('-----------------------result weight ransac threshold inlier--------------------------');
                    result_addition_ransac_inlier_w_list(image_i)
                    disp('-----------------------error--------------------------');
                    disp(abs(scale_list(image_i) - result_addition_ransac_inlier_w_list(image_i)));
                end
% % %                 if noise_test
% % %                     %--------------plot--------------------
% % %                     h1 = figure;
% % %                     hold on;
% % %                     for ind_plot = 1:length(error_list)
% % %                         if valid_list(ind_plot)
% % %                             plot(vectors_projection_ijk_range(ind_plot),error_list(ind_plot),'r.');
% % %                         end
% % %                     end
% % %                     title(['Pose ',num2str(image_i)]);
% % %                     hold off;                       
% % %                %     savefig(h1,[pwd,'\figure_save\',num2str(noise_number),'\valid\Pose_',num2str(image_i),'.fig'])
% % %                     close(h1);
% % %                     %----------------------------------------------------------------------------
% % %                     h2 = figure;
% % %                     hold on;
% % %                     for ind_plot = 1:length(error_list)
% % %                         if ransac_inlier_idx(ind_plot)
% % %                             plot(vectors_projection_ijk_range(ind_plot),error_list(ind_plot),'r.');
% % %                         end
% % %                     end
% % %                     title(['Pose ',num2str(image_i),' ransac inlier']);
% % %                     hold off;
% % %                %     savefig(h2,[pwd,'\figure_save\',num2str(noise_number),'\ransac\Pose_',num2str(image_i),'.fig'])
% % %                     close(h2);
% % %                     %-----------------------------------------------------------------------------
% % %                 end
                if  ~isempty(lambda_c_equation_valid) && compute_add_equ_flag
                    exist_flag = [1,1];
                    lambda_c_sol_valid = double(solve(lambda_c_equation_valid == 0,lambda_c));                
                    plus_ind = (lambda_c_sol_valid>0).*(imag(lambda_c_sol_valid)==0);
                    if sum(plus_ind)==1
                        scale_estimated_valid(image_i) = lambda_c_sol_valid(logical(plus_ind));
                    elseif sum(plus_ind)==2
                        [~,ind_min] = min(abs(lambda_c_sol_valid - [1,1]*result_addition_valid_list(image_i)));
                        scale_estimated_valid(image_i) = lambda_c_sol_valid(ind_min);
                    else
                        exist_flag(1) = 0;
                        disp('No proper solution! valid.');
                        flag_valid_list(image_i) = 0;
                        scale_estimated_valid(image_i) = 0;
                    end
                    %----------------------------------------------------------------------------------
                    lambda_c_sol_valid_w = double(solve(lambda_c_equation_valid_w == 0,lambda_c));
                    plus_ind = (lambda_c_sol_valid_w>0).*(imag(lambda_c_sol_valid_w)==0);
                    if sum(plus_ind)==1
                        scale_estimated_valid_w(image_i) = lambda_c_sol_valid_w(logical(plus_ind));
                    elseif sum(plus_ind)==2
                        [~,ind_min] = min(abs(lambda_c_sol_valid_w - [1,1]*result_addition_valid_w_list(image_i)));
                        scale_estimated_valid_w(image_i) = lambda_c_sol_valid_w(ind_min);
                    else
                        exist_flag(2) = 0;
                        disp('No proper solution! weights + valid.');
                        flag_valid_weights_list(image_i) = 0;
                        scale_estimated_valid_w(image_i) = 0;
                    end
                    %----------------------------------------------------------------------------------                
                    if disp_result_and_error && exist_flag(1)
                        disp('---------------------------initial output--------------------------');
                        disp('-----valid-----')
                        scale_estimated_valid(image_i)
                        disp('--------------------------------error------------------------------');
                        disp(abs(scale_list(image_i)-scale_estimated_valid(image_i)));
                    end
                    if disp_result_and_error && exist_flag(2)
                        disp('---------------------------initial output--------------------------');
                        disp('-----weights + valid-----')
                        scale_estimated_valid_w(image_i)
                        disp('--------------------------------error------------------------------');                               
                        disp(abs(scale_list(image_i)-scale_estimated_valid_w(image_i)));
                    end
                end
                %----------------------------------------------------------------------------------          
                if  ~isempty(lambda_c_equation_range_ransac)&&compute_add_equ_flag
                    exist_flag = [1,1];
                    lambda_c_sol_range_ransac = double(solve(lambda_c_equation_range_ransac == 0,lambda_c));   
                    plus_ind = (lambda_c_sol_range_ransac>0).*(imag(lambda_c_sol_range_ransac)==0);
                    if sum(plus_ind)==1
                        scale_estimated_range_ransac(image_i) = lambda_c_sol_range_ransac(logical(plus_ind));
                    elseif sum(plus_ind)==2
                        [~,ind_min] = min(abs(lambda_c_sol_range_ransac - [1,1]*result_addition_ransac_inlier_list(image_i)));
                        scale_estimated_range_ransac(image_i) = lambda_c_sol_range_ransac(ind_min);
                    else
                        exist_flag(1) = 0;
                        disp('No proper solution! range + ransac.');
                        flag_ransac_list(image_i) = 0;   
                        scale_estimated_range_ransac(image_i) = 0;
                    end
                    %----------------------------------------------------------------------------------
                    lambda_c_sol_range_weights_ransac = double(solve(lambda_c_equation_weights_ransac == 0,lambda_c));
                    plus_ind = (lambda_c_sol_range_weights_ransac>0).*(imag(lambda_c_sol_range_weights_ransac)==0);
                    if sum(plus_ind)==1
                        scale_estimated_weights_ransac(image_i) = lambda_c_sol_range_weights_ransac(logical(plus_ind));
                    elseif sum(plus_ind)==2
                        [~,ind_min] = min(abs(lambda_c_sol_range_weights_ransac - [1,1]*result_addition_ransac_inlier_w_list(image_i)));
                        scale_estimated_weights_ransac(image_i) = lambda_c_sol_range_weights_ransac(ind_min);
                    else
                        exist_flag(2) = 0;
                        disp('No proper solution! weights + ransac.');
                        flag_ransac_weights_list(image_i) = 0;
                        scale_estimated_weights_ransac(image_i) = 0;
                    end
                    if disp_result_and_error && exist_flag(1)
                        disp('---------------------------initial output--------------------------');
                        disp('-----range + ransac-----')
                        scale_estimated_range_ransac(image_i)
                        disp('--------------------------------error------------------------------');                               
                        disp(abs(scale_list(image_i)-scale_estimated_range_ransac(image_i)));
                    end
                    if disp_result_and_error && exist_flag(2)
                        disp('---------------------------initial output--------------------------');
                        disp('-----weights + ransac-----')
                        scale_estimated_weights_ransac(image_i)
                        disp('--------------------------------error------------------------------');
                        disp(abs(scale_list(image_i)-scale_estimated_weights_ransac(image_i)));
                    end
                    disp('=========================initial end===============================');
                end
                %---------------------------------------------------------------------------------------------------------------                
                if logical(optimization_flag)
                    disp('========================= optimization(one unknowns)=========================');
                    options = optimoptions('lsqnonlin','Display','iter','Algorithm','levenberg-marquardt');
                    %-----------------------------------------------------------------------------------------------------------
                    disp('------------------ optimization --------------------');
                    if flag_ransac_weights_list
                        ini_r_w = scale_estimated_weights_ransac(image_i);
                    else
                        ini_r_w = result_addition_ransac_inlier_w_list(image_i);
                    end
                    tic;
                    [output_refined_valid,~,~,~,~] = lsqnonlin(@(variable)error_func_2020_inlier_weights_one(variable,lambda_c_equation_list_separate,valid_constraints,valid_list,vectors_projection_ijk_range,0,0),...
                                                    ini_r_w,[],[],options);
                    disp('time: valid + no weights');
                    toc;
                    scale_estimated_refined_valid(image_i) = output_refined_valid;                   
                    %------------------------------------------------------------
                    tic;
                    [output_refined_ransac,~,~,~,~] = lsqnonlin(@(variable)error_func_2020_inlier_weights_one(variable,lambda_c_equation_list_separate,valid_constraints,ransac_inlier_idx,vectors_projection_ijk_range,0,0),...
                                                    ini_r_w,[],[],options);
                    disp('time: ransac + no weights');
                    toc;
                    scale_estimated_refined_ransac(image_i) = output_refined_ransac;
                    %------------------------------------------------------------
                    tic;
                    [output_refined_weights,~,~,~,~] = lsqnonlin(@(variable)error_func_2020_inlier_weights_one(variable,lambda_c_equation_list_separate,valid_constraints,valid_list,vectors_projection_ijk_range,1,0),...
                                                    ini_r_w,[],[],options); 
                    disp('time: valid + weights');
                    toc;
                    scale_estimated_refined_weights(image_i) = output_refined_weights;
                    %------------------------------------------------------------
                    tic;
                    [output_refined_weights_ransac,~,~,~,~] = lsqnonlin(@(variable)error_func_2020_inlier_weights_one(variable,lambda_c_equation_list_separate,valid_constraints,ransac_inlier_idx,vectors_projection_ijk_range,1,0),...
                                                    ini_r_w,[],[],options);
                    disp('time: ransac + weights');
                    toc;
                    scale_estimated_refined_weights_ransac(image_i) = output_refined_weights_ransac;
%%%%%%%%%%%%%%%%%%%%%=====================================================================================================================
                    if flag_valid_list(image_i)
                        ini_v = scale_estimated_valid(image_i);
                    else
                        ini_v = result_addition_valid_list(image_i);
                    end
                    tic;
                    [output_refined_valid,~,~,~,~] = lsqnonlin(@(variable)error_func_2020_BA_one(variable,correspondences_sonar,coordinates_3d_lambda_list,...
                        R_i2j,t_i2j,R_k2j,t_k2j,valid_list,vectors_projection_ijk_range,0),...
                        ini_v,[],[],options);
                    disp('time: valid + no weights + BA');
                    toc;
                    scale_estimated_refined_valid_BA(image_i) = output_refined_valid;
                    %---------------------------------------------------------------------
                    if flag_ransac_list(image_i)
                        ini_r = scale_estimated_range_ransac(image_i);
                    else
                        ini_r = result_addition_ransac_inlier_list(image_i);
                    end                                        
                    tic;
                    [output_refined_ransac,~,~,~,~] = lsqnonlin(@(variable)error_func_2020_BA_one(variable,correspondences_sonar,coordinates_3d_lambda_list,...
                        R_i2j,t_i2j,R_k2j,t_k2j,ransac_inlier_idx,vectors_projection_ijk_range,0),...
                        ini_r,[],[],options);
                    disp('time: ransac + no weights + BA');
                    toc;
                    scale_estimated_refined_ransac_BA(image_i) = output_refined_ransac;
                    %---------------------------------------------------------------------
                    if flag_valid_weights_list(image_i)
                        ini_w = scale_estimated_valid_w(image_i);
                    else
                        ini_w = result_addition_valid_w_list(image_i);
                    end
                    tic;
                    [output_refined_weights,~,~,~,~] = lsqnonlin(@(variable)error_func_2020_BA_one(variable,correspondences_sonar,coordinates_3d_lambda_list,...
                        R_i2j,t_i2j,R_k2j,t_k2j,valid_list,vectors_projection_ijk_range,1),...
                        ini_w,[],[],options);
                    disp('time: valid + weights + BA');
                    toc;
                    scale_estimated_refined_weights_BA(image_i) = output_refined_weights;
                    %-------------------------------------------------------------------------------    
                    if flag_ransac_weights_list
                        ini_r_w = scale_estimated_weights_ransac(image_i);
                    else
                        ini_r_w = result_addition_ransac_inlier_w_list(image_i);
                    end
                    tic;
                    [output_refined_weights_ransac,~,~,~,~] = lsqnonlin(@(variable)error_func_2020_BA_one(variable,correspondences_sonar,coordinates_3d_lambda_list,...
                        R_i2j,t_i2j,R_k2j,t_k2j,ransac_inlier_idx,vectors_projection_ijk_range,1),...
                        ini_r_w,[],[],options);
                    disp('time: ransac + weights + BA');
                    toc;
                    scale_estimated_refined_weights_ransac_BA(image_i) = output_refined_weights_ransac;
                    %=====================================same initial different object functions =============================================
                    tic;
                    [output_refined_all,~,~,~,~] = lsqnonlin(@(variable)error_func_2020_BA_one(variable,correspondences_sonar,coordinates_3d_lambda_list,...
                        R_i2j,t_i2j,R_k2j,t_k2j,ones(1,length(index_common1)),vectors_projection_ijk_range,0),...
                        ini_r_w,[],[],options);
                    disp('time: all points + BA + RW_ini');
                    toc;
                    scale_estimated_refined_all_BA_RW(image_i) = output_refined_all;                   
                    %---------------------------------------------------------------------
                    tic;
                    [output_refined_valid,~,~,~,~] = lsqnonlin(@(variable)error_func_2020_BA_one(variable,correspondences_sonar,coordinates_3d_lambda_list,...
                        R_i2j,t_i2j,R_k2j,t_k2j,valid_list,vectors_projection_ijk_range,0),...
                        ini_r_w,[],[],options);
                    disp('time: valid + no weights + BA + RW_ini');
                    toc;
                    scale_estimated_refined_valid_BA_RW(image_i) = output_refined_valid;
                    %---------------------------------------------------------------------                                  
                    tic;
                    [output_refined_ransac,~,~,~,~] = lsqnonlin(@(variable)error_func_2020_BA_one(variable,correspondences_sonar,coordinates_3d_lambda_list,...
                        R_i2j,t_i2j,R_k2j,t_k2j,ransac_inlier_idx,vectors_projection_ijk_range,0),...
                        ini_r_w,[],[],options);
                    disp('time: ransac + no weights + BA + RW_ini');
                    toc;
                    scale_estimated_refined_ransac_BA_RW(image_i) = output_refined_ransac;
                    %---------------------------------------------------------------------
                    tic;
                    [output_refined_weights,~,~,~,~] = lsqnonlin(@(variable)error_func_2020_BA_one(variable,correspondences_sonar,coordinates_3d_lambda_list,...
                        R_i2j,t_i2j,R_k2j,t_k2j,valid_list,vectors_projection_ijk_range,1),...
                        ini_r_w,[],[],options);
                    disp('time: valid + weights + BA + RW_ini');
                    toc;
                    scale_estimated_refined_weights_BA_RW(image_i) = output_refined_weights;
                    %----------------------------------------------------------------------------------------------------------------------
                    disp('=============================output================================');
                    disp('-----------------------refine valid output-----------------------');
                    disp(scale_estimated_refined_valid(image_i));
                    disp('--------------------------------error------------------------------');
                    disp(abs(scale_list(image_i)-scale_estimated_refined_valid(image_i)));
                    disp('-----------------------refine ransac output-----------------------');
                    disp(scale_estimated_refined_ransac(image_i));
                    disp('--------------------------------error------------------------------');
                    disp(abs(scale_list(image_i)-scale_estimated_refined_ransac(image_i)));
                    disp('-----------------------refine weights output-----------------------');
                    disp(scale_estimated_refined_weights(image_i));
                    disp('--------------------------------error------------------------------');
                    disp(abs(scale_list(image_i)-scale_estimated_refined_weights(image_i)));
                    disp('-----------------------refine weights ransac output-----------------------');
                    disp(scale_estimated_refined_weights_ransac(image_i));
                    disp('--------------------------------error------------------------------');
                    disp(abs(scale_list(image_i)-scale_estimated_refined_weights_ransac(image_i)));
                    disp('===================================================================');
                    disp('-----------------------refine valid output BA-----------------------');
                    disp(scale_estimated_refined_valid_BA(image_i));
                    disp('--------------------------------error------------------------------');
                    disp(abs(scale_list(image_i)-scale_estimated_refined_valid_BA(image_i)));
                    disp('-----------------------refine ransac output BA-----------------------');
                    disp(scale_estimated_refined_ransac_BA(image_i));
                    disp('--------------------------------error------------------------------');
                    disp(abs(scale_list(image_i)-scale_estimated_refined_ransac_BA(image_i)));
                    disp('-----------------------refine weights output BA-----------------------');
                    disp(scale_estimated_refined_weights_BA(image_i));
                    disp('--------------------------------error------------------------------');
                    disp(abs(scale_list(image_i)-scale_estimated_refined_weights_BA(image_i)));
                    disp('-----------------------refine weights ransac output BA-----------------------');
                    disp(scale_estimated_refined_weights_ransac_BA(image_i));
                    disp('--------------------------------error------------------------------');
                    disp(abs(scale_list(image_i)-scale_estimated_refined_weights_ransac_BA(image_i)));
                    disp('===================================================================');
                    disp('-----------------------refine all output BA-----------------------');
                    disp(scale_estimated_refined_all_BA_RW(image_i));
                    disp('--------------------------------error------------------------------');
                    disp(abs(scale_list(image_i)-scale_estimated_refined_all_BA_RW(image_i)));
                    disp('-----------------------refine valid output BA-----------------------');
                    disp(scale_estimated_refined_valid_BA_RW(image_i));
                    disp('--------------------------------error------------------------------');
                    disp(abs(scale_list(image_i)-scale_estimated_refined_valid_BA_RW(image_i)));
                    disp('-----------------------refine ransac output BA-----------------------');
                    disp(scale_estimated_refined_ransac_BA_RW(image_i));
                    disp('--------------------------------error------------------------------');
                    disp(abs(scale_list(image_i)-scale_estimated_refined_ransac_BA_RW(image_i)));
                    disp('-----------------------refine weights output BA-----------------------');
                    disp(scale_estimated_refined_weights_BA_RW(image_i));
                    disp('--------------------------------error------------------------------');
                    disp(abs(scale_list(image_i)-scale_estimated_refined_weights_BA_RW(image_i)));
                end
                %---------------------------------------------------------------------------------------------------------------
            else
                disp('No proper correspondence for scale estimation!');
                valid_poses(image_i) = 3;
            end
        else
            disp('No correspondence among three sonar views!');
            valid_poses(image_i) = 0;
        end
    elseif image_j > start_frame 
        disp(['----------------------Pose_',num2str(image_j-1),'-----------------------------']);
        % flag_essential_get == 0
        if valid_poses(image_j) == 2    % the pose between j and k can't be computed  
            disp('there is no enough match points for Essential matrix estimation.');
            disp('can not estimate the absolute scale, for there is not enough match points in color images.');
        end
        if valid_poses(image_j-1) == 2 % the pose between i and j can't be computed 
            disp('Can not estimate the absolute scale, for the previous pose (one of the two relative pose among three viewpoints) cannot be computed.');
        end
    end
end
% % % disp('-----ground truth-----')
% % % scale_list(start_frame:end_frame-1)
% % % scale_ratio_gt = zeros(1,end_frame-1);
% % % for ind_ratio = start_frame:end_frame-1
% % %     scale_ratio_gt(ind_ratio) = scale_list(ind_ratio + 1)/scale_list(ind_ratio);
% % % end
if disp_result_and_error        
    disp('-----valid-----')
    scale_estimated_valid(start_frame:end_frame-1)
    error_abs = abs(scale_estimated_valid(start_frame:end_frame-1)-scale_list(start_frame:end_frame-1));
    %----------------------
    error_valid = error_abs;
    %----------------------
    disp('--average--')
    sum(error_abs)/(end_frame-start_frame)
    disp('--max value--')
    max(error_abs)
    figure;
    %plot(scale_ratio_gt(start_frame:end_frame-1),error_abs,'r.');
    plot(scale_list(start_frame:end_frame-1),error_abs,'b.');
    title('valid');
    %--------------------------------------------------------------------------
    disp('-----ransac-----')
    scale_estimated_range_ransac(start_frame:end_frame-1)
    error_abs = abs(scale_estimated_range_ransac(start_frame:end_frame-1)-scale_list(start_frame:end_frame-1));
    %----------------------
    error_ransac = error_abs;
    %----------------------
    disp('--average--')
    sum(error_abs)/(end_frame-start_frame)
    disp('--max value--')
    max(error_abs)
    figure;
    %plot(scale_ratio_gt(start_frame:end_frame-1),error_abs,'r.');
    plot(scale_list(start_frame:end_frame-1),error_abs,'b.');
    title('ransac');
    %--------------------------------------------------------------------------
    disp('-----weights + valid-----')
    scale_estimated_valid_w(start_frame:end_frame-1)
    error_abs = abs(scale_estimated_valid_w(start_frame:end_frame-1)-scale_list(start_frame:end_frame-1));
    %----------------------
    error_valid_weights = error_abs;
    %----------------------
    disp('--average--')
    sum(error_abs)/(end_frame-start_frame)
    disp('--max value--')
    max(error_abs)
    figure;
    %plot(scale_ratio_gt(start_frame:end_frame-1),error_abs,'r.');
    plot(scale_list(start_frame:end_frame-1),error_abs,'b.');
    title('weights + valid');
    %--------------------------------------------------------------------------
    disp('-----weights + ransac----')
    scale_estimated_weights_ransac(start_frame:end_frame-1)
    error_abs = abs(scale_estimated_weights_ransac(start_frame:end_frame-1)-scale_list(start_frame:end_frame-1));
    %----------------------
    error_ransac_weights = error_abs;
    %----------------------
    disp('--average--')
    sum(error_abs)/(end_frame-start_frame)
    disp('--max value--')
    max(error_abs)
    figure;
    %plot(scale_ratio_gt(start_frame:end_frame-1),error_abs,'r.');
    plot(scale_list(start_frame:end_frame-1),error_abs,'b.');
    title('weights + ransac');
    %=============================================================================================================
    disp('-----addition results valid-----')
    result_addition_valid_list(start_frame:end_frame-1)
    disp('--average--')
    sum(abs(result_addition_valid_list(start_frame:end_frame-1)-scale_list(start_frame:end_frame-1)))/(end_frame-start_frame)
    
    disp('-----addition results valid weight-----')
    result_addition_valid_w_list(start_frame:end_frame-1)
    disp('--average--')
    sum(abs(result_addition_valid_w_list(start_frame:end_frame-1)-scale_list(start_frame:end_frame-1)))/(end_frame-start_frame)
    
    disp('-----addition results ransac-----')
    result_addition_ransac_inlier_list(start_frame:end_frame-1)
    disp('--average--')
    sum(abs(result_addition_ransac_inlier_list(start_frame:end_frame-1)-scale_list(start_frame:end_frame-1)))/(end_frame-start_frame)
    
    disp('-----addition results ransac weight-----')
    result_addition_ransac_inlier_w_list(start_frame:end_frame-1)
    disp('--average--')
    sum(abs(result_addition_ransac_inlier_w_list(start_frame:end_frame-1)-scale_list(start_frame:end_frame-1)))/(end_frame-start_frame)    
end
if logical(optimization_flag)
    disp('-----refined valid -----')
    scale_estimated_refined_valid(start_frame:end_frame-1)
    error_abs = abs(scale_estimated_refined_valid(start_frame:end_frame-1)-scale_list(start_frame:end_frame-1));
    disp('--average--')
    sum(error_abs)/(end_frame-start_frame)
    disp('--error sort--')
    sort(error_abs,'descend')
    
    disp('-----refined ransac -----')
    scale_estimated_refined_ransac(start_frame:end_frame-1)
    error_abs = abs(scale_estimated_refined_ransac(start_frame:end_frame-1)-scale_list(start_frame:end_frame-1));
    disp('--average--')
    sum(error_abs)/(end_frame-start_frame)
    disp('--error sort--')
    sort(error_abs,'descend')
    
    disp('-----refined weights-----')
    scale_estimated_refined_weights(start_frame:end_frame-1)
    error_abs = abs(scale_estimated_refined_weights(start_frame:end_frame-1)-scale_list(start_frame:end_frame-1));
    disp('--average--')
    sum(error_abs)/(end_frame-start_frame)
    disp('--error sort--')
    sort(error_abs,'descend')
    
    disp('-----refined weights + ransac -----')
    scale_estimated_refined_weights_ransac(start_frame:end_frame-1)
    error_abs = abs(scale_estimated_refined_weights_ransac(start_frame:end_frame-1)-scale_list(start_frame:end_frame-1));
    disp('--average--')
    sum(error_abs)/(end_frame-start_frame)
    disp('--error sort--')
    sort(error_abs,'descend')    
    %===============================================================================================
end
total_num_list        
number_of_valid_list
ransac_valid_num_list     
%----------------------------------------------save----------------------------------------------------------
if noise_test
    save([pwd,'\estimated_results\',num2str(noise_number),'\estimated_initial_values_start(',num2str(start_frame),')_end(',num2str(end_frame-1),').mat'],...
    'start_frame','end_frame','scale_estimated_valid','scale_estimated_range_ransac','scale_estimated_valid_w','scale_estimated_weights_ransac');
%---------------save initial results--------------------
    save([pwd,'\estimated_results\',num2str(noise_number),'\add_result_start(',num2str(start_frame),')_end(',num2str(end_frame-1),').mat'],...
    'result_addition_valid_list','result_addition_valid_w_list','result_addition_ransac_inlier_list','result_addition_ransac_inlier_w_list');
%---------------save refined results--------------------
    save([pwd,'\estimated_results\',num2str(noise_number),'\refined_values_start(',num2str(start_frame),')_end(',num2str(end_frame-1),').mat'],...
    'scale_estimated_refined_valid','scale_estimated_refined_ransac','scale_estimated_refined_weights','scale_estimated_refined_weights_ransac',...
    'scale_estimated_refined_valid_BA','scale_estimated_refined_ransac_BA','scale_estimated_refined_weights_BA','scale_estimated_refined_weights_ransac_BA',...
    'scale_estimated_refined_all_BA_RW','scale_estimated_refined_valid_BA_RW','scale_estimated_refined_ransac_BA_RW','scale_estimated_refined_weights_BA_RW',...
    'total_num_list','number_of_valid_list','ransac_valid_num_list','residual_thres_ransac_list');
%---------------save flags------------------------------
    save([pwd,'\estimated_results\',num2str(noise_number),'\flags_start(',num2str(start_frame),')_end(',num2str(end_frame-1),').mat'],...
        'valid_poses','flag_valid_list','flag_ransac_list','flag_valid_weights_list','flag_ransac_weights_list'); 
%---------------save result errors------------------------------
    save([pwd,'\estimated_results\',num2str(noise_number),'\result_errors_start(',num2str(start_frame),')_end(',num2str(end_frame-1),').mat'],...
        'error_ransac_weights','error_valid_weights','error_ransac','error_valid');
%---------------save all------------------------------
save([pwd,'\estimated_results\',num2str(noise_number),'\all_DATA(',num2str(start_frame),')_end(',num2str(end_frame-1),').mat']);
%---------------save result errors------------------------------
    if use_pre_compute_pose == 0
        save([pwd,'\estimated_results\',num2str(noise_number),'\camera_pose_perspective_errors_start(',num2str(start_frame),')_end(',num2str(end_frame-1),').mat'],...
            'estimated_pose_j_k');
    end
end
%============================================ boxplot ========================================================
error_valid = abs(scale_estimated_valid(1:190)-scale_list(1:190)).'./scale_list(1:190).';
error_ransac = abs(scale_estimated_range_ransac(1:190)-scale_list(1:190)).'./scale_list(1:190).';
error_valid_w = abs(scale_estimated_valid_w(1:190)-scale_list(1:190)).'./scale_list(1:190).';
error_ransac_w = abs(scale_estimated_weights_ransac(1:190)-scale_list(1:190)).'./scale_list(1:190).';
figure;
boxplot([error_valid,error_ransac,error_valid_w,error_ransac_w]);
title('RANSAC');
ylim([0,0.2]);
%-----------------------------------------refined results-------------------------------------
error_refined_valid = abs(scale_estimated_refined_valid(1:190)-scale_list(1:190)).'./scale_list(1:190).';
error_refined_valid_BA = abs(scale_estimated_refined_valid_BA(1:190)-scale_list(1:190)).'./scale_list(1:190).';
error_refined_ransac = abs(scale_estimated_refined_ransac(1:190)-scale_list(1:190)).'./scale_list(1:190).';
error_refined_ransac_BA = abs(scale_estimated_refined_ransac_BA(1:190)-scale_list(1:190)).'./scale_list(1:190).';
error_refined_weights = abs(scale_estimated_refined_weights(1:190)-scale_list(1:190)).'./scale_list(1:190).';
error_refined_weights_BA = abs(scale_estimated_refined_weights_BA(1:190)-scale_list(1:190)).'./scale_list(1:190).';
error_refined_weights_ransac = abs(scale_estimated_refined_weights_ransac(1:190)-scale_list(1:190)).'./scale_list(1:190).';
error_refined_weights_ransac_BA = abs(scale_estimated_refined_weights_ransac_BA(1:190)-scale_list(1:190)).'./scale_list(1:190).';
figure;
boxplot([error_valid ,error_refined_valid,error_refined_valid_BA]);
title('valid');
figure;
boxplot([error_ransac,error_refined_ransac,error_refined_ransac_BA]);
title('RANSAC');
ylim([0,0.2]);
figure;
boxplot([error_valid_w,error_refined_weights,error_refined_weights_BA]);
title('weights');
ylim([0,0.2]);
figure;
boxplot([error_ransac_w,error_refined_weights_ransac,error_refined_weights_ransac_BA]);
title('weights + ransac');
ylim([0,0.2]);
figure;
boxplot([   error_valid,error_refined_valid_BA,...
            error_ransac,error_refined_ransac_BA,...
            error_valid_w,error_refined_weights_BA,...
            error_ransac_w,error_refined_weights_ransac_BA],'symbol','');
%ylim_pre = get(gca,'YLim');
ylim([0,0.18]);
title('compare 4 configurations and corresponding 4 object functions for optimization');
%====================================================================================================
error_refined_all_BA_RW = abs(scale_estimated_refined_all_BA_RW(1:190)-scale_list(1:190)).'./scale_list(1:190).';
error_refined_valid_BA_RW = abs(scale_estimated_refined_valid_BA_RW(1:190)-scale_list(1:190)).'./scale_list(1:190).';
error_refined_ransac_BA_RW = abs(scale_estimated_refined_ransac_BA_RW(1:190)-scale_list(1:190)).'./scale_list(1:190).';
error_refined_weights_BA_RW = abs(scale_estimated_refined_weights_BA_RW(1:190)-scale_list(1:190)).'./scale_list(1:190).';
figure;
boxplot([   error_ransac_w,...
            error_refined_all_BA_RW,...
            error_refined_valid_BA_RW,...
            error_refined_ransac_BA_RW,...
            error_refined_weights_BA_RW,...
            error_refined_weights_ransac_BA],'symbol','');
ylim_pre = get(gca,'YLim');
title('compare the same initial value under 5 object functions configurations for optimization')
disp('======================================== end ==================================================');


% error_valid1 = error_valid;
% error_ransac1 = error_ransac;
% error_valid_w1 = error_valid_w;
% error_ransac_w1 = error_ransac_w;
% boxplot([error_valid,error_valid1,error_ransac,error_ransac1,error_valid_w,error_valid_w1,error_ransac_w,error_ransac_w1])

% error_refined_valid_BA1 = error_refined_valid_BA;
% error_refined_ransac_BA1 = error_refined_ransac_BA;
% error_refined_weights_BA1 = error_refined_weights_BA;
% error_refined_weights_ransac_BA1 = error_refined_weights_ransac_BA;
