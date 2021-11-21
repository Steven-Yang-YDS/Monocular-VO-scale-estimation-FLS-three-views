clc;
close all;
clear;
format long;
%--------------------------------------------------------------------------------------------------------
re_estimate_camera_pose = 0;
use_preCompute_pose = 1;
use_R2017b = 0;
%parentDir = '..';
%--------------------------------------------------------------------------------------------------------
addpath([pwd,'\auxiliary_subfunc'],[pwd,'\PointClasses_subfunc'],[pwd,'\PoseClasses_subfunc'],[pwd,'\plotAxesAndPoints_subfunc']);
%--------------------------------------------------------------------------------------------------------
focalLength = 0.01*5;                              % focal length in world units                      
focalxy = focalLength*[ 20 , 20 ]*1000;            % a two-element vector, [fx, fy].the number of pixels per world unit in the x and y direction respectively.
principalPoint = [ 960 , 540 ];                    % a two-element vector, [cx,cy], in pixels.
CameraMatFun = [principalPoint(1);0;focalxy(1);
                0;principalPoint(2);focalxy(2);
                                         0;0;1];

Orientation = cell(2,1);
Location = cell(2,1);
%--------------------------------------------------
noise_number = 3;
%--------------------------------------------------
if noise_number<10
    str_noise_num = ['0',num2str(noise_number)];
elseif mod(noise_number,10) == 0
    str_noise_num = num2str(noise_number/10);
else
    str_noise_num = num2str(noise_number);
end  
load([parentDir,'\ground_truth_data\ground_truth_200poses.mat']);
load([parentDir,'\input_data\sensor_measurements_noise_(mean_0_sigma_0.0',str_noise_num,')_200poses.mat']);
camera_image_list = camera_image_list_noise;
camera_image_list_pixel = camera_image_list_pixel_noise; 

start_frame = 1;
end_frame = 201;
%estimated_pose_j_k = cell(1,end_frame);
estimated_pose_j_k_opencv = cell(1,end_frame);
estimated_pose_j_k_BA = cell(1,end_frame);
mask_list =  cell(1,end_frame);
pointNum_list_camera = zeros(1,end_frame);
for image_j = start_frame:end_frame
    disp(['---------------------- Pose_',num2str(image_j),' ----------------------'])
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
    pointNum = length(common_jk);
    pointNum_list_camera(image_j) = pointNum;
    if pointNum >=5 
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
%         [E_mat, inliers_c] = estimateEssentialMatrix(input_1, input_2, camera_intrinsics);  %%

        points1 = cell(1,pointNum);%zeros(length(index_a),2);
        points2 = cell(1,pointNum);%zeros(length(index_b),2);    
        for ind1 = 1:pointNum%length(index_a)
            points1{ind1} = camera_image_list{image_j}{index_a2(ind1)}.coordinate.';
        end
        for ind2 = 1:pointNum%length(index_b)
            points2{ind2} = camera_image_list{image_k}{index_b2(ind2)}.coordinate.';
        end        
        %-----------------------------------------------------------------------------------------------------------------%
%         points1_inliers = points1(logical(inliers_c));
%         points2_inliers = points2(logical(inliers_c));
%         pointsMat1 = cell2mat(points1_inliers.');
%         pointsMat2 = cell2mat(points2_inliers.');
%         % ------------------------ using normalized Point coordinates -------------------
%         IntrinsicMatrix = [1,0,0;0,1,0;0,0,1];
%         radialDistortion = [0,0];
%         cameraParamsEye = cameraParameters('IntrinsicMatrix',IntrinsicMatrix,'RadialDistortion',radialDistortion); 
%         [orientation,location] = relativeCameraPose(E_mat,cameraParamsEye,pointsMat1, pointsMat2);
%         % ------------------------------------------------------------------------
%         % orientation and location describe the pose of frame2 relative to frame1.
%         % In Matlab, points coordinates are expressed with row vector (P1 = P2R + t).
%         % so it needs to transform to column vector and use the transform from P1 to P2.
%         % -------------------------------- R, t transforming ----------------------------------------
%         R_jk_inter_func = orientation;
%         t_jk_inter_func = - orientation * location.';        
        %------------------------------------------------------------------------------------------------------------------%%%
        if use_preCompute_pose == 0
            [E_mat_opencv, mask] = cv.findEssentialMat(input_1, input_2, 'Method','Ransac','CameraMatrix',camera_intrinsics.IntrinsicMatrix.');
            points1_mask = points1(logical(mask));
            points2_mask = points2(logical(mask));
            [R_jk_opencv,t_jk_opencv, ~, ~] = cv.recoverPose(E_mat_opencv, points1_mask, points2_mask);          
        %-------------------------------------------------------------------%
%         disp('-------- error of camera estimated R ----------' );
%         disp(sum(sum(abs(R_jk_inter_func-R_list{image_j+1}))));
%         disp('---------------- estimated t ------------------' );
%         disp(t_jk_inter_func);
%         disp('-----------------------------------------------' );
        %--------------------------------------------------------------------%%
        disp('-------- OPENCV error of camera estimated R ----------' );
        disp(sum(sum(abs(R_jk_opencv-R_list{image_j+1}))));
        disp('---------------- OPENCV estimated t ------------------' );
        disp(t_jk_opencv);
        disp('-----------------------------------------------' );
        %---------------------------------------------------------
        % estimated_pose_j_k{image_j} = [R_jk_inter_func,t_jk_inter_func/norm(t_jk_inter_func);0,0,0,1];  % save the estimated pose
            estimated_pose_j_k_opencv{image_j} = [R_jk_opencv,t_jk_opencv/norm(t_jk_opencv);0,0,0,1];  % save the estimated pose
            mask_list{image_j} = mask;
        elseif use_preCompute_pose == 1
            load([pwd,'\intermediate_results\preComputePose.mat']);
            mask = mask_list{image_j};    
            R_jk_opencv = estimated_pose_j_k_opencv{image_j}(1:3,1:3);
            t_jk_opencv = estimated_pose_j_k_opencv{image_j}(1:3,4);
        end
        %--------------------------------- refine with BA -----------------------------------------
        xyzPoints = zeros(sum(mask),3);
        points1_mask_pix = input_1(logical(mask),:);
        points2_mask_pix = input_2(logical(mask),:);
        for index_common = 1:size(points1_mask_pix,1)
            coord_c_j = points1_mask_pix(index_common,:);
            coord_c_k = points2_mask_pix(index_common,:);
            P_j = (camera_intrinsics.IntrinsicMatrix.')\[coord_c_j(1);coord_c_j(2);1];
            P_k = (camera_intrinsics.IntrinsicMatrix.')\[coord_c_k(1);coord_c_k(2);1];

            A_jk = cross_mat(P_j)*[eye(3),[0;0;0]];
            B_jk = cross_mat(P_k)*estimated_pose_j_k_opencv{image_j}(1:3,:);
            C_jk = [A_jk(1:2,:);...
                    B_jk(1:2,:)];        
            [~,~,V_jk] = svd(C_jk);
            position_3D_e_augment_jk = V_jk(:,end);   
            position_3D_e_jk = position_3D_e_augment_jk(1:3)/position_3D_e_augment_jk(4);
            if position_3D_e_jk(3) < 0
               position_3D_e_jk = -position_3D_e_jk;
            end
            xyzPoints(index_common,:) = position_3D_e_jk.';
            pointTracks(index_common) = pointTrack([image_j,image_k],[coord_c_j;coord_c_k]); 
        end
        ViewId = [uint32(image_j);uint32(image_k)];
         % the R and t is the transformation from camera to world   
        if use_R2017b == 1
            Orientation{1} = eye(3);      % R2017b
            Location{1} = [0,0,0];
            Orientation{2} = eul2rotm(rotm2eul(R_jk_opencv));      % R2017b
            Location{2} = -(t_jk_opencv/norm(t_jk_opencv)).'*R_jk_opencv;        
            cameraPoses = table(ViewId,Orientation,Location);
        elseif use_R2017b == 0                                     % R2020b
            AbsolutePose(1,1) = rigid3d(eul2rotm(rotm2eul(eye(3))),[0,0,0]);
            AbsolutePose(2,1) = rigid3d(eul2rotm(rotm2eul(R_jk_opencv)), -(t_jk_opencv/norm(t_jk_opencv)).'*R_jk_opencv);
            cameraPoses = table(ViewId,AbsolutePose);
        end
        [xyzRefinedPoints,refinedPoses] = bundleAdjustment(xyzPoints,pointTracks,cameraPoses,camera_intrinsics);%,'PointsUndistorted',true); 
        if use_R2017b == 1                                         % R2020b
            R_jk_BA = refinedPoses.Orientation{2};
            t_jk_BA = -(refinedPoses.Location{2}*refinedPoses.Orientation{2}).';
            t_jk_BA = t_jk_BA/norm(t_jk_BA);
        elseif use_R2017b == 0                                     % R2020b
            R_jk_BA = refinedPoses.AbsolutePose(2).Rotation;
            t_jk_BA = -(refinedPoses.AbsolutePose(2).Translation*refinedPoses.AbsolutePose(2).Rotation).';
            t_jk_BA = t_jk_BA/norm(t_jk_BA);
        end
        estimated_pose_j_k_BA{image_j} = [R_jk_BA,t_jk_BA;0,0,0,1];
        %--------------------------------------------------------------------%%
        disp('-------- error of Bundle Adjustment refined R ----------' );
        disp(sum(sum(abs(R_jk_BA-R_list{image_j+1}))));
        disp('---------------- Bundle Adjustment refined t ------------------' );
        disp(t_jk_BA);
        disp('-----------------------------------------------' );
        %---------------------------------------------------------
        clear pointTracks;
    end
end
if use_preCompute_pose == 0
    save([pwd,'\intermediate_results\preComputePose.mat'],'estimated_pose_j_k_opencv','mask_list');
end
if re_estimate_camera_pose == 1
    save([pwd,'\camera_pose_estimation\noise_',num2str(noise_number),'_camera_pose_perspective_errors_start(',num2str(start_frame),')_end(',num2str(end_frame-1),').mat'],...
        'estimated_pose_j_k_opencv');
end

load([parentDir,'\ground_truth_data\ground_truth_200poses.mat']);
R_gt = R_list;
error_R_opencv = 0;
error_t_opencv = 0;
error_R_BA = 0;  %
error_t_BA = 0;  %
for i=1:length(estimated_pose_j_k_opencv)   
    error_R_opencv = error_R_opencv + sum(sum(abs(estimated_pose_j_k_opencv{i}(1:3,1:3)-R_gt{i+1})));
    error_t_opencv = error_t_opencv + sum(abs(estimated_pose_j_k_opencv{i}(1:3,4)-[0;0;-1])); 
    error_R_BA = error_R_BA + sum(sum(abs(estimated_pose_j_k_BA{i}(1:3,1:3)-R_gt{i+1})));  %
    error_t_BA = error_t_BA + sum(abs(estimated_pose_j_k_BA{i}(1:3,4)-[0;0;-1]));  %
end
error_R_opencv
error_t_opencv
error_R_BA  
error_t_BA  
