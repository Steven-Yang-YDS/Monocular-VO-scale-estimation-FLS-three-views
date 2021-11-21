clc;
close all;
clear;
format long;
%------------------------------------------------------------
addpath([pwd,'\PointClasses'],[pwd,'\PoseClasses'],[pwd,'\plotAxesAndPoints']);
%-----------------------------------------------------------------------------------------------------------------
noise_test = 1;                     % whether to add camera image noise 
fix_noise_set_num = 6;              % camera noise group  ,notice: the noises in all groups are with a noise level of sigma == 1 pixel
%--------------------------------------------------------
%load([pwd,'\ground_truth_data\ground_truth_233.mat']);
%load([pwd,'\input_data\sensor_measurements_233.mat']);
%load([pwd,'\input_data_plane\sensor_measurement_noise(mean_0_sigma_0.001)_50posesPlane.mat']);
approximate_plane = 1;
if approximate_plane == 0
    filename = '50posesPlane';
elseif approximate_plane == 1
    filename = '50posesPlane_approximate';
end
if noise_test == 0
    load([pwd,'\input_data_plane\sensor_measurements_',filename,'.mat'])
elseif noise_test == 1
    if fix_noise_set_num<10
        str_noise_num = ['0',num2str(fix_noise_set_num)];
    elseif mod(fix_noise_set_num,10) == 0
        str_noise_num = num2str(fix_noise_set_num/10);
    else
        str_noise_num = num2str(fix_noise_set_num);
    end
    load([pwd,'\input_data_plane\sensor_measurements_noise_(mean_0_sigma_0.0',str_noise_num,')_',filename,'.mat']);
    camera_image_list = camera_image_list_noise;
end
%-----------------------------extrinsic_parameters--------------------------------------------
R_c2s = eularAngle([0,0,-90]/180*pi,'zyx');
t_c2s = [0;0;-0.05];
focalLength = 0.01*5;                              % focal length in world units                      
focalxy = focalLength*[ 20 , 20 ]*1000;            % a two-element vector, [fx, fy].the number of pixels per world unit in the x and y direction respectively.
principalPoint = [ 960 , 540 ];                    % a two-element vector, [cx,cy], in pixels.
imageSize = [ 1920,1080 ];                         % Image size produced by the camera, specified as a two-element vector, [mrows,ncols]
bound_x = [-imageSize(1)/focalxy(1)/2,imageSize(1)/focalxy(1)/2];
bound_y = [-imageSize(2)/focalxy(2)/2,imageSize(2)/focalxy(2)/2];
%---------------------------------------------------------------------------------------------
show_correspondences = 1;
start_frame = 1;
end_frame = 50;    %the number of image_i should be no greater than max_value -1 (51-1)
%=================================================display matching process=======================================
%draw the corresponding point betweent image i and j
%---------------------------------------------------------------------------------------------
%valid poses: need only to consider case 2
valid_poses = ones(1,end_frame);               %the n-th valid flag indicates that the validation of pose from n to n+1
% meaning: 0 -- no correspondences among three sonar views
%          1 -- the three-view method works
%          2 -- one of the two relative pose is unsolvable for no correspondence between color images
%          3 -- the correspondences in three sonar views contain heavily noises and are not proper data.
%-----------------------------------------------------------------------------------------------
estimated_pose = cell(1,end_frame);
estimated_normal_vector = cell(1,end_frame);
ratio_list = zeros(1,end_frame-1);  
point_3D_aver_list = zeros(3,end_frame-1);

for image_i = start_frame:end_frame
    disp(['============================= pose_',num2str(image_i),' ===============================']);
    image_j = image_i + 1;
    %---------------------------------- find correspondences ------------------------------------
    offset_x = 2 * bound_x(2);
    list1 = zeros(1,length(camera_image_list{image_i}));
    list2 = zeros(1,length(camera_image_list{image_j}));
    if show_correspondences
        figure;
        set(gca,'YDir','reverse');
        axis equal;
        set(gca,'XLim',[bound_x(1),3*bound_x(2)]);
        set(gca,'YLim',bound_y);
        xlabel('x');
        ylabel('y');        
    end
    hold on;
    for index_correspond = 1:length(camera_image_list{image_i})
        if show_correspondences
            point_vec = camera_image_list{image_i}{index_correspond}.coordinate;
            plot(point_vec(1),point_vec(2),'g.');   
        end
        list1(index_correspond) = camera_image_list{image_i}{index_correspond}.feature_index;
    end
    for index_correspond = 1:length(camera_image_list{image_j})
        if show_correspondences
            point_vec = camera_image_list{image_j}{index_correspond}.coordinate;
            plot((point_vec(1)+ offset_x),point_vec(2),'m.');
        end
        list2(index_correspond) = camera_image_list{image_j}{index_correspond}.feature_index;
    end
    [common, index_a, index_b] = intersect(list1, list2);
    if show_correspondences
        plot([bound_x(1),bound_x(1)],bound_y,'k-');
        plot([bound_x(2),bound_x(2)],bound_y,'k-');
        plot([3*bound_x(2),3*bound_x(2)],bound_y,'k-');
        plot([bound_x(1),3*bound_x(2)],[bound_y(1),bound_y(1)],'k-');
        plot([bound_x(1),3*bound_x(2)],[bound_y(2),bound_y(2)],'k-');
    end
    corres_i_homo = zeros(length(common),2);
    corres_j_homo = zeros(length(common),2);

    for ind_match_line = 1:length(common)
        start_x = camera_image_list{image_i}{index_a(ind_match_line)}.coordinate(1);
        start_y = camera_image_list{image_i}{index_a(ind_match_line)}.coordinate(2);
        end_x = camera_image_list{image_j}{index_b(ind_match_line)}.coordinate(1);
        end_y = camera_image_list{image_j}{index_b(ind_match_line)}.coordinate(2);
        if show_correspondences
            plot([start_x,end_x + offset_x],[start_y,end_y],'k-');
        end
        corres_i_homo(ind_match_line,1) = start_x;
        corres_i_homo(ind_match_line,2) = start_y;
        corres_j_homo(ind_match_line,1) = end_x;
        corres_j_homo(ind_match_line,2) = end_y;
    end
    hold off;
    %------------------------------------------------------------------------------------
    %------------------------------------------------------------------------------------
    if length(common) >= 4     % need at least 4 point to compute the homography matrix
        srcPoints = corres_i_homo;
        dstPoints = corres_j_homo;
        [Homo_mat,inlier_mask] = cv.findHomography(srcPoints, dstPoints);
        pointsMat1_inlier = srcPoints(logical(inlier_mask),:);
        pointsMat2_inlier = dstPoints(logical(inlier_mask),:);

        %[transform,pointsMat1_inlier,pointsMat2_inlier,status] = estimateGeometricTransform(srcPoints, dstPoints,'projective');
    % % % % %     if status == 0
    % % % % %         Homo_mat = transform.T.';   %  transpose,matlab uses row vectors for position representation 
    % % % % %     elseif status == 1 || status == 2
    % % % % %         disp('No enough points found.');
    % % % % %     end
        if det(Homo_mat)<0
            Homo_mat = -Homo_mat;
        end    
    % % %     IntrinsicMatrix = [1,0,0;0,1,0;0,0,1];
    % % %     radialDistortion = [0,0];
    % % %     cameraParamsEye = cameraParameters('IntrinsicMatrix',IntrinsicMatrix,'RadialDistortion',radialDistortion); 
    % % %     [orientation,location] = relativeCameraPose(Homo_mat,cameraParamsEye,pointsMat1_inlier, pointsMat2_inlier);

        %h_n{image_i} = Homo_mat
        %det(Homo_mat)
        K = eye(3);
        [motions, nsols] = cv.decomposeHomographyMat(Homo_mat, K);
        %=====================validation====================================
        %R_possible = cell(1,2);
        %t_possible = cell(1,2);
        %n_possible = cell(1,2);
        point_3D_aver = zeros(3,2);

        times_count_valid =0;

        positive_num = zeros(1,length(motions.R));
        point_3D_aver_all = zeros(3,length(motions.R));
        for ind_motion = 1:length(motions.R)
            for ind_inlier = 1:length(pointsMat1_inlier)
                prod1 = [pointsMat1_inlier(ind_inlier,:),1]*motions.n{ind_motion};
                prod2 = [pointsMat2_inlier(ind_inlier,:),1]*motions.R{ind_motion}*motions.n{ind_motion};
                if prod1>0 && prod2>0
                    A = cross_mat(pointsMat1_inlier(ind_inlier,:))*[eye(3),[0;0;0]];
                    B = cross_mat(pointsMat2_inlier(ind_inlier,:))*[motions.R{ind_motion},(motions.t{ind_motion}/norm(motions.t{ind_motion}))];
                    C = [A(1:2,:);...
                        B(1:2,:)];        
                    [~,~,V] = svd(C);
                    position_3D_e = V(:,end); 
                    if position_3D_e(3)/position_3D_e(4)<0
                        disp('point is not visible.');
                    end                
                    positive_num(ind_motion) = positive_num(ind_motion) + 1;
                    point_3D_aver_all(:,ind_motion) = point_3D_aver_all(:,ind_motion) + position_3D_e(1:3)/position_3D_e(4);
                end
    % %             if position_3D_e(3)/position_3D_e(4) > 0
    % %                 positive_num(ind_motion) = positive_num(ind_motion) + 1;
    % %                 point_3D_aver_all(:,ind_motion) = point_3D_aver_all(:,ind_motion) + position_3D_e(1:3)/position_3D_e(4);
    % %             end
            end
        end
        ind_pose = 1;
        for ind_motion = 1:length(motions.R)
            if positive_num(ind_motion)/length(pointsMat1_inlier) > 0.8
                R_possible{ind_pose} = motions.R{ind_motion};
                t_possible{ind_pose} = motions.t{ind_motion}/norm(motions.t{ind_motion});
                n_possible{ind_pose} = motions.n{ind_motion};            
                point_3D_aver(:,ind_pose) = point_3D_aver_all(:,ind_motion)/positive_num(ind_motion);
                ind_pose = ind_pose + 1;
            end
        end
        value_valid_cos = zeros(1,length(R_possible));
        value_valid_dist = zeros(1,length(R_possible));
        if image_i~=1
            n_in_Nframe = estimated_pose{image_i-1}(1:3,1:3)*estimated_normal_vector{image_i-1};
            for ind_product = 1:length(R_possible)
                value_valid_cos(ind_product) = n_in_Nframe.'* n_possible{ind_product};
                value_valid_dist(ind_product) = norm(n_in_Nframe - n_possible{ind_product});
            end 
            R_possible = R_possible(value_valid_cos>0);
            t_possible = t_possible(value_valid_cos>0);
            n_possible = n_possible(value_valid_cos>0);
            value_valid_dist = value_valid_dist(value_valid_cos>0);        
            point_3D_aver = point_3D_aver(:,value_valid_cos>0);
            if length(n_possible) > 1
                [~,ind_min] = min(value_valid_dist);
                estimated_pose{image_i} = [R_possible{ind_min},t_possible{ind_min};0,0,0,1];
                estimated_normal_vector{image_i} = n_possible{ind_min}; 
                point_3D_aver = point_3D_aver(:,ind_min);
            elseif length(n_possible) == 1
                estimated_pose{image_i} = [R_possible{1},t_possible{1};0,0,0,1];
                estimated_normal_vector{image_i} = n_possible{1};
            else
                disp("no result !");
            end       
        else
            if length(n_possible)>1
                times_count = 0;
                for ind_n = 1:length(n_possible)
                    n_inS_temp = R_c2s * n_possible{ind_n};
                    if n_inS_temp(3)<0 
                        times_count = times_count +1;
                        estimated_pose{image_i} = [R_possible{ind_n},t_possible{ind_n};0,0,0,1];
                        estimated_normal_vector{image_i} = n_possible{ind_n};  
                        point_3D_aver = point_3D_aver(:,ind_n);
                    end                
                end
                if times_count == 2
                    disp("There are two solutions for initial n !");
                end  
            elseif length(n_possible)==1
                estimated_pose{image_i} = [R_possible{1},t_possible{1};0,0,0,1];
                estimated_normal_vector{image_i} = n_possible{1};
            else      % 0
                 disp("Can't find the initial n !");
            end
        end
        %------------------------------compute relative scale-------------------------------
        point_3D_aver_list(:,image_i) =  point_3D_aver;
        if image_i>1
            plane_distant_pre = (estimated_pose{image_i-1}(1:3,1:3) * estimated_normal_vector{image_i-1}).'*(estimated_pose{image_i-1}(1:3,1:3)*point_3D_aver_list(:,image_i-1) + estimated_pose{image_i-1}(1:3,4));
            plane_distant_cur = estimated_normal_vector{image_i}.' * point_3D_aver_list(:,image_i);
            ratio_list(image_i-1) = plane_distant_pre/plane_distant_cur;
        end
    else
        valid_poses(image_j) = 2;
    end
end
if noise_test == 0
    if approximate_plane == 0
        save([pwd,'\estimated_results_plane\withoutNoise\estimated_cameraPoses_planeNormals.mat'],'estimated_pose','estimated_normal_vector','point_3D_aver_list','ratio_list','valid_poses');
    elseif approximate_plane == 1
        save([pwd,'\estimated_results_plane\withoutNoise\estimated_cameraPoses_planeNormals_approximate.mat'],'estimated_pose','estimated_normal_vector','point_3D_aver_list','ratio_list','valid_poses');
    end
elseif  noise_test == 1
    if approximate_plane == 0
        save([pwd,'\estimated_results_plane\',num2str(fix_noise_set_num),'\estimated_cameraPoses_planeNormals.mat'],'estimated_pose','estimated_normal_vector','point_3D_aver_list','ratio_list','valid_poses');
    elseif approximate_plane == 1
        save([pwd,'\estimated_results_plane\',num2str(fix_noise_set_num),'\estimated_cameraPoses_planeNormals_approximate.mat'],'estimated_pose','estimated_normal_vector','point_3D_aver_list','ratio_list','valid_poses');
    end
end
% % 
% % %-------------------------------test with ground truth-------------------------------------
% % load([pwd,'\ground_truth_data_plane\ground_truth_50posesPlane.mat']);
% % 
% % abs_re = 0;
% % abs_e = 0;
% % for i=1:49%length(estimated_pose)
% %     error_re(i) = sum(sum(abs(estimated_pose{i}(1:3,1:3)-R_list{i+1})));
% %     abs_re = abs_re + error_re(i);
% %     abs_e = abs_e + norm(estimated_pose{i}(1:3,4)+t_list{i+1}/norm(t_list{i+1}));
% % end
% % abs_re
% % abs_e
% % ratio_gt = zeros(1,length(scale_list)-1);
% % ratio_gt2 = zeros(1,length(scale_list)-1);
% % 
% % for i = 1:49
% %     ratio_gt(i)=scale_list(i+1)/scale_list(i);
% %     ratio_gt2(i) = norm(t_list{i+1})/norm(t_list{i});
% % end
% % ratio_gt
% % ratio_gt2
% % ratio_list

%============================================ find sonar correspondences ===============================================
% % % %     list1_s = zeros(1,length(sonar_image_list{image_i}));
% % % %     list2_s = zeros(1,length(sonar_image_list{image_j}));
% % % %     figure;
% % % %     %set(gca,'YDir','reverse');
% % % %     axis equal;
% % % %     %set(gca,'XLim',[bound_x(1),3*bound_x(2)]);
% % % %     %set(gca,'YLim',bound_y);
% % % %     xlabel('x'),ylabel('y')
% % % %     hold on;
% % % %     for index_correspond = 1:length(sonar_image_list{image_i})
% % % %         point_vec = sonar_image_list{image_i}{index_correspond}.coordinate;
% % % %         plot(point_vec(1),point_vec(2),'g.');   
% % % %         list1_s(index_correspond) = sonar_image_list{image_i}{index_correspond}.feature_index;
% % % %     end
% % % %     for index_correspond = 1:length(sonar_image_list{image_j})
% % % %         point_vec = sonar_image_list{image_j}{index_correspond}.coordinate;
% % % %         plot((point_vec(1)+ offset_x),point_vec(2),'m.');
% % % %         list2_s(index_correspond) = sonar_image_list{image_j}{index_correspond}.feature_index;
% % % %     end
% % % %     [common_s, index_a_s, index_b_s] = intersect(list1_s, list2_s);
% % % %     
% % % %     corres_i_homo_s = zeros(length(common_s),2);
% % % %     corres_j_homo_s = zeros(length(common_s),2);
% % % % 
% % % %     for ind_match_line = 1:length(common_s)
% % % %         start_x = sonar_image_list{image_i}{index_a_s(ind_match_line)}.coordinate(1);
% % % %         start_y = sonar_image_list{image_i}{index_a_s(ind_match_line)}.coordinate(2);
% % % %         end_x = sonar_image_list{image_j}{index_b_s(ind_match_line)}.coordinate(1);
% % % %         end_y = sonar_image_list{image_j}{index_b_s(ind_match_line)}.coordinate(2);
% % % %         plot([start_x,end_x + offset_x],[start_y,end_y],'k-');
% % % % 
% % % %         corres_i_homo_s(ind_match_line,1) = start_x;
% % % %         corres_i_homo_s(ind_match_line,2) = start_y;
% % % %         corres_j_homo_s(ind_match_line,1) = end_x;
% % % %         corres_j_homo_s(ind_match_line,2) = end_y;
% % % %     end
% % % %     hold off;
% % % % %     srcPoints_s = corres_i_homo_s(1:4,:);
% % % % %     dstPoints_s = corres_j_homo_s(1:4,:);
% % % % %     Homo_mat_s = cv.findHomography(srcPoints_s, dstPoints_s);
% % % % %     if det(Homo_mat_s)<0
% % % % %         Homo_mat_s = -Homo_mat_s;
% % % % %     end
% % % %     sonar_point_num = 3;
% % % %     
% % % %     range_R1 = norm(corres_i_homo_s(sonar_point_num,:));
% % % %     sin_theta1 = corres_i_homo_s(sonar_point_num,1)/range_R1;
% % % %     cos_theta1 = corres_i_homo_s(sonar_point_num,2)/range_R1;
% % % %     range_R2 = norm(corres_j_homo_s(sonar_point_num,:));
% % % %     sin_theta2 = corres_j_homo_s(sonar_point_num,1)/range_R2;
% % % %     cos_theta2 = corres_j_homo_s(sonar_point_num,2)/range_R2;
% % % %     %================================================================================================    
% % % %     R_c = estimated_pose{image_i}.R_mat;
% % % %     t_c_hat = estimated_pose{image_i}.t_vector/norm(estimated_pose{image_i}.t_vector);
% % % %     n_c_hat = estimated_pose{image_i}.n_vector/norm(estimated_pose{image_i}.n_vector);
% % % %     A = cross_mat(srcPoints(1,:))*[eye(3),[0;0;0]];
% % % %     B = cross_mat(dstPoints(1,:))*[R_c,t_c_hat];
% % % %     C = [A(1:2,:);...
% % % %          B(1:2,:)];        
% % % %     [~,~,V_test] = svd(C);
% % % %     position_3D_e_augment = V_test(:,end);
% % % %     position_3D_e_eular_pre = position_3D_e_augment(1:3)/position_3D_e_augment(4);
% % % %     position_3D_e_eular_cur = R_c * position_3D_e_eular_pre + t_c_hat;
% % % %     distance_pre = position_3D_e_eular_pre.'*n_c_hat;    %the distance from camera origin to the plane when the magnitude ||t_o||=1
% % % %     distance_cur = position_3D_e_eular_cur.'*(R_c * n_c_hat);
% % % % %     if distance <0
% % % % %         distance = -distance;
% % % % %     end
% % % %     syms scale_c k_s1 k_s2
% % % %     %------------------------------------------------------------------------------------------------
% % % %     k_c1 = 1/(scale_c*distance_pre);              % the scale of the plane normal in camera frame
% % % %     k_c2 = 1/(scale_c*distance_cur);
% % % %     %k_s1 = k_c1/(1+k_c1*t_c2s.'*R_c2s*n_c_hat);
% % % %     %k_s2 = k_c2/(1+k_c2*t_c2s.'*R_c2s*R_c*n_c_hat);
% % % %     %-------------------------------------------------------------------------------------------------  
% % % %     %scale_c1 = solve(k_s1 == k_c1/(1+k_c1*t_c2s.'*(R_c2s*n_c_hat)),scale_c); %用k_s1表示scale_c
% % % %     k_s1_scale = solve(k_s1 == k_c1/(1+k_c1*t_c2s.'*(R_c2s*n_c_hat)),k_s1);
% % % %     k_s2_scale = solve(k_s2 == k_c2/(1+k_c2*t_c2s.'*(R_c2s*(R_c*n_c_hat))),k_s2);
% % % %     R_s = R_c2s*R_c*R_c2s.';
% % % %     t_s = t_c2s - R_s * t_c2s + R_c2s * t_c_hat * scale_c;                  %t_s包含 k_s1
% % % %     n_s_hat1 = R_c2s*n_c_hat;
% % % %     n_s_hat2 = R_c2s*R_c*n_c_hat;
% % % %     n_sx1 = n_s_hat1(1);
% % % %     n_sy1 = n_s_hat1(2);
% % % %     n_sz1 = n_s_hat1(3);
% % % %     n_sx2 = n_s_hat2(1);
% % % %     n_sy2 = n_s_hat2(2);
% % % %     n_sz2 = n_s_hat2(3);
% % % %     Q = R_s + k_s1 * t_s * n_s_hat1.';
% % % %     solution_list = zeros(1,4);
% % % %     syms sin_phi1 sin_phi2
% % % %     sin_phi1_k_s1_solutions = solve(k_s1 * ((n_sx1 * sin_theta1 + n_sy1 * cos_theta1) * sqrt(1-sin_phi1^2) + n_sz1 * sin_phi1) * range_R1 == 1,sin_phi1);  %用k_s1表示sin_phi1
% % % %     sin_phi2_k_s2_solutions = solve(k_s2 * ((n_sx2 * sin_theta2 + n_sy2 * cos_theta2) * sqrt(1-sin_phi2^2) + n_sz2 * sin_phi2) * range_R2 == 1,sin_phi2);  %用k_s2表示sin_phi2
% % % %     %--------------------------solution1--------------------------------
% % % %     sin_phi1_k_s1 = sin_phi1_k_s1_solutions(1);
% % % %     sin_phi2_k_s2 = sin_phi2_k_s2_solutions(1);
% % % %     cos_phi1_k_s1 = sqrt(1-sin_phi1_k_s1^2);
% % % %     cos_phi2_k_s2 = sqrt(1-sin_phi2_k_s2^2);
% % % %     eq_row1 = [cos_phi1_k_s1/cos_phi2_k_s2*Q(1,1),cos_phi1_k_s1/cos_phi2_k_s2*Q(1,2),range_R1*sin_phi1_k_s1/cos_phi2_k_s2*Q(1,3)]*[corres_i_homo_s(sonar_point_num,1);corres_i_homo_s(sonar_point_num,2);1]-corres_j_homo_s(sonar_point_num,1);
% % % %     equ = subs(eq_row1,[k_s1,k_s2],[k_s1_scale,k_s2_scale]);
% % % %     solution_list(1) = vpasolve(equ,scale_c);
% % % %     %--------------------------solution2--------------------------------
% % % %     sin_phi1_k_s1 = sin_phi1_k_s1_solutions(1);
% % % %     sin_phi2_k_s2 = sin_phi2_k_s2_solutions(2);
% % % %     cos_phi1_k_s1 = sqrt(1-sin_phi1_k_s1^2);
% % % %     cos_phi2_k_s2 = sqrt(1-sin_phi2_k_s2^2);
% % % %     eq_row1 = [cos_phi1_k_s1/cos_phi2_k_s2*Q(1,1),cos_phi1_k_s1/cos_phi2_k_s2*Q(1,2),range_R1*sin_phi1_k_s1/cos_phi2_k_s2*Q(1,3)]*[corres_i_homo_s(sonar_point_num,1);corres_i_homo_s(sonar_point_num,2);1]-corres_j_homo_s(sonar_point_num,1);
% % % %     equ = subs(eq_row1,[k_s1,k_s2],[k_s1_scale,k_s2_scale]);
% % % %     solution_list(2) = vpasolve(equ,scale_c);
% % % %     %--------------------------solution3--------------------------------
% % % %     sin_phi1_k_s1 = sin_phi1_k_s1_solutions(2);
% % % %     sin_phi2_k_s2 = sin_phi2_k_s2_solutions(1);
% % % %     cos_phi1_k_s1 = sqrt(1-sin_phi1_k_s1^2);
% % % %     cos_phi2_k_s2 = sqrt(1-sin_phi2_k_s2^2);
% % % %     eq_row1 = [cos_phi1_k_s1/cos_phi2_k_s2*Q(1,1),cos_phi1_k_s1/cos_phi2_k_s2*Q(1,2),range_R1*sin_phi1_k_s1/cos_phi2_k_s2*Q(1,3)]*[corres_i_homo_s(sonar_point_num,1);corres_i_homo_s(sonar_point_num,2);1]-corres_j_homo_s(sonar_point_num,1);
% % % %     equ = subs(eq_row1,[k_s1,k_s2],[k_s1_scale,k_s2_scale]);
% % % %     solution_list(3) = vpasolve(equ,scale_c);
% % % %     %--------------------------solution4--------------------------------
% % % %     sin_phi1_k_s1 = sin_phi1_k_s1_solutions(2);
% % % %     sin_phi2_k_s2 = sin_phi2_k_s2_solutions(2);
% % % %     cos_phi1_k_s1 = sqrt(1-sin_phi1_k_s1^2);
% % % %     cos_phi2_k_s2 = sqrt(1-sin_phi2_k_s2^2);
% % % %     eq_row1 = [cos_phi1_k_s1/cos_phi2_k_s2*Q(1,1),cos_phi1_k_s1/cos_phi2_k_s2*Q(1,2),range_R1*sin_phi1_k_s1/cos_phi2_k_s2*Q(1,3)]*[corres_i_homo_s(sonar_point_num,1);corres_i_homo_s(sonar_point_num,2);1]-corres_j_homo_s(sonar_point_num,1);
% % % %     equ = subs(eq_row1,[k_s1,k_s2],[k_s1_scale,k_s2_scale]);
% % % %     solution_list(4) = vpasolve(equ,scale_c);
% % % %     %--------------------------------------------------------------------
% % % %     disp(['-------------------scale_',num2str(image_i),'----------------------']);
% % % %     multi_solution_count = 0;
% % % %     for solution_index = 1:4
% % % %         if imag(solution_list(solution_index)) == 0
% % % %             if solution_list(solution_index)>0
% % % %                 k_s1_solution = double(subs(k_s1_scale,scale_c,solution_list(solution_index)));
% % % %                 k_s2_solution = double(subs(k_s2_scale,scale_c,solution_list(solution_index)));
% % % %                 switch solution_index
% % % %                     case  1
% % % %                         sin_phi1_solution = double(subs(sin_phi1_k_s1_solutions(1),k_s1,k_s1_solution));
% % % %                         sin_phi2_solution = double(subs(sin_phi2_k_s2_solutions(1),k_s2,k_s2_solution));
% % % %                     case  2
% % % %                         sin_phi1_solution = double(subs(sin_phi1_k_s1_solutions(1),k_s1,k_s1_solution));
% % % %                         sin_phi2_solution = double(subs(sin_phi2_k_s2_solutions(2),k_s2,k_s2_solution));
% % % %                     case  3
% % % %                         sin_phi1_solution = double(subs(sin_phi1_k_s1_solutions(2),k_s1,k_s1_solution));
% % % %                         sin_phi2_solution = double(subs(sin_phi2_k_s2_solutions(1),k_s2,k_s2_solution));
% % % %                     case  4
% % % %                         sin_phi1_solution = double(subs(sin_phi1_k_s1_solutions(2),k_s1,k_s1_solution));
% % % %                         sin_phi2_solution = double(subs(sin_phi2_k_s2_solutions(2),k_s2,k_s2_solution));
% % % %                 end            
% % % %                 if sin_phi1_solution >= sind(bound_phi(1)) && sin_phi1_solution <= sind(bound_phi(2)) && sin_phi2_solution >= sind(bound_phi(1)) && sin_phi2_solution <= sind(bound_phi(2))
% % % %                     scale_esimated(image_i) = solution_list(solution_index); 
% % % %                     disp(solution_list(solution_index));
% % % %                     multi_solution_count = multi_solution_count + 1;
% % % %                     if multi_solution_count>1
% % % %                         
% % % %                         disp('there are multiple solutions!');
% % % %                     end
% % % %                 end
% % % %             end
% % % %         end
% % % %     end
% % % %     if multi_solution_count == 0
% % % %         disp('no solution for scale estimation!');
% % % %     end
