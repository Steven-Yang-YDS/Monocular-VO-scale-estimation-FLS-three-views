clc;
close all;
clear;
format long;
warning('off');
%------------------------------------------------------------
addpath([pwd,'\PointClasses'],[pwd,'\PoseClasses'],[pwd,'\plotAxesAndPoints']);
%-----------------------------------------------------------------------------------------------------------------
noise_test = 1;                     % whether to add sonar image noise 
noise_number =15;                   % sonar image noise level
%--------------------------------------------------------------------------
fix_noise_set_num = 6;              % camera image noise group  ,notice: the noises in all groups are with a noise level of sigma == 1 pixel
%--------------------------------------------------------
approximate_plane = 1;
if approximate_plane == 0
    filename = '50posesPlane';
elseif approximate_plane == 1
    filename = '50posesPlane_approximate';
end
%load([pwd,'\ground_truth_data\ground_truth_233.mat']);
%load([pwd,'\input_data\sensor_measurements_233.mat']);
%load([pwd,'\input_data_plane\sensor_measurement_noise(mean_0_sigma_0.001)_50posesPlane.mat']);
if noise_test == 0
    load([pwd,'\input_data_plane\sensor_measurements_',filename,'.mat']);
    if approximate_plane == 0
        load([pwd,'\estimated_results_plane\estimated_cameraPoses_planeNormals.mat']);
    elseif approximate_plane == 1
        load([pwd,'\estimated_results_plane\estimated_cameraPoses_planeNormals_approximate.mat']);
    end
elseif noise_test == 1
    if noise_number<10
        str_noise_num = ['0',num2str(noise_number)];
    elseif mod(noise_number,10) == 0
        str_noise_num = num2str(noise_number/10);
    else
        str_noise_num = num2str(noise_number);
    end
    load([pwd,'\input_data_plane\sensor_measurements_noise_(mean_0_sigma_0.0',str_noise_num,')_',filename,'.mat']);
    if approximate_plane == 0        
        load([pwd,'\estimated_results_plane\',num2str(fix_noise_set_num),'\estimated_cameraPoses_planeNormals.mat']);
    elseif approximate_plane == 1
        load([pwd,'\estimated_results_plane\',num2str(fix_noise_set_num),'\estimated_cameraPoses_planeNormals_approximate.mat']);
    end
    sonar_image_list = sonar_image_list_noise;
end
%------------------------------------------------------------------------
R_c2s = eularAngle([0,0,-90]/180*pi,'zyx');
t_c2s = [0;0;-0.05];
bound_phi = [-10,10];
%==============================================
show_correspondences_sonar = 0;
start_frame = 1;
end_frame = 50;
%=================================================display matching process=======================================
%draw the corresponding point betweent image i and j
%multi_solution_flag = zeros(1,end_frame);
statu_flag = ones(1,end_frame);
multiSolution_count = zeros(1,end_frame);     % counting the times of obtaining multi-solutions  
invalidSolution_count = zeros(1,end_frame);   % counting the times of obtaining unsolvable input
%==============================================
scale_estimated_poses = cell(1,end_frame-start_frame+1);
scale_estimated_points_list = cell(1,end_frame-start_frame+1);
time_pose = zeros(1,end_frame-start_frame+1);
time_point_list = cell(1,end_frame-start_frame+1);
valid_list_list = cell(1,end_frame-start_frame+1);
for image_i = start_frame :end_frame
    tStart = tic;
    image_j = image_i + 1;
    %============================================ find sonar correspondences ===============================================
    list1_s = zeros(1,length(sonar_image_list{image_i}));
    list2_s = zeros(1,length(sonar_image_list{image_j}));
    if show_correspondences_sonar
        figure;
        %set(gca,'YDir','reverse');
        axis equal;
        %set(gca,'XLim',[bound_x(1),3*bound_x(2)]);
        %set(gca,'YLim',bound_y);
        xlabel('x'),ylabel('y');
        hold on;
    end
    for index_correspond = 1:length(sonar_image_list{image_i})
        if show_correspondences_sonar
            point_vec = sonar_image_list{image_i}{index_correspond}.coordinate;
            plot(point_vec(1),point_vec(2),'g.');
        end
        list1_s(index_correspond) = sonar_image_list{image_i}{index_correspond}.feature_index;
    end
    for index_correspond = 1:length(sonar_image_list{image_j})
        if show_correspondences_sonar
            point_vec = sonar_image_list{image_j}{index_correspond}.coordinate;
            plot((point_vec(1)+ offset_x),point_vec(2),'m.');
        end
        list2_s(index_correspond) = sonar_image_list{image_j}{index_correspond}.feature_index;
    end
    [common_s, index_a_s, index_b_s] = intersect(list1_s, list2_s);
    
    corres_i_homo_s = zeros(length(common_s),2);
    corres_j_homo_s = zeros(length(common_s),2);
    
    percent = 0.10;
    for ind_match_line = 1:length(common_s)
        start_x = sonar_image_list{image_i}{index_a_s(ind_match_line)}.coordinate(1);
        start_y = sonar_image_list{image_i}{index_a_s(ind_match_line)}.coordinate(2);
        end_x = sonar_image_list{image_j}{index_b_s(ind_match_line)}.coordinate(1);
        end_y = sonar_image_list{image_j}{index_b_s(ind_match_line)}.coordinate(2);
        if show_correspondences_sonar
            plot([start_x,end_x + offset_x],[start_y,end_y],'k-');
        end
        corres_i_homo_s(ind_match_line,1) = start_x;
        corres_i_homo_s(ind_match_line,2) = start_y;
        corres_j_homo_s(ind_match_line,1) = end_x;
        corres_j_homo_s(ind_match_line,2) = end_y;
    end
    if show_correspondences_sonar
        hold off;
    end
%     srcPoints_s = corres_i_homo_s(1:4,:);
%     dstPoints_s = corres_j_homo_s(1:4,:);
%     Homo_mat_s = cv.findHomography(srcPoints_s, dstPoints_s);
%     if det(Homo_mat_s)<0
%         Homo_mat_s = -Homo_mat_s;
%     end
    valid_list = ones(1,length(common_s));
    scale_estimated_points = cell(1,length(common_s));
    time_point = zeros(1,length(common_s));

    disp(['-------------------scale_',num2str(image_i),'----------------------']);
    for sonar_point_num = 1:length(common_s)        % 使用每个点来计算    
        tic;
        range_R1 = norm(corres_i_homo_s(sonar_point_num,:));
        sin_theta1 = corres_i_homo_s(sonar_point_num,1)/range_R1;
        cos_theta1 = corres_i_homo_s(sonar_point_num,2)/range_R1;
        range_R2 = norm(corres_j_homo_s(sonar_point_num,:));
        sin_theta2 = corres_j_homo_s(sonar_point_num,1)/range_R2;
        cos_theta2 = corres_j_homo_s(sonar_point_num,2)/range_R2;
        %================================================================================================    
        R_c = estimated_pose{image_i}(1:3,1:3);
        t_c_hat = estimated_pose{image_i}(1:3,4);
        n_c_hat = estimated_normal_vector{image_i};
        point_3D_pre_frame = point_3D_aver_list(:,image_i);
        point_3D_next_frame = R_c * point_3D_aver_list(:,image_i) + t_c_hat;
        plane_distance_pre = point_3D_pre_frame.'* n_c_hat;
        plane_distance_next = point_3D_next_frame.'* (R_c * n_c_hat); %the distance from camera origin to the plane when the magnitude ||t_o||=1
    %     if distance <0
    %         distance = -distance;
    %     end
        syms scale_c k_s1 k_s2
        %------------------------------------------------------------------------------------------------
        k_c1 = 1/(scale_c*plane_distance_pre);              % the scale of the plane normal in camera frame
        k_c2 = 1/(scale_c*plane_distance_next);
        %k_s1 = k_c1/(1+k_c1*t_c2s.'*R_c2s*n_c_hat);
        %k_s2 = k_c2/(1+k_c2*t_c2s.'*R_c2s*R_c*n_c_hat);
        %-------------------------------------------------------------------------------------------------  
        %scale_c1 = solve(k_s1 == k_c1/(1+k_c1*t_c2s.'*(R_c2s*n_c_hat)),scale_c); %用k_s1表示scale_c
        k_s1_scale = solve(k_s1 == k_c1/(1+k_c1*t_c2s.'*(R_c2s*n_c_hat)),k_s1);         % contain unknows scale_c  描述了平面法向量在相机和声呐坐标系中各自模长的对应关系
        k_s2_scale = solve(k_s2 == k_c2/(1+k_c2*t_c2s.'*(R_c2s*(R_c*n_c_hat))),k_s2);   % contain unknows scale_c
        R_s = R_c2s*R_c*R_c2s.';
        t_s = t_c2s - R_s * t_c2s + R_c2s * t_c_hat * scale_c;                  %t_s contain unknows scale_c
        n_s_hat1 = R_c2s*n_c_hat;        % sonar view1 坐标系下的单位平面法向量   contain no unknows
        n_s_hat2 = R_c2s*R_c*n_c_hat;    % sonar view2 坐标系下的单位平面法向量   contain no unknows
        n_sx1 = n_s_hat1(1);           
        n_sy1 = n_s_hat1(2);
        n_sz1 = n_s_hat1(3);
        n_sx2 = n_s_hat2(1);
        n_sy2 = n_s_hat2(2);
        n_sz2 = n_s_hat2(3);    
        Q = R_s + k_s1 * t_s * n_s_hat1.';  % 两个声呐坐标系下的平面点的变换关系用Q描述  k_s1=1/d1 contain unknows k_s1
        solution_list = zeros(1,4);
        syms sin_phi1 sin_phi2
        sin_phi1_k_s1_solutions = solve(k_s1 * ((n_sx1 * sin_theta1 + n_sy1 * cos_theta1) * sqrt(1-sin_phi1^2) + n_sz1 * sin_phi1) * range_R1 == 1,sin_phi1);  %用k_s1表示sin_phi1
        sin_phi2_k_s2_solutions = solve(k_s2 * ((n_sx2 * sin_theta2 + n_sy2 * cos_theta2) * sqrt(1-sin_phi2^2) + n_sz2 * sin_phi2) * range_R2 == 1,sin_phi2);  %用k_s2表示sin_phi2
        %--------------------------solution1--------------------------------
        sin_phi1_k_s1 = sin_phi1_k_s1_solutions(1);
        sin_phi2_k_s2 = sin_phi2_k_s2_solutions(1);
        cos_phi1_k_s1 = sqrt(1-sin_phi1_k_s1^2);
        cos_phi2_k_s2 = sqrt(1-sin_phi2_k_s2^2);
        eq_row1 = [cos_phi1_k_s1/cos_phi2_k_s2*Q(1,1),cos_phi1_k_s1/cos_phi2_k_s2*Q(1,2),range_R1*sin_phi1_k_s1/cos_phi2_k_s2*Q(1,3)]*[corres_i_homo_s(sonar_point_num,1);corres_i_homo_s(sonar_point_num,2);1]-corres_j_homo_s(sonar_point_num,1);
        equ = subs(eq_row1,[k_s1,k_s2],[k_s1_scale,k_s2_scale]);
        temp_solution = vpasolve(equ,scale_c);
        if ~isempty(temp_solution)
            solution_list(1) = temp_solution;
        else
            solution_list(1) = -1;
        end
        %--------------------------solution2--------------------------------
        sin_phi1_k_s1 = sin_phi1_k_s1_solutions(1);
        sin_phi2_k_s2 = sin_phi2_k_s2_solutions(2);
        cos_phi1_k_s1 = sqrt(1-sin_phi1_k_s1^2);
        cos_phi2_k_s2 = sqrt(1-sin_phi2_k_s2^2);
        eq_row1 = [cos_phi1_k_s1/cos_phi2_k_s2*Q(1,1),cos_phi1_k_s1/cos_phi2_k_s2*Q(1,2),range_R1*sin_phi1_k_s1/cos_phi2_k_s2*Q(1,3)]*[corres_i_homo_s(sonar_point_num,1);corres_i_homo_s(sonar_point_num,2);1]-corres_j_homo_s(sonar_point_num,1);
        equ = subs(eq_row1,[k_s1,k_s2],[k_s1_scale,k_s2_scale]);
        temp_solution = vpasolve(equ,scale_c);
        if ~isempty(temp_solution)
            solution_list(2) = temp_solution;
        else
            solution_list(2) = -1;
        end
        %--------------------------solution3--------------------------------
        sin_phi1_k_s1 = sin_phi1_k_s1_solutions(2);
        sin_phi2_k_s2 = sin_phi2_k_s2_solutions(1);
        cos_phi1_k_s1 = sqrt(1-sin_phi1_k_s1^2);
        cos_phi2_k_s2 = sqrt(1-sin_phi2_k_s2^2);
        eq_row1 = [cos_phi1_k_s1/cos_phi2_k_s2*Q(1,1),cos_phi1_k_s1/cos_phi2_k_s2*Q(1,2),range_R1*sin_phi1_k_s1/cos_phi2_k_s2*Q(1,3)]*[corres_i_homo_s(sonar_point_num,1);corres_i_homo_s(sonar_point_num,2);1]-corres_j_homo_s(sonar_point_num,1);
        equ = subs(eq_row1,[k_s1,k_s2],[k_s1_scale,k_s2_scale]);
        temp_solution = vpasolve(equ,scale_c);
        if ~isempty(temp_solution)
            solution_list(3) = temp_solution;
        else
            solution_list(3) = -1;
        end
        %--------------------------solution4--------------------------------
        sin_phi1_k_s1 = sin_phi1_k_s1_solutions(2);
        sin_phi2_k_s2 = sin_phi2_k_s2_solutions(2);
        cos_phi1_k_s1 = sqrt(1-sin_phi1_k_s1^2);
        cos_phi2_k_s2 = sqrt(1-sin_phi2_k_s2^2);
        eq_row1 = [cos_phi1_k_s1/cos_phi2_k_s2*Q(1,1),cos_phi1_k_s1/cos_phi2_k_s2*Q(1,2),range_R1*sin_phi1_k_s1/cos_phi2_k_s2*Q(1,3)]*[corres_i_homo_s(sonar_point_num,1);corres_i_homo_s(sonar_point_num,2);1]-corres_j_homo_s(sonar_point_num,1);
        equ = subs(eq_row1,[k_s1,k_s2],[k_s1_scale,k_s2_scale]);
        temp_solution = vpasolve(equ,scale_c);
        if ~isempty(temp_solution)
            solution_list(4) = temp_solution;
        else
            solution_list(4) = -1;
        end
        %--------------------------------------------------------------------
        
        multi_solution_count = 0;
        temp_solution_list = [];
        for solution_index = 1:4
            if imag(solution_list(solution_index)) == 0
                if solution_list(solution_index)>0
                    k_s1_solution = double(subs(k_s1_scale,scale_c,solution_list(solution_index)));
                    k_s2_solution = double(subs(k_s2_scale,scale_c,solution_list(solution_index)));
                    switch solution_index
                        case  1
                            sin_phi1_solution = double(subs(sin_phi1_k_s1_solutions(1),k_s1,k_s1_solution));
                            sin_phi2_solution = double(subs(sin_phi2_k_s2_solutions(1),k_s2,k_s2_solution));
                        case  2
                            sin_phi1_solution = double(subs(sin_phi1_k_s1_solutions(1),k_s1,k_s1_solution));
                            sin_phi2_solution = double(subs(sin_phi2_k_s2_solutions(2),k_s2,k_s2_solution));
                        case  3
                            sin_phi1_solution = double(subs(sin_phi1_k_s1_solutions(2),k_s1,k_s1_solution));
                            sin_phi2_solution = double(subs(sin_phi2_k_s2_solutions(1),k_s2,k_s2_solution));
                        case  4
                            sin_phi1_solution = double(subs(sin_phi1_k_s1_solutions(2),k_s1,k_s1_solution));
                            sin_phi2_solution = double(subs(sin_phi2_k_s2_solutions(2),k_s2,k_s2_solution));
                    end        
                    if sin_phi1_solution >= sind(bound_phi(1)) && sin_phi1_solution <= sind(bound_phi(2)) && sin_phi2_solution >= sind(bound_phi(1)) && sin_phi2_solution <= sind(bound_phi(2))
                        multi_solution_count = multi_solution_count + 1;
                        temp_solution_list(multi_solution_count)= solution_list(solution_index); 
                        %disp(solution_list(solution_index));
                        if multi_solution_count > 1
                            %multi_solution_flag(image_i) = 1;
                            disp('There are multiple solutions!');    % flag  2
                            scale_estimated_points{sonar_point_num} = temp_solution_list;
                            valid_list(sonar_point_num) = 2;
                            multiSolution_count(image_i) = multiSolution_count(image_i) + 1;
                        end                   
                    end

                end
            end
        end
        if multi_solution_count == 0
            disp('No proper estimated solution !');      % flag  3
            disp('**** possible_solutions ****')
            disp(solution_list);
            scale_estimated_points{sonar_point_num} = -1;
            valid_list(sonar_point_num) = 0;            
            invalidSolution_count(image_i) = invalidSolution_count(image_i) + 1;
        else
            scale_estimated_points{sonar_point_num} = temp_solution_list;
        end
        if sonar_point_num >= round(percent *length(common_s))
            disp([num2str(percent/0.10*10),'% points have been tested.']);
            percent = percent + 0.10;
        end
        time_point(sonar_point_num) = toc;
    end
    sum_results = 0;
    valid_num = 0;
    for index = 1:length(common_s)
        if valid_list(index) == 1
            sum_results = sum_results + scale_estimated_points{index};
            valid_num = valid_num + 1;
        end
    end
    if valid_num > 0
        scale_estimated_poses{image_i} = sum_results/valid_num;
    else
        scale_estimated_poses{image_i} = -1;
        statu_flag{image_i} = 0; 
    end
    scale_estimated_points_list{image_i} = scale_estimated_points;
    time_pose(image_i) = toc(tStart);
    valid_list_list{image_i} = valid_list; 
    time_point_list{image_i} = time_point;    
end
%------------------------------ error test ------------------------------
if approximate_plane == 0
    load([pwd,'\ground_truth_data_plane\ground_truth_50posesPlane.mat']);
elseif approximate_plane == 1
    load([pwd,'\ground_truth_data_plane\ground_truth_50posesPlane_approximate.mat']);
end
error_list = zeros(1,length(scale_estimated_poses));
for index_error = 1:length(scale_estimated_poses)
    if scale_estimated_poses{index_error} ~= -1
        error_list(index_error) = abs(scale_estimated_poses{index_error}- scale_list(index_error+1));
    else
        error_list(index_error) = -1;
    end
end
%------------------------------------------------------------------------
if noise_test == 0
    if approximate_plane == 0
        save([pwd,'\estimated_results_plane\withoutNoise\pre_method_2010\estimated_result_2010_start(',num2str(start_frame),')_end(',num2str(end_frame-1),').mat'],...
                'start_frame','end_frame','statu_flag','scale_estimated_poses','multiSolution_count','invalidSolution_count','scale_estimated_points_list','time_point_list','valid_list_list','time_pose');
    elseif approximate_plane == 1
        save([pwd,'\estimated_results_plane\withoutNoise\pre_method_2010\estimated_result_2010_start(',num2str(start_frame),')_end(',num2str(end_frame-1),')_approximate.mat'],...
                'start_frame','end_frame','statu_flag','scale_estimated_poses','multiSolution_count','invalidSolution_count','scale_estimated_points_list','time_point_list','valid_list_list','time_pose');
    end
elseif noise_test == 1
    if approximate_plane == 0
        save([pwd,'\estimated_results_plane\',num2str(noise_number),'\pre_method_2010\estimated_result_2010_start(',num2str(start_frame),')_end(',num2str(end_frame-1),').mat'],...
                'start_frame','end_frame','statu_flag','scale_estimated_poses','multiSolution_count','invalidSolution_count','scale_estimated_points_list','time_point_list','valid_list_list','time_pose');
    elseif approximate_plane == 1
        save([pwd,'\estimated_results_plane\',num2str(noise_number),'\pre_method_2010\estimated_result_2010_start(',num2str(start_frame),')_end(',num2str(end_frame-1),')_approximate.mat'],...
                'start_frame','end_frame','statu_flag','scale_estimated_poses','multiSolution_count','invalidSolution_count','scale_estimated_points_list','time_point_list','valid_list_list','time_pose');
    end
end

