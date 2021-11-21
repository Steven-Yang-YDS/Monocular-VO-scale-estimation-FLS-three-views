clc;
close all;
clear;
%---------------------------------------------------------------------------------------------
addpath([pwd,'\PointClasses'],[pwd,'\PoseClasses'],[pwd,'\plotAxesAndPoints']);
approximate_plane = 1;                                   % use a approximate plane model 
if approximate_plane == 0
    data_file_name = '_50posesPlane';
elseif approximate_plane == 1
    data_file_name = '_50posesPlane_approximate';
end
load([pwd,'\ground_truth_data_plane\ground_truth',data_file_name,'.mat']);
XYZcolors = 'rgbcmy';
axeslength = 0.1;                                        % 1.5
markersize = 5;                                          % feature marker
start_index = 20;
index_draw = [start_index,start_index+20];
%-----------------------------extrinsic_parameters--------------------------------------------
R_c2s = eularAngle([0,0,-90]/180*pi,'zyx');
t_c2s = [0;0;-0.05];
%---------------------------------------------------------------------------------------------
is_first = 1;
mean_c = 0;
sigma_c = 1;                                           %pixel
mean_s = 0;
%sigma_s = 0.005;                                       % meter
for sigma_s = 0.001:0.001:0.015                              % meter
    disp(['-------------------------Inputs generation with sonar noise (mean_s,sigma_s) = (',num2str(mean_s),',',num2str(sigma_s),')-------------------------']);
    %=================================================projection_on_camera_imaging_plane==============================================================
    focalLength = 0.01*5;                              % focal length in world units                      
    focalxy = focalLength*[ 20 , 20 ]*1000;            % a two-element vector, [fx, fy].the number of pixels per world unit in the x and y direction respectively.
    principalPoint = [ 960 , 540 ];                    % a two-element vector, [cx,cy], in pixels.
    imageSize = [ 1920,1080 ];                         % Image size produced by the camera, specified as a two-element vector, [mrows,ncols]
    camera_intrinsics = cameraIntrinsics(focalxy,principalPoint,imageSize);

    bound_x = [-imageSize(1)/focalxy(1)/2,imageSize(1)/focalxy(1)/2];
    bound_y = [-imageSize(2)/focalxy(2)/2,imageSize(2)/focalxy(2)/2];

    camera_range_bound = 7;                           % 7m

    camera_image_list = cell(1,length(R_list));
    camera_image_list_pixel = cell(1,length(R_list));
    camera_image_list_pixel_noise = cell(1,length(R_list));
    camera_image_list_noise = cell(1,length(R_list));
    figure;
    col = 4;
    row = 5;
    idx_plot = 1;
    for index_pose = 1:length(R_list)
        point_2d_list = cell(1,1);
        point_2d_list_noise = cell(1,1);
        point_2d_list_pixel = cell(1,1);
        point_2d_list_pixel_noise = cell(1,1);
        index_2d = 0;
        if idx_plot >= index_draw(1) && idx_plot < index_draw(2)
            subplot(row,col,idx_plot-index_draw(1)+1);
            set(gca,'YDir','reverse');
            set(gca,'XLim',bound_x);
            set(gca,'YLim',bound_y);            
        end
        hold on;        
        for index_local_coord = 1:length(point_3d_list)
            point_3d_inWC = R_c2s\(point_3d_list{index_local_coord}.coordinate - t_c2s); 
            point_3d_inNC = point_3d_inWC;
            for index_trans_forward = 1:index_pose
                t_p2c = R_list{index_trans_forward}\t_list{index_trans_forward};
                point_3d_inNC = R_list{index_trans_forward}*(point_3d_inNC - t_p2c);
            end
            if point_3d_inNC(3) > 0
                %--------------------------------------------camera_projection-----------------------------------------------------------    
                point_2d = [point_3d_inNC(1);point_3d_inNC(2)]/point_3d_inNC(3);
                if point_2d(1) > bound_x(1) && point_2d(1) < bound_x(2) && point_2d(2) > bound_y(1) && point_2d(2) < bound_y(2) && norm(point_3d_inNC) < camera_range_bound
                    %--------------------------------------------------------------------------------------------------------------------
                    if idx_plot >= index_draw(1) && idx_plot < index_draw(2)
                        plot(point_2d(1),point_2d(2),'r.');
                    end
                    %--------------------------------------------------------------------------------------------------------------------
                    point_2d_obj = {FeaturePoint2DProjection(point_2d,index_local_coord)};
                    point_2d_list = [point_2d_list, point_2d_obj];
                    %---------------------------------------camera_noise_data_generation-----------------------------------------------------
                    noise_c = [normrnd(mean_c,sigma_c);normrnd(mean_c,sigma_c);0];
                    point_2d_pixel = camera_intrinsics.IntrinsicMatrix.' * [point_2d;1];
                    point_2d_pixel_noise = point_2d_pixel + noise_c;     % pixel plane
                    point_2d_obj_pixel = {FeaturePoint2DProjection(point_2d_pixel,index_local_coord)};
                    point_2d_list_pixel = [point_2d_list_pixel, point_2d_obj_pixel];
                    point_2d_obj_pixel_noise = {FeaturePoint2DProjection(point_2d_pixel_noise,index_local_coord)};
                    point_2d_list_pixel_noise = [point_2d_list_pixel_noise, point_2d_obj_pixel_noise];
                    
                    %-------------------------------------------------------------------------------------------------------------------------
                    point_2d_noise = camera_intrinsics.IntrinsicMatrix.'\point_2d_pixel_noise;               % normalized plane
                    point_2d_obj_noise = {FeaturePoint2DProjection(point_2d_noise,index_local_coord,noise_c)};
                    point_2d_list_noise = [point_2d_list_noise, point_2d_obj_noise];
                    index_2d = index_2d + 1;
                end
            end
        end
        hold off;
        if idx_plot >= index_draw(1) && idx_plot < index_draw(2)            
            axis equal;
        end
        idx_plot = idx_plot + 1;
        %point_2d_list{1} = index_2d;    %the first cell denotes the total feature number in image i
        point_2d_list = point_2d_list(2:end);
        point_2d_list_pixel =  point_2d_list_pixel(2:end);
        point_2d_list_pixel_noise = point_2d_list_pixel_noise(2:end);
        point_2d_list_noise = point_2d_list_noise(2:end);
        camera_image_list{index_pose} = point_2d_list;
        camera_image_list_pixel{index_pose} = point_2d_list_pixel;  
        camera_image_list_pixel_noise{index_pose} = point_2d_list_pixel_noise;  
        camera_image_list_noise{index_pose} = point_2d_list_noise;  
        %%%%%index_2d 
        %point_2d_list{1}
    end
    % for index_image = 1:1%length(camera_image_list)
    %     list1 = zeros(1,length(camera_image_list{index_image}));
    %     list2 = zeros(1,length(camera_image_list{index_image+1}));
    %     for index_list1 = 1:length(camera_image_list{index_image}) 
    %         list1(index_list1) = camera_image_list{index_image}{index_list1}.feature_index;    
    %     end    
    %     for index_list2 = 1:length(camera_image_list{index_image+1}) 
    %         list2(index_list2) = camera_image_list{index_image+1}{index_list2}.feature_index;
    %     end
    % %     list1(:)
    % %     list2(:)
    %     [common, index_a, index_b] = intersect(list1, list2);
    %     points1 = cell(1,5);%zeros(length(index_a),2);
    %     points2 = cell(1,5);%zeros(length(index_b),2);    
    %     
    %     for ind1 = 1:5%length(index_a)
    %         points1{ind1} = camera_image_list{index_image}{index_a(ind1)}.coordinate;
    %     end
    %     for ind2 = 1:5%length(index_b)
    %         points2{ind2} = camera_image_list{index_image+1}{index_b(ind2)}.coordinate;
    %     end
    % %     [E,mask] = cv.findEssentialMat(points1, points2);
    % %     [R, t, ~, ~] = cv.recoverPose(E, points1, points2);
    % end

    %=================================================projection_on_sonar_imaging_plane==============================================================
    bound_theta = [-65,65];                                         % unit: degree
    bound_phi = [-10,10];                              
    sonar_range_bound = [0.5,10];                                   % 100m
    sample_times = 30;                                              %768; %  sampling times of the displaying curve bound 
    bound_x_s = sonar_range_bound(2)*[sind(bound_theta(1)),sind(bound_theta(2))];
    bound_y_s = [0,sonar_range_bound(2)];
    % 
    sonar_image_list = cell(1,length(R_list));
    sonar_image_list_noise = cell(1,length(R_list));
    figure;
    col = 4;
    row = 5;
    idx_plot = 1;
    sample_point_list = zeros(4,sample_times+1);
    for index_pose = 1:length(R_list)
        point_2d_list_s = cell(1,1);
        point_2d_list_s_noise = cell(1,1);
        index_2d_s = 0;
        if idx_plot >= index_draw(1) && idx_plot < index_draw(2)
            subplot(row,col,idx_plot-index_draw(1)+1);
            hold on;       
            for ind_sample = 0:sample_times 
                theta_sample = ind_sample*(bound_theta(2) - bound_theta(1))/sample_times-65;
                sample_point_list(1,ind_sample+1) = sonar_range_bound(1)*sind(theta_sample);
                sample_point_list(2,ind_sample+1) = sonar_range_bound(1)*cosd(theta_sample);
                sample_point_list(3,ind_sample+1) = sonar_range_bound(2)*sind(theta_sample);
                sample_point_list(4,ind_sample+1) = sonar_range_bound(2)*cosd(theta_sample);
            end
            for ind_connect = 1:sample_times
                plot([sample_point_list(1,ind_connect),sample_point_list(1,ind_connect+1)],[sample_point_list(2,ind_connect),sample_point_list(2,ind_connect+1)],'k-');
                plot([sample_point_list(3,ind_connect),sample_point_list(3,ind_connect+1)],[sample_point_list(4,ind_connect),sample_point_list(4,ind_connect+1)],'k-');
            end
            plot([sample_point_list(1,1),sample_point_list(3,1)],[sample_point_list(2,1),sample_point_list(4,1)],'k-');
            plot([sample_point_list(1,end),sample_point_list(3,end)],[sample_point_list(2,end),sample_point_list(4,end)],'k-');
            hold off;
        end
        
        %set(gca,'YDir','reverse');
        %set(gca,'XLim',bound_x_s);
        %set(gca,'YLim',bound_y_s);
        hold on; 
        for index_local_coord = 1:length(point_3d_list_s)
            point_3d_s_inWC = R_c2s\(point_3d_list_s{index_local_coord}.coordinate - t_c2s); 
            point_3d_s_inNC = point_3d_s_inWC;
            for index_trans_forward = 1:index_pose
                t_p2c = R_list{index_trans_forward}\t_list{index_trans_forward};
                point_3d_s_inNC = R_list{index_trans_forward}*(point_3d_s_inNC - t_p2c);
            end
            point_3d_s_inNS = R_c2s*point_3d_s_inNC + t_c2s;          %transform to the Nth sonar frame 
            if point_3d_s_inNS(2) > 0                                 %Ys should be greater than 0
                range_gt = norm(point_3d_s_inNS);                     % gt: ground truth
                tan_theta_gt = point_3d_s_inNS(1)/point_3d_s_inNS(2);
                cos_phi_gt = norm(point_3d_s_inNS(1:2))/range_gt;
                %--------------------------------------------sonar_projection----------------------------------------------------------- 
                point_2d_s = [point_3d_s_inNS(1);point_3d_s_inNS(2)]/cos_phi_gt;
                if range_gt <= sonar_range_bound(2) && tan_theta_gt >= tand(bound_theta(1)) && tan_theta_gt <= tand(bound_theta(2)) && cos_phi_gt >= cosd(bound_phi(2))
                    if idx_plot >= index_draw(1) && idx_plot < index_draw(2)
                        plot(point_2d_s(1),point_2d_s(2),'b.');
                    end
                    point_2d_obj_s = {FeaturePoint2DProjection(point_2d_s,index_local_coord)};
                    point_2d_list_s = [point_2d_list_s, point_2d_obj_s];
                %---------------------------------------sonar_noise_data_generation------------------------------------------------------------
                    noise_s = [normrnd(mean_s,sigma_s);normrnd(mean_s,sigma_s)];
                    point_2d_obj_s_noise = {FeaturePoint2DProjection(point_2d_s + noise_s,index_local_coord,noise_s)};
                    point_2d_list_s_noise = [point_2d_list_s_noise, point_2d_obj_s_noise];
                    index_2d_s = index_2d_s + 1;
                end
            end
        end
        hold off;
        if idx_plot >= index_draw(1) && idx_plot < index_draw(2)
            axis equal;
        end
        idx_plot = idx_plot + 1;
        %point_2d_list{1} = index_2d;    %the first cell denotes the total feature number in image i
        point_2d_list_s = point_2d_list_s(2:end);
        sonar_image_list{index_pose} = point_2d_list_s;
        point_2d_list_s_noise = point_2d_list_s_noise(2:end);
        sonar_image_list_noise{index_pose} = point_2d_list_s_noise;
        %%%%%index_2d_s 
        %point_2d_list{1}
    end
    if is_first == 1
        save([pwd,'\input_data_plane\sensor_measurements',data_file_name,'.mat'],'camera_image_list','sonar_image_list','camera_image_list_pixel','camera_intrinsics');
        is_first = 0;
    end
        save([pwd,'\input_data_plane\sensor_measurements_noise_(mean_',num2str(mean_s),'_sigma_',num2str(sigma_s),')',data_file_name,'.mat'],'camera_image_list_noise','camera_image_list_pixel_noise','sonar_image_list_noise','camera_intrinsics');
    disp('-------------------------Finished-------------------------');
end