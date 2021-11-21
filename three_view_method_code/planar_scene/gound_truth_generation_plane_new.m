clc;
close all;
clear;
useold_data = 1;        % do not generate new ground truth, use data from files.
approximate_plane = 1 ;
%-------------------------------------------------------------------------
addpath([pwd,'\PointClasses'],[pwd,'\PoseClasses'],[pwd,'\plotAxesAndPoints']);
if ~logical(useold_data)
    vector_nums = 51;                                        % point_nums = vector_nums + 1 , the origin point
    %feature_point_Num = 10;
    %feature_point_Num_S = 10;
    data_file_name = '50posesPlane';
    axeslength = 0.1;                                        % 1.5
    XYZcolors = 'rgbcmy';
    markersize = 5;                                          % feature marker
    %-----------------------------extrinsic_parameters--------------------------------------------
    R_c2s = eularAngle([0,0,-90]/180*pi,'zyx');
    t_c2s = [0;0;-0.05];                                     % meter
    %-------------------------------------------------------------------------
    plane_z_distance = -1.0;       %in sonar frame
    x_rot_bound = [20,45];
    y_rot_bound = [-10,10];
    z_rot_bound = [-6,6];
    move_bound = [0.1,0.3];
    t_vec_upper_bound_inS = [0.5;1.03;0.3];
    t_vec_lower_bound_inS = [-0.5;0.03;-0.3];
    %-------------------------------------------------------------------------
    position_list = cell(1,vector_nums);
    R_list= cell(1,vector_nums);
    t_list= cell(1,vector_nums);
    scale_list= zeros(1,vector_nums);
    %-------------------------------------------------------------------------
    h_fig = figure(1);
    hold on;
    plot3([0,1*axeslength],[0,0],[0,0],'LineWidth',2,'Color','k');
    plot3([0,0],[0,1*axeslength],[0,0],'LineWidth',2,'Color','k');
    plot3([0,0],[0,0],[0,1*axeslength],'LineWidth',2,'Color','k');
    axex_wc_inS = R_c2s*[1;0;0]*axeslength;                  % unit vectors along the world frame axes at [0;0;0]
    axey_wc_inS = R_c2s*[0;1;0]*axeslength;
    axez_wc_inS = R_c2s*[0;0;1]*axeslength;
    origin_wc_inS = R_c2s*[0;0;0] + t_c2s;
    axex_point_w = axex_wc_inS + origin_wc_inS;
    axey_point_w = axey_wc_inS + origin_wc_inS;
    axez_point_w = axez_wc_inS + origin_wc_inS;

    plot3([origin_wc_inS(1),axex_point_w(1)],[origin_wc_inS(2),axex_point_w(2)],[origin_wc_inS(3),axex_point_w(3)],'LineWidth',2,'Color',XYZcolors(1));
    plot3([origin_wc_inS(1),axey_point_w(1)],[origin_wc_inS(2),axey_point_w(2)],[origin_wc_inS(3),axey_point_w(3)],'LineWidth',2,'Color',XYZcolors(2));
    plot3([origin_wc_inS(1),axez_point_w(1)],[origin_wc_inS(2),axez_point_w(2)],[origin_wc_inS(3),axez_point_w(3)],'LineWidth',2,'Color',XYZcolors(3));

    for index_pose = 1:vector_nums  %length(pose_List)
        %disp(['%====================generate pose ',num2str(index_pose),'=======================']);
        rand_list = rand(1,3);
        angel_x_ramdom = x_rot_bound(1) + (x_rot_bound(2)-x_rot_bound(1))*rand_list(1);
        angel_y_ramdom = y_rot_bound(1) + (y_rot_bound(2)-y_rot_bound(1))*rand_list(2);
        angel_z_ramdom = z_rot_bound(1) + (z_rot_bound(2)-z_rot_bound(1))*rand_list(3);
        R_total = eularAngle([angel_z_ramdom,angel_x_ramdom,angel_y_ramdom]/180*pi,'zxy');
        R_list{index_pose} = R_total;
        if index_pose > 1
            for index_R = 1:index_pose-1
                R_list{index_pose} = R_list{index_pose}/R_list{index_R};
            end
        end    
        scale_list(index_pose) = move_bound(1) + (move_bound(2) - move_bound(1)) * rand(1,1); 
        rand_list_t = rand(1,3).';
        t_vec_inS = t_vec_lower_bound_inS + (t_vec_upper_bound_inS - t_vec_lower_bound_inS) .* rand_list_t;   
        t_vec_inS = t_vec_inS/norm(t_vec_inS);
        t_list{index_pose} = scale_list(index_pose) * R_total * (R_c2s\t_vec_inS);
        position_list{index_pose} =  R_list{index_pose}\t_list{index_pose};   
        %---------------------------------draw_axes------------------------------------------
        axex_c = R_list{index_pose}\[1;0;0]*axeslength;
        axey_c = R_list{index_pose}\[0;1;0]*axeslength;
        axez_c = R_list{index_pose}\[0;0;1]*axeslength;

        origin_s = R_list{index_pose}\(R_c2s\([0;0;0] - t_c2s));
        axex_s = R_list{index_pose}\(R_c2s\([1;0;0]*axeslength - t_c2s));
        axey_s = R_list{index_pose}\(R_c2s\([0;1;0]*axeslength - t_c2s));
        axez_s = R_list{index_pose}\(R_c2s\([0;0;1]*axeslength - t_c2s));
        %---------------------------------draw_axes------------------------------------------    
        if index_pose>1
            for index_R = index_pose-1:-1:1
            %position_list{index_pose} = position_list{index_pose} + position_list{index_R};
                position_list{index_pose} = R_list{index_R}\position_list{index_pose};  %灏嗗綋鍓嶅鍔犵殑鍚戦噺涔樹互鏃嬭浆鐭╅樀鐨勯?嗭紝杩唬鎭㈠鍒板湪鍓嶄竴涓潗鏍囩郴涓嬬殑鍧愭爣

                axex_c = R_list{index_R}\axex_c;
                axey_c = R_list{index_R}\axey_c;
                axez_c = R_list{index_R}\axez_c;

                origin_s = R_list{index_R}\origin_s;
                axex_s = R_list{index_R}\axex_s;
                axey_s = R_list{index_R}\axey_s;
                axez_s = R_list{index_R}\axez_s;
            end    
                position_list{index_pose} = position_list{index_pose} + position_list{index_pose-1};
        end
        position_inS = R_c2s*position_list{index_pose} + t_c2s;

        axex_c_inS = R_c2s*axex_c;         %just unit vectors along the frame at pose index represented in world frame
        axey_c_inS = R_c2s*axey_c;
        axez_c_inS = R_c2s*axez_c;
        %----------------------------------------------------------------------------------------------------------------
        origin_c_inS = position_inS;
        axex_point = axex_c_inS + origin_c_inS;
        axey_point = axey_c_inS + origin_c_inS;
        axez_point = axez_c_inS + origin_c_inS;

        plot3([origin_c_inS(1),axex_point(1)],[origin_c_inS(2),axex_point(2)],[origin_c_inS(3),axex_point(3)],'LineWidth',2,'Color',XYZcolors(1));
        plot3([origin_c_inS(1),axey_point(1)],[origin_c_inS(2),axey_point(2)],[origin_c_inS(3),axey_point(3)],'LineWidth',2,'Color',XYZcolors(2));
        plot3([origin_c_inS(1),axez_point(1)],[origin_c_inS(2),axez_point(2)],[origin_c_inS(3),axez_point(3)],'LineWidth',2,'Color',XYZcolors(3));
        %----------------------------------------------------------------------------------------------------------------
        origin_s_inS = R_c2s*(origin_s + position_list{index_pose}) + t_c2s;

        axex_point_s = R_c2s*(axex_s + position_list{index_pose}) + t_c2s;      %the point coordinate in world frame
        axey_point_s = R_c2s*(axey_s + position_list{index_pose}) + t_c2s;
        axez_point_s = R_c2s*(axez_s + position_list{index_pose}) + t_c2s;

        plot3([origin_s_inS(1),axex_point_s(1)],[origin_s_inS(2),axex_point_s(2)],[origin_s_inS(3),axex_point_s(3)],'LineWidth',2,'Color',XYZcolors(4));
        plot3([origin_s_inS(1),axey_point_s(1)],[origin_s_inS(2),axey_point_s(2)],[origin_s_inS(3),axey_point_s(3)],'LineWidth',2,'Color',XYZcolors(5));
        plot3([origin_s_inS(1),axez_point_s(1)],[origin_s_inS(2),axez_point_s(2)],[origin_s_inS(3),axez_point_s(3)],'LineWidth',2,'Color',XYZcolors(6));

    end
    hold off;
    %=========================================================================
    position_list=[[0;0;0],position_list];    %add the origin point
%     R_origin =[1,0,0;...
%                0,1,0;...
%                0,0,1];                        %origin pose
%     t_origin = [0;0;0];
%     R_list = [{R_origin},R_list];
%     t_list = [{t_origin},t_list];
    %============================================================draw_path_in_sonar_frame======================================================
    position_list_inS = zeros(3,length(position_list));
    hold on;
    
    for index_draw = 2:vector_nums+1
        %if index_draw>1        
        position_inS_p = R_c2s*position_list{index_draw-1} + t_c2s;      %previous in sonar frame
        position_list_inS(:,index_draw-1) = position_inS_p;
        position_inS_c = R_c2s*position_list{index_draw} + t_c2s;        %current in sonar frame
        plot3([position_inS_c(1),position_inS_p(1)],[position_inS_c(2),position_inS_p(2)],[position_inS_c(3),position_inS_p(3)],'LineWidth',1,'Color','k','LineStyle','--');
        %plot3([position_list{index_draw}(1),position_list{index_draw-1}(1)],[position_list{index_draw}(2),position_list{index_draw-1}(2)],[position_list{index_draw}(3),position_list{index_draw-1}(3)],'LineWidth',2,'Color','k');
        %else
        %    plot3([position_list{index_draw}(1),0],[position_list{index_draw}(2),0],[position_list{index_draw}(3),0],'LineWidth',2,'Color','k');
        %end
    end
    hold off;
    %=========================================================================
    view(3);
    axis equal;
    xlabel('x'),ylabel('y'),zlabel('z');
    %=========================================================================================================================================
    %t_list = t_list;                    % 51 poses (52 positions) o---o
    %scale_list = scale_list;            % 51 poses (52 positions)
    %R_list = R_list;                    % 51 poses (52 positions)
    position_list = position_list(2:end);      % 51 do not include the original point [0;0;0] 从第二个点开始
    save([pwd,'\ground_truth_data_plane\internal_save_data\internal_save.mat'],'-regexp','[^h_fig]');
    savefig(h_fig,[pwd,'\ground_truth_data_plane\internal_save_data\trajectory_only.fig']);    
else
    load([pwd,'\ground_truth_data_plane\internal_save_data\internal_save.mat']);
    h_fig = openfig([pwd,'\ground_truth_data_plane\internal_save_data\trajectory_only.fig'],'reuse');
end
offset_x = 3;
offset_y = 4.5;
plane_area_bound_x = [min(position_list_inS(1,:))-offset_x,max(position_list_inS(1,:))+offset_x];
plane_area_bound_y = [min(position_list_inS(2,:)),max(position_list_inS(2,:))+offset_y];
if approximate_plane == 1
    plane_area_bound_z = [-20,20] * 0.001;     %unit:meter
end
area_dimension = (plane_area_bound_x(2)-plane_area_bound_x(1))*(plane_area_bound_y(2)-plane_area_bound_y(1));
cam_point_dense = 32;
sonar_point_dense = 18;
camera_feature_num = round(area_dimension*cam_point_dense);
sonar_feature_num = round(area_dimension*sonar_point_dense);
point_3d_list = cell(1,camera_feature_num);
point_3d_list_s = cell(1,sonar_feature_num);
index_num_c = 1;
index_num_s = 1;
hold on;
for indexP_c_3d = 1:camera_feature_num
    rand_list_p = rand(1,2);
    x_rand = plane_area_bound_x(1)+(plane_area_bound_x(2)-plane_area_bound_x(1))*rand_list_p(1);
    y_rand = plane_area_bound_y(1)+(plane_area_bound_y(2)-plane_area_bound_y(1))*rand_list_p(2);
    z_rand = plane_z_distance;
    if approximate_plane == 1
        rand_z = rand(1,1);
        z_offset_rand = plane_area_bound_z(1) + (plane_area_bound_z(2) - plane_area_bound_z(1))*rand_z;
        z_rand = plane_z_distance + z_offset_rand;
    end    
    point_rand = [x_rand;y_rand;z_rand];
    plot3(point_rand(1),point_rand(2),point_rand(3),'r.','MarkerSize',markersize);
    point_3d_list{index_num_c} = FeaturePointIn3DSpace(point_rand,index_num_c);
    index_num_c = index_num_c + 1;
end
 for indexP_s_3d = 1:sonar_feature_num
    rand_list_p = rand(1,2);
    x_rand = plane_area_bound_x(1)+(plane_area_bound_x(2)-plane_area_bound_x(1))*rand_list_p(1);
    y_rand = plane_area_bound_y(1)+(plane_area_bound_y(2)-plane_area_bound_y(1))*rand_list_p(2); 
    z_rand = plane_z_distance;
    if approximate_plane == 1
        rand_z = rand(1,1);
        z_offset_rand = plane_area_bound_z(1) + (plane_area_bound_z(2) - plane_area_bound_z(1))*rand_z;
        z_rand = plane_z_distance + z_offset_rand;
    end    
    point_rand = [x_rand;y_rand;z_rand];
    plot3(point_rand(1),point_rand(2),point_rand(3),'b.','MarkerSize',markersize);
    point_3d_list_s{index_num_s} = FeaturePointIn3DSpace(point_rand,index_num_s);
    index_num_s = index_num_s + 1;
end   
%-------------------------------------------------------------------------
hold off;    
if approximate_plane == 0
    save([pwd,'\ground_truth_data_plane\ground_truth_',data_file_name,'.mat'],'position_list','scale_list','R_list','t_list','point_3d_list','point_3d_list_s');
    savefig(h_fig,[pwd,'\ground_truth_data_plane\ground_truth_trajectory.fig']);
elseif approximate_plane == 1
    save([pwd,'\ground_truth_data_plane\ground_truth_',data_file_name,'_approximate.mat'],'position_list','scale_list','R_list','t_list','point_3d_list','point_3d_list_s');
    savefig(h_fig,[pwd,'\ground_truth_data_plane\ground_truth_trajectory_approximate.fig']);
end









