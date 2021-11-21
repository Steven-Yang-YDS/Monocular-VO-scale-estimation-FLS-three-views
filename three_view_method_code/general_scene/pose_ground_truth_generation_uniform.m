clc;
close all;
clear;
%-----------------------------------------------------------------------------------------------------
useold_data = 0;
addpath([pwd,'\PointClasses_subfunc'],[pwd,'\PoseClasses_subfunc'],[pwd,'\plotAxesAndPoints_subfunc']);
%------------------------------------------------------------------------
if ~logical(useold_data)
vector_nums = 201;                                        % point_nums = vector_nums + 1 , the origin point
%feature_point_Num = 10;
%feature_point_Num_S = 10;
data_file_name = '200poses';
%-------------------------------------------------------------------------------------------------------------
axeslength = 0.1;                                        % 1.5
XYZcolors = 'rgbcmy';
markersize = 5;                                          % feature marker
%-----------------------------extrinsic_parameters--------------------------------------------
R_c2s = eularAngle([0,0,-90]/180*pi,'zyx');
t_c2s = [0;0;-0.05];
%------------------------------------------------------------------------

x_rot_bound = [-10,10];
y_rot_bound = [-20,20];
z_rot_bound = [-3,3];
move_bound = [0.1,0.25];
    
position_list = cell(1,vector_nums);
R_list= cell(1,vector_nums);
t_list= cell(1,vector_nums);
scale_list= zeros(1,vector_nums);
%fig_handle =
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
    rand_list = rand(1,4);
    angel_x_ramdom = x_rot_bound(1) + (x_rot_bound(2)-x_rot_bound(1))*rand_list(1);
    angel_y_ramdom = y_rot_bound(1) + (y_rot_bound(2)-y_rot_bound(1))*rand_list(2);
    angel_z_ramdom = z_rot_bound(1) + (z_rot_bound(2)-z_rot_bound(1))*rand_list(3);
    scale_list(index_pose) = move_bound(1) + (move_bound(2)-move_bound(1))*rand_list(4); 
    R_list{index_pose} = eularAngle([angel_z_ramdom,angel_x_ramdom,angel_y_ramdom]/180*pi,'zxy');
    t_list{index_pose} = [0;0;scale_list(index_pose)];
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
            position_list{index_pose} = R_list{index_R}\position_list{index_pose};  %将当前增加的向量乘以旋转矩阵的逆，迭代恢复到在前一个坐标系下的坐标
            
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
%     origin_s = R_c2s*(R_list{1}\R_c2s\([0;0;0] - t_c2s) + position_list{1}) + t_c2s;
%     axex_point_s = R_c2s*(R_list{1}\R_c2s\([1;0;0]*axeslength - t_c2s) + position_list{1}) + t_c2s;
%     axey_point_s = R_c2s*(R_list{1}\R_c2s\([0;1;0]*axeslength - t_c2s) + position_list{1}) + t_c2s;
%     axez_point_s = R_c2s*(R_list{1}\R_c2s\([0;0;1]*axeslength - t_c2s) + position_list{1}) + t_c2s;
%     plot3([origin_s(1),axex_point_s(1)],[origin_s(2),axex_point_s(2)],[origin_s(3),axex_point_s(3)],'LineWidth',2,'Color',XYZcolors(4));
%     plot3([origin_s(1),axey_point_s(1)],[origin_s(2),axey_point_s(2)],[origin_s(3),axey_point_s(3)],'LineWidth',2,'Color',XYZcolors(5));
%     plot3([origin_s(1),axez_point_s(1)],[origin_s(2),axez_point_s(2)],[origin_s(3),axez_point_s(3)],'LineWidth',2,'Color',XYZcolors(6));
hold off;
position_list=[[0;0;0],position_list];    %add the origin point
R_origin =[1,0,0;...
           0,1,0;...
           0,0,1];                        %origin pose
t_origin = [0;0;0];
R_list = [{R_origin},R_list];
t_list = [{t_origin},t_list];
%============================================================draw_path_in_sonar_frame======================================================
hold on;
for index_draw = 2:vector_nums+1
    %if index_draw>1
    position_inS_p = R_c2s*position_list{index_draw-1} + t_c2s;      %previous in sonar frame
    position_inS_c = R_c2s*position_list{index_draw} + t_c2s;        %current in sonar frame
    plot3([position_inS_c(1),position_inS_p(1)],[position_inS_c(2),position_inS_p(2)],[position_inS_c(3),position_inS_p(3)],'LineWidth',1,'Color','k','LineStyle','--');
    %plot3([position_list{index_draw}(1),position_list{index_draw-1}(1)],[position_list{index_draw}(2),position_list{index_draw-1}(2)],[position_list{index_draw}(3),position_list{index_draw-1}(3)],'LineWidth',2,'Color','k');
    %else
    %    plot3([position_list{index_draw}(1),0],[position_list{index_draw}(2),0],[position_list{index_draw}(3),0],'LineWidth',2,'Color','k');
    %end
end
hold off;
%==========================================================================================================================================
view(3);
axis equal;
xlabel('x'),ylabel('y'),zlabel('z');
%save([pwd,'\ground_truth_data\ground_truth.mat'],'position_list','scale_list','R_list','t_list');
%=========================================================================================================================================
    save([pwd,'\ground_truth_data\internal_save_data\internal_save.mat'],'-regexp','[^h_fig]');
    savefig(h_fig,[pwd,'\ground_truth_data\internal_save_data\trajectory_only.fig']);    
else
    load([pwd,'\ground_truth_data\internal_save_data\internal_save.mat']);
    h_fig = openfig([pwd,'\ground_truth_data\internal_save_data\trajectory_only.fig'],'reuse');
end
%=========================================================================================================================================
%feature points generated
%point_list_local = cell(1,feature_point_Num);
%========================================generate_feature_point_AND_draw_in_sonar_frame==========================================================
length_sum = 0;
for ind_length = 1:length(t_list) 
    length_sum = length_sum + norm(t_list{ind_length});
end
%------------------------------------------------------------------------
point_num_dense = 125;
point_num_dense_sonar = 300;
%------------------------------------------------------------------------
hold on;
point_3d_list = cell(1,round(point_num_dense * length_sum));
point_3d_list_s = cell(1,round(point_num_dense_sonar * length_sum));
%indexP_c_3d = 1;
%indexP_s_3d = 1;
%-------------------------------------------------------------------------
pose_coordX_list = zeros(1,length(position_list));
pose_coordY_list = zeros(1,length(position_list));
pose_coordZ_list = zeros(1,length(position_list));
for ind_xyz = 1:length(position_list)
    position_inS = R_c2s*position_list{ind_xyz} + t_c2s;
    pose_coordX_list(ind_xyz) = position_inS(1);
    pose_coordY_list(ind_xyz) = position_inS(2);
    pose_coordZ_list(ind_xyz) = position_inS(3);
end
maxX = max(pose_coordX_list);
minX = min(pose_coordX_list);
maxY = max(pose_coordY_list);
minY = min(pose_coordY_list);
maxZ = max(pose_coordZ_list);
minZ = min(pose_coordZ_list);
X_range = maxX-minX;
Y_range = maxY-minY;
Z_range = maxZ-minZ;
extendX = 15;
extendY =15;
extendZ = 15;
for indexP_c_3d = 1:round(point_num_dense * length_sum) %length(position_list)
    rand_list_p = rand(1,3);
    X_coord = rand_list_p(1) * (X_range + extendX) + minX - 0.5 * extendX;
    Y_coord = rand_list_p(2) * (Y_range + extendY) + minY - 0.5 * extendY;
    Z_coord = rand_list_p(3) * (Z_range + extendZ) + minZ - 0.5 * extendZ;
    point_local = [X_coord; Y_coord; Z_coord];
    plot3(point_local(1),point_local(2),point_local(3),'r.','MarkerSize',markersize);
    point_3d_list{indexP_c_3d} = FeaturePointIn3DSpace(point_local,indexP_c_3d);
end
%------------------------------------------------------------------------
extendX_s = 22;
extendY_s = 22;
extendZ_s = 22;
for indexP_s_3d = 1:round(point_num_dense_sonar * length_sum) %length(position_list)
   rand_list_p = rand(1,3);
    X_coord = rand_list_p(1) * (X_range + extendX_s) + minX - 0.5 * extendX_s;
    Y_coord = rand_list_p(2) * (Y_range + extendY_s) + minY - 0.5 * extendY_s;
    Z_coord = rand_list_p(3) * (Z_range + extendZ_s) + minZ - 0.5 * extendZ_s;
    point_local = [X_coord; Y_coord; Z_coord];
    plot3(point_local(1),point_local(2),point_local(3),'b.','MarkerSize',markersize);
    point_3d_list_s{indexP_s_3d} = FeaturePointIn3DSpace(point_local,indexP_s_3d);
end
%-------------------------------------------------------------------------
hold off;
save([pwd,'\ground_truth_data\ground_truth_',data_file_name,'.mat'],'position_list','scale_list','R_list','t_list','point_3d_list','point_3d_list_s');
savefig(h_fig,[pwd,'\ground_truth_data\ground_truth_trajectory.fig']);


