clear;
close all;
clc;
%--------------------------------------------------------------------
noise_num = 13;
pose_num= 17;
end_num = 63;
markerSize = 4;
lineWidth = 0.5;
load(['..\estimated_results_plane\',num2str(noise_num),'\pre_method_2010\estimated_result_2010_start(1)_end(49)_approximate.mat']);
time_pre_method = zeros(1,end_num);

for ind = 1:end_num
    %time_without_ransac_without_optimize(ind)=sum(abs(time_pose(ind)-time_ransac_pose_fixed(ind)-time_optimization_pose_fixed(ind)));
    time_pre_method(ind) = sum(abs(time_point_list{pose_num + 1}(1:ind*5)));
end
alpha = 0.5;

plotx = (1:end_num)*5;
figure
hold on;
plot_1 = plot(plotx,time_pre_method,'m*-','LineWidth',lineWidth,'MarkerSize',markerSize);
hold off;

load([pwd,'\',num2str(noise_num),'\three_view_method_sonar_pose_fixed_',num2str(pose_num),'_approximate.mat']);
time_with_ransac_with_optimize = zeros(1,end_num);
for ind = 1:end_num
    time_with_ransac_with_optimize(ind) = sum(abs(time_point_list{ind})) + time_ransac_pose_fixed(ind) + time_optimization_pose_fixed(ind);
    %time_with_ransac_without_optimize(ind)=sum(abs(time_pose(ind)-time_optimization_pose_fixed(ind)));
end
time_with_ransac_without_optimize = zeros(1,end_num);
for ind = 1:end_num
    time_with_ransac_without_optimize(ind) = sum(abs(time_point_list{ind})) + time_ransac_pose_fixed(ind);
    %time_with_ransac_without_optimize(ind)=sum(abs(time_pose(ind)-time_optimization_pose_fixed(ind)));
end
time_without_ransac_without_optimize = zeros(1,end_num);
for ind = 1:end_num
    %time_without_ransac_without_optimize(ind)=sum(abs(time_pose(ind)-time_ransac_pose_fixed(ind)-time_optimization_pose_fixed(ind)));
    time_without_ransac_without_optimize(ind)=sum(abs(time_point_list{ind}));
end

hold on;
plot_2 = plot(plotx,time_with_ransac_with_optimize(1:end_num),'*-','Color',[0.4660, 0.6740, 0.1880],'LineWidth',lineWidth,'MarkerSize',markerSize);
plot_3 = plot(plotx,time_with_ransac_without_optimize(1:end_num),'*-','Color',[0.3010, 0.7450, 0.9330],'LineWidth',lineWidth,'MarkerSize',markerSize);
plot_4 = plot(plotx,time_without_ransac_without_optimize(1:end_num),'*-','Color',[0.9290 0.6940 0.1250],'LineWidth',lineWidth,'MarkerSize',markerSize);
plot_2.Color(4) = alpha;
plot_3.Color(4) = alpha;
plot_4.Color(4) = alpha;
hold off;
box;