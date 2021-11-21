clear all;
close all;
clc;
%for i =1:15
%  i=10;
%  load([pwd,'\',num2str(i),'\pre_method_2010\estimated_result_2010_start(1)_end(49)_approximate.mat']);
%  numberOfPoint = zeros(1,length(time_point_list));
%  for j = 1:length(valid_list_list)
%      %numberOfPoint(j) = length(time_point_list{j});
%      numberOfPoint(j) = sum(valid_list_list{j});
%  end
%  [pointNum_sort,pointNum_sort_index] = sort(numberOfPoint);
%  plotx1 = pointNum_sort;
%  ploty1 = time_pose(pointNum_sort_index);
 %==================================================
i = 8;
load([pwd,'\',num2str(i),'\pre_method_2010\estimated_result_2010_start(1)_end(49)_approximate.mat']);
numberOfPoint_pre = zeros(1,length(time_point_list));
for j = 1:length(time_point_list)
      %numberOfPoint(j) = length(time_point_list{j});
 numberOfPoint_pre(j) = sum(length(valid_list_list{j}));
end
time_point_total_pre = [];% = zeros(1,sum(numberOfPointValid));
 
for j = 1:length(time_point_list)
 time_point_total_pre = [time_point_total_pre,time_point_list{j}(logical(valid_list_list{j}))];
end
%length(time_point_total)
%sum(numberOfPointValid)

numberOfPoint_pre = numberOfPoint_pre(1:end-1);
time_pose = time_pose(1:end-1);
[pointNum_sort,pointNum_sort_index] = sort(numberOfPoint_pre);
plotx_pre = pointNum_sort;
ploty_pre = time_pose(pointNum_sort_index);
%--------------------------------------------------------
alpha = 0.5;
lineWidth = 1.5;
figure;
hold on;
%plot(plotx1,ploty1,'r');
plot1 = plot(plotx_pre,ploty_pre,'m*-','LineWidth',lineWidth);
hold off;
load([pwd,'\',num2str(i),'\three_view_method\estimated_result_three_views_start(1)_end(50)_approximate_laptop1.mat']);
[pointNum_sort,pointNum_sort_index] = sort(number_of_valid_list);
plotx_our_optimized = pointNum_sort;
ploty_our_optimized = time_pose(pointNum_sort_index);
time_weights_ransac = zeros(1,length(time_point_list));
for j=1:length(time_point_list)
    time_weights_ransac(j) = time_pose(j)-time_optimization(j);        
end
ploty_our_with_ransac = time_weights_ransac(pointNum_sort_index);
%-----------------------------------------------------------------------
time_point_total_three_view = [];
time_weights_only = zeros(1,length(time_point_list));
%time_point_total_three_view = [];
load([pwd,'\',num2str(i),'\three_view_method\estimated_result_three_views_start(1)_end(50)_approximate_weight.mat']);
for j=1:length(time_point_list)    
    time_point_total_three_view = [time_point_total_three_view,abs(time_point_list{j})];    % no weights
    time_weights_only(j) = time_pose(j)-time_optimization(j);        
    
    %time_point_total_three_view = [time_point_total_three_view,time_point_list{i}(time_point_list{i}>0)];
    %time_pose_with_RANSAC_optimize_for_one_point(i) = (time_pose{i}-time_pose_total_three_view(i))/sum(time_point_list{i}>0);
end
%ploty4 = time_pose(pointNum_sort_index) - time_optimization(pointNum_sort_index);
ploty_our_with_weights = time_weights_only(pointNum_sort_index);
%-------------------------------------------------------------------------
hold on;%[0.9290 0.6940 0.1250]
plot_2 = plot(plotx_our_optimized,ploty_our_with_weights,'*-','Color',[0.9290 0.6940 0.1250],'LineWidth',lineWidth);
plot_3 = plot(plotx_our_optimized,ploty_our_with_ransac,'*-','Color',[0.3010, 0.7450, 0.9330],'LineWidth',lineWidth);  %without
plot_4 = plot(plotx_our_optimized,ploty_our_optimized,'*-','Color',[0.4660, 0.6740, 0.1880],'LineWidth',lineWidth);
plot_2.Color(4) = alpha;
plot_3.Color(4) = alpha;
plot_4.Color(4) = alpha;
%plot(plotx3,ploty4,'m');
%==================================================================================

%==============================================================================================
hold off;
legend('Method in [23]','Our method with weights','Our method with weights & Ransac','Our method with Ransac & optimization');
box on;
%------------------------------------------------------------------------------------

%------------------------------------------------------------------------------------
figure;
hold on;
plot_4 = plot(plotx_our_optimized,ploty_our_with_ransac,'*-','Color',[0 0.4470 0.7410],'LineWidth',lineWidth);
%plot_5 = plot(plotx_our_optimized,ploty_our_optimized,'*-','Color',[0.9290 0.6940 0.1250],'LineWidth',lineWidth);
%plot(plotx3,ploty4,'m')
plot_4.Color(4) = alpha;
%plot_5.Color(4) = alpha;
%set(gca,'YTick',[0:150:1500]);
hold off;
% % hold on;
% % plot(1:49,numberOfPointValid,'r');
% % plot(1:49,number_of_valid_list,'b');
% % plot(1:49,number_of_valid_list,'g');
% % hold off;
% % legend('Method in [24]','Our method with Ransac & optimization','Our method w/o Ransac/optimization');
% mean(numberOfPointValid)
% mean(number_of_valid_list)
figure;
boxplot(time_point_total_pre,'Color',[1 0 1]);
median(time_point_total_pre)
set(gca,'ylim',[0.2,15]);
disp('time_point_total_pre');
disp(length(time_point_total_pre));
figure;
boxplot(time_point_total_three_view,'Color',[0 0.4470 0.7410]);
median(time_point_total_three_view)
set(gca,'ylim',[0.2,0.25]);
disp('time_point_total_three_view');
disp(length(time_point_total_three_view));





%length(time_point_total_three_view)
% % figure;
% % group = [repmat({'Method in [24]'}, length(time_point_total_pre), 1);...
% %     repmat({'Our method w/o Ransac/optimization'}, length(time_point_total_three_view), 1)...
% %     repmat({'computation Time for RANSAC'}length(time_pose_with_RANSAC_optimize_for_one_point),1)];
% % combined_time_list = [time_point_total_pre,time_point_total_three_view,time_pose_with_RANSAC_optimize];
% % hold on;
% % boxplot(combined_time_list,group);
% % hold off;
