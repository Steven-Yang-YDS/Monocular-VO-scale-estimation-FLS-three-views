close all;
clear;
clc;
%-----------------------------------------------------------------------
%subfolder = '\addCameraNoise';
%subfolder = '\addFixedCameraNoise';
%subfolder = '\addFixedCameraNoisePreCompute';
subfolder = [];

path_str = pwd;
posit_slash = strfind(path_str,'\');
fa_path = path_str(1:posit_slash(end)-1);   

load([fa_path,'\ground_truth_data_plane\ground_truth_50posesPlane.mat']);
noise_level_list = 1:15;
% Gc1 = cell(1,15);
% Gc2 = cell(1,15);
% Gc3 = cell(1,15);
% Gc4 = cell(1,15);
% Gc5 = cell(1,15);
Gc1 = cell(1,1);
Gc2 = cell(1,1);
%Gc3 = cell(1,1);
Gc4 = cell(1,1);
Gc5 = cell(1,1);
Gc6 = cell(1,1);
avg1 = zeros(1,15);
avg2 = zeros(1,15);
%avg3 = zeros(1,15);
avg4 = zeros(1,15);
avg5 = zeros(1,15);
avg6 = zeros(1,15);

med1 = zeros(1,15);
med2 = zeros(1,15);
med4 = zeros(1,15);
med5 = zeros(1,15);
med6 = zeros(1,15);
for noise_index = noise_level_list
    load([pwd,'\',num2str(noise_index),'\pre_method_2010\estimated_result_2010_start(1)_end(49)_approximate.mat'],'scale_estimated_poses','valid_list_list');
    %-----previous method------------------------------------------------------------------------
    error_list_pre = zeros(1,length(scale_estimated_poses));
    sum_valid_num_pre = 0;
    for index_error = 1:length(scale_estimated_poses)
        error_list_pre(index_error) = abs(scale_estimated_poses{index_error}- scale_list(index_error+1))/ scale_list(index_error+1);
        if index_error ~= 50
            sum_valid_num_pre = sum_valid_num_pre + sum(valid_list_list{index_error});
        end
    end
    valid_list_pre_noise_level(noise_index) = sum_valid_num_pre;
    load([pwd,'\',num2str(noise_index),'\three_view_method\estimated_result_three_views_start(1)_end(50)_approximate_laptop1.mat'],...
        'scale_estimated_weights_ransac','scale_estimated_refined_weights_ransac','ransac_valid_num_list');
    %-----our method-----
    error_list_new = abs(scale_estimated_weights_ransac - scale_list(2:length(scale_estimated_weights_ransac)+1))./scale_list(2:length(scale_estimated_weights_ransac)+1);
    error_list_new_refined = abs(scale_estimated_refined_weights_ransac - scale_list(2:length(scale_estimated_refined_weights_ransac)+1))./scale_list(2:length(scale_estimated_weights_ransac)+1);
    valid_list_three_noise_level(noise_index) = sum(ransac_valid_num_list);
    %--------------------------------------------------------------------------------------------------------    
    load([pwd,'\',num2str(noise_index),'\three_view_method\estimated_result_three_views_start(1)_end(50)_approximate_weight.mat'],'scale_estimated_weights','scale_estimated_refined_weights');
    
    %-----our method-----
    error_list_new_only_w = abs(scale_estimated_weights - scale_list(2:length(scale_estimated_weights)+1))./scale_list(2:length(scale_estimated_weights)+1);
    %error_list_new_refined_only_w = abs(scale_estimated_refined_weights_ransac - scale_list(2:length(scale_estimated_refined_weights_ransac)+1));
    %--------------------------------------------------------------------------------------------------------    
    length_min = min(length(error_list_pre),length(error_list_new));
%     Gc1{1} = error_list_pre(1:length_min);
%     Gc2{1} = error_list_new_only_w(1:length_min);
%     Gc3{1} = error_list_new_refined_only_w(1:length_min);
%     Gc4{1} = error_list_new(1:length_min);
%     Gc5{1} = error_list_new_refined(1:length_min);
    Gc1{noise_index} = error_list_pre(1:length_min);
    Gc2{noise_index} = error_list_new_only_w(1:length_min);
    %Gc3{noise_index} = error_list_new_refined_only_w(1:length_min);
    Gc4{noise_index} = error_list_new(1:length_min);
    Gc5{noise_index} = error_list_new_refined(1:length_min);
%-------------------------------------------------------------------------
    avg1(noise_index) = sum(Gc1{noise_index})/length(Gc1{noise_index});
    avg2(noise_index) = sum(Gc2{noise_index})/length(Gc2{noise_index});
    %avg3(noise_index) = sum(Gc3{noise_index})/length(Gc3{noise_index});
    avg4(noise_index) = sum(Gc4{noise_index})/length(Gc4{noise_index});
    avg5(noise_index) = sum(Gc5{noise_index})/length(Gc5{noise_index});
    %-------------------------------------------------------------------
    med1(noise_index) = median(Gc1{noise_index});
    med2(noise_index) = median(Gc2{noise_index});
    %avg3(noise_index) = sum(Gc3{noise_index})/length(Gc3{noise_index});
    med4(noise_index) = median(Gc4{noise_index});
    med5(noise_index) = median(Gc5{noise_index});
    load(['..\optimization_result_and_time\',num2str(noise_index),'\optimization_result_and_time.mat']);
    error_pre_BA = abs(output_refined_pre_list - scale_list(2:length(output_refined_pre_list)+1))./scale_list(2:length(output_refined_pre_list)+1);
    Gc6{noise_index} = error_pre_BA(1:length_min);
    avg6(noise_index) = sum(Gc6{noise_index})/length(Gc6{noise_index});
    med6(noise_index) = median(Gc6{noise_index});
end

data1=vertcat(Gc1,Gc6,Gc2,Gc4,Gc5);%,Gc5,Gc6,Gc7,Gc8);
figure;
%set(gca, 'LooseInset', get(gca,'TightInset')) 
alpha = 0.5;
colors = [0.4660, 0.6740, 0.1880,1*alpha;
          0, 0.4470, 0.7410, 1*alpha;    
          0.3010, 0.7450, 0.9330,1*alpha;
         % 0, 0.4470, 0.7410, 1*alpha;    
          0.9290, 0.6940, 0.1250,1*alpha;
          1,0,1,1*alpha ];                                                   
xlab = {'1','2','3','4','5','6','7','8','9','10','11','12','13','14','15'};
Mlab = {'Method in [24]','Method in [24] + BA','Our method + weight','Our method + weights + RANSAC','Our method + weights + RANSAC + optimization'};%,'Our method + weights + optimization'
multiple_boxplot(data1.',xlab,Mlab,colors);
set(gca,'YLim',[0 0.1])
ytickLabel_num = 100*get(gca,'ytick');
ytickLabels_text = cell(1,length(ytickLabel_num));
for i = 1:length(ytickLabel_num)
    ytickLabels_text{i}=num2str(ytickLabel_num(i));
end
yticklabels(ytickLabels_text)

extend_percent= 0.03;
xlim_range = get(gca,'XLim');
extend_length = (xlim_range(2)-xlim_range(1))*extend_percent;
xlim_range = xlim_range + [-extend_length,extend_length];
set(gca,'XLim',xlim_range)
% figure;
% start_num = 91;
% end_num = 125;
% hold on;
% plot(start_num:end_num,gt(start_num:end_num),'k-','LineWidth',2);
% plot(start_num:end_num,scale_estimated_valid(start_num:end_num),'m--','LineWidth',2);
% plot(start_num:end_num,scale_estimated_range_ransac(start_num:end_num),'r--','LineWidth',2);
% plot(start_num:end_num,scale_estimated_valid_seperate_w(start_num:end_num),'--','Color',[0.9290 0.6940 0.1250],'LineWidth',2);
% plot(start_num:end_num,scale_estimated_weights_ransac(start_num:end_num),'b--','Color',[0 0.4470 0.7410],'LineWidth',2);
% legend('ground truth','w/o ransac / weights','with ransac','with weights','with ransac & weight');
% hold off;

%======================================================
figure;
title('mean');
hold on;
plot1 = plot(1:15,avg1,'m*-','LineWidth',1.5);   % pre method
plot6 = plot(1:15,avg6,'*-','Color',[0 0.4470 0.7410],'LineWidth',1.5);   % pre method + BA
plot2 = plot(1:15,avg2,'*-','Color',[0.9290 0.6940 0.1250],'LineWidth',1.5);   % weights
%plot3 = plot(1:15,avg3,'*-','Color',[0 0.4470 0.7410],'LineWidth',1.5);   % 
plot4 = plot(1:15,avg4,'*-','Color',[0.3010, 0.7450, 0.9330],'LineWidth',1.5);   %  weights  +  ransac
plot5 = plot(1:15,avg5,'*-','Color',[0.4660, 0.6740, 0.1880],'LineWidth',1.5);   % weights  +  ransac + optimizatioin
plot1.Color(4) = alpha;
plot6.Color(4) = alpha;
plot2.Color(4) = alpha;
%plot3.Color(4) = alpha;
plot4.Color(4) = alpha;
plot5.Color(4) = alpha;
legend('Method in [22]','Method in [24] + BA','Our method + weight','Our method + weights + RANSAC','Our method + weights + RANSAC + optimization');%,'Our method + weights + optimization'
%set(gca,'xticklabel',get(gca,'xtick'),'yticklabel',get(gca,'ytick'));
hold off;
ytickLabel_num = 100*get(gca,'ytick');
ytickLabels_text = cell(1,length(ytickLabel_num));
for i = 1:length(ytickLabel_num)
    ytickLabels_text{i}=num2str(ytickLabel_num(i));
end
yticklabels(ytickLabels_text)
xlim([0,16]);
xticks(0:1:16);
box on;
%======================================================
figure;
hold on;
plot1 = plot(1:15,med1,'m*-','LineWidth',1.5);   % pre method
plot6 = plot(1:15,med6,'*-','Color',[0 0.4470 0.7410],'LineWidth',1.5);   % pre method + BA
plot2 = plot(1:15,med2,'*-','Color',[0.9290 0.6940 0.1250],'LineWidth',1.5);   % weights
%plot3 = plot(1:15,avg3,'*-','Color',[0 0.4470 0.7410],'LineWidth',1.5);   % 
plot4 = plot(1:15,med4,'*-','Color',[0.3010, 0.7450, 0.9330],'LineWidth',1.5);   %  weights  +  ransac
plot5 = plot(1:15,med5,'*-','Color',[0.4660, 0.6740, 0.1880],'LineWidth',1.5);   % weights  +  ransac + optimizatioin
plot1.Color(4) = alpha;
plot6.Color(4) = alpha;
plot2.Color(4) = alpha;
%plot3.Color(4) = alpha;
plot4.Color(4) = alpha;
plot5.Color(4) = alpha;
legend('Method in [22]','Method in [24] + BA','Our method + weight','Our method + weights + RANSAC','Our method + weights + RANSAC + optimization');%,'Our method + weights + optimization'
%set(gca,'xticklabel',get(gca,'xtick'),'yticklabel',get(gca,'ytick'));
hold off;
ytickLabel_num = 100*get(gca,'ytick');
ytickLabels_text = cell(1,length(ytickLabel_num));
for i = 1:length(ytickLabel_num)
    ytickLabels_text{i}=num2str(ytickLabel_num(i));
end
yticklabels(ytickLabels_text)
xlim([0,16]);
xticks(0:1:16);
box on;

%======================================================
%ylim([0.02,0.125]);
%xlim([0,16]);
%xticks(0:1:16);
% or
%%figure;
%%multiple_boxplot(data)

% 
% 
% 
% MRL2 = {ratio1, ratio2, ratio3, ratio4, ratio5, ratio6, ratio7, ratio8, ratio9};
%  
% % Create example data
% G1=MRL2{1};
% G2=MRL2{2};
% G3=MRL2{3};
% G4=MRL2{4};
% G5=MRL2{5};
% G6=MRL2{6};
% G7=MRL2{7};
% G8=MRL2{8};
% G9=MRL2{9};
%  
% % prepare data
% data=cell(6,9);
% for ii=1:size(data,1)
% Gc1{ii}=G1(:,ii);
% Gc2{ii}=G2(:,ii);
% Gc3{ii}=G3(:,ii);
% Gc4{ii}=G4(:,ii);
% Gc5{ii}=G5(:,ii);
% Gc6{ii}=G6(:,ii);
% Gc7{ii}=G7(:,ii);
% Gc8{ii}=G8(:,ii);
% Gc9{ii}=G9(:,ii);
% end
% data=vertcat(Gc1,Gc2,Gc3,Gc4,Gc5,Gc6,Gc7,Gc8,Gc9);
% 
% figure
% %set(gca, 'LooseInset', get(gca,'TightInset'))
%  
% multiple_boxplot(data')
% % or
% multiple_boxplot(data)