close all;
clear;
clc;
%-----------------------------------------------------------------------
%subfolder = '\addCameraNoise';
%subfolder = '\addFixedCameraNoise';
weights_used = 1;
noise_camera = 1;

if noise_camera == 0
    subfolder = '\weights_used_for_RANSAC_noCameraNoise';
elseif noise_camera == 1
    if weights_used == 1
        subfolder = '\weights_used_for_RANSAC';
    else
        subfolder = '\no_weights_used_for_RANSAC';
    end
end

path_str = pwd;
posit_slash = strfind(path_str,'\');
fa_path = path_str(1:posit_slash(end)-1);   %programming2021
fa_fa_path = '..\..\';
load([fa_fa_path,'\ground_truth_data\ground_truth_200poses.mat']);
gt = scale_list;
startNoiseLevel = 1;
maxNoiseLevel = 15;

noise_level_list = startNoiseLevel:maxNoiseLevel;
Gc1 = cell(1,maxNoiseLevel);
Gc2 = cell(1,maxNoiseLevel);
Gc3 = cell(1,maxNoiseLevel);
Gc4 = cell(1,maxNoiseLevel);
Gc5 = cell(1,maxNoiseLevel);
Gc6 = cell(1,maxNoiseLevel);
Gc7 = cell(1,maxNoiseLevel);
Gc8 = cell(1,maxNoiseLevel);
Gc9 = cell(1,maxNoiseLevel);
Gc10 = cell(1,maxNoiseLevel);
Gc11 = cell(1,maxNoiseLevel);
Gc12 = cell(1,maxNoiseLevel);
Gc13 = cell(1,maxNoiseLevel);
Gc14 = cell(1,maxNoiseLevel);
Gc15 = cell(1,maxNoiseLevel);
Gc16 = cell(1,maxNoiseLevel);
Gc17 = cell(1,maxNoiseLevel);
Gc18 = cell(1,maxNoiseLevel);
Gc19 = cell(1,maxNoiseLevel);
Gc20 = cell(1,maxNoiseLevel);
Gc21 = cell(1,maxNoiseLevel);
Gc22 = cell(1,maxNoiseLevel);
Gc23 = cell(1,maxNoiseLevel);
Gc24 = cell(1,maxNoiseLevel);
Gc25 = cell(1,maxNoiseLevel);
Gc26 = cell(1,maxNoiseLevel);
Gc27 = cell(1,maxNoiseLevel);



avg1 = zeros(1,maxNoiseLevel);
avg2 = zeros(1,maxNoiseLevel);
avg3 = zeros(1,maxNoiseLevel);
avg4 = zeros(1,maxNoiseLevel);
avg5 = zeros(1,maxNoiseLevel);
avg6 = zeros(1,maxNoiseLevel);
avg7 = zeros(1,maxNoiseLevel);
avg8 = zeros(1,maxNoiseLevel);

avg9 = zeros(1,maxNoiseLevel);

avg15 = zeros(1,maxNoiseLevel);
avg16 = zeros(1,maxNoiseLevel);
avg17 = zeros(1,maxNoiseLevel);
avg18 = zeros(1,maxNoiseLevel);

med1 = zeros(1,maxNoiseLevel);
med2 = zeros(1,maxNoiseLevel);
med3 = zeros(1,maxNoiseLevel);
med4 = zeros(1,maxNoiseLevel);
med5 = zeros(1,maxNoiseLevel);
med6 = zeros(1,maxNoiseLevel);
med7 = zeros(1,maxNoiseLevel);
med8 = zeros(1,maxNoiseLevel);
med9 = zeros(1,maxNoiseLevel);
med10 = zeros(1,maxNoiseLevel);
med12 = zeros(1,maxNoiseLevel);
%---------------no camera noise refined-----------------
med13 = zeros(1,maxNoiseLevel);
med14 = zeros(1,maxNoiseLevel);
med15 = zeros(1,maxNoiseLevel);
med16 = zeros(1,maxNoiseLevel);

med17 = zeros(1,maxNoiseLevel);
med18 = zeros(1,maxNoiseLevel);
med19 = zeros(1,maxNoiseLevel);
med20 = zeros(1,maxNoiseLevel);



start_frame_setting = 1;
end_frame_setting = 190;
subfolder=[];
for noise_index = noise_level_list
    load([pwd,'\',num2str(noise_index),'\weights_used_for_RANSAC_noCameraNoise','\estimated_initial_values_start(1)_end(199).mat']);
    error_valid_abs_noCamNoise = abs(scale_estimated_valid(start_frame_setting:end_frame_setting)-gt(start_frame_setting:end_frame_setting))./gt(start_frame_setting:end_frame_setting);
    %-----------------------------------------------------------------------------------------------------
    error_ransac_abs_noCamNoise = abs(scale_estimated_range_ransac(start_frame_setting:end_frame_setting)-gt(start_frame_setting:end_frame_setting))./gt(start_frame_setting:end_frame_setting);
    %-----------------------------------------------------------------------------------------------------
    error_weights_valid_abs_noCamNoise = abs(scale_estimated_valid_w(start_frame_setting:end_frame_setting)-gt(start_frame_setting:end_frame_setting))./gt(start_frame_setting:end_frame_setting);
    %-----------------------------------------------------------------------------------------------------
    error_weights_ransac_abs_noCamNoise = abs(scale_estimated_weights_ransac(start_frame_setting:end_frame_setting)-gt(start_frame_setting:end_frame_setting))./gt(start_frame_setting:end_frame_setting);
    %-----------------------------------------------------------------------------------------------------
    load([pwd,'\',num2str(noise_index),subfolder,'\estimated_initial_values_start(1)_end(199).mat']);
    %-----valid-----
    error_valid_abs = abs(scale_estimated_valid(start_frame_setting:end_frame_setting)-gt(start_frame_setting:end_frame_setting))./gt(start_frame_setting:end_frame_setting);
    %-----ransac-----
    error_ransac_abs = abs(scale_estimated_range_ransac(start_frame_setting:end_frame_setting)-gt(start_frame_setting:end_frame_setting))./gt(start_frame_setting:end_frame_setting);
    %-----weights + valid-----
    %load([pwd,subfolder,'\',num2str(noise_index),'\result_errors_start(1)_end(199).mat'],'error_valid_weights');
    error_weights_valid_abs = abs(scale_estimated_valid_w(start_frame_setting:end_frame_setting)-gt(start_frame_setting:end_frame_setting))./gt(start_frame_setting:end_frame_setting);
    %error_weights_valid_abs = abs(scale_estimated_valid_w(start_frame:end_frame-1)-gst(start_frame:end_frame-1));
    %-----weights + ransac----
    error_weights_ransac_abs = abs(scale_estimated_weights_ransac(start_frame_setting:end_frame_setting)-gt(start_frame_setting:end_frame_setting))./gt(start_frame_setting:end_frame_setting);
%     %-----valid_seperate-----
%     error_valid_seperate_abs = abs(scale_estimated_valid_seperate(start_frame:end_frame-1)-gt(start_frame:end_frame-1));
%     %-----ransac2-----
%     error_ransac2_abs = abs(scale_estimated_range_ransac2(start_frame:end_frame-1)-gt(start_frame:end_frame-1));
%     %-----weights + valid_seperate-----
%     error_weights_valid_seperate_abs = abs(scale_estimated_valid_seperate_w(start_frame:end_frame-1)-gt(start_frame:end_frame-1));    
%     %-----weights + ransac2-----
%     error_weights_ransac2_abs = abs(scale_estimated_weights_ransac2(start_frame:end_frame-1)-gt(start_frame:end_frame-1));
    %--------------------------------------------------------------------------------------------------------
%     load([pwd,subfolder,'\',num2str(noise_index),'\add_result_start(1)_end(199).mat']);
%     error_valid_abs_addition = abs(result_addition_valid_list(start_frame_setting:end_frame_setting-1)-gt(start_frame_setting:end_frame_setting-1))./gt(start_frame_setting:end_frame_setting-1);
%     Gc9{noise_index} = error_valid_abs_addition;
%     error_ransac_abs_addition = abs(result_addition_ransac_inlier_list(start_frame_setting:end_frame_setting-1)-gt(start_frame_setting:end_frame_setting-1))./gt(start_frame_setting:end_frame_setting-1);
%     Gc10{noise_index} = error_ransac_abs_addition;
%         error_weights_valid_abs_addition = abs(result_addition_valid_w_list(start_frame_setting:end_frame_setting-1)-gt(start_frame_setting:end_frame_setting-1))./gt(start_frame_setting:end_frame_setting-1);
%     Gc11{noise_index} = error_weights_valid_abs_addition;
%         error_weights_ransac_addition = abs(result_addition_ransac_inlier_w_list(start_frame_setting:end_frame_setting-1)-gt(start_frame_setting:end_frame_setting-1))./gt(start_frame_setting:end_frame_setting-1);
%     Gc12{noise_index} = error_weights_ransac_addition;    
    
    Gc1{noise_index} = error_valid_abs_noCamNoise;
    Gc2{noise_index} = error_ransac_abs_noCamNoise;
    Gc3{noise_index} = error_weights_valid_abs_noCamNoise;
    Gc4{noise_index} = error_weights_ransac_abs_noCamNoise;
    
    Gc9{noise_index} = error_valid_abs;
    Gc10{noise_index} = error_ransac_abs;
    Gc11{noise_index} = error_weights_valid_abs;
    Gc12{noise_index} = error_weights_ransac_abs;
    
%     Gc5{noise_index} = error_valid_seperate_abs;
%     Gc6{noise_index} = error_ransac2_abs;
%     Gc7{noise_index} = error_weights_valid_seperate_abs;
%     Gc8{noise_index} = error_weights_ransac2_abs;
%-------------------------------------------------------------------------
    load([pwd,'\',num2str(noise_index),subfolder,'\refined_values_start(1)_end(199).mat']);
    Gc17{noise_index} = abs(scale_estimated_refined_valid_BA_RW(start_frame_setting:end_frame_setting)-gt(start_frame_setting:end_frame_setting))./gt(start_frame_setting:end_frame_setting);
    Gc18{noise_index} = abs(scale_estimated_refined_ransac_BA_RW(start_frame_setting:end_frame_setting)-gt(start_frame_setting:end_frame_setting))./gt(start_frame_setting:end_frame_setting);
    Gc19{noise_index} = abs(scale_estimated_refined_weights_BA_RW(start_frame_setting:end_frame_setting)-gt(start_frame_setting:end_frame_setting))./gt(start_frame_setting:end_frame_setting);
    Gc20{noise_index} = abs(scale_estimated_refined_weights_ransac_BA(start_frame_setting:end_frame_setting)-gt(start_frame_setting:end_frame_setting))./gt(start_frame_setting:end_frame_setting);
    
    Gc5{noise_index} = abs(scale_estimated_refined_valid(start_frame_setting:end_frame_setting)-gt(start_frame_setting:end_frame_setting))./gt(start_frame_setting:end_frame_setting);
    Gc6{noise_index} = abs(scale_estimated_refined_ransac(start_frame_setting:end_frame_setting)-gt(start_frame_setting:end_frame_setting))./gt(start_frame_setting:end_frame_setting);
    Gc7{noise_index} = abs(scale_estimated_refined_weights(start_frame_setting:end_frame_setting)-gt(start_frame_setting:end_frame_setting))./gt(start_frame_setting:end_frame_setting);
    Gc8{noise_index} = abs(scale_estimated_refined_weights_ransac(start_frame_setting:end_frame_setting)-gt(start_frame_setting:end_frame_setting))./gt(start_frame_setting:end_frame_setting);
    
    load([pwd,'\',num2str(noise_index),'\weights_used_for_RANSAC_noCameraNoise','\refined_values_start(1)_end(199).mat']);
    Gc13{noise_index} = abs(scale_estimated_refined_valid(start_frame_setting:end_frame_setting)-gt(start_frame_setting:end_frame_setting))./gt(start_frame_setting:end_frame_setting);
    Gc14{noise_index} = abs(scale_estimated_refined_ransac(start_frame_setting:end_frame_setting)-gt(start_frame_setting:end_frame_setting))./gt(start_frame_setting:end_frame_setting);
    Gc15{noise_index} = abs(scale_estimated_refined_weights(start_frame_setting:end_frame_setting)-gt(start_frame_setting:end_frame_setting))./gt(start_frame_setting:end_frame_setting);
    Gc16{noise_index} = abs(scale_estimated_refined_weights_ransac(start_frame_setting:end_frame_setting)-gt(start_frame_setting:end_frame_setting))./gt(start_frame_setting:end_frame_setting);
    
%     avg1(noise_index) = sum(Gc9{noise_index})/length(Gc9{noise_index});
%     avg2(noise_index) = sum(Gc10{noise_index})/length(Gc10{noise_index});
%     avg3(noise_index) = sum(Gc11{noise_index})/length(Gc11{noise_index});
%     avg4(noise_index) = sum(Gc12{noise_index})/length(Gc12{noise_index});
    avg1(noise_index) = sum(Gc17{noise_index})/length(Gc17{noise_index});
    avg2(noise_index) = sum(Gc18{noise_index})/length(Gc18{noise_index});
    avg3(noise_index) = sum(Gc19{noise_index})/length(Gc19{noise_index});
    avg4(noise_index) = sum(Gc20{noise_index})/length(Gc20{noise_index});
    avg5(noise_index) = sum(Gc5{noise_index})/length(Gc5{noise_index});
    avg6(noise_index) = sum(Gc6{noise_index})/length(Gc6{noise_index});
    avg7(noise_index) = sum(Gc7{noise_index})/length(Gc7{noise_index});
    avg8(noise_index) = sum(Gc8{noise_index})/length(Gc8{noise_index});

    
%     avg15(noise_index) = sum(Gc5{noise_index})/length(Gc5{noise_index});
%     avg16(noise_index) = sum(Gc6{noise_index})/length(Gc6{noise_index});
%     avg17(noise_index) = sum(Gc7{noise_index})/length(Gc7{noise_index});
%     avg18(noise_index) = sum(Gc8{noise_index})/length(Gc8{noise_index});
    
%     med1(noise_index) = median(Gc9{noise_index});
%     med2(noise_index) = median(Gc10{noise_index});
%     med3(noise_index) = median(Gc11{noise_index});
%     med4(noise_index) = median(Gc12{noise_index});
    
    
    med5(noise_index) = median(Gc5{noise_index});
    med6(noise_index) = median(Gc6{noise_index});
    med7(noise_index) = median(Gc7{noise_index});
    med8(noise_index) = median(Gc8{noise_index});
    
    med17(noise_index) = median(Gc17{noise_index});
    med18(noise_index) = median(Gc18{noise_index});
    med19(noise_index) = median(Gc19{noise_index});
    med20(noise_index) = median(Gc20{noise_index});
    
    med9(noise_index) = median((Gc10{noise_index}+Gc12{noise_index})/2);
      
    med12 (noise_index) = median(Gc12{noise_index});
    %---------------no camera noise refined-----------------
    med13 (noise_index) = median(Gc13{noise_index});
    med14 (noise_index) = median(Gc14{noise_index});
    med15 (noise_index) = median(Gc15{noise_index});
    med16 (noise_index) = median(Gc16{noise_index});
    
    med4(noise_index) = median(Gc4{noise_index});
end
%data1=vertcat(Gc1,Gc9,Gc2,Gc10,Gc3,Gc11,Gc4,Gc12);
data_noCamNoise = vertcat(Gc1,Gc2,Gc3,Gc4);
figure;
%set(gca, 'LooseInset', get(gca,'TightInset')) 
multiple_boxplot(data_noCamNoise.');
title('without camera noise');
ylim([0,1]);
ytickLabel_num = 100*get(gca,'ytick');
ytickLabels_text = cell(1,length(ytickLabel_num));
for i = 1:length(ytickLabel_num)
    ytickLabels_text{i}=num2str(ytickLabel_num(i));
end
yticklabels(ytickLabels_text)
%-------------------------------------------------------
figure;
title('without camera noise');
hold on;
% % % % % plot(1:maxNoiseLevel,avg1,'k*-','LineWidth',1.5);   % valid
% % % plot(1:maxNoiseLevel,avg2,'r*-','LineWidth',1.5);   % ransac
% % % plot(1:maxNoiseLevel,avg3,'*-','Color',[0.6350 0.0780 0.1840],'LineWidth',1.5);   % weights
% % % plot(1:maxNoiseLevel,avg4,'b*-','LineWidth',1.5);   % ransac weights
% % % % % plot(1:maxNoiseLevel,avg5,'d--','Color',[0.3010 0.7450 0.9330],'LineWidth',1.5);
% % % plot(1:maxNoiseLevel,avg6,'d--','Color',[0.9290 0.6940 0.1250],'LineWidth',1.5);
% % % plot(1:maxNoiseLevel,avg7,'d--','Color',[0.8500 0.3250 0.0980],'LineWidth',1.5);
% % % plot(1:maxNoiseLevel,avg8,'d--','Color',[0.4660 0.6740 0.1880],'LineWidth',1.5);

plot(1:maxNoiseLevel,med13,'k*-','LineWidth',1.5);   % valid
%plot(1:maxNoiseLevel,med2,'r*-','LineWidth',1.5);   % ransac
%plot(1:maxNoiseLevel,med3,'*-','Color',[0.6350 0.0780 0.1840],'LineWidth',1.5);   % weights
%plot(1:maxNoiseLevel,med4,'b*-','LineWidth',1.5);   % ransac weights
%%plot(1:maxNoiseLevel,med5,'d--','Color',[0.3010 0.7450 0.9330],'LineWidth',1.5);
plot(1:maxNoiseLevel,med14,'d--','Color',[0.9290 0.6940 0.1250],'LineWidth',1.5);
plot(1:maxNoiseLevel,med15,'d--','Color',[0.3010, 0.7450, 0.9330],'LineWidth',1.5);%plot(1:maxNoiseLevel,med15,'d--','Color',[0.8500 0.3250 0.0980],'LineWidth',1.5);
plot(1:maxNoiseLevel,med16,'d--','Color',[0.4660 0.6740 0.1880],'LineWidth',1.5);
legend('initial','optimization with ransac','optimization with weights','optimization with ransac & weight');
%legend('initial','with ransac','with weights','with ransac & weight','optimization with ransac','optimization with ransac & weight');
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





%-------------------------------------------------------
data1=vertcat(Gc9,Gc10,Gc11,Gc12);%,Gc9,Gc10,Gc11,Gc12);
figure;
%set(gca, 'LooseInset', get(gca,'TightInset'))
color_exist = multiple_boxplot(data1.');
title('with camera noise');
ylim([0,1]);
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



data2=vertcat(Gc4,Gc15,Gc5,Gc16,Gc6,Gc17,Gc7,Gc18,Gc8);%,Gc5,Gc6,Gc7,Gc8);
disp('------ini------');
median(Gc4{1})
disp('------valid------');
median(Gc5{1})
disp('------ransac------');
median(Gc6{1})
disp('------weights------');
median(Gc7{1})
disp('------weights+ransac------');
median(Gc8{1})

%data2=vertcat(Gc4,Gc15,Gc16,Gc17,Gc18);
%data2=vertcat(Gc4,Gc5,Gc6,Gc7,Gc8);
figure;
%set(gca, 'LooseInset', get(gca,'TightInset'))
 
multiple_boxplot(data2.');
ylim([0,0.3]);

% % % % figure;
% % % % start_num = 91;
% % % % end_num = 125;
% % % % hold on;
% % % % plot(start_num:end_num,gt(start_num:end_num),'k-','LineWidth',2);
% % % % plot(start_num:end_num,scale_estimated_valid(start_num:end_num),'m--','LineWidth',2);
% % % % plot(start_num:end_num,scale_estimated_range_ransac(start_num:end_num),'r--','LineWidth',2);
% % % % plot(start_num:end_num,scale_estimated_valid_w(start_num:end_num),'--','Color',[0.9290 0.6940 0.1250],'LineWidth',2);
% % % % plot(start_num:end_num,scale_estimated_weights_ransac(start_num:end_num),'--','Color',[0 0.4470 0.7410],'LineWidth',2);
% % % % legend('ground truth','w/o ransac / weights','with ransac','with weights','with ransac & weight');
% % % % hold off;

figure;
plot_v = plot(1:maxNoiseLevel,med5,'d--','Color',color_exist(4,1:3),'LineWidth',1.5);%plot(1:maxNoiseLevel,med5,'d--','Color',[0.3010 0.7450 0.9330],'LineWidth',1.5);
plot_v.Color(4) = 0.5;
legend('optimization w/o ransac/weights');
ylim([0.3,1]);
ytickLabel_num = 100*get(gca,'ytick');
ytickLabels_text = cell(1,length(ytickLabel_num));
for i = 1:length(ytickLabel_num)
    ytickLabels_text{i}=num2str(ytickLabel_num(i));
end
yticklabels(ytickLabels_text)
xlim([0,16]);
xticks(0:1:16);


figure;
title('with camera noise');
hold on;
% % % % % plot(1:maxNoiseLevel,avg1,'k*-','LineWidth',1.5);   % valid
% % % plot(1:maxNoiseLevel,avg2,'r*-','LineWidth',1.5);   % ransac
% % % plot(1:maxNoiseLevel,avg3,'*-','Color',[0.6350 0.0780 0.1840],'LineWidth',1.5);   % weights
% % % plot(1:maxNoiseLevel,avg4,'b*-','LineWidth',1.5);   % ransac weights
% % % % % plot(1:maxNoiseLevel,avg5,'d--','Color',[0.3010 0.7450 0.9330],'LineWidth',1.5);
% % % plot(1:maxNoiseLevel,avg6,'d--','Color',[0.9290 0.6940 0.1250],'LineWidth',1.5);
% % % plot(1:maxNoiseLevel,avg7,'d--','Color',[0.8500 0.3250 0.0980],'LineWidth',1.5);
% % % plot(1:maxNoiseLevel,avg8,'d--','Color',[0.4660 0.6740 0.1880],'LineWidth',1.5);

%plot(1:maxNoiseLevel,med9,'k*-','LineWidth',1.5);   % valid
plot(1:maxNoiseLevel,med12,'k*-','LineWidth',1.5);   % initial value
%plot(1:maxNoiseLevel,med2,'r*-','LineWidth',1.5);   % ransac
%plot(1:maxNoiseLevel,med3,'*-','Color',[0.6350 0.0780 0.1840],'LineWidth',1.5);   % weights
%plot(1:maxNoiseLevel,med4,'b*-','LineWidth',1.5);   % ransac weights
%%plot(1:maxNoiseLevel,med5,'d--','Color',[0.3010 0.7450 0.9330],'LineWidth',1.5);
plot_r = plot(1:maxNoiseLevel,med6,'d--','Color',color_exist(3,1:3),'LineWidth',1.5);   % ;[0.9290 0.6940 0.1250]        r
plot_r.Color(4) = 0.5;
plot_w = plot(1:maxNoiseLevel,med7,'d--','Color',[0.4660 0.6740 0.1880],'LineWidth',1.5); %[ 0.3010, 0.7450, 0.9330]  color_exist(2,1:3)  w
plot_w.Color(4) = 0.5;
plot_wr = plot(1:maxNoiseLevel,med8,'d--','Color',[0.9290 0.6940 0.1250],'LineWidth',1.5); %[0.4660 0.6740 0.1880]  color_exist(1,1:3)    wr
plot_wr.Color(4) = 0.5;
legend('initial','optimization with ransac','optimization with weights','optimization with ransac & weight');
%legend('initial','with ransac','with weights','with ransac & weight','optimization with ransac','optimization with ransac & weight');
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
% % ylim([0.02,0.125]);
%'w/o ransac / weights',,'optimization w/o ransac/weights','optimization with weights'


figure;
hold on;
plot(1:maxNoiseLevel,avg2,'r*-','LineWidth',1.5);
plot(1:maxNoiseLevel,avg9,'b*-','LineWidth',1.5);
hold off;

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