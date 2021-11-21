clc;
close all;
clear;
format long;
%-------------------------------------------------------------------------
path_str = pwd;
posit_slash = strfind(path_str,'\');
fa_path = path_str(1:posit_slash(end)-1);   %programming2021
%-------------------------------------------------------------------------
noiseNum = 7;
frameNum = 77;
load([fa_path,'\ground_truth_data\ground_truth_200poses.mat']);
open([fa_path,'\figure_save\',num2str(noiseNum),'\valid\\Pose_',num2str(frameNum),'.fig']);
ph = findall(gca,'type','Line');
xc=get(ph,'xdata');
yc=get(ph,'ydata');
yc_new = zeros(1,length(xc));
for i=1:length(xc)
    yc_new(i) = yc{i}/scale_list(frameNum);
end
figure;
hold on;
for i=1:length(xc)  
    plot(xc{i},yc_new(i),'r.','Markersize',5);
end
hold off;
ylim([0,1]);
ytickLabel_num = 100*get(gca,'ytick');
ytickLabels_text = cell(1,length(ytickLabel_num));
for i = 1:length(ytickLabel_num)
    ytickLabels_text{i}=num2str(ytickLabel_num(i));
end
yticklabels(ytickLabels_text)
box on;