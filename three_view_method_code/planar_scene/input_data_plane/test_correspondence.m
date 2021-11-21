clear;
close all;
clc;
path_str = pwd;
posit_slash = strfind(path_str,'\');
fa_path = path_str(1:posit_slash(end)-1);
addpath([fa_path,'\PointClasses'],[fa_path,'\PoseClasses'],[fa_path,'\plotAxesAndPoints']);
load([pwd,'\sensor_measurements_50posesPlane.mat'])
for i=1:48
    list1=[];
    list2=[];
    list3=[];
    for j = 1:length(sonar_image_list{i})
        list1(j) = sonar_image_list{i}{j}.feature_index;
    end
    for j = 1:length(sonar_image_list{i+1})
        list2(j) = sonar_image_list{i+1}{j}.feature_index;
    end
    for j = 1:length(sonar_image_list{i+2})
        list3(j) = sonar_image_list{i+2}{j}.feature_index;
    end
    [common1, index_a1, index_b1] = intersect(list1, list2);
    [common2, index_a2, index_b2] = intersect(list2, list3);
    [common_test, ind_test1, ind_test2] = intersect(common1, common2);
    length_list(i) = length(common_test);
end
length_list