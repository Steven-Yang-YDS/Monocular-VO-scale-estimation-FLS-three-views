function [unit_vectorXYZ,origin] = plot_axes(CameraViewPoint,referenceFrame,axeslength,XYZcolors,AxesTexts)
%CameraViewPoint incoporates the camera pose with respect to sonar.
%SonarViewPoint incoporates the world pose with respect to sonar.
    %R_sonar = SonarViewPoint.R_matrix;
    ref_R = referenceFrame.R_matrix;
    ref_T = referenceFrame.T_vector;
    R_c2s = CameraViewPoint.R_matrix;    
    T_c2s = CameraViewPoint.T_vector;

    origin = ref_R*T_c2s + ref_T;
    axex_c = ref_R*(R_c2s *[1;0;0]*axeslength + T_c2s) + ref_T;
    axey_c = ref_R*(R_c2s *[0;1;0]*axeslength + T_c2s) + ref_T;
    axez_c = ref_R*(R_c2s *[0;0;1]*axeslength + T_c2s) + ref_T;
    
    hold on;
    plot3([origin(1),axex_c(1)],[origin(2),axex_c(2)],[origin(3),axex_c(3)],'LineWidth',2,'Color',XYZcolors(1));
    plot3([origin(1),axey_c(1)],[origin(2),axey_c(2)],[origin(3),axey_c(3)],'LineWidth',2,'Color',XYZcolors(2));
    plot3([origin(1),axez_c(1)],[origin(2),axez_c(2)],[origin(3),axez_c(3)],'LineWidth',2,'Color',XYZcolors(3));
    
    unit_vectorX = axex_c - origin;
    unit_vectorY = axey_c - origin;
    unit_vectorZ = axez_c - origin;
    unit_vectorXYZ = cat(2,unit_vectorX,unit_vectorY,unit_vectorZ);
    
    name_x = unit_vectorX*1.3 + origin;
    name_y = unit_vectorY*1.3 + origin;
    name_z = unit_vectorZ*1.3 + origin;

%     X_name = ['X_',AxesTexts];
%     Y_name = ['Y_',AxesTexts];
%     Z_name = ['Z_',AxesTexts];
    text(name_x(1),name_x(2),name_x(3),['X_{',AxesTexts,'}'],'FontSize',15);       %为坐标轴添加标注
    text(name_y(1),name_y(2),name_y(3),['Y_{',AxesTexts,'}'],'FontSize',15);
    text(name_z(1),name_z(2),name_z(3),['Z_{',AxesTexts,'}'],'FontSize',15);
    hold off

end
%     R_s2c = CameraViewPoint.R_matrix;
% 
%     %T_sonar = SonarViewPoint.T_vector;
%     T_s2c = CameraViewPoint.T_vector;
% 
% % unitx = R_sonar * R_s2c'*[1;0;0];
% % unity = R_sonar * R_s2c'*[0;1;0];
% % unitz = R_sonar * R_s2c'*[0;0;1];
% origin = R_s2c.'*([0;0;0]- T_s2c);
% axex_s = R_s2c.'*([1;0;0]*axeslength -T_s2c);
% axey_s = R_s2c.'*([0;1;0]*axeslength -T_s2c);
% axez_s = R_s2c.'*([0;0;1]*axeslength -T_s2c);
% 
% hold on;
% 
% 
% plot3([origin(1),axex_s(1)],[origin(2),axex_s(2)],[origin(3),axex_s(3)],'LineWidth',2,'Color',XYZcolors(1));
% 
% plot3([origin(1),axey_s(1)],[origin(2),axey_s(2)],[origin(3),axey_s(3)],'LineWidth',2,'Color',XYZcolors(2));
% 
% plot3([origin(1),axez_s(1)],[origin(2),axez_s(2)],[origin(3),axez_s(3)],'LineWidth',2,'Color',XYZcolors(3));

