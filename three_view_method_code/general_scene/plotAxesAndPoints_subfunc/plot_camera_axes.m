function plot_camera_axes(CameraViewPoint,axeslength,XYZcolors)
%CameraViewPoint incoporates the camera pose with respect to sonar.
%SonarViewPoint incoporates the world pose with respect to sonar.
    %R_sonar = SonarViewPoint.R_matrix;
    R_c2s = CameraViewPoint.R_matrix;
    
    T_c2s = CameraViewPoint.T_vector;

    origin = T_c2s;
    axex_c = R_c2s *[1;0;0]*axeslength + T_c2s;
    axey_c = R_c2s *[0;1;0]*axeslength + T_c2s;
    axez_c = R_c2s *[0;0;1]*axeslength + T_c2s;
    
    hold on;
    plot3([origin(1),axex_c(1)],[origin(2),axex_c(2)],[origin(3),axex_c(3)],'LineWidth',2,'Color',XYZcolors(1));
    plot3([origin(1),axey_c(1)],[origin(2),axey_c(2)],[origin(3),axey_c(3)],'LineWidth',2,'Color',XYZcolors(2));
    plot3([origin(1),axez_c(1)],[origin(2),axez_c(2)],[origin(3),axez_c(3)],'LineWidth',2,'Color',XYZcolors(3));
    

    name_x = (axex_c - origin)*1.2 + origin;
    name_y = (axey_c - origin)*1.2 + origin;
    name_z = (axez_c - origin)*1.2 + origin;

    text(name_x(1),name_x(2),name_x(3),'X_c','FontSize',15);       %为坐标轴添加标注
    text(name_y(1),name_y(2),name_y(3),'Y_c','FontSize',15);
    text(name_z(1),name_z(2),name_z(3),'Z_c','FontSize',15);
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

