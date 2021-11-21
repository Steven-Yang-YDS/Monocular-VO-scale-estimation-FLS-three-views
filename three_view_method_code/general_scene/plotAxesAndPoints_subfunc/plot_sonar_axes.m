function plot_sonar_axes(axeslength,XYZcolors)
%sonarview = SonarViewPoint(20,30,40,20,30,40);
%axeslength = (SonarViewPoint.T_vector(1)+SonarViewPoint.T_vector(2)+SonarViewPoint.T_vector(3))/3;
% unitx = SonarViewPoint.R_matrix*[1;0;0];
% unity = SonarViewPoint.R_matrix*[0;1;0];
% unitz = SonarViewPoint.R_matrix*[0;0;1];
% unitx_w = unitx*axeslength + SonarViewPoint.T_vector;
% unity_w = unity*axeslength + SonarViewPoint.T_vector;
% unitz_w = unitz*axeslength + SonarViewPoint.T_vector;
unitx = [1;0;0];
unity = [0;1;0];
unitz = [0;0;1];
unitx_w = unitx*axeslength;
unity_w = unity*axeslength;
unitz_w = unitz*axeslength;
view(3);
axis equal;
hold on;
% if (~isequal(SonarViewPoint.RT_matrix, eye(4)))
%     x0 = axeslength*[1;0;0];
%     y0 = axeslength*[0;1;0];
%     z0 = axeslength*[0;0;1];
% 
%     plot3([0,x0(1)],[0,x0(2)],[0,x0(3)],'LineWidth',2);
%     plot3([0,y0(1)],[0,y0(2)],[0,y0(3)],'LineWidth',2);
%     plot3([0,z0(1)],[0,z0(2)],[0,z0(3)],'LineWidth',2);
%     text(x0(1)*1.2,x0(2)*1.2,x0(3)*1.2,'X_w','FontSize',15);       %为坐标轴添加标注
%     text(y0(1)*1.2,y0(2)*1.2,y0(3)*1.2,'Y_w','FontSize',15);
%     text(z0(1)*1.2,z0(2)*1.2,z0(3)*1.2,'Z_w','FontSize',15);
% end

xlabel('axis X');
ylabel('axis Y');
zlabel('axis Z');

plot3([0,unitx_w(1)],[0,unitx_w(2)],[0,unitx_w(3)],'LineWidth',2,'Color',XYZcolors(1));
plot3([0,unity_w(1)],[0,unity_w(2)],[0,unity_w(3)],'LineWidth',2,'Color',XYZcolors(1));
plot3([0,unitz_w(1)],[0,unitz_w(2)],[0,unitz_w(3)],'LineWidth',2,'Color',XYZcolors(1));
axis_x = unitx_w *1.2;
axis_y = unity_w *1.2;
axis_z = unitz_w *1.2;

% plot3([SonarViewPoint.pos_x,unitx_w(1)],[SonarViewPoint.pos_y,unitx_w(2)],[SonarViewPoint.pos_z,unitx_w(3)],'LineWidth',2);
% 
% plot3([SonarViewPoint.pos_x,unity_w(1)],[SonarViewPoint.pos_y,unity_w(2)],[SonarViewPoint.pos_z,unity_w(3)],'LineWidth',2);
% 
% plot3([SonarViewPoint.pos_x,unitz_w(1)],[SonarViewPoint.pos_y,unitz_w(2)],[SonarViewPoint.pos_z,unitz_w(3)],'LineWidth',2);

% vector_x = [unitx_w(1)-SonarViewPoint.pos_x,unitx_w(2)-SonarViewPoint.pos_y,unitx_w(3)-SonarViewPoint.pos_z];
% vector_y = [unity_w(1)-SonarViewPoint.pos_x,unity_w(2)-SonarViewPoint.pos_y,unity_w(3)-SonarViewPoint.pos_z];
% vector_z = [unitz_w(1)-SonarViewPoint.pos_x,unitz_w(2)-SonarViewPoint.pos_y,unitz_w(3)-SonarViewPoint.pos_z];

% axis_x = (unitx_w - SonarViewPoint.T_vector)*1.2 + SonarViewPoint.T_vector;
% axis_y = (unity_w - SonarViewPoint.T_vector)*1.2 + SonarViewPoint.T_vector;
% axis_z = (unitz_w - SonarViewPoint.T_vector)*1.2 + SonarViewPoint.T_vector;

text(axis_x(1),axis_x(2),axis_x(3),'X_s','FontSize',15);       %为坐标轴添加标注
text(axis_y(1),axis_y(2),axis_y(3),'Y_s','FontSize',15);
text(axis_z(1),axis_z(2),axis_z(3),'Z_s','FontSize',15);
hold off

end