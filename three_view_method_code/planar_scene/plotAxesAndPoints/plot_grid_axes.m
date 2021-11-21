function plot_grid_axes(varargin)%SonarViewPoint,TransCoordinate,text)
%sonarview = SonarViewPoint(20,30,40,20,30,40);
if nargin == 3
    %SonarViewPoint = varargin{1};
    TransCoordinate = varargin{1};
    R_matrix = TransCoordinate.R_matrix;
    T_vector = TransCoordinate.T_vector;
    %R_matrix_s = SonarViewPoint.R_matrix;
    %T_vector_s = SonarViewPoint.T_vector;
    text_x = 'X_g';
    text_y = 'Y_g';
    text_z = 'Z_g';
    axeslength = varargin{2};
    XYZcolors = varargin{3};
end
%--------draw a frame according to RT matrix----------
if nargin == 4
    %SonarViewPoint = varargin{1};
    RT_matrix = varargin{1};
    R_matrix = RT_matrix(1:3,1:3);
    T_vector = RT_matrix(1:3,4);
    %R_matrix_s = SonarViewPoint.R_matrix;
    %T_vector_s = SonarViewPoint.T_vector;
    text_x = ['X_','{',varargin{2},'}'];
    text_y = ['Y_','{',varargin{2},'}'];
    text_z = ['Z_','{',varargin{2},'}'];
    axeslength = varargin{3};
    XYZcolors = varargin{4};
end

    %axeslength = (T_vector_s(1) + T_vector_s(2) + T_vector_s(3))/3;
    unitx_s = R_matrix*[1;0;0]*axeslength;
    unity_s = R_matrix*[0;1;0]*axeslength ;
    unitz_s = R_matrix*[0;0;1]*axeslength;
    

%     unitx_w = R_matrix_s*unitx_s + T_vector_s;
%     unity_w = R_matrix_s*unity_s + T_vector_s;
%     unitz_w = R_matrix_s*unitz_s + T_vector_s;

    hold on;
    plot3([T_vector(1),T_vector(1)+unitx_s(1)],[T_vector(2),T_vector(2)+unitx_s(2)],[T_vector(3),T_vector(3)+unitx_s(3)],'LineWidth',2,'Color',XYZcolors(1));
    plot3([T_vector(1),T_vector(1)+unity_s(1)],[T_vector(2),T_vector(2)+unity_s(2)],[T_vector(3),T_vector(3)+unity_s(3)],'LineWidth',2,'Color',XYZcolors(2));
    plot3([T_vector(1),T_vector(1)+unitz_s(1)],[T_vector(2),T_vector(2)+unitz_s(2)],[T_vector(3),T_vector(3)+unitz_s(3)],'LineWidth',2,'Color',XYZcolors(3));

    axis_x = unitx_s *1.2 + T_vector;
    axis_y = unity_s *1.2 + T_vector;
    axis_z = unitz_s *1.2 + T_vector;

    text(axis_x(1),axis_x(2),axis_x(3),text_x,'FontSize',15);       %为坐标轴添加标注
    text(axis_y(1),axis_y(2),axis_y(3),text_y,'FontSize',15);
    text(axis_z(1),axis_z(2),axis_z(3),text_z,'FontSize',15);
    hold off
%     originalPoint =R_matrix_s*T_vector + T_vector_s;

%     plot3([originalPoint(1),unitx_w(1)],[originalPoint(2),unitx_w(2)],[originalPoint(3),unitx_w(3)],'LineWidth',2);
%     plot3([originalPoint(1),unity_w(1)],[originalPoint(2),unity_w(2)],[originalPoint(3),unity_w(3)],'LineWidth',2);
%     plot3([originalPoint(1),unitz_w(1)],[originalPoint(2),unitz_w(2)],[originalPoint(3),unitz_w(3)],'LineWidth',2);
% 
%     axis_x = (unitx_w - originalPoint)*1.2 + originalPoint;
%     axis_y = (unity_w - originalPoint)*1.2 + originalPoint;
%     axis_z = (unitz_w - originalPoint)*1.2 + originalPoint;
% 
%     text(axis_x(1),axis_x(2),axis_x(3),text_x,'FontSize',15);       %为坐标轴添加标注
%     text(axis_y(1),axis_y(2),axis_y(3),text_y,'FontSize',15);
%     text(axis_z(1),axis_z(2),axis_z(3),text_z,'FontSize',15);

    

    
end