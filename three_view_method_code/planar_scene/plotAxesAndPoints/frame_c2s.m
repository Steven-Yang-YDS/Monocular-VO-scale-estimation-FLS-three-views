function Point_s = frame_c2s(Point_c,SonarViewPoint,CameraViewPoint,scaleForview)
%FRAME_C2S 此处显示有关此函数的摘要
%   
    R_sonar = SonarViewPoint.R_matrix;
    R_camera = CameraViewPoint.R_matrix;
    T_sonar = SonarViewPoint.T_vector;
    T_camera = CameraViewPoint.T_vector;    
    Point_s = R_sonar * (R_camera')*(scaleForview*Point_c - T_camera) + T_sonar; 
    
end

