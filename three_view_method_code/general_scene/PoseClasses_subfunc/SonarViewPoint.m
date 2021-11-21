classdef SonarViewPoint < TransCoordinate
    properties
        min_fov_angle = -65;
        max_fov_angle = 65;
        fov_angle = [-65,65];
        min_pitch_angle = 10;
        max_pitch_angle = -10;
        pitch_angle = [-10,10];
    end
    methods
        function obj = SonarViewPoint(x,y,z,angle,order,FOV,PITCH)
            obj = obj@TransCoordinate(x,y,z,angle,order);
            obj.fov_angle = FOV;
            obj.min_fov_angle = FOV(1);
            obj.max_fov_angle = FOV(2);
            obj.pitch_angle = PITCH;
            obj.min_pitch_angle = PITCH(1);
            obj.max_pitch_angle = PITCH(2);       
        end
    end
end