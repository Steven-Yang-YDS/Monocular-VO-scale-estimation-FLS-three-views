classdef FeaturePointIn3DSpace
    properties
       pos_x = 0;
       pos_y = 0;
       pos_z = 0;
       coordinate = [0;0;0];
       feature_index = 0;        %in default
       Is_observed = false;      %in default: not be observed by cameras
    end
    methods
        function obj = FeaturePointIn3DSpace(point,index)
            obj.pos_x = point(1);
            obj.pos_y = point(2);
            obj.pos_z = point(3);
            obj.coordinate = [point(1);point(2);point(3)];
            obj.feature_index = index;
        end
    end
end