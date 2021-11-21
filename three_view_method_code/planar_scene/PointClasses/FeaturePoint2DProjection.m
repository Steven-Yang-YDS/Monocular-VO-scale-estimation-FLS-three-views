classdef FeaturePoint2DProjection
    properties
       pos_x = 0;
       pos_y = 0;
       pos_z = 0;
       coordinate = [0;0];
       feature_index = 0;        %in default
       Is_observed = false;      %in default: not be observed by cameras
       Is_noise = false;
       noise_gt = 0;             %ground truth  for camera: pixel, for sonar: meter 
    end
    methods
        function obj = FeaturePoint2DProjection(varargin)        %(point,index)
            narginchk(2,3);
            if nargin == 2           
                obj.pos_x = varargin{1}(1);
                obj.pos_y = varargin{1}(2);
                obj.coordinate = [varargin{1}(1);varargin{1}(2)];
                obj.feature_index = varargin{2};
            elseif nargin == 3
                obj.pos_x = varargin{1}(1);
                obj.pos_y = varargin{1}(2);
                obj.coordinate = [varargin{1}(1);varargin{1}(2)];
                obj.feature_index = varargin{2};
                obj.Is_noise = true;
                obj.noise_gt = varargin{3};                      %noise_x,noise_y
            end
        end
    end
end