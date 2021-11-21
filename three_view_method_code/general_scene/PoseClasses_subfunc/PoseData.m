classdef PoseData
    properties
        R_mat = eye(3);
        t_vector = [0;0;0];
        n_vector = [0;0;0];       
    end
    methods
        function obj = PoseData(R,t,n)
            obj.R_mat = R;
            obj.t_vector = t;
            obj.n_vector = n;
        end
    end
end