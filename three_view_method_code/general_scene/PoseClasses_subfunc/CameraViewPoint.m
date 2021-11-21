classdef CameraViewPoint < TransCoordinate
    properties
    focalLength = 1;
%     x0 = 16;
%     y0 = 9;
%     intrinsicPara = [1 0 0;
%                      0 1 0;
%                      0 0 1];
    intrinsic_matrix = [ 1     0    0;
                         0     1    0;
                         0     0    1];
    camera_intrinsics = cameraIntrinsics([1 1],[1 1],[1 1]);
    end
    methods
        function obj = CameraViewPoint(x,y,z,angle,order,f,focalxy,principalPoint,imageSize)
            obj = obj@TransCoordinate(x,y,z,angle,order);
            obj.focalLength = f;
            %obj.camera_intrinsics = cameraIntrinsics();
            obj.camera_intrinsics = cameraIntrinsics(focalxy,principalPoint,imageSize);            
%             obj.x0 = x0;
%             obj.y0 = y0;
            obj.intrinsic_matrix = [obj.camera_intrinsics.FocalLength(1)       0                    obj.camera_intrinsics.PrincipalPoint(1);
                                                 0        obj.camera_intrinsics.FocalLength(2)      obj.camera_intrinsics.PrincipalPoint(2);
                                                 0                          0                                          1               ];
% cameraIntrinsics(focalxy,principalPoint,imageSize);
        end
    end
end