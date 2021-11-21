classdef TransCoordinate
    properties
       pos_x = 0;
       pos_y = 0;
       pos_z = 0;
       alpha = 0;
       beta = 0;
       gamma = 0;
       R_matrix = [ 1 0 0;
                    0 1 0;
                    0 0 1 ];
       T_vector = [ 0 ; 0 ; 0 ];
       RT_matrix = [ 1 0 0 0;
                     0 1 0 0;
                     0 0 1 0;
                     0 0 0 1];
    end
    methods
        function obj = TransCoordinate(x,y,z,angle,order)
            obj.pos_x = x;
            obj.pos_y = y;
            obj.pos_z = z;
%             obj.alpha = angle(1);
%             obj.beta = angle(2);
%             obj.gamma = angle(3);
            %-----------------rotation matrix-------------- description the rotation of vectors    而对应的坐标系旋转角度为角度的负值 
            alpha_array = @(alpha)  [1,0,0;
                                     0,cosd(alpha),-sind(alpha);
                                     0,sind(alpha),cosd(alpha)];

             beta_array = @(beta)   [cosd(beta),0,sind(beta);
                                     0,1,0;
                                     -sind(beta),0,cosd(beta)];               %使用旋转矩阵alpha--x,alpha--y,gamma--z表示

            gamma_array = @(gamma)  [cosd(gamma),-sind(gamma),0;
                                     sind(gamma),cosd(gamma),0;
                                     0,0,1];
            Rotation = zeros(3,3,3);
            for i=1:3
                if(order(i)=='x')
                    Rotation(:,:,i) = alpha_array(angle(i));
                elseif(order(i)=='y')
                    Rotation(:,:,i) = beta_array(angle(i));
                elseif(order(i)=='z')
                    Rotation(:,:,i) = gamma_array(angle(i));
                end                
            end


%             alpha_array = [1,0,0;
%                            0,cosd(obj.alpha),-sind(obj.alpha);
%                            0,sind(obj.alpha),cosd(obj.alpha)];
%                        
%              beta_array = [cosd(obj.beta),0,sind(obj.beta);
%                            0,1,0;
%                            -sind(obj.beta),0,cosd(obj.beta)];               %使用旋转矩阵alpha--x,alpha--y,gamma--z表示
%                        
%             gamma_array = [cosd(obj.gamma),-sind(obj.gamma),0;
%                            sind(obj.gamma),cosd(obj.gamma),0;
%                            0,0,1];
            obj.R_matrix = Rotation(:,:,1) * Rotation(:,:,2) * Rotation(:,:,3);          %按照 order 的顺序旋转
            obj.T_vector = [ obj.pos_x ; obj.pos_y ; obj.pos_z ];
            obj.RT_matrix(1:3,1:3) = obj.R_matrix;
            obj.RT_matrix(1:3,4) = obj.T_vector;
            obj.RT_matrix(4,1:4) = [0 0 0 1];            
        end
    end
end