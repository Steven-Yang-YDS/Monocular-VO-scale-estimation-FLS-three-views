function R_matrix = eularAngle(angle,order)
%angle : a vector of three degrees; order : a string of 'x','y','z' denote rotate order
%example: order = 'zyx' , theorder is   R = Rz*Ry*Rx
    alpha_array = @(alpha)  [1,0,0;
                             0,cos(alpha),-sin(alpha);
                             0,sin(alpha),cos(alpha)];

     beta_array = @(beta)   [cos(beta),0,sin(beta);
                             0,1,0;
                             -sin(beta),0,cos(beta)];               %使用旋转矩阵alpha--x,alpha--y,gamma--z表示

    gamma_array = @(gamma)  [cos(gamma),-sin(gamma),0;
                             sin(gamma),cos(gamma),0;
                             0,0,1];
    Rotation = zeros(3,3,3);
    for i=1:3
        if(order(i)=='x'||order(i)=='X')
            Rotation(:,:,i) = alpha_array(angle(i));
        elseif(order(i)=='y'||order(i)=='Y')
            Rotation(:,:,i) = beta_array(angle(i));
        elseif(order(i)=='z'||order(i)=='Z')
            Rotation(:,:,i) = gamma_array(angle(i));
        end                
    end
    R_matrix = Rotation(:,:,1) * Rotation(:,:,2) * Rotation(:,:,3);          %按照z轴，y轴，x轴的顺序旋转
end

