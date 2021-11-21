function plotPoints(GridPoint_S,ImagePoint_s,ImagePoint_c,PixelPoint_c,SonarVisible,CameraVisible,gridRow,gridCol,SonarViewPoint,CameraViewPoint,axeslength,scaleForview,figurename)
% 
% 
    figure('NumberTitle', 'off', 'Name',[figurename,'_imagingModel']);
    plot_sonar_axes(axeslength,'rrr');
    plot_camera_axes(CameraViewPoint,axeslength,'bbb');
    hold on;
    for i = 1:gridRow
        for j = 1:gridCol                              
            plot3(GridPoint_S{i,j}(1),GridPoint_S{i,j}(2),GridPoint_S{i,j}(3),'*b');         %grids在声呐坐标系中的坐标(3D)
        end
    end
    %------------------图像点在声呐坐标系的坐标--------------------------------
    for i = 1:gridRow
        for j = 1:gridCol 
            if SonarVisible(i,j)
                plot3(ImagePoint_s{i,j}(1),ImagePoint_s{i,j}(2),ImagePoint_s{i,j}(3),'or');  %绘制在声呐坐标系中
            end
        end
    end  
    %---------------------图像点在声呐坐标系的坐标---------------------
    ImagePoint_Sc = cell(gridRow,gridCol);
    for i = 1:gridRow
        for j = 1:gridCol
            if(CameraVisible(i,j))
                ImagePoint_Sc{i,j} = frame_c2s(ImagePoint_c{i,j},SonarViewPoint,CameraViewPoint,scaleForview);
                %ImagePoint_Sc{i,j} = R_sonar * (R_s2c')*(scaleForview*ImagePoint_c{i,j}-T_s2c) + T_sonar;    %图像点在声呐坐标系中的坐标,          
                plot3(ImagePoint_Sc{i,j}(1),ImagePoint_Sc{i,j}(2),ImagePoint_Sc{i,j}(3),'og');  %绘制在世界坐标系中
            end
        end
    end  
%---------------------draw projection line---------------------------------    
%     for i = 1:gridCol
%         for j = 1:gridRow
%             line([GridPoint_S{i,j}(1),(ImagePoint_Sc{i,j}(1)-GridPoint_S{i,j}(1))*1.5+GridPoint_S{i,j}(1)],[GridPoint_S{i,j}(2),(ImagePoint_Sc{i,j}(2)-GridPoint_S{i,j}(2))*1.5+GridPoint_S{i,j}(2)],[GridPoint_S{i,j}(3),(ImagePoint_Sc{i,j}(3)-GridPoint_S{i,j}(3))*1.5+GridPoint_S{i,j}(3)]);
%         end
%     end     
%---------------------draw camera view bounds--------------------------------
    xbound = CameraViewPoint.camera_intrinsics.ImageSize(2);
    ybound = CameraViewPoint.camera_intrinsics.ImageSize(1); 
    sx = CameraViewPoint.camera_intrinsics.FocalLength(1)/CameraViewPoint.focalLength;
    sy = CameraViewPoint.camera_intrinsics.FocalLength(2)/CameraViewPoint.focalLength;
    px = [xbound/2/sx,-xbound/2/sx,-xbound/2/sx,xbound/2/sx];
    py = [ybound/2/sy,ybound/2/sy,-ybound/2/sy,-ybound/2/sy];
    boundPoint = zeros(3,4);
    for i=1:4
        boundPoint(:,i) = frame_c2s([px(i);py(i);CameraViewPoint.focalLength],SonarViewPoint,CameraViewPoint,scaleForview);
    end
   
    plot3([boundPoint(1,1),boundPoint(1,2)],[boundPoint(2,1),boundPoint(2,2)],[boundPoint(3,1),boundPoint(3,2)],'-k'); 
    plot3([boundPoint(1,2),boundPoint(1,3)],[boundPoint(2,2),boundPoint(2,3)],[boundPoint(3,2),boundPoint(3,3)],'-k'); 
    plot3([boundPoint(1,3),boundPoint(1,4)],[boundPoint(2,3),boundPoint(2,4)],[boundPoint(3,3),boundPoint(3,4)],'-k'); 
    plot3([boundPoint(1,4),boundPoint(1,1)],[boundPoint(2,4),boundPoint(2,1)],[boundPoint(3,4),boundPoint(3,1)],'-k');
    hold off;
%-------------draw points on xSy------------------------    
    figure('NumberTitle', 'off', 'Name',[figurename,'_sonarImage']);
    axis equal;
    hold on;
    for i = 1:gridRow
        for j = 1:gridCol
            if(SonarVisible(i,j))
                plot(ImagePoint_s{i,j}(1),ImagePoint_s{i,j}(2),'or');             %projection of the grids in sonar image coordinates
            end        
        end
    end
    hold off;
%-------------draw points on xCy------------------------      
    figure('NumberTitle', 'off', 'Name',[figurename,'_cameraImagePlane']);
    axis equal;
    set(gca,'YDir','reverse')                                                     %对Y方向反转
    hold on;
    for i = 1:gridRow
        for j = 1:gridCol
            if(CameraVisible(i,j))
                plot(ImagePoint_c{i,j}(1),ImagePoint_c{i,j}(2),'og');             %图像点在摄像机像平面xCy的坐标
            end                     
        end
    end
    hold off;    
%-------------draw points on xCy in pixel ------------------------      
    figure('NumberTitle', 'off', 'Name',[figurename,'_cameraPixelPlane']);
    axis equal;
    set(gca,'YDir','reverse');                                                     %对Y方向反转
    xlim([0 CameraViewPoint.camera_intrinsics.ImageSize(2)]);
    ylim([0 CameraViewPoint.camera_intrinsics.ImageSize(1)]);
    hold on;
    for i = 1:gridRow
        for j = 1:gridCol
            if(CameraVisible(i,j))
                plot(PixelPoint_c{i,j}(1), PixelPoint_c{i,j}(2),'ok');             %图像点在摄像机pixel平面xCy的坐标
            end                     
        end
    end
    hold off;     
end

