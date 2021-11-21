function error_ijk = reprojErrSonar(obs, points, Tij, Tjk, ind)
%REPROJERRSONAR Summary of this function goes here
%   Detailed explanation goes here
error_ijk = [];
    if ind(1) ~= 0
        for i = ind
            if i==1
                reproj_3d = Tij\([points,1].');   
            elseif i==2
                reproj_3d = [points,1].';   
            elseif i==3
                reproj_3d = Tjk*([points,1].');   
            end
            reproj_coord = [reproj_3d(1), reproj_3d(2)]/norm(reproj_3d(1:2)) * norm(reproj_3d(1:3));
            error_ijk = [error_ijk,obs(2*i - 1:2*i) - reproj_coord];
        end
    elseif ind(1) == 0
        for i = 1:3
            if i==1
                reproj_3d = Tij\([points,1].');   
            elseif i==2
                reproj_3d = [points,1].';   
            elseif i==3
                reproj_3d = Tjk*([points,1].');   
            end
            reproj_coord = [reproj_3d(1), reproj_3d(2)]/norm(reproj_3d(1:2)) * norm(reproj_3d(1:3));
            error_ijk = [error_ijk,obs(2*i - 1:2*i) - reproj_coord];
        end
    end
end

