function [R,T] = disambiguateRelativePose(Rots,u3,points0_h,points1_h,K1,K2)
% DISAMBIGUATERELATIVEPOSE- finds the correct relative camera pose (among
% four possible configurations) by returning the one that yields points
% lying in front of the image plane (with positive depth).
%
% Arguments:
%   Rots -  3x3x2: the two possible rotations returned by decomposeEssentialMatrix
%   u3   -  a 3x1 vector with the translation information returned by decomposeEssentialMatrix
%   p1   -  3xN homogeneous coordinates of point correspondences in image 1
%   p2   -  3xN homogeneous coordinates of point correspondences in image 2
%   K1   -  3x3 calibration matrix for camera 1
%   K2   -  3x3 calibration matrix for camera 2
%
% Returns:
%   R -  3x3 the correct rotation matrix
%   T -  3x1 the correct translation vector
%
%   where [R|t] = T_C2_C1 = T_C2_W is a transformation that maps points
%   from the world coordinate system (identical to the coordinate system of camera 1)
%   to camera 2.

M1 = K1 * [eye(3),zeros(3,1)];
T = zeros(3,4,4); % used this name to indicate the transofrmation matrix
T(:,:,1) = [Rots(:,:,1), u3];
T(:,:,2) = [Rots(:,:,2), u3];
T(:,:,3) = [Rots(:,:,1), -u3];
T(:,:,4) = [Rots(:,:,2), -u3];
n_pos_points = zeros(4,1);
for solution = 1 : 4
    M2 = K2 * T(:,:,solution);
    P = linearTriangulation(points0_h,points1_h,M1,M2);
    point_pos_depth = find(P(3,:) > 0);
    n_pos_points(solution) = length(point_pos_depth);
end
maximum = max(n_pos_points);
idx_T = find(n_pos_points == maximum);
R = T(1:3,1:3,idx_T);
T = T(1:3, end, idx_T);

