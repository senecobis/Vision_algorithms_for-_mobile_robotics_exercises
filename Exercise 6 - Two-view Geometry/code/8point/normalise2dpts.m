function [pts_tilda, T] = normalise2dpts(pts)
% NORMALISE2DPTS - normalises 2D homogeneous points
%
% Function translates and normalises a set of 2D homogeneous points
% so that their centroid is at the origin and their mean distance from
% the origin is sqrt(2).
%
% Usage:   [pts_tilda, T] = normalise2dpts(pts)
%
% Argument:
%   pts -  3xN array of 2D homogeneous coordinates
%
% Returns:
%   pts_tilda -  3xN array of transformed 2D homogeneous coordinates.
%   T      -  The 3x3 transformation matrix, pts_tilda = T*pts

p1 = pts(1:2,:);
N = width(p1);
mu_1 = sum(p1,2)/N;

sigma2_1 = sum(vecnorm(p1-mu_1,2,1).^2) /N;

T = [sqrt(2/sigma2_1), 0, -sqrt(2/sigma2_1)*mu_1(1);
      0, sqrt(2/sigma2_1), -sqrt(2/sigma2_1)*mu_1(2);
      0, 0, 1];

pts_tilda = T * pts;



