function [T_save,T_norm_save,T_mean_save,T_std_save,img_map] = InitTemplates_sift(tsize, numT, img, cpt)
% generate templates from single image
%   (r1,c1) ***** (r3,c3)            (1,1) ***** (1,cols)
%     *             *                  *           *
%      *             *       ----->     *           *
%       *             *                  *           *
%     (r2,c2) ***** (r4,c4)              (rows,1) **** (rows,cols)
% r1,r2,r3;
% c1,c2,c3

%% prepare templates geometric parameters
p{1}= cpt;
for i=2:numT
    p{i} = cpt+randn(2,3)*0.6;
end
% p{2} = cpt + [-1 0 0; 0 0 0];
% p{3} = cpt + [1 0 0; 0 0 0];
% p{4} = cpt + [0 -1 0; 0 0 0];
% p{5} = cpt + [0 1 0; 0 0 0];
% p{6} = cpt + [0 0 1; 0 0 0];
% p{7} = cpt + [0 0 0; -1 0 0];
% p{8} = cpt + [0 0 0; 1 0 0];
% p{9} = cpt + [0 0 0; 0 -1 0];
% p{10} = cpt + [0 0 0; 0 1 0];

%% Initializating templates and image
T_save	    = [];
T_norm_save	= [];
T_mean_save	= [];
T_std_save	= [];
%% cropping and normalizing templates
for n=1:numT
    [T,T_norm,T_mean,T_std,img_map] = ...
		corner2image_sift(img, p{n}, tsize);   
    T_save = [T_save T];   
    T_norm_save= [T_save T]; 
    T_mean_save= [T_save T]; 
    T_std_save = [T_save T]; 
end