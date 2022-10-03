function [crop,crop_norm,crop_mean,crop_std,img_map] = corner2image_sift(img, p, tsize)
%   (r1,c1) ***** (r3,c3)            (1,1) ***** (1,cols)
%     *             *                  *           *
%      *             *       ----->     *           *
%       *             *                  *           *
%     (r2,c2) ***** (r4,c4)              (rows,1) **** (rows,cols)
afnv_obj = corners2affine( p, tsize);
map_afnv = afnv_obj.afnv;
img_map = IMGaffine_c(double(img), map_afnv, tsize);

%lbp_img_map= lbp(uint8(img_map));
sift_img_map= do_sift(img_cut, 'Verbosity', 1, 'NumOctaves', 4, 'Threshold',  0.2/3/2);

[m1,n1]=size(sift_img_map);
sift_img_map=sift_img_map';
a=[m1,n1];
crop= reshape(sift_img_map,max(a), 1);
[crop,crop_mean,crop_std] = whitening(crop); %crop is a vector
%[crop,crop_mean,crop_std] = whitening( reshape(img_map, prod(tsize), 1) ); %crop is a vector
crop_norm = norm(crop);
crop = crop/crop_norm;
