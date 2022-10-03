function [crop,crop_norm,crop_mean,crop_std,lbp_img_map] = corner2image(img, p, tsize)
%   (r1,c1) ***** (r3,c3)            (1,1) ***** (1,cols)
%     *             *                  *           *
%      *             *       ----->     *           *
%       *             *                  *           *
%     (r2,c2) ***** (r4,c4)              (rows,1) **** (rows,cols)
afnv_obj = corners2affine( p, tsize);
map_afnv = afnv_obj.afnv;
img_map = IMGaffine_c(double(img), map_afnv, tsize);
lbp_img_map= lbp(uint8(img_map));
[crop,crop_mean,crop_std] = whitening( reshape(img_map, prod(tsize), 1) ); %crop is a vector¡–œÚ¡ø
lbp_img_map=crop;
crop_norm = norm(crop);
crop = crop/crop_norm;
