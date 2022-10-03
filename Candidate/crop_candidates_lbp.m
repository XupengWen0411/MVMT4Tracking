function [gly_crop, gly_inrange] = crop_candidates_lbp(img_frame, curr_samples, template_size)
%create gly_crop, gly_inrange

nsamples = size(curr_samples,1);

gly_inrange = zeros(nsamples,1);
gly_crop = [];

for n = 1:nsamples
    curr_afnv = curr_samples(n, :);    
    
    %    [img_cut, gly_inrange(n)] = IMGaffine_r(img_frame, curr_afnv, template_size);
    [img_cut, gly_inrange(n)] = IMGaffine_c(img_frame, curr_afnv, template_size);
    lbp_img_cut= lbp(img_cut);
    %lbp_img_cut= hogfeat(img_cut);
    
    [m1,n1]=size(lbp_img_cut);
    a=[m1,n1];
    gly = reshape(lbp_img_cut, max(a) , 1);
    gly_crop = [gly_crop gly];
end
