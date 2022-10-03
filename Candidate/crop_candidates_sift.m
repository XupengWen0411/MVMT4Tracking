function [gly_crop, gly_inrange] = crop_candidates_sift(img_frame, curr_samples, template_size)
%create gly_crop, gly_inrange

nsamples = size(curr_samples,1);

gly_inrange = zeros(nsamples,1);
gly_crop = [];

for n = 1:nsamples
    curr_afnv = curr_samples(n, :);    
    
    %    [img_cut, gly_inrange(n)] = IMGaffine_r(img_frame, curr_afnv, template_size);
    [img_cut, gly_inrange(n)] = IMGaffine_c(img_frame, curr_afnv, template_size);
    
    img_cut=img_cut-min(img_cut(:)) ;
    img_cut=img_cut/max(img_cut(:)) ;

    sift_img_cut = do_sift( img_cut, 'Verbosity', 1, 'NumOctaves', 4, 'Threshold',  0.2/3/2 ) ; %0.04/3/2

    
    [m1,n1]=size(sift_img_cut);
    a=[m1,n1];
    gly = reshape(sift_img_cut, max(a) , 1);
    gly_crop = [gly_crop gly];
end
