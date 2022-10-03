function [track_res,output] = MTView_Tracking_APG8(s_frames, paraT)
 

%% Initialize templates T
%-Generate T from single image
init_pos = paraT.init_pos;
n_sample=paraT.n_sample;
sz_T=paraT.sz_T;
rel_std_afnv = paraT.rel_std_afnv;
nT=paraT.nT;

%generate the initial templates for the 1st frame
img = imread(s_frames{1});
if(size(img,3) == 3)
    img = rgb2gray(img);
end
%% 模板初始化
[T_lbp,T_norm_lbp,T_mean_lbp,T_std_lbp] = InitTemplates_lbp(sz_T,nT,img,init_pos);%初始化模板尺寸、模板数量、图像、模板截取初始坐标

[T_hog,T_norm_hog,T_mean_hog,T_std_hog] = InitTemplates_hog(sz_T,nT,img,init_pos);%初始化模板尺寸、模板数量、图像、模板截取初始坐标

[T_hu,T_norm_hu,T_mean_hu,T_std_hu] = InitTemplates_hu(sz_T,nT,img,init_pos);%初始化模板尺寸、模板数量、图像、模板截取初始坐标

%[T_sift,T_norm_sift,T_mean_sift,T_std_sift] = InitTemplates_sift(sz_T,nT,img,init_pos);%初始化模板尺寸、模板数量、图像、模板截取初始坐标

%% 模板归一化
norms_lbp = T_norm_lbp.*T_std_lbp; %template norms 模板规范化

norms_hog = T_norm_hog.*T_std_hog; %template norms 模板规范化

norms_hu = T_norm_hu.*T_std_hu; %template norms 模板规范化

%norms_sift = T_norm_sift.*T_std_sift; %template norms 模板规范化

T=[T_lbp;T_hog;T_hu];

occlusionNf_lbp = 0;

occlusionNf_hog = 0;

occlusionNf_hu = 0;

%occlusionNf_sift = 0;


%% L1 function settings
angle_threshold = paraT.angle_threshold;
para.Lambda = paraT.lambda;
para.nT = paraT.nT;
para.Lip = paraT.Lip;
para.Maxit = paraT.Maxit;
% disp(T_lbp);
dim_T_lbp	= size(T_lbp,1);	%number of elements in one template, sz_T(1)*sz_T(2)=12x15 = 180
A_lbp		= [T_lbp eye(dim_T_lbp)]; %data matrix is composed of T, positive trivial T.
[m_A_lbp n_A_lbp]=size(A_lbp);

dim_T_hog	= size(T_hog,1);	%number of elements in one template, sz_T(1)*sz_T(2)=12x15 = 180
A_hog		= [T_hog eye(dim_T_hog)]; %data matrix is composed of T, positive trivial T.
[m_A_hog n_A_hog]=size(A_hog);

dim_T_hu	= size(T_hu,1);	%number of elements in one template, sz_T(1)*sz_T(2)=12x15 = 180
A_hu		= [T_hu eye(dim_T_hu)]; %data matrix is composed of T, positive trivial T.
[m_A_hu n_A_hu]=size(A_hu);

% dim_T_sift	= size(T_sift,1);	%number of elements in one template, sz_T(1)*sz_T(2)=12x15 = 180
% A_sift		= [T_sift eye(dim_T_sift)]; %data matrix is composed of T, positive trivial T.
% [m_A_sift n_A_sift]=size(A_sift);



if n_A_lbp>n_A_hog&&n_A_lbp>n_A_hu%&&n_A_lbp>n_A_sift
buchong_hog=zeros(m_A_hog,abs((n_A_lbp-n_A_hog)));
A_hog=[A_hog buchong_hog];
buchong_hu=zeros(m_A_hu,abs(n_A_lbp-n_A_hu));
A_hu=[A_hu buchong_hu];
% buchong_sift=zeros(m_A_sift,abs(n_A_lbp-n_A_sift));
% A_sift=[A_sift buchong_sift];
end
if n_A_hog>n_A_lbp&&n_A_hog>n_A_hu%&&n_A_hog>n_A_sift
buchong_lbp=zeros(m_A_lbp,abs(n_A_lbp-n_A_hog));
A_lbp=[A_lbp buchong_lbp];
buchong_hu=zeros(m_A_hu,abs(n_A_hog-n_A_hu));
A_hu=[A_hu buchong_hu];
% buchong_sift=zeros(m_A_sift,abs(n_A_hog-n_A_sift));
% A_sift=[A_sift buchong_sift];
end
% if n_A_sift>n_A_lbp&&n_A_sift>n_A_hog&&n_A_sift>n_A_hu
% buchong_lbp=zeros(m_A_lbp,abs(n_A_lbp-n_A_sift));
% A_lbp=[A_lbp buchong_lbp];
% buchong_hog=zeros(m_A_hog,abs(n_A_hog-n_A_sift));
% A_hog=[A_hog buchong_hog];
% buchong_hu=zeros(m_A_hu,abs(n_A_hu-n_A_sift));
% A_hu=[A_hu buchong_hu];
% end

alpha = 50;%this parameter is used in the calculation of the likelihood of particle filter
aff_obj = corners2affine(init_pos, sz_T); %get affine transformation parameters from the corner points in the first frame
map_aff = aff_obj.afnv;%aff_obj.afnv： affine parameters
aff_samples = ones(n_sample,1)*map_aff;

T_id	= -(1:nT);	% template IDs, for debugging

fixT_lbp = T_lbp(:,1)/nT; % first template is used as a fixed template
fixT_hog = T_hog(:,1)/nT; % first template is used as a fixed template
fixT_hu = T_hu(:,1)/nT; % first template is used as a fixed template
%fixT_sift = T_sift(:,1)/nT; % first template is used as a fixed template
%Temaplate Matrix
Temp1_lbp = [T_lbp,fixT_lbp]*pinv([T_lbp,fixT_lbp]);
Temp1_hog = [T_hog,fixT_hog]*pinv([T_hog,fixT_hog]);
Temp1_hu = [T_hu,fixT_hu]*pinv([T_hu,fixT_hu]);
%Temp1_sift = [T_sift,fixT_sift]*pinv([T_sift,fixT_sift]);
%% Tracking

% initialization
nframes	= length(s_frames);
track_res	= zeros(6,nframes);
Time_record = zeros(nframes,1);
%Coeff = zeros(size([A fixT],2),nframes);
Min_Err = zeros(nframes,1);
count = zeros(nframes,1);
Time = zeros(n_sample,1); % L1 norm time recorder
ratio = zeros(nframes,1);% energy ratio

for t = 1:nframes
    fprintf('Frame number: %d \n',t);
    
    img_color	= imread(s_frames{t});
    if(size(img_color,3) == 3)
        img     = double(rgb2gray(img_color));
    else
        img     = double(img_color);
    end
    
    tic
    %-Draw transformation samples from a Gaussian distribution
    sc			= sqrt(sum(map_aff(1:4).^2)/2);%map_aff=aff_obj.afnv： affine parameters
    std_aff		= rel_std_afnv.*[1, sc, sc, 1, sc, sc];
    map_aff		= map_aff + 1e-14;
    aff_samples = draw_sample(aff_samples, std_aff); %draw transformation samples from a Gaussian distribution
    
    %利用粒子跟踪算法获取可能的跟踪目标-Crop candidate targets "Y" according to the transformation samples
    [Y_lbp, Y_inrange_lbp] = crop_candidates_lbp(im2double(img), aff_samples(:,1:6), sz_T);
    [Y_hog, Y_inrange_hog] = crop_candidates_hog(im2double(img), aff_samples(:,1:6), sz_T);
    [Y_hu, Y_inrange_hu] = crop_candidates_hu(im2double(img), aff_samples(:,1:6), sz_T);
    %[Y_sift, Y_inrange_sift] = crop_candidates_sift(im2double(img), aff_samples(:,1:6), sz_T);
   
    if(sum(Y_inrange_lbp==0) == n_sample)||(sum(Y_inrange_hog==0) == n_sample)...
            ||(sum(Y_inrange_hu==0) == n_sample)%||(sum(Y_inrange_sift==0) == n_sample)
        sprintf('Target is out of the frame!\n');
    end
    
    [Y_lbp,Y_crop_mean_lbp,Y_crop_std_lbp] = whitening(Y_lbp);	 % zero-mean-unit-variance
    [Y_lbp, Y_crop_norm_lbp] = normalizeTemplates(Y_lbp); %norm one
    
    [Y_hog,Y_crop_mean_hog,Y_crop_std_hog] = whitening(Y_hog);	 % zero-mean-unit-variance
    [Y_hog, Y_crop_norm_hog] = normalizeTemplates(Y_hog); %norm one
    
    [Y_hu,Y_crop_mean_hu,Y_crop_std_hu] = whitening(Y_hu);	 % zero-mean-unit-variance
    [Y_hu, Y_crop_norm_hu] = normalizeTemplates(Y_hu); %norm one
    
%     [Y_sift,Y_crop_mean_sift,Y_crop_std_sift] = whitening(Y_sift);	 % zero-mean-unit-variance
%     [Y_sift, Y_crop_norm_sift] = normalizeTemplates(Y_sift); %norm one
    
   
    %-L1-LS for each candidate target
    eta_max	= -inf;%inf：无穷大
   
    p=zeros(n_sample,1); % observation likelihood initialization 初始化样本检查的可能性
    X_train=cell(3,1);
    X_train{1}=A_lbp;
    X_train{2}=A_hog;
    X_train{3}=A_hu;
    %X_train{4}=A_sift;
    %% 参数设置，不改
    opts.init = 0;      % guess start point from data. 
    opts.tFlag = 1;     % terminate after relative objective value does not changes much.
    opts.tol = 10^-3;   % tolerance. 
    opts.maxIter = 100; % maximum iteration number of optimization.
    rho_1 = 0.5;%2*sqrt(n)/train_num;%   rho1: P
    rho_2 = 0.5;%3*sqrt(n)/train_num; %   rho2: Q

    % first stage L2-norm bounding    
    for j = 1:n_sample
        if (Y_inrange_lbp(j)==0 || sum(abs(Y_lbp(:,j)))==0)||(Y_inrange_hog(j)==0 || sum(abs(Y_hog(:,j)))==0)...
                ||(Y_inrange_hu(j)==0 || sum(abs(Y_hu(:,j)))==0)%||(Y_inrange_sift(j)==0 || sum(abs(Y_sift(:,j)))==0)
            continue;
        end
        disp(j);
        Y_label=cell(3,1);
        Y_label{1}=Y_lbp(:,j);
        Y_label{2}=Y_hog(:,j);
        Y_label{3}=Y_hu(:,j);
        %Y_label{4}=Y_sift(:,j);
        
        [W funcVal_lbp P_lbp Q_lbp] = Least_rMTFL(X_train, Y_label, rho_1, rho_2, opts);
        
        D_s_lbp = (Y_lbp(:,j) - [A_lbp(:,1:nT) fixT_lbp]*[W(1:nT,1); W(end,1)]).^2;%reconstruction error 重构误差
        D_s_hog = (Y_hog(:,j) - [A_hog(:,1:nT) fixT_hog]*[W(1:nT,2); W(end,2)]).^2;%reconstruction error 重构误差
        D_s_hu = (Y_hu(:,j) - [A_hu(:,1:nT) fixT_hu]*[W(1:nT,3); W(end,3)]).^2;%reconstruction error 重构误差
        %D_s_sift = (Y_sift(:,j) - [A_sift(:,1:nT) fixT_sift]*[W(1:nT,4); W(end,4)]).^2;%reconstruction error 重构误差
        D_s=[D_s_lbp;D_s_hog;D_s_hu];   
        
        p(j) = exp(-alpha*(sum(D_s))); % probability w.r.t samples
        if(sum(W(1:nT,1))<0||sum(W(1:nT,2))<0||sum(W(1:nT,3))<0)%||sum(W(1:nT,4))<0) %remove the inverse intensity patterns
            continue;
        elseif(p(j)>eta_max)
            id_max	= j;%可能性最大的测试样本的ID
            
            c_max_lbp= W(:,1);%可能性最大的稀疏表示系数
            c_max_hog= W(:,2);%可能性最大的稀疏表示系数
            c_max_hu = W(:,3);%可能性最大的稀疏表示系数
            %c_max_sift = W(:,4);%可能性最大的稀疏表示系数
            
            eta_max = p(j);
            Min_Err(t) = sum(D_s);%第t帧的最小误差
        end
           
    end
   
    % resample according to probability
    map_aff = aff_samples(id_max,1:6); %target transformation parameters with the maximum probability
    %% 自己改
    a_max_lbp	= c_max_lbp(1:nT);%nT指模板的数量，a_max为模板的稀疏表示系数
    a_max_hog	= c_max_hog(1:nT);%nT指模板的数量，a_max为模板的稀疏表示系数
    a_max_hu 	= c_max_hu(1:nT);%nT指模板的数量，a_max为模板的稀疏表示系数
    %a_max_sift 	= c_max_sift(1:nT);%nT指模板的数量，a_max为模板的稀疏表示系数
    
    [aff_samples, ~] = resample(aff_samples,p,map_aff); %resample the samples wrt. the probability
    %% 自己改
    [~, indA_lbp] = max(a_max_lbp);
    min_angle_lbp = images_angle(Y_lbp(:,id_max),A_lbp(:,indA_lbp));
    
    [~, indA_hog] = max(a_max_hog);
    min_angle_hog = images_angle(Y_hog(:,id_max),A_hog(:,indA_hog));
    
    [~, indA_hu] = max(a_max_hu);
    min_angle_hu = images_angle(Y_hu(:,id_max),A_hu(:,indA_hu));
    
%     [~, indA_sift] = max(a_max_sift);
%     min_angle_sift = images_angle(Y_sift(:,id_max),A_sift(:,indA_sift));
%     
     %-Template update
     occlusionNf_lbp = occlusionNf_lbp-1;
     level = 0.03;
    if( min_angle_lbp > angle_threshold && occlusionNf_lbp<0 )        
        disp('LBP Update!')
        [~,indW] = min(a_max_lbp(1:nT));
        % insert new template
        T_lbp(:,indW)	= Y_lbp(:,id_max);
        T_mean_lbp(indW)= Y_crop_mean_lbp(id_max);
        T_id(indW)	= t; %track the replaced template for debugging
        norms(indW) = Y_crop_std_lbp(id_max)*Y_crop_norm_lbp(id_max);
        
        [T_lbp, ~] = normalizeTemplates(T_lbp);
        A_lbp(:,1:nT)	= T_lbp;
        
         %Temaplate Matrix
        Temp1_lbp = [T_lbp,fixT_lbp]*pinv([T_lbp,fixT_lbp]);
        elseif occlusionNf_hog<0
            para.Lambda(3) = paraT.lambda(3);
    end
    
    
    occlusionNf_hog = occlusionNf_hog-1;
    if( min_angle_hog > angle_threshold  && occlusionNf_hog<0 )
        disp('HOG Update!')
        [~,indW] = min(a_max_hog(1:nT));
        % insert new template
        T_hog(:,indW)	= Y_hog(:,id_max);
        T_mean_hog(indW)= Y_crop_mean_hog(id_max);
        T_id(indW)	= t; %track the replaced template for debugging
        norms(indW) = Y_crop_std_hog(id_max)*Y_crop_norm_hog(id_max);
        
        [T_hog, ~] = normalizeTemplates(T_hog);
        A_hog(:,1:nT)	= T_hog;
        
         %Temaplate Matrix
        Temp1_hog = [T_hog,fixT_hog]*pinv([T_hog,fixT_hog]);
        elseif occlusionNf_hog<0
            para.Lambda(3) = paraT.lambda(3);
    end
    
    occlusionNf_hu = occlusionNf_hu-1;
    if( min_angle_hu > angle_threshold  && occlusionNf_hu<0 )
        disp('HU Update!')
        [~,indW] = min(a_max_hu(1:nT));
        % insert new template
        T_hu(:,indW)	= Y_hu(:,id_max);
        T_mean_hu(indW)= Y_crop_mean_hu(id_max);
        T_id(indW)	= t; %track the replaced template for debugging
        norms(indW) = Y_crop_std_hu(id_max)*Y_crop_norm_hu(id_max);
        
        [T_hu, ~] = normalizeTemplates(T_hu);
        A_hu(:,1:nT)	= T_hu;
        
         %Temaplate Matrix
        Temp1_hu = [T_hu,fixT_hu]*pinv([T_hu,fixT_hu]);
        elseif occlusionNf_hu<0
            para.Lambda(3) = paraT.lambda(3);
    end
    
%     occlusionNf_sift = occlusionNf_sift-1;
%     if( min_angle_sift > angle_threshold  && occlusionNf_sift<0 )
%         disp('SIFT Update!')
%         [~,indW] = min(a_max_sift(1:nT));
%         % insert new template
%         T_sift(:,indW)	= Y_sift(:,id_max);
%         T_mean_sift(indW)= Y_crop_mean_sift(id_max);
%         T_id(indW)	= t; %track the replaced template for debugging
%         norms(indW) = Y_crop_std_sift(id_max)*Y_crop_norm_sift(id_max);
%         
%         [T_sift, ~] = normalizeTemplates(T_sift);
%         A_sift(:,1:nT)	= T_sift;
%         
%          %Temaplate Matrix
%         Temp1_sift = [T_sift,fixT_sift]*pinv([T_sift,fixT_sift]);
%         elseif occlusionNf_sift<0
%             para.Lambda(3) = paraT.lambda(3);
%     end
    
    Time_record(t) = toc;

    %-Store tracking result
    track_res(:,t) = map_aff';
    
    %-Demostration and debugging
    if paraT.bDebug
        s_debug_path = paraT.s_debug_path;
        % print debugging information
        fprintf('minimum angle: %f\n', min_angle);
        fprintf('Minimum error: %f\n', Min_Err(t));
        fprintf('T are: ');
        for i = 1:nT
            fprintf('%d ',T_id(i));
        end
        fprintf('\n');
        fprintf('coffs are: ');
        for i = 1:nT
            fprintf('%.3f ',c_max(i));
        end
        fprintf('\n\n');
        
%%         draw tracking results
        img_color	= double(img_color);
        img_color	= showTemplates(img_color, T, T_mean, norms, sz_T, nT);
        imshow(uint8(img_color));
        text(5,10,num2str(t),'FontSize',18,'Color','r');
        color = [1 0 0];
        drawAffine(map_aff, sz_T, color, 2);
        drawnow;
        
        if ~exist(s_debug_path,'dir')
            fprintf('Path %s not exist!\n', s_debug_path);
        else
            s_res	= s_frames{t}(1:end-4);
            s_res	= fliplr(strtok(fliplr(s_res),'/'));
            s_res	= fliplr(strtok(fliplr(s_res),'\'));
            s_res	= [s_debug_path s_res '_L1_APG.jpg'];
            saveas(gcf,s_res)
        end
     end
end
 
output.time = Time_record; % cpu time of APG method for each frame
output.minerr = Min_Err; % reconstruction error for each frame
% %output.coeff = Coeff;  % best coefficients for each frame
output.ratio = ratio;  % the energy of trivial templates
