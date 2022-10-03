function [track_res,output] = MTT_Tracking_APG(s_frames, paraT)
 

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
[T,T_norm,T_mean,T_std] = InitTemplates(sz_T,nT,img,init_pos);%初始化模板尺寸、模板数量、图像、模板截取初始坐标
norms = T_norm.*T_std; %template norms 模板规范化
occlusionNf = 0;

%% L1 function settings
angle_threshold = paraT.angle_threshold;
para.Lambda = paraT.lambda;
para.nT = paraT.nT;
para.Lip = paraT.Lip;
para.Maxit = paraT.Maxit;

dim_T	= size(T,1);	%number of elements in one template, sz_T(1)*sz_T(2)=12x15 = 180
A		= [T eye(dim_T)]; %data matrix is composed of T, positive trivial T.
alpha = 50;%this parameter is used in the calculation of the likelihood of particle filter
aff_obj = corners2affine(init_pos, sz_T); %get affine transformation parameters from the corner points in the first frame
map_aff = aff_obj.afnv;%aff_obj.afnv： affine parameters
aff_samples = ones(n_sample,1)*map_aff;

T_id	= -(1:nT);	% template IDs, for debugging
fixT = T(:,1)/nT; % first template is used as a fixed template

%Temaplate Matrix
Temp = [A fixT];
Dict = Temp'*Temp;
Temp1 = [T,fixT]*pinv([T,fixT]);

%% Tracking

% initialization
nframes	= length(s_frames);
track_res	= zeros(6,nframes);
Time_record = zeros(nframes,1);
Coeff = zeros(size([A fixT],2),nframes);
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
    [Y, Y_inrange] = crop_candidates(im2double(img), aff_samples(:,1:6), sz_T);
    if(sum(Y_inrange==0) == n_sample)
        sprintf('Target is out of the frame!\n');
    end
    
    [Y,Y_crop_mean,Y_crop_std] = whitening(Y);	 % zero-mean-unit-variance
    [Y, Y_crop_norm] = normalizeTemplates(Y); %norm one
    
    %-L1-LS for each candidate target
    eta_max	= -inf;%inf：无穷大
    q   = zeros(n_sample,1); % minimal error bound initialization
   
    % first stage L2-norm bounding    
    for j = 1:n_sample
        if Y_inrange(j)==0 || sum(abs(Y(:,j)))==0
            continue;
        end
        
        % L2 norm bounding L2准则边界
        q(j) = norm(Y(:,j)-Temp1*Y(:,j));
        q(j) = exp(-alpha*q(j)^2);
    end
    %  sort samples according to descend order of q 按照q数值大小从小到大排列样本
    [q,indq] = sort(q,'descend');% [Y, I] = sort(A, dim, mode) 功能：对矩阵A的各列或各行重新排序，I记录Y中的元素在排序前A中位置   
    
    % second stage
    p	= zeros(n_sample,1); % observation likelihood initialization 初始化样本检查的可能性
    n = 1;
    tau = 0;    
   %% 自己添加
    X1=cell(1 ,1);
    X_train =[]; 
    Y1=cell(1 ,1);
    Y_label =[];
   
    while (n<n_sample)&&(q(n)>=tau)        

       %% 自己添加
        X1{1}=Temp;
        X_train =[X_train,X1];
        Y1{1}=Y(:,indq(n));
        Y_label =[Y_label,Y1];
        
        [c] = APGLASSOup(Temp'*Y(:,indq(n)),Dict,para);
        
        D_s = (Y(:,indq(n)) - [A(:,1:nT) fixT]*[c(1:nT); c(end)]).^2;%reconstruction error 重构误差
        p(indq(n)) = exp(-alpha*(sum(D_s))); % probability w.r.t samples
        tau = tau + p(indq(n))/(2*n_sample-1);%update the threshold                 
        n = n+1;
    end
    
    X_train =X_train';
    Y_label =Y_label';
    
    opts.init = 0;      % guess start point from data. 
    opts.tFlag = 1;     % terminate after relative objective value does not changes much.
    opts.tol = 10^-3;   % tolerance. 
    opts.maxIter = 100; % maximum iteration number of optimization.
    rho_1 = 0.5;%2*sqrt(n)/train_num;%   rho1: P
    rho_2 = 0.5;%3*sqrt(n)/train_num; %   rho2: Q
    
    [W funcVal P Q] = Least_rMTFL(X_train, Y_label, rho_1, rho_2, opts);
    
    n1=size(W,2);
    for i=1:n1
        D_s = (Y_label{i}(:) - [A(:,1:nT) fixT]*[W(1:nT,i); W(end,i)]).^2;%reconstruction error 重构误差
        p(indq(i)) = exp(-alpha*(sum(D_s))); % probability w.r.t samples         
        if(sum(W(1:nT,i))<0) %remove the inverse intensity patterns
            continue;
        elseif(p(indq(n))>eta_max) 
            id_max	= indq(i);%可能性最大的测试样本的ID
            c_max	= W(:,i);%可能性最大的稀疏表示系数
            eta_max = p(indq(i));
            Min_Err(t) = sum(D_s);%第t帧的最小误差
        end
    end
    
    count(t) = n;%particles used to calculate the L1 minimization in each frame    
    
    % resample according to probability
    map_aff = aff_samples(id_max,1:6); %target transformation parameters with the maximum probability
    a_max	= c_max(1:nT);%nT指模板的数量，a_max为模板的稀疏表示系数
    [aff_samples, ~] = resample(aff_samples,p,map_aff); %resample the samples wrt. the probability
    [~, indA] = max(a_max);
    min_angle = images_angle(Y(:,id_max),A(:,indA));
    ratio(t) = norm(c_max(nT:end-1));
    Coeff (:,t) = c_max;%将稀疏表示系数存储到矩阵Coeff    
    
     %-Template update
     occlusionNf = occlusionNf-1;
     level = 0.03;
    if( min_angle > angle_threshold && occlusionNf<0 )        
        disp('Update!')
        trivial_coef = c_max(nT+1:end-1);
        trivial_coef = reshape(trivial_coef, sz_T);
        
        trivial_coef = im2bw(trivial_coef, level);

        se = [0 0 0 0 0;
            0 0 1 0 0;
            0 1 1 1 0;
            0 0 1 0 0'
            0 0 0 0 0];
        trivial_coef = imclose(trivial_coef, se);
        
        cc = bwconncomp(trivial_coef);
        stats = regionprops(cc, 'Area');
        areas = [stats.Area];
        
        % occlusion detection 遮挡检测
        if (max(areas) < round(0.25*prod(sz_T)))        
            % find the tempalte to be replaced
            [~,indW] = min(a_max(1:nT));
        
            % insert new template
            T(:,indW)	= Y(:,id_max);
            T_mean(indW)= Y_crop_mean(id_max);
            T_id(indW)	= t; %track the replaced template for debugging
            norms(indW) = Y_crop_std(id_max)*Y_crop_norm(id_max);
        
            [T, ~] = normalizeTemplates(T);
            A(:,1:nT)	= T;
        
            %Temaplate Matrix
            Temp = [A fixT];
            Dict = Temp'*Temp;
            Temp1 = [T,fixT]*pinv([T,fixT]);
        else
            occlusionNf = 5;
            % update L2 regularized term
            para.Lambda(3) = 0;
        end
    elseif occlusionNf<0
        para.Lambda(3) = paraT.lambda(3);
    end
    
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
output.coeff = Coeff;  % best coefficients for each frame
output.count = count;  % particles used to calculate the L1 minimization in each frame
output.ratio = ratio;  % the energy of trivial templates
