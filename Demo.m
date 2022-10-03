clear;
close all
clc


addpath('InitTemplates\'); % load function 
addpath('Candidate\'); % load function 
addpath('Tracking\'); % load function 
addpath('MVMT\'); % load function

addpath('feature_ext\'); 
addpath('feature_ext\hog\'); % load function 
addpath('feature_ext\lbp\'); % load function
addpath('feature_ext\huju\'); % load function
addpath('feature_ext\color\'); % load function
addpath('feature_ext\sift\'); % load function
addpath('feature_ext\sift\descriptor\'); % load function
addpath('feature_ext\sift\key-location\'); % load function
addpath('feature_ext\sift\orientation\'); % load function
addpath('feature_ext\sift\scale-space\'); % load function
addpath('feature_ext\sift\util\'); % load function
addpath('feature_ext\sift\match\');

times  = 1; %operate times; to avoid overwriting previous saved tracking result in the .mat format
% title = 'owl';
title = 'ship1';
res_path='results\';

num=1;%track number[1，2，3]
nframes=5; 

%% parameter setting for each sequence
switch num
    case 1
% switch title
        
        %Initialization for the first frame. 
        %Each column is a point indicating a corner of the target in the first image. 
        %The 1st row is the y coordinate and the 2nd row is for x.
        %Let [p1 p2 p3] be the three points, they are used to determine the affine parameters of the target, as following
        %    p1(65,55)-----------p3(170,53)
        %         | 				|		 
        %         |     target      |
        %         | 				|	        
        %   p2(64,140)--------------
       
%         video_name = 'ship11';
%         video_name = 'owl';
        video_name = 'ship1';
        video_path = fullfile('.\data\',video_name);
        m_start_frame =95; %starting frame number
        %nframes		= 20; %393;	 %number of frames to be tracked
        Imgext		= 'jpg';				%image format
        numzeros	= 3;	%number of digits for the frame index
        s_frames	= cell(nframes,1);
        nz			= strcat('%0',num2str(numzeros),'d'); %number of zeros in the name of image
        for t=1:nframes
            image_no	= m_start_frame + (t-1);
            fid			= sprintf(nz, image_no);
%             s_frames{t}	= strcat(video_path,'\',fid,'.',Imgext);
            s_frames{t}	= strcat(video_path,'\','00480.avi-',fid,'.',Imgext);
        end
        %% initialize bounding box
        first_frame= imread(s_frames{1});
        % figure(1),imshow(first_frame);
        %mov=mmreader('F:\matlab\an\bin\video\00480.avi');     %用mmreader读入视频文件
        %first_frame=read(mov,1);%获取图像帧;
        figure(1),imshow(first_frame);
        hold on
        %% 截取目标框
        [temp1,rect1]=imcrop(first_frame);
        
        v1=rect1(1);
        v2=rect1(2);
        v3=rect1(3);
        v4=rect1(4);
       
        plot([v1,v1+v3],[v2,v2],[v1,v1],[v2,v2+v4],[v1,v1+v3],[v2+v4,v2+v4],[v1+v3,v1+v3],[v2,v2+v4],'LineWidth',2,'Color','y') 
        text(v1+round(rect1(3)/2),v2-20,['第',num2str(1),'个目标框'],'horiz','center','Color','g')
        
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%      
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
        %% 跟踪第一个目标框
        m_boundingbox = [rect1(1),rect1(2),rect1(3),rect1(4)];  % [left-top-x, left-top-y, width, height];

% init_pos	= SelectTarget(all_images{1});  % automatically get bounding box
% init_pos =  [p1 p2 p3];
% 			  p1-------------------p3
% 				\					\
% 				 \       target      \
% 				  \                   \
% 				  p2-------------------\  
        init_pos = [m_boundingbox(2)   m_boundingbox(2)+m_boundingbox(4)  m_boundingbox(2) ;
            m_boundingbox(1)   m_boundingbox(1)                   m_boundingbox(1)+m_boundingbox(3)];

        width = m_boundingbox(3);
        height = m_boundingbox(4);
        
       %% 	set object size including height and width based on the initialization		
       if min( 0.5*[height width]) < 25
           sz_T = 1.0 * [height width];
           if height > 80
               sz_T =  [ 0.5 *height width];  
           end
       else
           sz_T = 0.5 * [height width];
       end
       sz_T = ceil(sz_T);
       if min(sz_T>32)
           sz_T = [32 32];
       end
       sz_T = [32 32];
% end

%prepare the path for saving tracking results
res_path=[res_path title '\'];
if ~exist(res_path,'dir')
    mkdir(res_path);
end
%% parameters setting for tracking
para.lambda = [0.2,0.001,10]; % lambda 1, lambda 2 for a_T and a_I respectively, lambda 3 for the L2 norm parameter
% set para.lambda = [a,a,0]; then this the old model
para.angle_threshold = 40;
para.Lip	= 8;
para.Maxit	= 5;
para.nT		= 10;%number of templates for the sparse representation
para.rel_std_afnv = [0.03,0.0005,0.0005,0.03,1,1];%diviation of the sampling of particle filter
para.n_sample	= 100;		%number of particles
para.sz_T		= sz_T;
para.init_pos	= init_pos;
para.bDebug		= 0;		%debugging indicator
bShowSaveImage	= 1;       %indicator for result image show and save after tracking finished
para.s_debug_path = res_path;

%% main function for tracking
[tracking_res_1_1,output_1_1]  = L1TrackingBPR_APGup(s_frames, para);
[tracking_res_1_2,output_1_2]  = MTT_Tracking_APG(s_frames, para);
[tracking_res_1_3,output_1_3]  = MTView_Tracking_APG8(s_frames, para);
disp(['fps: ' num2str(nframes/sum(output_1_1.time))])

%% Output tracking results

%save([res_path title '_L1_APG_' num2str(times) '.mat'], 'tracking_res1','sz_T','output1');

if ~para.bDebug&&bShowSaveImage
    for t = 1:nframes
        img_color	= imread(s_frames{t});
        img_color	= double(img_color);
        figure(2);
        imshow(uint8(img_color));
        text(5,10,num2str(t+m_start_frame),'FontSize',18,'Color','r');
        %%%L1跟踪结果
        color = [1 0 0];
        
        map_afnv	= tracking_res_1_1(:,t)';
        drawAffine(map_afnv, sz_T, color, 2);%draw tracking result on the figure
      
        %%%Multi-task跟踪结果
        color = [0 1 0];
        
        map_afnv	= tracking_res_1_2(:,t)';
        drawAffine(map_afnv, sz_T, color, 2);%draw tracking result on the figure
   
        %%%Multi-view跟踪结果
        color = [0 0 1];
       
        map_afnv	= tracking_res_1_3(:,t)';
        drawAffine(map_afnv, sz_T, color, 2);%draw tracking result on the figure
        
 
        %save tracking result image
        s_res	= s_frames{t}(1:end-4);
        s_res	= fliplr(strtok(fliplr(s_res),'/'));
        s_res	= fliplr(strtok(fliplr(s_res),'\'));
        s_res	= [res_path s_res '_L1_APG.jpg'];
        saveas(gcf,s_res)
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 2
% switch title
       
        %Initialization for the first frame. 
        %Each column is a point indicating a corner of the target in the first image. 
        %The 1st row is the y coordinate and the 2nd row is for x.
        %Let [p1 p2 p3] be the three points, they are used to determine the affine parameters of the target, as following
        %    p1(65,55)-----------p3(170,53)
        %         | 				|		 
        %         |     target      |
        %         | 				|	        
        %   p2(64,140)--------------
       
        video_name = 'ship11';
        video_path = fullfile('.\data\',video_name);
        m_start_frame = 1;  %starting frame number
       % nframes		= 5; %393;	 %number of frames to be tracked
        Imgext		= 'jpg';				%image format
        numzeros	= 4;	%number of digits for the frame index
        s_frames	= cell(nframes,1);
        nz			= strcat('%0',num2str(numzeros),'d'); %number of zeros in the name of image
        for t=1:nframes
            image_no	= m_start_frame + (t-1);
            fid			= sprintf(nz, image_no);
            s_frames{t}	= strcat(video_path,'\',fid,'.',Imgext);
        end
        %% initialize bounding box
         first_frame= imread(s_frames{1});
        % figure(1),imshow(first_frame);
        %mov=mmreader('F:\matlab\an\bin\video\00480.avi');     %用mmreader读入视频文件
        %first_frame=read(mov,1);%获取图像帧;
        figure(1),imshow(first_frame);
        hold on
        %% 截取目标框
        [temp1,rect1]=imcrop(first_frame);
        [temp2,rect2]=imcrop(first_frame);
               
        v1=rect1(1);
        v2=rect1(2);
        v3=rect1(3);
        v4=rect1(4);
        plot([v1,v1+v3],[v2,v2],[v1,v1],[v2,v2+v4],[v1,v1+v3],[v2+v4,v2+v4],[v1+v3,v1+v3],[v2,v2+v4],'LineWidth',2,'Color','y') 
        text(v1+round(rect1(3)/2),v2-20,['第',num2str(1),'个目标框'],'horiz','center','Color','g')
        v1=rect2(1);
        v2=rect2(2);
        v3=rect2(3);
        v4=rect2(4);
        plot([v1,v1+v3],[v2,v2],[v1,v1],[v2,v2+v4],[v1,v1+v3],[v2+v4,v2+v4],[v1+v3,v1+v3],[v2,v2+v4],'LineWidth',2,'Color','y') 
        text(v1+round(rect2(3)/2),v2-20,['第',num2str(2),'个目标框'],'horiz','center','Color','g')
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%      
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
        %% 跟踪第一个目标框
        m_boundingbox = [rect1(1),rect1(2),rect1(3),rect1(4)];  % [left-top-x, left-top-y, width, height];

% init_pos	= SelectTarget(all_images{1});  % automatically get bounding box
% init_pos =  [p1 p2 p3];
% 			  p1-------------------p3
% 				\					\
% 				 \       target      \
% 				  \                   \
% 				  p2-------------------\  
        init_pos = [m_boundingbox(2)   m_boundingbox(2)+m_boundingbox(4)  m_boundingbox(2) ;
            m_boundingbox(1)   m_boundingbox(1)                   m_boundingbox(1)+m_boundingbox(3)];

        width = m_boundingbox(3);
        height = m_boundingbox(4);
        
       %% 	set object size including height and width based on the initialization		
       if min( 0.5*[height width]) < 25
           sz_T = 1.0 * [height width];
           if height > 80
               sz_T =  [ 0.5 *height width];  
           end
       else
           sz_T = 0.5 * [height width];
       end
       sz_T = ceil(sz_T);
       if min(sz_T>32)
           sz_T = [32 32];
       end
       sz_T = [32 32];
% end

%prepare the path for saving tracking results
res_path=[res_path title '\'];
if ~exist(res_path,'dir')
    mkdir(res_path);
end
%% parameters setting for tracking
para.lambda = [0.2,0.001,10]; % lambda 1, lambda 2 for a_T and a_I respectively, lambda 3 for the L2 norm parameter
% set para.lambda = [a,a,0]; then this the old model
para.angle_threshold = 40;
para.Lip	= 8;
para.Maxit	= 5;
para.nT		= 10;%number of templates for the sparse representation
para.rel_std_afnv = [0.03,0.0005,0.0005,0.03,1,1];%diviation of the sampling of particle filter
para.n_sample	= 100;		%number of particles
para.sz_T		= sz_T;
para.init_pos	= init_pos;
para.bDebug		= 0;		%debugging indicator
bShowSaveImage	= 1;       %indicator for result image show and save after tracking finished
para.s_debug_path = res_path;

%% main function for tracking
[tracking_res_1_1,output_1_1]  = L1TrackingBPR_APGup(s_frames, para);
[tracking_res_1_2,output_1_2]  = MTT_Tracking_APG(s_frames, para);
[tracking_res_1_3,output_1_3]  = MTView_Tracking_APG8(s_frames, para);
disp(['fps: ' num2str(nframes/sum(output_1_1.time))])

%% 跟踪第二个目标框
m_boundingbox = [rect2(1),rect2(2),rect2(3),rect2(4)];  % [left-top-x, left-top-y, width, height];

init_pos = [m_boundingbox(2)   m_boundingbox(2)+m_boundingbox(4)  m_boundingbox(2) ;
            m_boundingbox(1)   m_boundingbox(1)                   m_boundingbox(1)+m_boundingbox(3)];
para.init_pos	= init_pos;      

%% main function for tracking
[tracking_res_2_1,output_2_1]  = L1TrackingBPR_APGup(s_frames, para);
[tracking_res_2_2,output_2_2]  = MTT_Tracking_APG(s_frames, para);
[tracking_res_2_3,output_2_3]  = MTView_Tracking_APG8(s_frames, para);
disp(['fps: ' num2str(nframes/sum(output_2_1.time))])

%% Output tracking results

%save([res_path title '_L1_APG_' num2str(times) '.mat'], 'tracking_res1','sz_T','output1');

if ~para.bDebug&&bShowSaveImage
    for t = 1:nframes
        img_color	= imread(s_frames{t});
        img_color	= double(img_color);
        figure(2);
        imshow(uint8(img_color));
        text(5,10,num2str(t+m_start_frame),'FontSize',18,'Color','r');
        %%%L1跟踪结果
        color = [1 0 0];
        
        map_afnv	= tracking_res_1_1(:,t)';
        drawAffine(map_afnv, sz_T, color, 2);%draw tracking result on the figure
       
        map_afnv	= tracking_res_2_1(:,t)';
        drawAffine(map_afnv, sz_T, color, 2);%draw tracking result on the figure
       
        %%%Multi-task跟踪结果
        color = [0 1 0];
        
        map_afnv	= tracking_res_1_2(:,t)';
        drawAffine(map_afnv, sz_T, color, 2);%draw tracking result on the figure
        
        map_afnv	= tracking_res_2_2(:,t)';
        drawAffine(map_afnv, sz_T, color, 2);%draw tracking result on the figure
        
      
        %%%Multi-view跟踪结果
        color = [0 0 1];
       
        map_afnv	= tracking_res_1_3(:,t)';
        drawAffine(map_afnv, sz_T, color, 2);%draw tracking result on the figure
        
        map_afnv	= tracking_res_2_3(:,t)';
        drawAffine(map_afnv, sz_T, color, 2);%draw tracking result on the figure
 
        drawnow
        %save tracking result image
        s_res	= s_frames{t}(1:end-4);
        s_res	= fliplr(strtok(fliplr(s_res),'/'));
        s_res	= fliplr(strtok(fliplr(s_res),'\'));
        s_res	= [res_path s_res '_L1_APG.jpg'];
        saveas(gcf,s_res)
    end
end

    case 3
% switch title      
        %Initialization for the first frame. 
        %Each column is a point indicating a corner of the target in the first image. 
        %The 1st row is the y coordinate and the 2nd row is for x.
        %Let [p1 p2 p3] be the three points, they are used to determine the affine parameters of the target, as following
        %    p1(65,55)-----------p3(170,53)
        %         | 				|		 
        %         |     target      |
        %         | 				|	        
        %   p2(64,140)--------------
       
        video_name = 'ship11';
        video_path = fullfile('.\data\',video_name);
        m_start_frame = 1;  %starting frame number
       % nframes		= 5; %393;	 %number of frames to be tracked
        Imgext		= 'jpg';				%image format
        numzeros	= 4;	%number of digits for the frame index
        s_frames	= cell(nframes,1);
        nz			= strcat('%0',num2str(numzeros),'d'); %number of zeros in the name of image
        for t=1:10:nframes
            image_no	= m_start_frame + (t-1);
            fid			= sprintf(nz, image_no);
            s_frames{t}	= strcat(video_path,'\',fid,'.',Imgext);
        end
        %% initialize bounding box
        first_frame= imread(s_frames{1});
        % figure(1),imshow(first_frame);
        %mov=mmreader('F:\matlab\an\bin\video\00480.avi');     %用mmreader读入视频文件
        %first_frame=read(mov,1);%获取图像帧;
        figure(1),imshow(first_frame);
        hold on
        %% 截取目标框
        [temp1,rect1]=imcrop(first_frame);
        [temp2,rect2]=imcrop(first_frame);
        [temp3,rect3]=imcrop(first_frame);
        
        v1=rect1(1);
        v2=rect1(2);
        v3=rect1(3);
        v4=rect1(4);
        plot([v1,v1+v3],[v2,v2],[v1,v1],[v2,v2+v4],[v1,v1+v3],[v2+v4,v2+v4],[v1+v3,v1+v3],[v2,v2+v4],'LineWidth',2,'Color','y') 
        text(v1+round(rect1(3)/2),v2-20,['第',num2str(1),'个目标框'],'horiz','center','Color','g')
        v1=rect2(1);
        v2=rect2(2);
        v3=rect2(3);
        v4=rect2(4);
        plot([v1,v1+v3],[v2,v2],[v1,v1],[v2,v2+v4],[v1,v1+v3],[v2+v4,v2+v4],[v1+v3,v1+v3],[v2,v2+v4],'LineWidth',2,'Color','y') 
        text(v1+round(rect2(3)/2),v2-20,['第',num2str(2),'个目标框'],'horiz','center','Color','g')
        v1=rect3(1);
        v2=rect3(2);
        v3=rect3(3);
        v4=rect3(4);
        plot([v1,v1+v3],[v2,v2],[v1,v1],[v2,v2+v4],[v1,v1+v3],[v2+v4,v2+v4],[v1+v3,v1+v3],[v2,v2+v4],'LineWidth',2,'Color','y') 
        text(v1+round(rect3(3)/2),v2-20,['第',num2str(3),'个目标框'],'horiz','center','Color','g')
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%      
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
        %% 跟踪第一个目标框
        m_boundingbox = [rect1(1),rect1(2),rect1(3),rect1(4)];  % [left-top-x, left-top-y, width, height];

% init_pos	= SelectTarget(all_images{1});  % automatically get bounding box
% init_pos =  [p1 p2 p3];
% 			  p1-------------------p3
% 				\					\
% 				 \       target      \
% 				  \                   \
% 				  p2-------------------\  
        init_pos = [m_boundingbox(2)   m_boundingbox(2)+m_boundingbox(4)  m_boundingbox(2) ;
            m_boundingbox(1)   m_boundingbox(1)                   m_boundingbox(1)+m_boundingbox(3)];

        width = m_boundingbox(3);
        height = m_boundingbox(4);
        
       %% 	set object size including height and width based on the initialization		
       if min( 0.5*[height width]) < 25
           sz_T = 1.0 * [height width];
           if height > 80
               sz_T =  [ 0.5 *height width];  
           end
       else
           sz_T = 0.5 * [height width];
       end
       sz_T = ceil(sz_T);
       if min(sz_T>32)
           sz_T = [32 32];
       end
       sz_T = [32 32];
% end

%prepare the path for saving tracking results
res_path=[res_path title '\'];
if ~exist(res_path,'dir')
    mkdir(res_path);
end
%% parameters setting for tracking
para.lambda = [0.2,0.001,10]; % lambda 1, lambda 2 for a_T and a_I respectively, lambda 3 for the L2 norm parameter
% set para.lambda = [a,a,0]; then this the old model
para.angle_threshold = 40;
para.Lip	= 8;
para.Maxit	= 5;
para.nT		= 10;%number of templates for the sparse representation
para.rel_std_afnv = [0.03,0.0005,0.0005,0.03,1,1];%diviation of the sampling of particle filter
para.n_sample	= 100;		%number of particles
para.sz_T		= sz_T;
para.init_pos	= init_pos;
para.bDebug		= 0;		%debugging indicator
bShowSaveImage	= 1;       %indicator for result image show and save after tracking finished
para.s_debug_path = res_path;

%% main function for tracking
[tracking_res_1_1,output_1_1]  = L1TrackingBPR_APGup(s_frames, para);
[tracking_res_1_2,output_1_2]  = MTT_Tracking_APG(s_frames, para);
[tracking_res_1_3,output_1_3]  = MTView_Tracking_APG8(s_frames, para);
disp(['fps: ' num2str(nframes/sum(output_1_1.time))])

%% 跟踪第二个目标框
m_boundingbox = [rect2(1),rect2(2),rect2(3),rect2(4)];  % [left-top-x, left-top-y, width, height];

init_pos = [m_boundingbox(2)   m_boundingbox(2)+m_boundingbox(4)  m_boundingbox(2) ;
            m_boundingbox(1)   m_boundingbox(1)                   m_boundingbox(1)+m_boundingbox(3)];
para.init_pos	= init_pos;      

%% main function for tracking
[tracking_res_2_1,output_2_1]  = L1TrackingBPR_APGup(s_frames, para);
[tracking_res_2_2,output_2_2]  = MTT_Tracking_APG(s_frames, para);
[tracking_res_2_3,output_2_3]  = MTView_Tracking_APG8(s_frames, para);
disp(['fps: ' num2str(nframes/sum(output_2_1.time))])


%% 跟踪第三个目标框
m_boundingbox = [rect3(1),rect3(2),rect3(3),rect3(4)];  % [left-top-x, left-top-y, width, height];

init_pos = [m_boundingbox(2)   m_boundingbox(2)+m_boundingbox(4)  m_boundingbox(2) ;
            m_boundingbox(1)   m_boundingbox(1)                   m_boundingbox(1)+m_boundingbox(3)];
para.init_pos	= init_pos;      

%% main function for tracking
[tracking_res_3_1,output_3_1]  = L1TrackingBPR_APGup(s_frames, para);
[tracking_res_3_2,output_3_2]  = MTT_Tracking_APG(s_frames, para);
[tracking_res_3_3,output_3_3]  = MTView_Tracking_APG8(s_frames, para);
disp(['fps: ' num2str(nframes/sum(output_2_1.time))])


%% Output tracking results

%save([res_path title '_L1_APG_' num2str(times) '.mat'], 'tracking_res1','sz_T','output1');

if ~para.bDebug&&bShowSaveImage
    for t = 1:nframes
        img_color	= imread(s_frames{t});
        img_color	= double(img_color);
        figure(2);
        imshow(uint8(img_color));
        text(5,10,num2str(t+m_start_frame),'FontSize',18,'Color','r');
        %%%L1跟踪结果
        color = [1 0 0];
        
        map_afnv	= tracking_res_1_1(:,t)';
        drawAffine(map_afnv, sz_T, color, 2);%draw tracking result on the figure
       
        map_afnv	= tracking_res_2_1(:,t)';
        drawAffine(map_afnv, sz_T, color, 2);%draw tracking result on the figure
        
        map_afnv	= tracking_res_3_1(:,t)';
        drawAffine(map_afnv, sz_T, color, 2);%draw tracking result on the figure
        
%         %%%Multi-task跟踪结果
        color = [0 1 0];
         
         map_afnv	= tracking_res_1_2(:,t)';
         drawAffine(map_afnv, sz_T, color, 2);%draw tracking result on the figure
         
         map_afnv	= tracking_res_2_2(:,t)';
         drawAffine(map_afnv, sz_T, color, 2);%draw tracking result on the figure
         
         map_afnv	= tracking_res_3_2(:,t)';
         drawAffine(map_afnv, sz_T, color, 2);%draw tracking result on the figure
        
        %%%Multi-view跟踪结果
        color = [0 0 1];
       
        map_afnv	= tracking_res_1_3(:,t)';
        drawAffine(map_afnv, sz_T, color, 2);%draw tracking result on the figure
        
        map_afnv	= tracking_res_2_3(:,t)';
        drawAffine(map_afnv, sz_T, color, 2);%draw tracking result on the figure
        
        map_afnv	= tracking_res_3_3(:,t)';
        drawAffine(map_afnv, sz_T, color, 2);%draw tracking result on the figure
        
        drawnow
        %save tracking result image
        s_res	= s_frames{t}(1:end-4);
        s_res	= fliplr(strtok(fliplr(s_res),'/'));
        s_res	= fliplr(strtok(fliplr(s_res),'\'));
        s_res	= [res_path s_res '_L1_APG.jpg'];
        saveas(gcf,s_res)
    end
end
end