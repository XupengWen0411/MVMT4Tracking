function [result]=M_GetU(im,p,q)
%% 中心距
% im：double型图像；
% p：x的阶数；
% q：y的阶数；
% 本程序中横向从左至右为x，纵向从上到下为y
% 子函数
%% 初始化
[height,width]=size(im);
[x,y]=meshgrid(1:width,1:height);
%% 求平均值
m00=M_GetM(im,0,0);
avgX=M_GetM(im,1,0)/m00;
avgY=M_GetM(im,0,1)/m00;
x=x-avgX;
y=y-avgY;
%% 计算pq阶中心距
d=im.*(x.^p.*y.^q);
result=sum(sum(d));