function [result]=M_GetN(im,p,q)
%% 归一化中心距
% im：double型图像；
% p：x的阶数；
% q：y的阶数；
% 本程序中横向从左至右为x，纵向从上到下为y
% 子函数
%%
result=M_GetU(im,p,q)/M_GetM(im,0,0)^((p+q+2)/2);