function [result]=M_GetM(im,p,q)
%% 计算原点矩
% im：double型图像；
% p：x的阶数；
% q：y的阶数；
% 本程序中横向从左至右为x，纵向从上到下为y
% 子函数
%%
% 如果是00阶直接加速返回结果
if p==0 && q==0
    result=sum(sum(im));
    return
else
    [height,width]=size(im);
    [x,y]=meshgrid(1:width,1:height);
    x=x.^p;
    y=y.^q;
    result=sum(sum(im.*x.*y));
end