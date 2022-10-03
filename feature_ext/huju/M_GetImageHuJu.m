function [result]=M_GetImageHuJu(im)
%% 计算图像的Hu矩
% im为double型的
% 本程序中横向从左至右为x，纵向从上到下为y
% 主函数
%% 计算各阶中心距
n20=M_GetN(im,2,0);
n11=M_GetN(im,1,1);
n02=M_GetN(im,0,2);
n30=M_GetN(im,3,0);
n21=M_GetN(im,2,1);
n12=M_GetN(im,1,2);
n03=M_GetN(im,0,3);
%% 计算7个不变量
f1=n20+n02;
f2=(n20-n02)^2+4*n11^2;
f3=(n30-3*n12)^2+(3*n21-n03)^2;
f4=(n30+n12)^2+(n21+n03)^2;
f5=(n30-3*n12)*(n30+n12)*((n30+n12)^2-3*(n21+n03)^2)+(3*n21-n03)*(n21+n03)*(3*(n30+n12)^2-(n21+n03)^2);
f6=(n20-n02)*((n30+n12)^2-(n21+n03)^2)+4*n11*(n30+n12)*(n21+n03);
f7=(3*n21-n03)*(n30+n12)*((n30+n12)^2-3*(n21+n03)^2)-(n30-3*n12)*(n21+n03)*(3*(n30+n12)^2-(n21+n03)^2);
f1=abs(log10(abs(f1)));
f2=abs(log10(abs(f2)));
f3=abs(log10(abs(f3)));
f4=abs(log10(abs(f4)));
f5=abs(log10(abs(f5)));
f6=abs(log10(abs(f6)));
f7=abs(log10(abs(f7)));
result=[f1,f2,f3,f4,f5,f6,f7];