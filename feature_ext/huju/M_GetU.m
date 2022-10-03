function [result]=M_GetU(im,p,q)
%% ���ľ�
% im��double��ͼ��
% p��x�Ľ�����
% q��y�Ľ�����
% �������к����������Ϊx��������ϵ���Ϊy
% �Ӻ���
%% ��ʼ��
[height,width]=size(im);
[x,y]=meshgrid(1:width,1:height);
%% ��ƽ��ֵ
m00=M_GetM(im,0,0);
avgX=M_GetM(im,1,0)/m00;
avgY=M_GetM(im,0,1)/m00;
x=x-avgX;
y=y-avgY;
%% ����pq�����ľ�
d=im.*(x.^p.*y.^q);
result=sum(sum(d));