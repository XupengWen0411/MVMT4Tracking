function [result]=M_GetM(im,p,q)
%% ����ԭ���
% im��double��ͼ��
% p��x�Ľ�����
% q��y�Ľ�����
% �������к����������Ϊx��������ϵ���Ϊy
% �Ӻ���
%%
% �����00��ֱ�Ӽ��ٷ��ؽ��
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