function [result]=M_GetN(im,p,q)
%% ��һ�����ľ�
% im��double��ͼ��
% p��x�Ľ�����
% q��y�Ľ�����
% �������к����������Ϊx��������ϵ���Ϊy
% �Ӻ���
%%
result=M_GetU(im,p,q)/M_GetM(im,0,0)^((p+q+2)/2);