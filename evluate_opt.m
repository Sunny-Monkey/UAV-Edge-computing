function LOSS=evluate_opt(x1,height,h_max,C,N,n1,bc,user_location,Bm)
Zqn=zeros(N,N);%��ʼ��
xnm=zeros(N,n1);%��ʼ��
ymq=zeros(n1,N);%��ʼ��
x_ind=[x1(1,1:2);x1(1,3:4);x1(1,5:6)];%���˻�λ��
x_list=x1(7:9); %���
member=[];
for i=1:1:n1
    x_uav=[x_ind(i,:),height];
    distance=[];nj=[];
    for j=1:1:size(user_location,1)
       if ~any(j==member) %�Ѿ���������̾�����û��������ڷ��䣬��ʾ����µ��û��������Ѿ�����ĲŶ���������˻�����������
           dl1=sqrt(sum((x_uav-[user_location(j,:),0]).^2));%���˻����û���λ��
           if (dl1 > 0 & dl1 < h_max)
               Zqn(x_list(i),j)=1;
               distance=[distance,dl1];
               nj=[nj,j];
           end
       end
    end
    [distance,I]=sort(distance,'ascend');%�����������,���ݾ��볤�����Ƚ��л��棬ʹ�ôﵽ�����������
    if length(I)<=C
        I1=I(1:min([length(I),C]));
        distance1=distance(1:min([length(I),C]));
        nj1=nj(1:min([length(I),C])); 
    else
        I1=I(1:C);
        distance1=distance(1:C);
        nj1=nj(1:C);
    end
    member=[member,nj1];
    xnm(nj1',i)=1;
    ymq(i,x_list(i))=1;
end
%����Task
task=0;
for i2=1:1:n1%���˻�
    T_B=0;
   for i1=1:1:N %�û�
       for i3=1:1:N%λ��
            T_B=T_B+xnm(i1,i2)*ymq(i2,i3)*Zqn(i3,i1);
       end
   end
    task=task+min([Bm(i2)/bc,T_B]);
end
LOSS=task;
end