function [x_ind,best_task]=one_step(user_location,height,h_max,C,N,n1,bc,Bm)
%���˻���A�������λ��Q
Q=[];
Q=[Q;user_location];%�����û�
%�����û�����
for i=1:1:size(user_location,1)-1
    for j=i+1:1:size(user_location,1)
        Q=[Q;[(user_location(i,:)+user_location(j,:))/2]];
    end
end
%�����û�������
for i=1:1:size(user_location,1)-2
    for j=i+1:1:size(user_location,1)-1
        for k=j+1:1:size(user_location,1)
        Q=[Q;[(user_location(i,:)+user_location(j,:)+user_location(k,:))/3]];
        end
    end
end

%%�ҳ�һ��ÿ���û����ӵ����˻�λ��
for i=1:1:N
    idl=[];
    for j=1:1:size(Q,1)
        d1=sqrt(sum(([Q(j,:),height]-[user_location(i,:),0]).^2));
        if (d1 > 0 & d1 < h_max)
            idl=[idl;j];%%%%%
        else
            continue;
        end
    end
    location_uav(i,:)=Q(idl(randperm(length(idl),1)),:);%�ϲ�����һ����
end
%Zqn
%X��ʾ���˻���λ��---���������˻���λ��
uav_m=[];
for i=1:1:N
   for j=1:1:N
       for k=1:1:N
           if (i~=j & j~=k & i~=k)
               uav_m=[uav_m;[location_uav(i,:),location_uav(j,:),location_uav(k,:),i,j,k]];
           end
       end
   end
end
%%%%%%%%%%�Ż�����%%%%%%%
parfor ip=1:1:size(uav_m,1)
    LOSS(ip)=evluate_opt(uav_m(ip,:),height,h_max,C,N,n1,bc,user_location,Bm);
end
[~,DEN]=find(max(LOSS)==LOSS);
IND=DEN(randperm(length(DEN),1));
best=uav_m(IND,1:6);
x_ind=[[best(1,1:2) height];[best(1,3:4) height];[best(1,5:6) height]];%���˻�λ��
best_task=LOSS(IND);
%��ͼƬ��ʱ���� user_location��x_ind
