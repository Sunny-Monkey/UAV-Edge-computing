function [x_ind,best_task]=one_step(user_location,height,h_max,C,N,n1,bc,Bm)
%无人机在A区域可能位置Q
Q=[];
Q=[Q;user_location];%包含用户
%两两用户中心
for i=1:1:size(user_location,1)-1
    for j=i+1:1:size(user_location,1)
        Q=[Q;[(user_location(i,:)+user_location(j,:))/2]];
    end
end
%三个用户的中心
for i=1:1:size(user_location,1)-2
    for j=i+1:1:size(user_location,1)-1
        for k=j+1:1:size(user_location,1)
        Q=[Q;[(user_location(i,:)+user_location(j,:)+user_location(k,:))/3]];
        end
    end
end

%%找出一个每个用户连接的无人机位置
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
    location_uav(i,:)=Q(idl(randperm(length(idl),1)),:);%合并到第一个点
end
%Zqn
%X表示无人机的位置---遍历的无人机的位置
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
%%%%%%%%%%优化参数%%%%%%%
parfor ip=1:1:size(uav_m,1)
    LOSS(ip)=evluate_opt(uav_m(ip,:),height,h_max,C,N,n1,bc,user_location,Bm);
end
[~,DEN]=find(max(LOSS)==LOSS);
IND=DEN(randperm(length(DEN),1));
best=uav_m(IND,1:6);
x_ind=[[best(1,1:2) height];[best(1,3:4) height];[best(1,5:6) height]];%无人机位置
best_task=LOSS(IND);
%绘图片的时候用 user_location和x_ind
