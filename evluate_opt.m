function LOSS=evluate_opt(x1,height,h_max,C,N,n1,bc,user_location,Bm)
Zqn=zeros(N,N);%初始化
xnm=zeros(N,n1);%初始化
ymq=zeros(n1,N);%初始化
x_ind=[x1(1,1:2);x1(1,3:4);x1(1,5:6)];%无人机位置
x_list=x1(7:9); %标号
member=[];
for i=1:1:n1
    x_uav=[x_ind(i,:),height];
    distance=[];nj=[];
    for j=1:1:size(user_location,1)
       if ~any(j==member) %已经分配后的最短距离的用户不可以在分配，表示如果新的用户不属于已经分配的才对其进行无人机接收其任务
           dl1=sqrt(sum((x_uav-[user_location(j,:),0]).^2));%无人机和用户的位置
           if (dl1 > 0 & dl1 < h_max)
               Zqn(x_list(i),j)=1;
               distance=[distance,dl1];
               nj=[nj,j];
           end
       end
    end
    [distance,I]=sort(distance,'ascend');%传输距离排序,根据距离长短优先进行缓存，使得达到最大的任务分配
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
%计算Task
task=0;
for i2=1:1:n1%无人机
    T_B=0;
   for i1=1:1:N %用户
       for i3=1:1:N%位置
            T_B=T_B+xnm(i1,i2)*ymq(i2,i3)*Zqn(i3,i1);
       end
   end
    task=task+min([Bm(i2)/bc,T_B]);
end
LOSS=task;
end