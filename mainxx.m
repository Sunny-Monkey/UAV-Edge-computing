clc
close all
clear all
disp('无人机优化模型')
%参数准备
%区域用户位置
d1=100; %区域A长度
d2=80; %区域A长度
N=30;%用户个数
C=6;%天线个数
bc=1;%电池损耗/每个内容---假设容量相等 电池消耗相同
Bm=[10 10 10];%无人机容量
%索引位置tuple={content k,UAV m，location q}
height=10;
h_max=20;
delta_d=0.5;%location interval
n1=3;%无人机个数
%discrete the location
%用户位置
%discrete the location
d1a=0:delta_d:d1;
d2a=0:delta_d:d2;
%离散位置
index=[];
for i=1:1:length(d1a)
    for j=1:1:length(d2a)
        index=[index;[d1a(i),d2a(j)]];
    end
end
num_index=size(index,1);%number of location
%新增人群位置计算
%新增人群不可能出现位置
d1a1=[5,d1-5];
d2a1=[5,d2-5];
[len,~]=find((index(:,1)>=d1a1(1) & index(:,1)<=d1a1(2)) & (index(:,2)>=d2a1(1) & index(:,2)<=d2a1(2)));
id_new=setdiff(1:1:size(index,1),len');  %新增用户编号
new_candi=index(id_new',:);%新增人群位置
%随机生成用户
user_index=randperm(num_index,N);
user_location=[];
for i=1:1:length(user_index)
    user_location=[user_location;index(user_index(i),:)];%%N*2
end
%仿真时间T
T=5;%仿真时间T
v=1;%用户行走速度
delta_p=2;%单位时间变化人数
theta_max=2*pi;%用户随机行走方向
best_uav=cell(1,T);
best_task=zeros(1,T);
[x_ind,task]=one_step(user_location,height,h_max,C,N,n1,bc,Bm);  %主程序
best_uav{1,1}=x_ind;
best_task(1,1)=task;
old_ind=x_ind;%设置就得无人机位置
for t=2:1:T
    %更新用户
    new_user_location=[];
    %考虑原始用户行走
    for i=1:1:size(user_location,1) 
      user_location(i,1)=user_location(i,1)+v*cos(random('unif',0,theta_max));
      user_location(i,2)=user_location(i,2)+v*sin(random('unif',0,theta_max));
      %边界更新，如果超出边界 则认为用户离开
      if (user_location(i,1)>d1) | (user_location(i,1)<0) | (user_location(i,2)<0)|(user_location(i,2)>d2)
          continue;%用户离开不管
      else
          new_user_location=[new_user_location;user_location(i,:)]; %用户没离开保留
      end
    end
    %当前时刻新来客户
    num_new=random('poiss',delta_p);
    if num_new==0
        new_user_location=new_user_location;
    else
        new_id=randperm(length(id_new),num_new);
        new_coming=new_candi(new_id',:);
        new_user_location=[new_user_location; new_coming];
    end
    %用户个数更新
    N=size(new_user_location,1);
    %计算当前无人机位置
    [x_ind,task]=one_step(new_user_location,height,h_max,C,N,n1,bc,Bm);  %主程序
    %无人机位置分配--中心服务器向无人机分配无人机更新位置
    x_ind=allot_uav(x_ind,old_ind);
    best_uav{1,t}=x_ind;
    best_task(1,t)=task;
    %我下一时刻用户位置更新
    user_location=new_user_location;%上一时刻用户位置
    old_ind=x_ind;
    if mod(t,20)==0
    figure,
    plot(user_location(:,1),user_location(:,2),'r*')
    xlabel('x')
    ylabel('y')
    title(num2str(t))
    end
end
figure,plot(1:1:T,best_task,'->')
xlabel('T/(min)')
ylabel('Task/(n)')
grid on
ylim([0 40])
Q1=[];Q2=[];Q3=[];
for i=1:1:T
Q1=[Q1,(best_uav{1,i}(1,:))'];
Q2=[Q2,(best_uav{1,i}(2,:))'];
Q3=[Q3,(best_uav{1,i}(3,:))'];
end
figure,
plot3(Q1(1,:),Q1(2,:),Q1(3,:),'r->')
hold on
plot3(Q2(1,:),Q2(2,:),Q2(3,:),'b->')
hold on
plot3(Q3(1,:),Q3(2,:),Q3(3,:),'c->')
xlabel('x')
ylabel('y')
zlabel('height')
grid on
legend('UAV-1','UAV-2','UAV-3')
title('the location of UAV')

