function x_in=allot_uav(x,y)
%x表示新的UAV位置(x1,y1,z1)
%y表示旧的UAV位置(x2,y2,z2)
member=[];
for i=1:1:size(x,1)
    new=x(i,:);
    for j=1:1:size(y,1)
        if ~any(j==member)
           dist(j)=sqrt(sum((new-y(j,:)).^2));
        else
           dist(j)=0;
        end
    end
    [~,len]=find(0~=dist); %判断不为零
    dist1=dist(len);%把不为零的复制
    [~,len1]=find(min(dist1)==dist1); %判断不为零
    %找出最短距离
    x_in(len(len1),:)=new;
    member=[member,len(len1)];
end
end