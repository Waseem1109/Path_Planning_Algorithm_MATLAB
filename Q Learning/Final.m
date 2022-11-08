clc
clear
map=ones(14,20);
map(2:13,2:19)=0;
map([7 8],5:9)=1;
map([2 3],[7 8])=1;
map([9 10],[8 9])=1;
map(13,13:15)=1;
map(12,14:15)=1;
map(11,15)=1;
map(2:7,15:16)=1;
q_goal=[18,11];
StartPoint=[3,3];
epsilon=0.9;alpha=0.9;gamma=0.9;n_episodes=1000;n_iterations=2000; % greedy poicy is epsilon
[Q,Rewards,actions]=learn(map,q_goal,alpha,gamma,epsilon,n_episodes,n_iterations);
path=FIND_OPTIMAL_PATH(StartPoint,epsilon,Q,actions,Rewards);
M = binaryOccupancyMap(map);
show(M)
hold on
plot(path(:,1),path(:,2),'r+','LineWidth',2);
plot(StartPoint(1),StartPoint(2),'ro','LineWidth',2);
plot(q_goal(1),q_goal(2),'ro','LineWidth',2);
S=size(map);
for r=1:(S(2)-1)
    for c=1:(S(1)-1)
        H=Q(14-c,r,:);
        A=find(H==max(H));
        if(A==1)
            text(r,c,'\leftarrow');
        elseif(A==2)
            text(r,c,'\uparrow');
        elseif(A==3)
            text(r,c,'\rightarrow');
        elseif(A==4)
            text(r,c,'\downarrow');
        end
            
    end
end
