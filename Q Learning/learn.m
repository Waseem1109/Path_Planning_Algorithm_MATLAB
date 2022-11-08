function [Q,Rewards,actions]=learn(map,q_goal,alpha,gamma,epsilon,n_episodes,n_iterations)
S=size(map);
ROWS=S(1);
COLUMNS=S(2);
FreeCell_reward=-1;
OccupiedCell_reward=-100;
GoalCell_reward=1;
Rewards=ones(ROWS,COLUMNS);
for r=1:S(1)
    for c=1:S(2)
        if(map(r,c)==0)
            Rewards(r,c)=FreeCell_reward;
        elseif(map(r,c)==1)
            Rewards(r,c)=OccupiedCell_reward;
        end
    end
end
g=q_goal(1);
l=S(1)-q_goal(2);
Rewards(l,g)=GoalCell_reward;
Q=zeros(S(1),S(2),4);
actions=[1,2,3,4];
%%
for episode=1:n_episodes
    [C_R_I,C_C_I]=NonOccupied_random_location(Rewards);
    for i=1:n_iterations
        if(isOccupied(C_R_I,C_C_I,Rewards)==0)
            A_I=get_new_action(Q,C_R_I,C_C_I,epsilon);
            O_R_I=C_R_I;O_C_I=C_C_I;
            [C_R_I,C_C_I]=goto_location(C_R_I,C_C_I,actions,A_I);
            Reward=Rewards(C_R_I,C_C_I);
            O_Q_V=Q(O_R_I,O_C_I,A_I);
            Action_Vector=Q(C_R_I,C_C_I,:);
            N_Q_V=O_Q_V+alpha*(Reward+(gamma*max(Action_Vector)-O_Q_V));
            Q(O_R_I,O_C_I,A_I)=N_Q_V;  
        else
            break;
        end
    end
end
end