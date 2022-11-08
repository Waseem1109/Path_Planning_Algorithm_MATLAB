clc
clear
En_rows=11;
En_columns=11;
q_values=zeros(En_rows,En_columns,4);
%% defied Action:
%numeric code action, 0=up, 1=right, 2-down, 3-left
actions=["up","right","down","left"];
%%
rewards=-100*ones(En_rows,En_columns);
rewards(1,5)=100;
%%
rewards(2,2:10)=-1;
rewards(3,[2 8 10])=-1;
rewards(4,[2:8 10])=-1;
rewards(5,[4 8])=-1;
rewards(6,1:11)=-1;
rewards(7,6)=-1;
rewards(8,2:10)=-1;
rewards(9,[4 8])=-1;
rewards(10,1:11)=-1;
imagesc(rewards);
%%
epsilon=0.9;
discount_factor=0.9;
learning_rate=0.9;
for episode=1:1000
    [row_index,column_index]=get_starting_location(rewards);
    while(isTerminalState(row_index,column_index,rewards)==0)
        action_index=getNextAction(q_values,row_index,column_index,epsilon);
        old_row_index=row_index;old_column_index=column_index;
        [row_index,column_index]=get_next_location(actions,row_index,column_index,action_index);
        reward=rewards(row_index,column_index);
        old_q_value=q_values(old_row_index,old_column_index,action_index);
        d=q_values(row_index,column_index,:);
        temporal_difference=reward+(discount_factor*max(d)-old_q_value);
        new_q_value=old_q_value+(learning_rate*temporal_difference);
        q_values(old_row_index,old_column_index,action_index)=new_q_value;
    end
end
path=get_shortest_path(10,11,epsilon,q_values,actions,rewards)