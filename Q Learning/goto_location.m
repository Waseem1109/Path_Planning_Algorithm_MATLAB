function [N_R_I,N_C_I]=goto_location(C_R_I,C_C_I,actions,A_I)
N_R_I=C_R_I;N_C_I=C_C_I;
if(actions(A_I)==1)
    N_C_I=C_C_I-1;
elseif(actions(A_I)==2)
    N_R_I=C_R_I-1;
elseif(actions(A_I)==3)
    N_C_I=C_C_I+1;
elseif(actions(A_I)==4)
    N_R_I=C_R_I+1;
end
end