function PLOT_PLANSTATES(solnInfo,pthObj)
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-');
plot(pthObj.States(:,1),pthObj.States(:,2),'g-','LineWidth',2);
end