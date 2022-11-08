function [pthObj,solnInfo]=PLANING(map,TurningRadius,Validation_distance,MAX_Connection_distance,Iterations,pathpoints,CurrentPoint,GoalPoint)
bounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = TurningRadius;
stateValidator = validatorOccupancyMap(ss); 
stateValidator.Map = map;
stateValidator.ValidationDistance =Validation_distance;
planner = plannerRRTStar(ss,stateValidator);
planner.MaxConnectionDistance = MAX_Connection_distance;
planner.MaxIterations = Iterations;
planner.ContinueAfterGoalReached = true;
[pthObj, solnInfo] = plan(planner,CurrentPoint,GoalPoint);
interpolate(pthObj,pathpoints)
end