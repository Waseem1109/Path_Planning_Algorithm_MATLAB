function path=PLANNER_RRT(x_size,y_size,stepsize,max_iteration,Obstacles,Start_point, Goal_point,Connecting_radius)
x_max =x_size;
y_max =y_size;
EPS =stepsize;
numodes = max_iteration;

obstacles = Obstacles;
qstart.coord = [Start_point(1) Start_point(2)];
qstart.cost = 0;
qstart.parent = 0;
qgoal.coord = [Goal_point(1) Goal_point(2)];
qgoal.cost =0;

nodes(1) = qstart;
 figure(1)
 axis([0 x_max 0 y_max])
 os=size(obstacles);
 if os(1,1)~=0
     for i=1:1:os(1,1)
         rectangle('Position',obstacles(i,:),'FaceColor',[0 .5 .5])
     end
 end
hold on

for i=1:1:numodes
    q_rand = [floor(rand(1)*x_max) floor(rand(1)*y_max)];
    plot(q_rand(1), q_rand(2), 'x', 'Color',  [0 0.4470 0.7410])
    for j=1:1:length(nodes)
        if nodes(j).coord == qgoal.coord
            break
        end
    end
    ndist =[];
    for j=1:1:length(nodes)
        n=nodes(j);
        tmp = dist(n.coord,q_rand);
        ndist=[ndist tmp];
    end
    [val,index] = min(ndist);
    q_near=nodes(index);
    q_new.coord = steer(q_rand, q_near.coord, val, EPS);
    if noCollision(q_rand, q_near.coord, obstacles)
                 line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], 'Color', 'k', 'LineWidth', 2);
                 drawnow
                 hold on
        q_new.cost = dist(q_near.coord,q_new.coord)+q_near.cost;
        q_nearest=[];
        neighbor_count =1;
        r = Connecting_radius;
        for k=1:1:length(nodes)
            if(noCollision(q_new.coord, nodes(k).coord, obstacles) && dist(q_new.coord, nodes(k).coord) <=r)
                q_nearest(neighbor_count).coord = nodes(k).coord;
                q_nearest(neighbor_count).cost = nodes(k).cost;
                neighbor_count =neighbor_count+1;
            end
        end
        % Initialize cost to currently known value
        q_min = q_near;
        C_min = q_new.cost;
        
        % Iterate through all nearest neighbors to find alternate lower
        % cost paths
        
        for k = 1:1:length(q_nearest)
            if noCollision(q_nearest(k).coord, q_new.coord, obstacles) && q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord) < C_min
                q_min = q_nearest(k);
                C_min = q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord);
                line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], 'Color', 'g');
                hold on
            end
        end
        
        % Update parent to least cost-from node
        for j = 1:1:length(nodes)
            if nodes(j).coord == q_min.coord
                q_new.parent = j;
            end
        end
        nodes = [nodes q_new];
    end
end
D = [];
for j = 1:1:length(nodes)
    tmpdist = dist(nodes(j).coord, qgoal.coord);
    D = [D tmpdist];
end

% Search backwards from goal to start to find the optimal least cost path
[val, idx] = min(D);
q_final = nodes(idx);
qgoal.parent = idx;
q_end = qgoal;
nodes = [nodes qgoal];
pathi =[qgoal.coord];
while q_end.parent ~= 0
    start = q_end.parent;
    pathi = [pathi;nodes(start).coord];
    line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], 'Color', 'r', 'LineWidth', 2);
    hold on
    q_end = nodes(start);
end
path =[];
os=size(pathi);
for i=1:1:os(1,1)
    l=os(1,1)-i+1;
    path(i,:)=pathi(l,:);
end
disp(path)
end
