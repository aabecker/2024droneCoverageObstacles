% This code implements Matlab's built-in functions to cover a rect
% workspace with a static obstacle. Pretty graphics included
% 
% TODOs: - Avoid creating islands if possible to minimize need to backtrack

clear; close all; clc;

% Define workspace h and w
h = 20;
w = 20;
ws = ones(h,w);

% Create random rect obstacle with specified h and w
obs_h = 5;
obs_w = 5;
obs_y = randi(h-obs_h,1);
obs_x = randi(w-obs_w,1);
ws(obs_y:obs_y+obs_h,obs_x:obs_x+obs_w) = 0;

% Start at random (valid) position
robot = randi(h*w);
while(ws(robot)==0)
    robot = randi(h*w);
end
ws(robot) = 3;

% Theoretical minimal cost with this kind of setup
min_cost = length(find(ws>0))-1;

% Create graph to represent problem
S = [];
T = [];
ind = 1;
for c = 1:w
    for r = 1:h
        if(ws(ind))
            if(r<h && ws(ind+1))
                S = [S,ind];
                T = [T,ind+1];
            end
            if(c<w && ws(ind+h))
                S = [S,ind];
                T = [T,ind+h];
            end
        end
        ind = ind+1;
    end
end   
weights = ones(length(S),1);
G = graph(S,T,weights);
% T = minspantree(G,'Root',robot);

fprintf('Robot starts at %d\n',robot);
figure();
im = imagesc(ws); colormap hot; 

curr_cost = 0;
iter = 1;
while(iter<1e3)
    % Find the next possible paths that do not cycle back
    edges = G.Edges{:,:};
    paths = [];
    for e = 1:size(edges,1)
        if((edges(e,1)==robot || edges(e,2)==robot) && edges(e,3)==1)
            paths = [paths;edges(e,:)];
            prev = robot;
            robot = edges(e,1)+edges(e,2)-robot;
            ws(prev) = 2;
            ws(robot) = 3;
            break;
        end
    end
    
%     % Try to prevent disconnects
%     dend = 0;
%     for p = 1:size(paths,1)
%         maybe = ws;
%         maybe([paths(p,1),paths(p,2)]) = 2;
%     end
    
    if(isempty(paths))
        % Disconnect happened :(
        % Find the closest unvisited node
        path = closestP(ws,h,w,G,robot);
        for ind = 2:length(path)
            prev = robot;
            robot = path(ind);
            ws(prev) = 2;
            ws(robot) = 3;
            
            delete(im); im = imagesc(ws);
            curr_cost = curr_cost+1;
            title(sprintf('min cost = %d, curr cost = %d',min_cost,curr_cost));
            pause(.1);
        end 
    else
        delete(im); im = imagesc(ws);
        curr_cost = curr_cost+1;
        title(sprintf('min cost = %d, curr cost = %d',min_cost,curr_cost));
        pause(.1);
    end
    
    % Increase the cost to revisit the previous node
    for e = 1:size(edges,1)
        if(edges(e,1)==prev || edges(e,2)==prev)
            G.Edges{e,2} = 1e3;
        end
    end
    
    % End if all nodes have been visited
    if(isempty(find(G.Edges{:,2}<1e3)))
        break;
    end
    iter = iter+1;
end
hold off
    

% This function used matlabs shortestpath to find the closest unvisited
% node to current node s
function path = closestP(ws,h,w,G,s)    
    G.Edges{:,2} = 1;
    min_cost = 1e3;
    
    for ind = 1:w*h
        if(ws(ind)==1)
            [P,cost] = shortestpath(G,s,ind);
            if(cost<min_cost)
                min_cost = cost;
                path = P;
            end
        end
    end
end