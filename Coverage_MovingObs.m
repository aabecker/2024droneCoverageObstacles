% This code implements Matlab's built-in functions to cover a rect
% workspace with a moving obstacle. Pretty graphics included
% 
% TODOs: - Avoid creating islands if possible to minimize need to backtrack
%        - Implement horizon to avoid collision

clear; close all; clc;

% Define workspace h and w
h = 25;
w = 25;
ws = ones(h,w);
cover = 0*ws;

% Create random rect obstacle with specified h and w at an edge of ws
obs_d = [5,5;3,3;4,4];
obs_p = [];
obs_dir = [];
for o = 1:size(obs_d,1)
    obs_h = obs_d(o,1);
    obs_w = obs_d(o,2);

    side = randi(4,1);
    switch side
        case 1
            obs_x = 1;
            obs_y = randi(h-obs_h,1);
            obs_dir = [obs_dir;1];
        case 2
            obs_y = h-obs_d(1);
            obs_x = randi(w-obs_w,1);
            obs_dir = [obs_dir;2];
        case 3
            obs_x = h-obs_d(1);
            obs_y = randi(h-obs_h,1);
            obs_dir = [obs_dir;3];
        case 4
            obs_y = 1;
            obs_x = randi(w-obs_w,1);
            obs_dir = [obs_dir;4];
    end
    ws(obs_y:obs_y+obs_h,obs_x:obs_x+obs_w) = 0;
    obs_p = [obs_p;obs_y,obs_x];
end

% Start at random (valid) position
robot = randi(h*w);
while(ws(robot)==0)
    robot = randi(h*w);
end
ws(robot) = 3;

fprintf('Robot starts at %d\n',robot);
figure();
im = imagesc(ws); colormap hot; 

% Theoretical minimal cost with this kind of setup
min_cost = numel(ws)-1;

curr_cost = 0;
iter = 1;
while(iter<1e3)
    % Move the obstacles
    for o = 1:size(obs_d,1)
        obs_y = obs_p(o,1);
        obs_x = obs_p(o,2);
        obs_h = obs_d(o,1);
        obs_w = obs_d(o,2);
        ws(obs_y:obs_y+obs_h,obs_x:obs_x+obs_w) = 1;
    end
        
    for o = 1:size(obs_d,1)
        obs_y = obs_p(o,1);
        obs_x = obs_p(o,2);
        obs_h = obs_d(o,1);
        obs_w = obs_d(o,2);
        
        switch obs_dir(o)
            case 1
                if(obs_x+obs_w<w) obs_x = obs_x+1;
                else obs_dir(o) = 3;
                end
            case 2 
                if(obs_y>1) obs_y = obs_y-1;
                else obs_dir(o) = 4;
                end
            case 3
                if(obs_x>1) obs_x = obs_x-1;
                else obs_dir(o) = 1;
                end
            case 4
                if(obs_y+obs_h<h) obs_y = obs_y+1;
                else obs_dir(o) = 2;
                end
        end
        ws(obs_y:obs_y+obs_h,obs_x:obs_x+obs_w) = 0;
        obs_p(o,:) = [obs_y,obs_x];
    end
    
    % Remember prior coverage
    for ind = 1:w*h
        if(ws(ind)==1)
            ws(ind) = 1+cover(ind);
        end
    end
    
    % Create graph to represent problem
    S = [];
    T = [];
    weights = [];
    ind = 1;
    for c = 1:w
        for r = 1:h
            if(ws(ind))
                if(r<h && ws(ind+1))
                    S = [S,ind];
                    T = [T,ind+1];
                    if(cover(ind) || cover(ind+1)) weights = [weights,1e3];
                    else weights = [weights,1];
                    end
                end
                if(c<w && ws(ind+h))
                    S = [S,ind];
                    T = [T,ind+h];
                    if(cover(ind) || cover(ind+1)) weights = [weights,1e3];
                    else weights = [weights,1];
                    end
                end
            end
            ind = ind+1;
        end
    end   
    G = graph(S,T,weights);
    
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
            cover(prev) = 1;
            break;
        end
    end
    
    if(isempty(paths))
        % Disconnect happened :(
        % Find the closest unvisited node
        path = closestP(ws,h,w,G,robot);
        for ind = 2:length(path)
            prev = robot;
            robot = path(ind);
            ws(prev) = 2;
            ws(robot) = 3;
            cover(prev) = 1;
            
            delete(im); im = imagesc(ws);
            curr_cost = curr_cost+1;
            title(sprintf('min cost = %d, curr cost = %d',min_cost,curr_cost));
            pause(.01);
        end 
    else
        delete(im); im = imagesc(ws);
        curr_cost = curr_cost+1;
        title(sprintf('min cost = %d, curr cost = %d',min_cost,curr_cost));
        pause(.01);
    end
    
    % Increase the cost to revisit the previous node
    for e = 1:size(edges,1)
        if(edges(e,1)==prev || edges(e,2)==prev)
            G.Edges{e,2} = 1e3;
        end
    end
    
    % End if all nodes have been visited
    if(isempty(find(ws==1)))
        break;
    end
    
    delete(im); im = imagesc(ws);
    pause(.1);
    
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