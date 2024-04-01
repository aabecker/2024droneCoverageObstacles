function collisionSet = movingStraightCircularObstacle(ptStart, velocity, radius, w, h,steps)
% simulates an obstacle that is moving straight.
% Returns the collision set with a grid of width w and height h with left
% bottom at (0,0).
% The collision set is all grid cells that collide with the workspace at a
% given time

% at each time step, the collision set is the shape swept out by a circle
% from [ptStart+ (t-1)*velocity] to  [ptStart+ (t)*velocity], with radius
% radius.

% does the path intersect the grid?
if nargin < 1
    ptStart = [-20,10];
    velocity = [1/2,0];
    radius = 8;
    w = 40; %x width
    h = 20; %y height
    steps = 200;  % time
end

path = generateBoustrophedonCoveragePath(w,h,false);

collisionSet = zeros(w,h,steps);

f2 = figure(2);clf
set(f2,'name', 'Collision set and path')

for i = 1:steps
    v1 = ptStart + velocity*i;
    v2 = ptStart + velocity*(i+1);
    
  
    
    for r = 1:h   %y value
        for c = 1:w   %x value
            pt = [r,c];
            if point_to_line(pt, v1, v2) < radius
                collisionSet(r,c,i) = 1;
                
            end
            
        end
        
    end
    
end

numCollisions = sum(collisionSet(:));
f2 = figure(2);clf
hold on
axis equal
set(f2,'name', 'Collision set and path')
covered = zeros(w,h);
currentcovered = zeros(w,h);
colormap hot

counter = 1;
for i = 1:steps-1
    Obs = double(logical(collisionSet(:,:,i)+collisionSet(:,:,i+1)));
    if(Obs(path(counter+1,1),path(counter+1,2))==1)
        if(Obs(path(counter,1),path(counter,2))==1)
            counter = counter-1;
        end
    elseif(Obs(path(counter,1),path(counter,2))==1)
        counter = counter-1;
    else
        counter = counter + 1;
    end

    currentcovered = zeros(w,h);
    currentcovered(path(counter,1),path(counter,2)) = 100;
    covered = covered + currentcovered/100*20;

%      hBoustro = imagesc(covered);
%      hBoustroCurrent = imagesc(currentcovered);
     hSurf1 = imagesc(50*Obs + covered + currentcovered);
    
     %uistack(hBoustroCurrent,"top")

    refresh
    drawnow
    pause(0.2)
%     filename = strcat("Step",num2str(i),".png")
%     exportgraphics(gca,filename)
    delete(hSurf1)
    %delete(hBoustroCurrent)
    
    
end

% need an iterative deepening backup planner:
% startWithArray 'waitTimes' as long as the
% Step through the path.  If there is a collision, go back in time 1 step
% before the collision, and wait for one turn at that point.  


 


end

   function dist = point_to_line(pt, v1, v2)
        % calculates the distance between a dot and a line
        % point is at pt, line is from v1 to v2
        %https://www.mathworks.com/matlabcentral/fileexchange/97462-distance-between-point-and-line-segments

        a = v1 - v2;
        b = pt - v2;
        line_vec = a ;%vector(start, end) # (3.5, 0, -1.5)
        pnt_vec = b ;%vector(start, pnt)  # (1, 0, -1.5)
        line_len = sqrt(sum(line_vec.^2)); % # 3.808
        line_unitvec = line_vec/line_len; % # (0.919, 0.0, -0.394)
        pnt_vec_scaled = pnt_vec/line_len; %  # (0.263, 0.0, -0.393)
        t = dot(line_unitvec, pnt_vec_scaled); % # 0.397
        if t < 0.0
            t = 0.0;
        elseif t > 1.0
            t = 1.0;
        end
        nearest = line_vec* t; %    # (1.388, 0.0, -0.595)
        dist = sqrt(sum((nearest-pnt_vec).^2));% # 0.985
        nearest = nearest+v2;
    end


%     function newPath = firstCollision(path, ptStart, velocity, radius)
% 
%         for i = 1:numel(path(:,1))
%             v1 = ptStart + velocity*i;
%             v2 = ptStart + velocity*(i+1);
%             if point_to_line(path(i,:), v1, v2) < radius 
%                 %move backwards and see if waiting is ok
%                 while true
%                     if point_to_line(path(i-1,:), v1, v2) < radius 
% 
% 
% 
%         end