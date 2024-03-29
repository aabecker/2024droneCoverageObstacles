function path =  rrtCoverage(map, startPt)

showPlot = true;
if nargin<1
    map = zeros(10,20);


    map = [ ...
        0,0,0,0,0,1,0,0,0,0,0;
        0,0,0,0,0,1,0,0,0,0,0;
        0,0,0,0,0,1,1,1,1,0,0;
        0,0,0,0,0,1,0,0,0,0,0;
        0,0,0,0,0,0,0,0,0,0,0;
        0,0,0,0,0,1,0,0,0,0,0;
        0,0,0,0,0,1,0,0,0,0,0;
        0,0,0,0,0,1,1,1,1,0,0;
        0,0,0,0,0,0,0,0,0,0,0;
        0,0,0,0,0,1,0,0,0,0,0;
        0,0,0,0,0,1,0,0,0,0,0;
        1,1,0,0,0,1,0,0,0,0,0;
        0,0,0,0,0,0,0,0,0,0,0;
        0,0,0,0,0,1,0,0,0,0,0;
        0,0,0,0,0,1,0,0,0,0,0;];
    startPt = [1,1];
end



%step 1: generate a RRT to cover the entire grid
sz = size(map);
Ctiles = zeros(sz);
Ctiles(startPt(1),startPt(2)) = 1; % place the initial tile

n = sum(map(:)==0);


if showPlot
    figure(2); clf
    imagesc(map + 2*Ctiles)
end

%tree architecture:  each node has a list of children (never more than 4)
% children are positive indices  (-1 means not a child)
%
% [row,col] = ind2sub(sz,ind)
tree = -1*ones(numel(map),4);





% Does an RRT
while sum(Ctiles ==  1,'all') < n

    while 1
        target = randi(numel(Ctiles));  % generate a random configuration
        if map(target) == 0 && Ctiles(target) == 0 % if target is a free configuration space that has not been reached
            break;
        end
    end
    [tr,tc] = ind2sub(size(map),target); %target r,c, position
    [fr,fc] = find(Ctiles==1);  % row, column data for all tiles

    [~,minInd] = min( abs(fr-tr) + abs(fc-tc) ); % find the nearest free configuration
    rErr = tr-fr(minInd);
    cErr = tc-fc(minInd);
    if abs(rErr) > abs(cErr)  %move one step toward the target from the nearest free config
        oneStep = [fr(minInd)+sign(rErr),fc(minInd)];
    else
        oneStep = [fr(minInd),fc(minInd)+sign(cErr)];
    end
    if map(oneStep(1),oneStep(2)) == 0
        Ctiles(oneStep(1),oneStep(2)) = 1;  %mark this tile as reached
        

        % save the tree structure
        childNum = childInd([ oneStep(2)-fc(minInd), oneStep(1)- fr(minInd)  ] );
        parentInd = sub2ind(size(map), [fc(minInd),fr(minInd)]);
        tree(  parentInd, childNum   )   = sub2ind(sz,oneStep(1),oneStep(2)); 

        if showPlot
            line([fc(minInd),oneStep(2)],[fr(minInd), oneStep(1)],'LineStyle','-','Marker','o','Color','r','linewidth',2 )
            hold on
        end
    end
%         imshow(~map, 'InitialMagnification', 2000)
%      title(sprintf('A %dx%d with %d freespace',w,h,sum(map ==  0,'all')))
%      drawnow
end

if showPlot
    %figure(2)
    %imagesc(map + 2*Ctiles)
    title(sprintf('A %dx%d with %d freespace and %d-tile config',size(map,1),size(map,2),sum(map ==  0,'all'),sum(Ctiles ==  1,'all')))
    drawnow
    axis equal tight

    %draw grid
    [h,w] = size(map);
    for c = 0:0.5:w
        line(0.5+[c,c], 0.5+[0,h], 'color',0.7*[1,1,1])
    end
    for r = 0:0.5:h
        line(0.5+[0,w], 0.5+[r,r], 'color',0.7*[1,1,1])
    end

end
drawTree(tree,map)


%step 2: double the resolution of the grid-graph



% step 3: starting at the  startPt, right-wall follow the RRT



% step 4: return the path

function childNum = childInd(oneStep)
    if all(oneStep == [-1,0])
            childNum = 1; %left
    elseif all(oneStep == [0,1])
            childNum = 2; %up
    elseif all(oneStep == [1,0])
            childNum = 3; %right
    elseif all(oneStep == [0,-1])
            childNum = 4; %down
    else
            disp('mistake in child onestep')
    end

function drawTree(tree,map)

    figure(3); clf
    for i = 1:size(tree,1)
        [PtSx,PtSy]  = ind2sub(size(map), i);

        for j = 1:4
            if tree(i,j) ~= -1
                
                [PtEx,PtEy]  =  ind2sub(size(map), tree(i,j));

                [PtSx, PtSy]
                [PtEx, PtEy]
                line( [PtSx, PtEx],[PtSy, PtEy]  )
            end
        end
    end

