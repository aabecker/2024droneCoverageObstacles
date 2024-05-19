function [s,t] = GenRandomTree(SubGrid) 
    %This function generates a covering tree starting from a rectangular grid
    %of points.

    s = [];
    t = [];
    
    while(true)    
        if(all(SubGrid.visited))
            %If all nodes have been visited at least once, exit loop
            break
        else
            not_visited = find(SubGrid.visited==false);
            %Initialize random seed
            rng();
            %select at random a point that has not been visited
            point1 = not_visited(randi([1,numel(not_visited)]));

            %select at random a nearest neighbor of this point 
            indPoint2 = randi([1,nnz(SubGrid.NNind(point1,:))]);
            point2 = SubGrid.NNind(point1,indPoint2);

            %Define an edge between the two points
            s = [s,point1];
            t = [t,point2];

            %Change the visited status of the two points to true
            SubGrid.setVisited(true,point1);
            if(~SubGrid.visited(point2))
                SubGrid.setVisited(true,point2);
            end
        end
    end
    T = graph(s,t);
    %Once the while loop is over, we have a forest of subtrees, which need
    %to be connected together. First we collect the connected components of
    %the tree, then we call the ConnectComponents function that finds a
    %random connection.

    CC = conncomp(T);
    [s,t] = ConnectComponents(SubGrid,CC,s,t);

end


function [s,t] = ConnectComponents(SubGrid, CC, s, t)
    %This function connects a forest into a tree, by randomly connecting
    %subtrees

    while(max(CC)>1)
        %Until there is more than one connected component
        maxCC = max(CC);
        
        %Take a random connected component
        C1 = randi([1,maxCC]);
        %Consider the collection of its points
        [Points_In_C1] = find(CC==C1);
        
        findPointFlag = false;
        while(~findPointFlag)
            %Consider a random point in this component
            point1 = Points_In_C1(randi([1,numel(Points_In_C1)]));
            
            %Remove this point from the collection of points of this
            %component
            Points_In_C1 = Points_In_C1(Points_In_C1~=point1);
            
            %Consider the indices of its nearest neighbors
            NNind = SubGrid.NNind(point1,:);
            %Disregard the zeros (a point could be in a corner or along the
            %border of the grid)
            NNind = NNind(NNind~=0);
            
            %Determine the connected components of the nearest neighbors
            NNindCC = CC(NNind);
            
            %Disregard the nearest neighbors that belong to the same
            %connected component
            SuitablePoints = NNind(NNindCC~=C1);
    
            if(numel(SuitablePoints)>0)
                %If there is at least one nearest neighbor that belongs to
                %a different connected component, select one at random from
                %the list and assign it to point 2
                point2 = SuitablePoints(randi([1,numel(SuitablePoints)]));
                
                %Update the graph
                s = [s,point1];
                t = [t,point2];
                
                %Set the point found flag and exit the loop
                findPointFlag = true;
            end
            %Recompute the tree and the connected components
            T = graph(s,t);
            CC = conncomp(T);
            
        end
    
        
    end

end