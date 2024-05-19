set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex')
labelsize = 20;
titlesize = 30;



import CoverageWaypointClass.*
import GridClass.*

%Defines step of the subgrid
SGstep = 2;
halfSGstep = SGstep/2;
qtrSGstep = SGstep/4;

% %Size of the arrow (if drawarrow is used)
% headW = 2.5 * SGstep;
% headL = 10 * qtrSGstep;

x = 0.5:1:29.5;
y = 0.5:1:19.5;
subgridX = 1:SGstep:29;
subgridY = 1:SGstep:19;

%initialize map

figure(1);clf;
axis equal
hold on

%Draws the waypoints on the principal grid
for i = 1:numel(x)
    for j = 1:numel(y)
        plot(x(i),y(j),".b",MarkerSize=8)
    end
end


%Initializes the subgrid class
Subgrid = GridClass();
Subgrid.Coords = zeros(numel(subgridX) * numel(subgridY),2);
Subgrid.NN = zeros(numel(subgridX) * numel(subgridY),8);
Subgrid.NNind = zeros(numel(subgridX) * numel(subgridY),4);
Subgrid.visited = false(numel(subgridX) * numel(subgridY),1);

%Initializes the coverage waypoints class
CoverageWaypoints = CoverageWaypointClass();
CoverageWaypoints.Coords = zeros(4 * numel(subgridX) * numel(subgridY),2);

%Initializes the containers for the edges of the random tree on the subgrid
s = [];
t = [];

for i = 1:numel(subgridX)
    for j = 1:numel(subgridY)
        %Initializes the set of the nearest neighbors of each node of the
        %subgrid; they are at most 4, but can be 3 or 2 if the node is in a
        %corner or along the border; as each NN has 2 coordinates the
        %variable subNNset will have at most 8 components.
        subNNset = zeros(1,8);
        %Index for each node (nodes are saved sequentially)
        CurInd = (i-1)*numel(subgridY)+j;
        %Saves the coordinates of the current node
        Subgrid.Coords(CurInd,:) = [subgridX(i),subgridY(j)];
        
        %Each node of the subgrid gives rise to 4 points on the main grid,
        %that are the vertices of a square centered in the subgrid node;
        %these nodes are saved in the coverage waypoints class
        CoverageWaypoints.Coords(4*CurInd-3:4*CurInd,:) = [subgridX(i) - qtrSGstep, subgridY(j) - qtrSGstep;
                                                          subgridX(i) - qtrSGstep, subgridY(j) + qtrSGstep;
                                                          subgridX(i) + qtrSGstep, subgridY(j) - qtrSGstep;
                                                          subgridX(i) + qtrSGstep, subgridY(j) + qtrSGstep];
        
        %Plots the nodes of the subgrid
        plot(subgridX(i),subgridY(j),".r",MarkerSize=12)
        
        %Populates the nearest neighbors coordinates
        if(i-1>0)
            subNNset(find(ismember(subNNset,0),1):find(ismember(subNNset,0),1)+1) = [i-1,j];
        end
        if(j-1>0)
            subNNset(find(ismember(subNNset,0),1):find(ismember(subNNset,0),1)+1) = [i,j-1];
        end
        if(i+1<numel(subgridX))
            subNNset(find(ismember(subNNset,0),1):find(ismember(subNNset,0),1)+1) = [i+1,j];
        end
        if(j+1<numel(subgridY))
            subNNset(find(ismember(subNNset,0),1):find(ismember(subNNset,0),1)+1) = [i,j+1];
        end
        subNNset = padarray(subNNset,[0,8-numel(subNNset)],'post');
        subNNsetIndices = (subNNset(1:2:end)-1) * numel(subgridY) + subNNset(2:2:end);
        subNNsetIndices(subNNsetIndices<0) = 0;
        
        %Saves the coordinates and the indices of the nearest neighbors;
        Subgrid.setNN(subNNset,CurInd);
        Subgrid.setNNind(subNNsetIndices,CurInd);
        
        %Creates an edge to each nearest neighbor
        s = [s,CurInd * ones(1,numel(subNNsetIndices(subNNsetIndices~=0)))];
        t = [t, subNNsetIndices(subNNsetIndices~=0)];
        
    end
end

%Calls the function that generates a random covering tree on the subgrid

[stree,ttree] = GenRandomTree(Subgrid);

%Creates the tree out of the [s,t] variables, and plots it
Tfull = graph(stree,ttree);
plot(Tfull,"XData",Subgrid.Coords(:,1),"YData",Subgrid.Coords(:,2))


%Rotation matrix and identity matrix
R90 = [0, -1;
       1, 0];
I = [1, 0;
     0, 1];

%Initializes the [s,t] variables for the coverage path across the waypoints
Ss = [];
St = [];

for i = 1:numel(Tfull.Edges)
    %Determines the coordinates of the two nodes of the edge
    P1 = Subgrid.Coords(Tfull.Edges{i,:}(1),:);
    P2 = Subgrid.Coords( Tfull.Edges{i,:}(2),:);
    
    %For each edge of the subgrid, there are two corresponding edges on the
    %principal grid that coast the edge keeping it on their right. Coasting
    %by keeping the tree on the right is obtained by defining the
    %rotation matrix as a rotation of 90 degrees. If in the future it is
    %desired to coast the tree keeping it on the left, it is sufficient to
    %switch the sign of the rotation matrix, which equals to consider a
    %rotation of -90.
    
    %First edge
    sP1 = P1 + qtrSGstep / SGstep * (P2-P1) * (I + R90)';
    sP2 = P2 + qtrSGstep / SGstep * (P2-P1) * (-I + R90)';
    
    %Second edge
    sP3 = P2 + qtrSGstep / SGstep * (P1-P2) * (I + R90)';
    sP4 = P1 + qtrSGstep / SGstep * (P1-P2) * (-I + R90)';
    
    Ss = [Ss, find(~any(CoverageWaypoints.Coords - sP1,2)), find(~any(CoverageWaypoints.Coords - sP3,2))];
    St = [St, find(~any(CoverageWaypoints.Coords - sP2,2)), find(~any(CoverageWaypoints.Coords - sP4,2))];

end


%Considering the edges leaves out many points of the principal grid. To
%account for them, we consider all nodes of the subgrid with a degree of 1,
%2 or 3, and define all the additional edges that close the gaps between
%the edges found in the previous section.

for i = 1:numel(Subgrid.Coords)/2
    %Indices of the nearest neighbors of node i
    NN = [ Tfull.Edges{:,1}( Tfull.Edges{:,1}(:,1)==i,2); Tfull.Edges{:,1}( Tfull.Edges{:,1}(:,2)==i,1)];
    
    P1 = Subgrid.Coords(i,:);
    Pnn = Subgrid.Coords(NN,:);

    if (isscalar(NN))
        %If the node is a terminal, there will be three edges that join the
        %two edges on its sides. These three edges are defined by 4 points
        %which we define in the correct order: sP1->sP2->sP3->sP4
        sP1 = P1 + qtrSGstep / SGstep * (Pnn-P1) * (I - R90)';
        sP2 = P1 + qtrSGstep / SGstep * (Pnn-P1) * (-I - R90)';
        sP3 = P1 + qtrSGstep / SGstep * (Pnn-P1) * (-I + R90)';
        sP4 = P1 + qtrSGstep / SGstep * (Pnn-P1) * (I + R90)';
        
        Ss = [Ss, find(~any(CoverageWaypoints.Coords - sP1,2)), find(~any(CoverageWaypoints.Coords - sP2,2)), find(~any(CoverageWaypoints.Coords - sP3,2))];
        St = [St, find(~any(CoverageWaypoints.Coords - sP2,2)), find(~any(CoverageWaypoints.Coords - sP3,2)), find(~any(CoverageWaypoints.Coords - sP4,2))];


    elseif(numel(NN)== 2)
        %If there are two nearest neighbors, the two edges can form a right
        %angle or are antiparallel, meaning that the dot product of the two
        %edges will be either 0 or ~0
      
        dotprod = (Pnn(1,1)-P1(1))*(Pnn(2,1)-P1(1)) + (Pnn(1,2)-P1(2))*(Pnn(2,2)-P1(2));

        if(dotprod == 0)
            %If right angle:
            %The corner point is defined unambiguously by considering the
            %directions described by the two edges
            sP2 = P1 - qtrSGstep / SGstep * (Pnn(1,:) + Pnn(2,:) - 2 * P1);
            %The other two points are obtained considering that the knee
            %must be coasted in a clockwise fashion: sP1->sP2 and sP2->sP3
            sP1 = P1 + (sP2 - P1) * R90';
            sP3 = P1 + (sP2 - P1) * (-R90)';
            
            Ss = [Ss, find(~any(CoverageWaypoints.Coords - sP1,2)), find(~any(CoverageWaypoints.Coords - sP2,2))];
            St = [St, find(~any(CoverageWaypoints.Coords - sP2,2)), find(~any(CoverageWaypoints.Coords - sP3,2))];

        else
            %If linear:
            %Similarly, we define the four points that give rise to the two
            %additional edges: sP1->sP2 and sP3->sP4
            sP1 = P1 + qtrSGstep / 2 / SGstep * (Pnn(2,:)-Pnn(1,:)) * (-I + R90)';
            sP2 = P1 + qtrSGstep / 2 / SGstep * (Pnn(2,:)-Pnn(1,:)) * (I + R90)';
            sP3 = P1 + qtrSGstep / 2 / SGstep * (Pnn(2,:)-Pnn(1,:)) * (I - R90)';
            sP4 = P1 + qtrSGstep / 2 / SGstep * (Pnn(2,:)-Pnn(1,:)) * (-I - R90)';
           
            Ss = [Ss, find(~any(CoverageWaypoints.Coords - sP1,2)), find(~any(CoverageWaypoints.Coords - sP3,2))];
            St = [St, find(~any(CoverageWaypoints.Coords - sP2,2)), find(~any(CoverageWaypoints.Coords - sP4,2))];

        end
    elseif(numel(NN)==3)
        %In this case only an edge is missing, which is found easily by
        %summing the three edges: two cancel out and the third tells where
        %to place the two points. sP1->sP2
        sP1 = P1 + qtrSGstep / SGstep * (sum(Pnn,1) - 3 * P1) * (-I - R90)';
        sP2 = P1 + qtrSGstep / SGstep * (sum(Pnn,1) - 3 * P1) * (-I + R90)';
        
        Ss = [Ss, find(~any(CoverageWaypoints.Coords - sP1,2))];
        St = [St, find(~any(CoverageWaypoints.Coords - sP2,2))];
    end
    
end

%Defines the covering graph (it is actually a path)
G = digraph(Ss,St);
plot(G,XData=CoverageWaypoints.Coords(:,1),YData=CoverageWaypoints.Coords(:,2))

%Sets axis labels and title
xlabel("$x$ [m]", FontSize=labelsize)
ylabel("$y$ [m]", FontSize=labelsize)
title("Coverage using depth first search trees", FontSize=titlesize)

%Choose a random point on the map
Next = randi([1,size(CoverageWaypoints.Coords,1)]);

for i = 1:numel(G.Edges)
    %Starts covering the map following the path
    coords = CoverageWaypoints.Coords(Next,:) - 0.9 * [qtrSGstep,qtrSGstep];
    rectangle('Position', [coords,0.9 * halfSGstep, 0.9 * halfSGstep], "FaceColor",'g',FaceAlpha=0.3)
    CoverageWaypoints.covered(Next) = true;

    Next = G.Edges{:,1}( G.Edges{:,1}(:,1)==Next,2);
    pause(0.05)
end
