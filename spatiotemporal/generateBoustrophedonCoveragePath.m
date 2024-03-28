function path = generateBoustrophedonCoveragePath(w,h,showPlot)
% generates a simple coverage path over a rectangular region
% RETURNS path, an ordered list of (x,y) coor
% assume startpoint is at (0,0)

if nargin <2
    w = 20;
    h = 10;
    showPlot = true;
end

%grid = zeros(w,h);
path = ones(w*h,2);

goUP = true;
i= 1;
    for c =  1:w
        for r =  1:h
            if goUP
                path(i,:) = [c,r];
            else
                path(i,:) = [c,h-r+1];
            end

            if r == h
                goUP = ~goUP;
            end
            i = i+1;
        end
    end


%%%%%%%%% visualization of the path (optional) %%%%%%%%%%%%%%%%%%%%%%    
if showPlot
f1 = figure(1);   clf(f1);  
plot(path(:,1),path(:,2),'-b.' )
axis equal
hold on
plot(path(1,1),path(1,2),'*g' )
%draw grid
for c = 0:w
    line(0.5+[c,c], 0.5+[0,h], 'color',0.7*[1,1,1])
end
for r = 0:h
    line(0.5+[0,w], 0.5+[r,r], 'color',0.7*[1,1,1])
end
axis tight
legend('path','start','location','northoutside')
end