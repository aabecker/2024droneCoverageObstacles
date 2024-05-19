classdef GridClass < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        Coords
        NN
        NNind
        visited
    end

    methods
        
        function obj = setCoords(obj,value,index)
            %Sets coordinates
            obj.Coords(index,:) = value;
        end
        
        function obj = setNN(obj,value,index)
            %Sets the nearest neighbors coordinates
            obj.NN(index,:) = value;
        end
        function obj = setNNind(obj,value,index)
            %Sets the nearest neighbors indices
            obj.NNind(index,:) = value;
        end
        function obj = setVisited(obj,value,index)
            %Sets the visited flag for a node
            obj.visited(index) = value;
        end
        
        
    end
end