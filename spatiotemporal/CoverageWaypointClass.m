classdef CoverageWaypointClass < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        Coords
        covered
        safe
    end

    methods
        function obj = setCoords(obj, value, index)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.Coords(index,:) = value;
        end

        function obj = setCovered(obj, value, index)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.covered(index) = value;
        end

        function obj = setSafe(obj, value, index)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.safe(index) = value;
        end
    end
end