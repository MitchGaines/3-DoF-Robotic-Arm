function [ is_out ] = outsideOfTolerances( previous, current)
%outsideOfTolerances Determines if the position has changed within
% a given tolerance
    tolerance = 5;
    is_out = 0;
    if (current(1, 1) > previous(1, 1)+tolerance || current(1, 1) < previous(1, 1)-tolerance) || ...
            (current(1, 2) > previous(1, 2)+tolerance || current(1, 2) < previous(1, 2)-tolerance)
       is_out = 1;     
    end

end

