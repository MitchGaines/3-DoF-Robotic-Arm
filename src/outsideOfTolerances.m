function [ is_out ] = outsideOfTolerances( previous, current)
%outsideOfTolerances Determines if the position has changed within
% a given tolerance
    is_out = 0;
    diff = sum(current) - sum(previous);
    if diff > [10 10] | diff < [10 10] 
       is_out = 1;     
    end

end

