function [ out ] = quatnorm( q )
    c = sqrt(sum(q.^2,1));
    out = q./c;
end