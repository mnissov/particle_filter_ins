function [ xR_orth ] = orthonormalize( xR )
    [~,np] = size(xR);
    R = reshape(xR, 3, 3, np);
    R_orth = zeros(size(R));
    
    for i=1:np
        [U,~,V] = svd(R(:,:,i));
        R_orth(:,:,i) = U*V';
    end
    
    xR_orth = reshape(R_orth, [], np);
end