function [RHat] = DiagonalizeMatrix(R, Nc)
%DIAGONALIZE MATRIX
%   Function creates extended weight matrix using matrix cell array R.
%   If R is a single matrix, the result is the same as with cell array
%   Rcell = {R, R, R, R, R, ...} containing Nc matrices.

    n = size(R,1);
    m = size(R,2);
    p = size(R,3);
    RHat = zeros(n*Nc, m*Nc);
    if(p == 1)
        step = 0;
        for i=1:m:m*Nc
            for j=1:n
                RHat(step*n+j,i:i+m-1) = R(j,:);
            end
            step = step+1;
        end
    else
        step = 0;
        for i=1:m:m*Nc
            for j=1:n
                RHat(step*n+j,i:i+m-1) = R(j,:,step+1);
            end
            step = step+1;
        end
    end
end

