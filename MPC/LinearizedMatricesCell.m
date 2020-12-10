function [At,Bt] = LinearizedMatricesCell(statesCell, inputCell, T)
% LINEARIZED MATRICES CELL
%   The function linearizes the model along the sequence of state and 
%   input values.

N = length(inputCell);
At = cell(N,1);
Bt = cell(N,1);
for i=1:N
    [At{i},Bt{i}] = LinearizedMatrices(statesCell{i},inputCell{i}, T);
end
end

