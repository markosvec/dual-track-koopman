function [F,PHI, THETA] = PredictionMatrices(A, B, C, Np)

    % Calculation of prediction matrices of equation
    % Y = F * X  +  PHI * dU + THETA * d;

    % dU = [dU1(k+1), dU2(k+1), dU1(k+2), dU2(k+2) ..... dU1(Np), dU2(Np)]'
    % Y  = [ Y1(k+1),  Y2(k+1),  Y1(k+2),  Y2(k+2) .....  Y1(Np),  Y2(Np)]'
    % X  = [X1(k) X2(k) X3(k) ... Xn(k)]'
    % d  = [d(k) d(k+1) d(k+2) ... d(Np)]'

    % Determine number of inputs outputs and states
    [n_states, n_inputs] = size(B);
    n_outputs = size(C,1);
    
    %% %%%%%% F matrix %%%%%%% %%
    % Determine size of matrix F
    % n_f, m_f -  number of rows and columns of matrix F
    n_f = n_outputs * Np;
    m_f = n_states;

    % Initalize matrix F
    F = zeros(n_f, m_f);

    % Fill matrix F one prediction step at the time
    % Use previous results in variable F_aux
    F_aux = A;
    temp1 = C*F_aux;
    
    for i = 1 : n_outputs
        F(i, :) = temp1(i,:);
    end

    for i = 1 : (Np - 1)
        %Define first and last row
        f_row = (i * n_outputs) + 1;
        l_row = f_row + n_outputs - 1;

        %Calculate matirx based on previous ca
        F_aux = A*F_aux;
        temp2 = C*F_aux;
        for j = f_row : n_outputs : l_row
            for k = 1 : n_outputs
                F(j+k-1, :) = temp2(k,:);
            end
        end
    end


    %% %%%%%% PHI matrix %%%%%%% %%
    % INIT %
    % Determine size of matrix PHI
    % n_p, m_p -  number of rows and columns of matrix Phi
    n_phi = n_outputs * Np;
    m_phi = n_inputs  * Np;

    % Initalize matrix Phi
    PHI = zeros(n_phi, m_phi);

    % CALCULATE 1 %
    % Caculate first n_inputs columns from matrix F
    % CB, CAB, CA^2B ... 

    for i = 1 : Np
            f_PHI_row = (i - 1) * n_outputs + 1;    % First row of matrix PHI
            l_PHI_row = i * n_outputs;  % Last row of matrix PHI
            f_PHI_col = (i - 1) * n_inputs + 1; % First column of matrix PHI
            l_PHI_col = i * n_inputs; % Last column of matrix PHI
%             PHI(f_PHI_row : l_PHI_row, f_PHI_col : l_PHI_col) = C * B(:,:,i);
            temp3 = C * B;
            for k = f_PHI_row : n_outputs : l_PHI_row
                for x = 1 : n_outputs
                    PHI(k+x-1, f_PHI_col : l_PHI_col) = temp3(x,:);    
                end                
            end

            for j = 1 : Np - i

                % Define rows for iteration step of calculation of PHI 
                f_PHI_row = l_PHI_row + 1;    % First row of matrix PHI
                l_PHI_row = f_PHI_row + n_outputs - 1;  % Last row of matrix PHI

                % Calculate predefined rows
                M = MultiplyStateMatrices(A,i+1,i+j);
%                 PHI(f_PHI_row : l_PHI_row, f_PHI_col : l_PHI_col) = C* M * B(:,:,i);
                temp4 = C* M * B;
                for k = f_PHI_row : n_outputs : l_PHI_row
                    for x = 1: n_outputs
                        PHI(k+x-1, f_PHI_col : l_PHI_col) = temp4(x,:);
                    end
                end
            end
    end

    %% %%%%%% THETA matrix %%%%%%% %%
    % INIT %
    % Determine size of matrix THETA
    % n_p, m_p -  number of rows and columns of matrix Theta
    n_theta = n_outputs * Np;
    m_theta = n_states * Np;

    % Initalize matrix Theta
    THETA = zeros(n_theta, m_theta);

    for i = 1 : Np
            f_THETA_row = (i - 1) * n_outputs + 1;    % First row of matrix THETA
            l_THETA_row = i * n_outputs;  % Last row of matrix THETA
            f_THETA_col = (i - 1) * n_states + 1; % First column of matrix THETA
            l_THETA_col = i * n_states; % Last column of matrix THETA
%             THETA(f_THETA_row : l_THETA_row, f_THETA_col : l_THETA_col) = C;
            for j = f_THETA_row : n_outputs : l_THETA_row
                for k = 1 : n_outputs
                    THETA(j+k-1, f_THETA_col : l_THETA_col) = C(k,:);
                end
            end
                
            for j = 1 : Np - i

                % Define rows for iteration step of calculation of THETA 
                f_THETA_row = l_THETA_row + 1;    % First row of matrix THETA
                l_THETA_row = f_THETA_row + n_outputs - 1;  % Last row of matrix THETA

                % Calculate predefined rows
                M = MultiplyStateMatrices(A,i+1,i+j);
%                 THETA(f_THETA_row : l_THETA_row, f_THETA_col : l_THETA_col) = C*M;
                temp5 = C*M;
                for k = f_THETA_row : n_outputs : l_THETA_row
                    for x = 1:n_outputs
                        THETA(k+x-1, f_THETA_col : l_THETA_col) = temp5(x,:);
                    end
                end
            end
    end
    
end

function [M] = MultiplyStateMatrices(A, i, n)
%MULTIPLY STATE MATRICES 
% Functions calculates matrix product M = A(n) * A(n-1) * ... A(i-1) * A(i)  

    M = eye(size(A));
    for j=i:n
        M = A*M;
    end

end
