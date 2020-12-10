function [ref, steer] = LTVReference(totalRefVector, totalSteer, nSignal, nRef, N, Ts, time)
% LTV REFERENCE
%   Function creates sliding horizon reference vectors suitable for MPC

    totalRef = reshape(totalRefVector,[nSignal,nRef]);
    
    currTimeStepNum = round(time/Ts) + 1;
    ref = reshape(totalRef(currTimeStepNum : currTimeStepNum+N-1,:), [N,nRef]);
    steer = totalSteer(currTimeStepNum : currTimeStepNum+N-1);

end

