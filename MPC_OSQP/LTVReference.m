function [ref, steer] = LTVReference(totalRefVector, totalSteer, nSignal, nRef, N, Ts, time)
% LTV REFERENCE
%   Function creates sliding horizon reference vectors suitable for MPC

    totalRef = reshape(totalRefVector,[nSignal,nRef]);
    
    currTimeStepNum = round(time/Ts) + 1;
    refTemp = reshape(totalRef(currTimeStepNum : currTimeStepNum+N-1,:), [N,nRef]);

    ref = refTemp(:,[1,3]);
    steer = totalSteer(currTimeStepNum : currTimeStepNum+N-1);
end

