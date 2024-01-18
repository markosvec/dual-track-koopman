function [] = WriteToFile(fileName, reference, states, slipAngle, input, sampleStep, endTime)
%WRITE TO FILE
%   The function writes data to file prepared for plotting in Latex using 
%   pgfplot package.

% Prepare data
if(endTime)
    endIdx = find(states.time > endTime);
else
    endIdx = length(states.time);
end

time = states.time(1:sampleStep:endIdx);
ref_vx = 3.6*reference.signals.values(1:sampleStep:endIdx,1);
ref_w = 180/pi*reference.signals.values(1:sampleStep:endIdx,3);
vx = 3.6*states.signals.values(1:sampleStep:endIdx,1);
w = 180/pi*states.signals.values(1:sampleStep:endIdx,3);

steer = 180/pi*input.signals.values(1:sampleStep:endIdx,1);
sfl = 100*input.signals.values(1:sampleStep:endIdx,2);
sfr = 100*input.signals.values(1:sampleStep:endIdx,3);
srl = 100*input.signals.values(1:sampleStep:endIdx,4);
srr = 100*input.signals.values(1:sampleStep:endIdx,5);

alpha_fl = 180/pi*slipAngle.signals.values(1:sampleStep:endIdx,1);
alpha_fr = 180/pi*slipAngle.signals.values(1:sampleStep:endIdx,2);
alpha_rl = 180/pi*slipAngle.signals.values(1:sampleStep:endIdx,3);
alpha_rr = 180/pi*slipAngle.signals.values(1:sampleStep:endIdx,4);

% Open the file to write in
fileID = fopen(fileName, 'w');

% Print the text line to file
firstLine = ['time vxRef wRef vx w steer sfl sfr srl srr alphaFL alphaFR alphaRL alphaRR\n'];

fprintf(fileID, firstLine);

% Load number of continuous time steps and discrete time steps
lenCont = length(time);

dataFormat = ['%f %f %f %f %f %f %f %f %f %f %f %f %f %f\n'];

for i=1:lenCont
    fprintf(fileID, dataFormat, time(i), ref_vx(i), ref_w(i), vx(i), w(i), ...
                steer(i), sfl(i), sfr(i), srl(i), srr(i), ...
                alpha_fl(i), alpha_fr(i), alpha_rl(i), alpha_rr(i));   
end

fclose(fileID);
end