function [] = WriteToFile(fileName, time, Y, Y_ref)
%WRITE TO FILE
%   The function writes data to file prepared for plotting in Latex using 
%   pgfplot package.

% Prepare data
vx = Y(:,1);
vy = Y(:,2);
w = Y(:,3);

vxRef = Y_ref(:,1);
vyRef = Y_ref(:,2);
wRef = Y_ref(:,3);

% Open the file to write in
fileID = fopen(fileName, 'w');

% Print the text line to file
firstLine = ['time vxRef wRef vx w \n'];

fprintf(fileID, firstLine);

% Load number of continuous time steps and discrete time steps
lenCont = length(time);

dataFormat = ['%f %f %f %f %f\n'];

for i=1:lenCont
    fprintf(fileID, dataFormat, time(i), 3.6*vxRef(i), 180/pi*wRef(i), 3.6*vx(i), 180/pi*w(i));   
end

fclose(fileID);
end

