function [] = WriteToFile(fileName, time, Y)
%WRITE TO FILE
%   The function writes data to file prepared for plotting in Latex using 
%   pgfplot package.

% Prepare data
vx = Y(:,1);
vy = Y(:,2);
w = Y(:,3);

% Open the file to write in
fileID = fopen(fileName, 'w');

% Print the text line to file
firstLine = ['time vx vy w \n'];

fprintf(fileID, firstLine);

% Load number of continuous time steps and discrete time steps
lenCont = length(time);

dataFormat = ['%f %f %f %f \n'];

for i=1:lenCont
    fprintf(fileID, dataFormat, time(i), vx(i), vy(i), w(i));   
end

fclose(fileID);
end

