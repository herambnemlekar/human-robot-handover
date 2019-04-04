% This script generate data plot of each study subject. Additionally it 
% finds the peaks and valleys of the plot.
clear; close all; clc; 
load('DataPilot1\subject1C'); % Load the subject data. C=Collaborative, NC=Non Collaborative
x = length(human.rw); 
y = zeros(1,x);
for i = 1:x   
    hand = human.rw(i,1:3);
    y(1,i) = hand(2); % hand(1) = Subject data in the X-axis, hand(2) = Subject data in the Y-axis, hand(3) = Subject data in the Z-axis
end
figure(1)
index = 1:x;
% scatter(index,y,20) % plot data points
% hold on
plot(index,y) % plot a 2D graph of the subject data
hold on
title('Right Wrist Subject 1 - Collaborative Task')
axis tight
[maxtab, mintab] = peakdet(y, 0.01); % call the function that find the peaks and valleys.
scatter(mintab(:,1), mintab(:,2), 150, 'g*'); % plot the valleys of the subject data.
scatter(maxtab(:,1), maxtab(:,2), 150, 'r*'); % plot the peaks of the subject data.