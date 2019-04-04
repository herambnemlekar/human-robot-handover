% This script generate data plot of each study subject. Additionally it 
% finds the peaks and valleys of the plot.
clear; close all; clc; 
load('DataPilot1\subject1C'); % Load the subject data. C=Collaborative, NC=Non Collaborative
l = length(human.rw); 
y = zeros(1,l);
for i = 1:l   
    hand = human.rw(i,1:3);
    y(1,i) = hand(2);
    x(1,i) = hand(1);
    z(1,i) = hand(3);
    % hand(1) = Subject data in the X-axis, hand(2) = Subject data in the Y-axis, hand(3) = Subject data in the Z-axis
end
figure(1)
index = 1:l;
% scatter(index,y,20) % plot data points
% hold on
plot(index,z) % plot a 2D graph of the subject data
hold on
plot(index,x)
hold on
plot(index,y)
hold on
grid on 
title('Right Wrist Subject 1 - Collaborative Task')
axis tight
[maxtabz, mintabz] = peakdet(z, 0.06);
[maxtaby, mintaby] = peakdet(y, 0.06); % call the function that find the peaks and valleys.
[maxtab, mintab] = peakdet(x, 0.06);
scatter(mintabz(:,1), mintabz(:,2), 150, 'g*'); % plot the valleys of the subject data.
scatter(maxtabz(:,1), maxtabz(:,2), 150, 'r*');
scatter(mintaby(:,1), mintaby(:,2), 150, 'g*'); % plot the valleys of the subject data.
scatter(maxtaby(:,1), maxtaby(:,2), 150, 'r*');
scatter(mintab(:,1), mintab(:,2), 150, 'g*'); % plot the valleys of the subject data.
scatter(maxtab(:,1), maxtab(:,2), 150, 'r*'); % plot the peaks of the subject data.