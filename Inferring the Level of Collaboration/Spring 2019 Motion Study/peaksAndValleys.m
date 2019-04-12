
% This script generate data plot of each study subject. Additionally it 
% finds the peaks and valleys of the plot.
clear; close all; clc; 
load('DataPilot1\subject1C'); % Load the subject data. C=Collaborative, NC=Non Collaborative
len = length(human.rw);
x = zeros(1,len);
y = zeros(1,len);
z = zeros(1,len);
for i = 1:len   
    hand = human.rw(i,1:3);
    y(1,i) = hand(2);
    x(1,i) = hand(1);
    z(1,i) = hand(3);

    % hand(1) = Subject data in the X-axis, hand(2) = Subject data in the Y-axis, hand(3) = Subject data in the Z-axis
end
dist = zeros(1,len-1);
for i=1:1:len-1
    x_ed = single((x(1,i+1)- x(1,i))^(2));
    y_ed = single((y(1,i+1)- y(1,i))^(2));
    z_ed = single((z(1,i+1)- z(1,i))^(2));
    dist(i) = (sqrt (x_ed + y_ed + z_ed));
    
%     dist(i) =sqrt((single(x1(1,i+1)- x1(1,i))^(2))+ (single(y1(1,i+1)- y1(1,i))^(2))+(single(z1(1,i+1)- z1(1,i))^(2)));
end
    
figure(1) %EUCLIDIAN DISTANCE PLOT 
index1 = 1:(len-1);

plot(index1,dist);
 hold on;
findpeaks(dist,'MinPeakDistance',1300,'Threshold',0.09,'MinPeakProminence',0.05)
% [maxtabdist, mintabdist] = peakdet(dist, 0.25);
% scatter(maxtabdist(:,1), maxtabdist(:,2), 150, 'r*');
% m(index1)=-0.5
% for i=1:len-1
% if ((single(dist(i)))~=0)
%     %disp(dist(i));
%     m(i)=dist(i+1);
%     
% end
% end
% disp(m);
% plot(index1,m)
figure(2) 
index = 1:len;
hold on
plot(index,x,'m');
hold on;
plot(index,y,'b');
hold on;
plot(index,z,'k'); % plot a 2D graph of the subject data
hold on;
grid on ;
title('Right Wrist Subject 1 - Collaborative Task');
axis tight;
[maxtabz, mintabz] = peakdet(z, 0.2);
[maxtaby, mintaby] = peakdet(y, 0.1); % call the function that find the peaks and valleys.
[maxtab, mintab] = peakdet(x, 0.1);
scatter(mintabz(:,1), mintabz(:,2), 150, 'g*'); % plot the valleys of the subject data.
scatter(maxtabz(:,1), maxtabz(:,2), 150, 'r*');
scatter(mintaby(:,1), mintaby(:,2), 150, 'g*'); % plot the valleys of the subject data.
scatter(maxtaby(:,1), maxtaby(:,2), 150, 'r*');
scatter(mintab(:,1), mintab(:,2), 150, 'g*'); % plot the valleys of the subject data.
scatter(maxtab(:,1), maxtab(:,2), 150, 'r*'); % plot the peaks of the subject data.

% for i=1:18
% xline(maxtab(i,1));
% end
% tolerance = 500;
% tolerance2 = 1000; 
% for i = 1:length(maxtab)
%     for j= 1:length(maxtaby)
%         for k=1:length(mintabz)
%     if (abs(maxtab(i,1)-maxtaby(j,1))< tolerance)&& ((abs(maxtaby(j,1)-mintabz(k,1))< tolerance2)||(abs(maxtab(i,1)-mintabz(k,1))< tolerance2))
%         disp('Peak x:')
%         disp(maxtab(i,1));
%         xline((maxtab(i,1)+maxtaby(j,1))/2);
%         disp('Peak y:')
%         disp(maxtaby(j,1))
%         disp('Peak z:')
%         disp(mintabz(k,1))
%         if (abs(maxtab(i,1)-maxtaby(j,1))< tolerance)
%             xline((maxtab(i,1)+maxtaby(j,1))/2);
%     else if (abs(maxtabz(k,1)-maxtaby(j,1))< tolerance)&& ((abs(mintabz(k,1)-maxtab(i,1))< tolerance2)||(abs(maxtab(i,1)-maxtaby(j,1))< tolerance2))
%         xline((mintabz(k,1)+maxtaby(j,1))/2);
%         end
%    end
%        end  
 %   end
%end
