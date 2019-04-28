
% This script generate data plot of each study subject. Additionally it 
% finds the peaks and valleys of the plot.
clear; close all; clc; 

load('Actual1/subject1C'); % Load the subject data. C=Collaborative, NC=Non Collaborative
len = length(human.rw);
x = zeros(1,len);
y = zeros(1,len);
z = zeros(1,len);
for i = 1:len   
    hand = human.rw(i,1:3);
    y(1,i) = hand(3);
    x(1,i) = hand(1);
    z(1,i) = hand(2);

    % hand(1) = Subject data in the X-axis, hand(2) = Subject data in the Y-axis, hand(3) = Subject data in the Z-axis
end

len1 = length(human.o);
o = zeros(len1,3);
for i = 1:len1   
    o(i,:) = quat2eul(human.o(i,:));
end

% Distance
dist = smoothdata(rms([x',y',z'],2));

% Velocity
vel = smoothdata(diff(dist));
normvel = normalize(vel);

%--------------------------------------------------------------------------
% Distance calculation commented for now, delete at the end if not required
% dist = zeros(1,len-1);
% for i=1:1:len-1
%     x_ed = single((x(1,i+1)- x(1,i))^(2));
%     y_ed = single((y(1,i+1)- y(1,i))^(2));
%     z_ed = single((z(1,i+1)- z(1,i))^(2));
%     dist(i) = (sqrt (x_ed + y_ed + z_ed));
%     
% %     dist(i) =sqrt((single(x1(1,i+1)- x1(1,i))^(2))+ (single(y1(1,i+1)- y1(1,i))^(2))+(single(z1(1,i+1)- z1(1,i))^(2)));
% end
%--------------------------------------------------------------------------

figure(1) %EUCLIDIAN DISTANCE PLOT
index = 1:len;
index1 = 1:(len-1);
plot(index,dist)
hold on
%findpeaks(dist,'MinPeakDistance',1300,'Threshold',0.09,'MinPeakProminence',0.05)
plot(index1,normvel) 
legend("Distance","Velocity");
title('Distance and Velocity')
grid on

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

figure(2) % (x,y,z) PLOT 
x_sm = smoothdata(x,'SmoothingFactor',0.05);
y_sm = smoothdata(y,'SmoothingFactor',0.05);
z_sm = smoothdata(z,'SmoothingFactor',0.05);
plot(index,x_sm,'m');
hold on;
plot(index,y_sm,'b');
hold on;
plot(index,z_sm,'k'); % plot a 2D graph of the subject data
hold on;
plot(index,dist)
hold on
grid on ;
title('Right Wrist Subject 1 - Collaborative Task');
axis tight;
legend('X','Y','Z','Distance');
% [maxtabz, mintabz] = peakdet(z, 0.2);
% [maxtaby, mintaby] = peakdet(y, 0.1); % call the function that find the peaks and valleys.
% [maxtab, mintab] = peakdet(x, 0.1);
% scatter(mintabz(:,1), mintabz(:,2), 150, 'g*'); % plot the valleys of the subject data.
% scatter(maxtabz(:,1), maxtabz(:,2), 150, 'r*');
% scatter(mintaby(:,1), mintaby(:,2), 150, 'g*'); % plot the valleys of the subject data.
% scatter(maxtaby(:,1), maxtaby(:,2), 150, 'r*');
% scatter(mintab(:,1), mintab(:,2), 150, 'g*'); % plot the valleys of the subject data.
% scatter(maxtab(:,1), maxtab(:,2), 150, 'r*'); % plot the peaks of the subject data.

figure(3) %ORIENTATION PLOT
index_o=1:len1;
Euler_x = smoothdata(o(:,1),'SmoothingFactor',0.1);
Euler_y = smoothdata(o(:,2),'SmoothingFactor',0.1);
Euler_z = smoothdata(o(:,3),'SmoothingFactor',0.1);
plot(index_o,Euler_x);
hold on
plot(index_o,Euler_y);
hold on
plot(index_o,Euler_z);
hold on
title("Orientation Plot");
legend("x","y","z");
grid on

