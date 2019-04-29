% This script generate data plot of each study subject. Additionally it 
% finds the peaks and valleys of the plot.
clear; close all; clc; 
for i = [1,3:6,8:10]
    load(['Actual1/subject',num2str(i),'NC.mat'])
    Human{i} = human;
    len = length(human.rw);
    right = human.rw(:,[1,3,2]);
    left = human.lw(:,[1,3,2]);
    right(:,1) = -1.*right(:,1); left(:,1) = -1.*left(:,1);
    o = rad2deg(quat2eul(human.o));
    
    Right = smoothdata(right,1,'SmoothingFactor',0.05);
    Left = smoothdata(left,1,'SmoothingFactor',0.05);
 
    fig = figure('Name','1','units','normalized','outerposition',[0 0 1 1]);
    hold on;
    grid on;
    plot(1:length(Right(:,1)),Right(:,1),'b','Linewidth',1.5);
    plot(1:length(Right(:,2)),Right(:,2),'g','Linewidth',1.5);
    plot(1:length(Right(:,3)),Right(:,3),'r','Linewidth',1.5);
    plot(1:length(Left(:,1)),Left(:,1),'b--','Linewidth',1.5);
    plot(1:length(Left(:,2)),Left(:,2),'g--','Linewidth',1.5);
    plot(1:length(Left(:,3)),Left(:,3),'r--','Linewidth',1.5)
    axis tight
    %print(fig, ['Plots/rw',num2str(i),'.png'],'-dpng','-r720');
    legend('X','Y','Z')
    title(['rw',num2str(i)])
    hold off; 
 
    [otp_ind, ~] = ginput(12);
    otp_ind = round(otp_ind);
    
    %Human{i}.otp = human.rw(otp_ind,:)
    Humanl{i}.otp = human.lw(otp_ind,:)
    
    figure(2)
    o = smoothdata(o,1,'SmoothingFactor',0.05);
    plot(1:length(o),o(:,1),'b','Linewidth',1.5);
    hold on
    plot(1:length(o),o(:,2),'g','Linewidth',1.5);
    plot(1:length(o),o(:,3),'r','Linewidth',1.5);
    legend('Tilt left','Turn left (right)','Look up')
    hold off
    
end

save('HumanNC_left.mat','Humanl')
%save('HumanNC.mat','Human')