% This script generate data plot of each study subject. Additionally it 
% finds the peaks and valleys of the plot.
clear; close all; clc; 
for i = [1,3:6,8:10];
    load(['Actual1/subject',num2str(i),'C.mat'])
    Human{i} = human;
    len = length(human.rw);
    right = human.rw(:,[1,3,2]);
    left = human.lw(:,[1,3,2]);
    o = quat2eul(human.o);
    
    Right = smoothdata(right,1,'SmoothingFactor',0.05);
    Left = smoothdata(left,1,'SmoothingFactor',0.05);
 
    fig = figure('Name','1','units','normalized','outerposition',[0 0 1 1]);
    hold on;
    grid on;
    plot(1:len,Right(:,1),'b','Linewidth',1.5);
    plot(1:len,Right(:,2),'g','Linewidth',1.5);
    plot(1:len,Right(:,3),'r','Linewidth',1.5);
    plot(1:len,Left(:,1),'b--','Linewidth',1.5);
    plot(1:len,Left(:,2),'g--','Linewidth',1.5);
    plot(1:len,Left(:,3),'r--','Linewidth',1.5)
    axis tight
    %print(fig, ['Plots/rw',num2str(i),'.png'],'-dpng','-r720');
    legend('X','Y','Z')
    title(['rw',num2str(i)])
    hold off; 
 
    [otp_ind, ~] = ginput(12);
    otp_ind = round(otp_ind);
    
    HumanC{i}.otp = human.rw(otp_ind,:);
end

save('HumanC.mat','HumanC')