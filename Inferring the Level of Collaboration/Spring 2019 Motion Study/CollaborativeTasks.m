% This script generate data plot of each study subject. Additionally it 
% finds the peaks and valleys of the plot.
%
% ORIENTATION:
% quat2eul of orientation gives [Rot_z, Rot_y, Rot_x]
% Rot_y is negetive when the subject looks to his/her right
% Rot_y is positive when the subject looks to his/her left
% Rot_x is negetive when the subject looks down
% Rot_z is negetive when subject tilts head to right

clear; close all; clc; 
for i = [1,3:6,8:10]
    load(['Actual1/subject',num2str(i),'C.mat'])
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
    
    figure(2)
    hold on
    o = smoothdata(o,1,'SmoothingFactor',0.05);
    plot(1:length(o),o(:,1),'b','Linewidth',1.5);
    plot(1:length(o),o(:,2),'g','Linewidth',1.5);
    plot(1:length(o),o(:,3),'r','Linewidth',1.5);
    legend('Tilt left','Turn left (right)','Look up')
end

save('HumanC.mat','HumanC')