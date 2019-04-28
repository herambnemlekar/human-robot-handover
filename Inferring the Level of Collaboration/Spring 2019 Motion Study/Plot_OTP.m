 load('Human.mat')
 for i = 1:length(Human)
     otp = Human{i}.otp;
     hold on
     scatter3(otp(:,1),otp(:,2),otp(:,3),30,'b', 'filled')
     grid on
 end