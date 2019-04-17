% Inter-personal distance vs OTP distance
ipd = []; ma =[]; IPD = {}; SH = [];
    for i = 1:subj
        for j = 1:15
            h = Data{i}{j}.h;
            mw = Data{i}{j}.rw;
            al = Data{i}{j}.al;
            ma = [ma; al];
            ipd = [ipd, norm(mean(h(:,1:2:3)))]; 
            IPD{i}{j} = norm(mean(h(:,1:2:3)));
            H{i}{j} = mean(h(:,2));
            wh{i}{j} = min(mw(:,2));
            HP{i}{j} = mean(h(:,1:2:3));
            SH = [SH; Data{i}{j}.sh];
        end
    end

OTP_XYZ = X;
X_1 = []; X_2 = []; Y_1 = []; Y_2 = []; AL = []; X = []; Y = []; X2 = []; Y2 = []; 
O = []; YO = []; HPO = []; WH = [];
cc = linspace(0,1,20);
for i = 1:20
    col = [0, 0, 1];
    for j = 1:15
        if ((IPD{i}{j} - OTPD{i}{j} <= Data{i}{j}.al) && (0.5*Data{i}{j}.al  < IPD{i}{j} - OTPD{i}{j}))
                
                figure(2)    
                hold on
                scatter(OTPD{i}{j}, IPD{i}{j}-OTPD{i}{j}, 36, col, 'filled')
                figure(3)
                hold on
                plot(Data{i}{j}.al, IPD{i}{j}-OTPD{i}{j}, 'bo', 'linewidth', 2)
                   
                AL = [AL; Data{i}{j}.al];
                Y = [Y; IPD{i}{j}-OTPD{i}{j}];
                X = [X; OTPD{i}{j}];
                
                YO = [YO; OTP_XYZ{i}{j}];
                HPO = [HPO; HP{i}{j}];

                if IPD{i}{j} >= 2*Data{i}{j}.al
                    X_1 = [X_1, OTPD{i}{j}];
                    Y_1 = [Y_1, IPD{i}{j}-OTPD{i}{j}];
                else
                    X_2 = [X_2, OTPD{i}{j}];
                    Y_2 = [Y_2, IPD{i}{j}-OTPD{i}{j}];
                end
                
                if ismember(j,[1,2,3,10,11,12])
                    O = [O; -40];
                elseif ismember(j, [4,5,6])
                    O = [O; 0];
                elseif ismember(j, [7,8,9,13,14,15])
                    O = [O; 30];
                end
                
        end
        if ((OTP_XYZ{i}{j}(3) + abs((Data{i}{j}.hh - H{i}{j}))) < Data{i}{j}.hh) && ((OTP_XYZ{i}{j}(3) + (Data{i}{j}.hh - H{i}{j})) > Data{i}{j}.wh)
                figure(4)
                hold on
                plot(Data{i}{j}.hh, OTP_XYZ{i}{j}(3) + abs((Data{i}{j}.hh - H{i}{j})), 'bo', 'linewidth', 2)
                
                figure(5)
                hold on
                plot(Data{i}{j}.wh, OTP_XYZ{i}{j}(3) + (Data{i}{j}.hh - H{i}{j}), 'bo', 'linewidth', 2)
                
                Y2 = [Y2; OTP_XYZ{i}{j}(3) + (Data{i}{j}.hh - H{i}{j})];
                X2 = [X2; Data{i}{j}.hh];
                WH = [WH; Data{i}{j}.wh];
               
        end
    end
end

figure(1)
h_box = boxplot([Y+X, X, AL],'label',{'d_R,H','d_O,H','Arm Length'},'Colors','rgb','Symbol','ko');
mean_ipd = mean(Y+X)
mean_otp_d = mean(X)
mean_ma = mean(AL)
hold on
grid on
set(h_box,'LineWidth',2)
set(h_box,'MarkerSize',4)
ax = gca;
ax.FontSize = 18;
ax.LineWidth = 2;
ax.GridColor = 'k';
ax.GridAlpha = 0.15;
ax.YLim = [0,1.5];
ax.YLabel.String = 'Distance (m)';
ax.YLabel.FontSize = 18;
ax.YMinorGrid = 'on';
ax.FontWeight = 'bold';
print(figure(1),'OTPdistance_box.png','-dpng','-r720');

figure(2)
grid on;
box on;
nX_1 = normlz(X_1, X);
nY_1 = normlz(Y_1, Y);
lm1 = fitlm(nX_1, nY_1)
int1 = lm1.Coefficients{1,1};
slope1 = lm1.Coefficients{2,1};
plot(X_1,denormlz((slope1*nX_1)+int1, Y_1),'r-','linewidth',3);
hold on
nX_2 = normlz(X_2, X);
nY_2 = normlz(Y_2, Y);
lm2 = fitlm(nX_2, nY_2)
int2 = lm2.Coefficients{1,1};
slope2 = lm2.Coefficients{2,1};
plot(X_2,denormlz((slope2*nX_2)+int2,Y_2),'b-','linewidth',3);
axis equal
xlabel('d_R,H (m)','fontsize',18,'Interpreter','none');
ylabel('d_O,H from human (m)','fontsize',18,'Interpreter','none');
ax = gca;
ax.FontSize = 18;
ax.FontWeight = 'bold';
ax.LineWidth = 2;
ax.YMinorGrid = 'on';
ax.YLim = [0.4,1];
ax.XLim = [1.1,1.5];
axis equal
axis([1.05,1.5,0.5,0.9])
print(figure(2),'IPDvsOTPd.png','-dpng','-r720');

figure(3)
grid on;
hold on;
box on;
nAL = normlz(AL, AL);
nY = normlz(Y, Y);
lm3 = fitlm(nAL, nY)
int3 = lm3.Coefficients{1,1};
slope3 = lm3.Coefficients{2,1};
plot(AL,denormlz((slope3*nAL)+int3,Y),'r-','linewidth',3);
hold on
axis equal
legend('OTP data','location','northwest')
xlabel('Arm length (m)','fontsize',18,'Interpreter','none');
ylabel('d_O,H from human (m)','fontsize',18,'Interpreter','none');
ax = gca;
ax.FontSize = 18;
ax.FontWeight = 'bold';
ax.LineWidth = 2;
ax.YMinorGrid = 'on';
ax.YLim = [0.52,0.72];
ax.XLim = [0.575,0.725];
axis equal
axis([0.53,0.73,0.5,0.71])
print(figure(3),'ALvsOTPd.png','-dpng','-r720');
fitlm(AL, Y)

figure(4)
grid on;
hold on;
box on;
nX2 = normlz(X2, X2);
nY2 = normlz(Y2, Y2);
% lm4 = fitlm(nX2, nY2)
% int4 = lm4.Coefficients{1,1};
% slope4 = lm4.Coefficients{2,1};
% plot(X2, denormlz((slope4*nX2)+int4,Y2),'r-','linewidth',3);
hold on
axis equal
legend('OTP data','location','northwest')
xlabel('Eye height, h_E from ground (m)','fontsize',18);
ylabel('h_O from ground (m)','fontsize',18);
ax = gca;
ax.FontSize = 18;
ax.FontWeight = 'bold';
ax.LineWidth = 2;
ax.YMinorGrid = 'on';
axis equal
axis([1.3, 1.8, 0.95,1.45])
print(figure(4),'HHvsOTPy.png','-dpng','-r720');
    
figure(5)
grid on;
hold on;
box on;
nWH = normlz(WH, WH);
nY2 = normlz(Y2, Y2);
% lm5 = fitlm(nWH, nY2);
% int5 = lm5.Coefficients{1,1};
% slope5 = lm5.Coefficients{2,1};
% plot(WH, denormlz((slope5*nWH)+int5,Y2),'r-','linewidth',3);
hold on
axis equal
legend('OTP data','location','northwest')
xlabel('Initial wrist height, h_W from ground (m)','fontsize',18);
ylabel('h_O from ground (m)','fontsize',18);
ax = gca;
ax.FontSize = 18;
ax.FontWeight = 'bold';
ax.LineWidth = 2;
ax.YMinorGrid = 'on';
axis equal
axis([0.45,0.9,1,1.425])
print(figure(5),'WHvsOTPy.png','-dpng','-r720');
fitlm(WH, Y2)