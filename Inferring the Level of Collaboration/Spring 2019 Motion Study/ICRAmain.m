clear all; close all;

% Load subject data and save in a cell
subj = 20; % number of subjects
for i = 1:subj
    load(['human',num2str(i),'.mat'])
    Data{i} = D;
end

% Find object transfer point
X = {}; OTPD = {}; % initialise variables
for i = 1:20
    for j = 1:15
        rw = Data{i}{j}.rw; % right wrist position
        h = Data{i}{j}.h; % head position
        [~, ind2] = min(vecnorm(rw(:,1:2:3)')); % right wrist point nearest to robot in x-z plane
        [~, ind3] = max(rw(:,2));   % right wrist point of max height along y
        
        % OTP chosen as mean of nearest in xz and highest in y
        % NOTE: For OTP, x-y-z of kinect re-arranged to x-z-y in task space
        X{i}{j} = [(rw(ind2,1)+rw(ind3,1))/2, (rw(ind2,3)+rw(ind3,3))/2, (rw(ind2,2)+rw(ind3,2))/2];
        
        % OTP distance
        OTPD{i}{j} = vecnorm([mean(h(:,1)), mean(h(:,3))] - [X{i}{j}(1),X{i}{j}(2)]);
    end
end

ICRAplot