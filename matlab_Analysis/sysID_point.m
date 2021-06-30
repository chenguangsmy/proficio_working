%% Wrap Lee's code into function for analysis

function [] = sysID_point()
clear all
close all


pathh = '/Users/jhermus/Documents/School/MIT/Research/Schwartz_Collaboration/proficio_working/matlab_Analysis/dataFile_notTracked/';
%           filee1 = '20210115aft00.csv';iteration mat = 8
%           filee1 = '20210115aft01.csv'; cover removed itermax = 8
%           filee1 = '20210115aft03.csv'; % Removed nrmalization iter max = 8; preturbation amplitude 1
%           filee2 = '20210115aft04.csv'; % Remove normalization iter max = 1
%           filee1 = '20210115aft05.csv'; % Keep normalization, it-max = 8, Kx =1000, Bx = 20
%           filee1 = '20210115aft06.csv';
%           filee1 = '20210202aft00.csv';

%           filee = '20210203aft00.csv'; % | K_x = 1000, B_x = 20, it_Max = 8, f_pret[1]=0; 2N amplitude |
%           filee = '20210203aft01.csv'; % | K_x = 1000, B_x = 20, it_Max = 8, f_pret[0]=0; 2N amplitude |
%           filee1 = '20210203aft02.csv'; % | K_x = 1000, B_x = 20, it_Max = 8, f_pret[1]=0; 5N amplitude |
%           filee2 = '20210203aft03.csv'; % | K_x = 1000, B_x = 20, it_Max = 8, f_pret[0]=0; 5N amplitude |


%           filee = '20210203aft04.csv'; % | K_x = 0, B_x = 0, it_Max = 8, pretAmplitude = 0.0 |
%             filee1 = '20210203aft05.csv'; % | K_x = 1000, B_x = 20, it_Max = 8, pretAmplitude = 0.0 |
%             filee1 = '20210203aft06.csv'; % | K_x = 1000, B_x = 20, it_Max = 8, pretAmplitude =10.0 |
%             filee2 = '20210203aft07.csv'; % | K_x = 500,  B_x = 20, it_Max = 8, pretAmplitude =10.0 |
%           filee1 = '20210203aft08.csv'; % | K_x = 1000, B_x = 20, it_Max =15, pretAmplitude =10.0 |
%           filee4 = '20210203aft09.csv'; % | K_x = 500,  B_x = 20, it_Max =15, pretAmplitude =10.0 |
%           filee1 = '20210203aft10.csv'; % | K_x = 500,  B_x = 20, it_Max =15, pretAmplitudex =16.0, pretAmplitudex =8.0 |
%           filee2 = '20210203aft11.csv'; % | K_x = 500,  B_x = 20, it_Max =15, pretAmplitudex =16.0, pretAmplitudex =8.0, with I hold |

%           filee6 = '20210204aft00.csv'; % Kx = 500, Bx = 0, maxiter = 1, pret amplitude = 10
%           filee2 = '20210204aft01.csv'; % Kx = 500, Bx = 0, maxiter = 1, pret amplitude = 2

%           filee3 = '20210204aft02.csv'; % K_x=1000, B_x=20, itermax = 1, pretAmplitude = 10, not removed FT cable in 02 and 03
%             filee3 = '20210204aft03.csv'; % K_x=500,   B_x=0, itermax = 8, pretAmplitude = 10, really big shake in y, up to 10cm
%           filee1 = '20210204aft04.csv'; % K_x=2500, B_x=20, itermax = 8, pretAmplitude = 6
%           filee2 = '20210204aft05.csv'; % K_x=2500, B_x=20, itermax = 8, pretAmplitude = 6, with arm holded |
            
        % PretAmplitude = 6, iteration = 8
%         filee1 = '20210218aft05.csv'; % Kx=2500, Bx=10
%         filee2 = '20210218aft06.csv'; % Kx=2500, Bx=0
%         filee3 = '20210218aft07.csv'; % Kx=500, Bx=10
%         filee4 = '20210218aft08.csv'; % Kx=500, Bx=0
%         filee5 = '20210218aft09.csv'; % Kx=2500, Bx=[-10,10]
        
        % Spring Testing
        filee1 = '20210311aft00.csv';
        filee2 = '20210311aft01.csv';
        filee3 = '20210311aft02.csv';
        filee4 = '20210311aft03.csv';
        filee5 = '20210311aft04.csv';
        
        filee6 = '20210311aft05.csv';
        filee7 = '20210311aft06.csv';
        filee8 = '20210311aft07.csv';
        filee9 = '20210311aft08.csv';
        filee10 = '20210311aft09.csv';

%         filee1 = '20210311aft01.csv';
%         filee2 = '20210311aft06.csv';


% fileNamess = {[pathh,filee1],[pathh,filee2]};
% namesString =  {'robot','robot+spring'};


% fileNamess = {[pathh,filee1],[pathh,filee2],[pathh,filee3],[pathh,filee4],[pathh,filee5]};
% namesString =  {'K:2500, B:10','K:2500, B:0','K:500, B:20','K:500, B:0','Kx=2500, Bx=[-10,10]'};
% namesString =  {'1','2','3','4','5'};

fileNamess = {[pathh,filee1],[pathh,filee2],[pathh,filee3],[pathh,filee4],[pathh,filee5],...
              [pathh,filee6],[pathh,filee7],[pathh,filee8],[pathh,filee9],[pathh,filee10]};
namesString =  {'1','2','3','4','5','6','7','8','9','10'};



% fileNamess = {[pathh,filee1]};
% namesString =  {'Robot'};

% filee1 = '20210203aft06.csv'; % | K_x = 1000, B_x = 20, it_Max = 8, pretAmplitude = 10.0 |
% filee2 = '20210203aft07.csv'; % | K_x = 500,  B_x = 20, it_Max = 8, pretAmplitude = 10.0 |
% filee3 = '20210204aft03.csv'; % | K_x = 500,  B_x =  0, it_Max = 8, pretAmplitude = 10.0, really big shake in y, up to 10cm
% fileNamess = {[pathh,filee1],[pathh,filee2],[pathh,filee3]};
% namesString =  {'K:1000, B:20','K:500, B:20','K:500, B:0'};

% namesString =  {'Robot','Robot+Hand'};

for i = 1:length(fileNamess)
    A(i) = leeID(fileNamess{i});
end

% A(2) = leeIDArm('CH_K2500_B_40_P6_FPGA-18-Jan-2020-18',A(1));

% Create plots
plotStructt(A,namesString);

% model_robot = [A(1).TF_freq', A(1).Z22_mag, A(1).Z22_phi];
% save('/Users/jhermus/Documents/School/MIT/Research/Schwartz_Collaboration/BallisticreleaseAnalysis/matlab/data/model_robot.mat','model_robot')

end

function [A] = leeID(fileNamee)
% SS Dynamic Ankle Impedance Identification (Relaxed)
% Impedance Identification in principal axis directions (DP and IE)

%% Variables
nfft=5000;
nFreq = nfft/2+1;
Hz = 500;
runningTime = 5*55;

unwrapThreshold = -50*pi/180;

% Admittance of Anklebot + Ankle
Y11_s = zeros(nFreq,1);
Y22_s = zeros(nFreq,1);
Y12_s = zeros(nFreq,1);
Y21_s = zeros(nFreq,1);

% Impedance of Anklebot + Ankle
Z11_s = zeros(nFreq,1);     Z11_s_mag = zeros(nFreq,1);     Z11_s_phi = zeros(nFreq,1);
Z22_s = zeros(nFreq,1);     Z22_s_mag = zeros(nFreq,1);     Z12_s_phi = zeros(nFreq,1);
Z12_s = zeros(nFreq,1);     Z12_s_mag = zeros(nFreq,1);     Z21_s_phi = zeros(nFreq,1);
Z21_s = zeros(nFreq,1);     Z21_s_mag = zeros(nFreq,1);     Z22_s_phi = zeros(nFreq,1);

% Impedance of Ankle
Z11_l = zeros(nFreq,1);     Z11_l_mag = zeros(nFreq,1);     Z11_l_phi = zeros(nFreq,1);
Z22_l = zeros(nFreq,1);     Z22_l_mag = zeros(nFreq,1);     Z12_l_phi = zeros(nFreq,1);
Z12_l = zeros(nFreq,1);     Z12_l_mag = zeros(nFreq,1);     Z21_l_phi = zeros(nFreq,1);
Z21_l = zeros(nFreq,1);     Z21_l_mag = zeros(nFreq,1);     Z22_l_phi = zeros(nFreq,1);

PC11_s = zeros(nFreq,1);
PC22_s = zeros(nFreq,1);
PC12_s = zeros(nFreq,1);
PC21_s = zeros(nFreq,1);

% Anklebot Data
% data_abot = load('/Users/jhermus/Documents/School/MIT/Research/Limb_Impedance/PreliminaryData/Stocastic/SampleData.asc', 'r');
% data_abot_length = length(data_abot);
% cutOff_abot = data_abot_length-runningTime*Hz;

% joint info
% d_DP=data_abot(cutOff_abot:end-1,4);
% d_IE=data_abot(cutOff_abot:end-1,3);
% t_DP=data_abot(cutOff_abot+1:end,8);
% t_IE=data_abot(cutOff_abot+1:end,7);


% robot info
% d_DP=data_abot(cutOff_abot:end-1,10);
% d_IE=data_abot(cutOff_abot:end-1,13);
% t_DP=data_abot(cutOff_abot+1:end,11);
% t_IE=data_abot(cutOff_abot+1:end,14);

% Import data and convert to X1, X2, Y1, Y2
% load('C:\Users\Newman Lab\Desktop\InMotion2\Matlab\Matlabdata\C_K2500_B40_P6_FPGA-17-Jan-2020-17-01.mat');
% load('C:\Users\Newman Lab\Desktop\InMotion2\Matlab\Matlabdata\C_K2500_B20_P6_FPGA-17-Jan-2020-17-01.mat');
% load('C:\Users\Newman Lab\Desktop\InMotion2\Matlab\Matlabdata\C_K2500_B1_P6_FPGA-17-Jan-2020-17-01.mat');


data = importdata(fileNamee);

dexStart = 20*Hz; %2e+4;
dexEnd = size(data,1)-20*Hz;
dexRange = [dexStart:dexEnd];
t = data(dexRange,1);
X_m = data(dexRange,10);
Y_m = data(dexRange,11);
Pret_X_N = data(dexRange,20);
Pret_Y_N = data(dexRange,21);

% % Sanity check
% figure(1);
% plot(Y_m); hold on;
% % Select points
% % [dex_X,dex_Y] = ginput(1);
% dex = find(diff(Pret_X_N)>1);
% dex_X = dex(5*Hz):(dex(5*Hz)+Hz*runningTime);
% % dex_X = [round(dex_X):(round(dex_X)+Hz*runningTime)];
% plot([dex_X(1), dex_X(end)],[0,0],'ok','linewidth',2.5);
% title((dex_X(end)-dex_X(1))/(Hz*60));

X1=Pret_X_N;
X2=Pret_Y_N;
Y1=X_m;
Y2=Y_m;

figure; 
subplot(2,2,1); plot(t,X1);
subplot(2,2,2); plot(t,X2);
subplot(2,2,3); plot(t,Y1);
subplot(2,2,4); plot(t,Y2);

% figure; plot(t(1:500)-t(1), X1(1:500),'linewidth',2.5);
% xlabel('Time (s)'); ylabel('Force (N)'); ylim([-8 8]); 
% set(gca, 'box', 'off','linewidth',2,'fontsize',18);
% 
% figure; 
% subplot(2,1,1); plot(t-t(1), (Y1-mean(Y1))*1000,'linewidth',2.5);
% xlabel('Time (s)'); ylabel('X (mm)'); xlim([0 max(t-t(1))]); ylim([-6 6]);  
% set(gca, 'box', 'off','linewidth',2,'fontsize',18);
% 
% subplot(2,1,2); plot(t-t(1), (Y2-mean(Y2))*1000,'linewidth',2.5);
% xlabel('Time (s)'); ylabel('Y (mm)'); xlim([0 max(t-t(1))]); ylim([-6 6]); 
% set(gca, 'box', 'off','linewidth',2,'fontsize',18);



%% Filter measurements
% cf = 500; % cutoff freqnency
% [b,a] = butter(3,cf/(Hz/2)); % make filter
% Y1=filtfilt(b,a,X_m(dex_X));
% Y2=filtfilt(b,a,Y_m(dex_X));

% Check xcorr preturbations
% [c,lags] = xcorr(X1,X1,220000,'coeff'); title('xcorr(X1,X1)');
% figure; subplot(3,1,1); stem(lags,c); hold on;
% [c,lags] = xcorr(X2,X2,220000,'coeff'); title('xcorr(X2,X2)');
% subplot(3,1,2); stem(lags,c);
% [c,lags] = xcorr(X1,X2,220000,'coeff'); title('xcorr(X1,X2)');
% subplot(3,1,3); stem(lags,c);

Y1 = detrend(Y1);
Y2 = detrend(Y2);

TF_freq = Hz/2*linspace(0,1,nfft/2+1);
A.TF_freq = TF_freq;

% Cross power spectral density
[Px1x1,Fx1x1] = cpsd(X1,X1,hamming(nfft/2),nfft/4,nfft,Hz);
[Px1x2,Fx1x2] = cpsd(X2,X1,hamming(nfft/2),nfft/4,nfft,Hz);
[Px1y1,Fx1y1] = cpsd(Y1,X1,hamming(nfft/2),nfft/4,nfft,Hz);
[Px1y2,Fx1y2] = cpsd(Y2,X1,hamming(nfft/2),nfft/4,nfft,Hz);

[Px2x1,Fx2x1] = cpsd(X1,X2,hamming(nfft/2),nfft/4,nfft,Hz);
[Px2x2,Fx2x2] = cpsd(X2,X2,hamming(nfft/2),nfft/4,nfft,Hz);
[Px2y1,Fx2y1] = cpsd(Y1,X2,hamming(nfft/2),nfft/4,nfft,Hz);
[Px2y2,Fx2y2] = cpsd(Y2,X2,hamming(nfft/2),nfft/4,nfft,Hz);

[Py1x1,Fy1x1] = cpsd(X1,Y1,hamming(nfft/2),nfft/4,nfft,Hz);
[Py1x2,Fy1x2] = cpsd(X2,Y1,hamming(nfft/2),nfft/4,nfft,Hz);
[Py1y1,Fy1y1] = cpsd(Y1,Y1,hamming(nfft/2),nfft/4,nfft,Hz);
[Py1y2,Fy1y2] = cpsd(Y2,Y1,hamming(nfft/2),nfft/4,nfft,Hz);

[Py2x1,Fy2x1] = cpsd(X1,Y2,hamming(nfft/2),nfft/4,nfft,Hz);
[Py2x2,Fy2x2] = cpsd(X2,Y2,hamming(nfft/2),nfft/4,nfft,Hz);
[Py2y1,Fy2y1] = cpsd(Y1,Y2,hamming(nfft/2),nfft/4,nfft,Hz);
[Py2y2,Fy2y2] = cpsd(Y2,Y2,hamming(nfft/2),nfft/4,nfft,Hz);

%% Sanity Check

% Sanity check xx
% figure;
% subplot(2,2,1); loglog(Fx1x1,abs(Px1x1)); title('x_1 x_1'); grid on;
% subplot(2,2,2); loglog(Fx1x2,abs(Px1x2)); title('x_1 x_2'); grid on;
% subplot(2,2,3); loglog(Fx2x1,abs(Px2x1)); title('x_2 x_1'); grid on;
% subplot(2,2,4); loglog(Fx2x2,abs(Px2x2)); title('x_2 x_2'); grid on;
%
% % Sanity check yy
% figure;
% subplot(2,2,1); loglog(Fy1y1,abs(Py1y1)); title('y_1 y_1'); grid on;
% subplot(2,2,2); loglog(Fy1y2,abs(Py1y2)); title('y_1 y_2'); grid on;
% subplot(2,2,3); loglog(Fy2y1,abs(Py2y1)); title('y_2 y_1'); grid on;
% subplot(2,2,4); loglog(Fy2y2,abs(Py2y2)); title('y_2 y_2'); grid on;
%
% % Sanity check xy
% figure;
% subplot(2,2,1); loglog(Fx1y1,abs(Px1y1)); title('x_1 y_1'); grid on;
% subplot(2,2,2); loglog(Fx1y2,abs(Px1y2)); title('x_1 y_2'); grid on;
% subplot(2,2,3); loglog(Fx2y1,abs(Px2y1)); title('x_2 y_1'); grid on;
% subplot(2,2,4); loglog(Fx2y2,abs(Px2y2)); title('x_2 y_2'); grid on;
%
% % Sanity check yx
% figure;
% subplot(2,2,1); loglog(Fy1x1,abs(Py1x1)); title('y_1 x_1'); grid on;
% subplot(2,2,2); loglog(Fy1x2,abs(Py1x2)); title('y_1 x_2'); grid on;
% subplot(2,2,3); loglog(Fy2x1,abs(Py2x1)); title('y_2 x_1'); grid on;
% subplot(2,2,4); loglog(Fy2x2,abs(Py2x2)); title('y_2 x_2'); grid on;

%% Ordinary Coherence
[OCx1x2,OCx1x2_F] = mscohere(X2,X1,hamming(nfft/2),nfft/4,nfft,Hz);
[OCx2x1,OCx2x1_F] = mscohere(X1,X2,hamming(nfft/2),nfft/4,nfft,Hz);
[OCx2y1,OCx2y1_F] = mscohere(Y1,X2,hamming(nfft/2),nfft/4,nfft,Hz);
[OCx1y2,OCx1y2_F] = mscohere(Y2,X1,hamming(nfft/2),nfft/4,nfft,Hz);
[OCx1y1,OCx1y1_F] = mscohere(Y1,X1,hamming(nfft/2),nfft/4,nfft,Hz);
[OCx2y2,OCx2y2_F] = mscohere(Y2,X2,hamming(nfft/2),nfft/4,nfft,Hz);

% figure;
% subplot(4,4,2); semilogx(OCx1x2_F,OCx1x2); title('C: x_1 x_2');grid on;
% subplot(4,4,5); semilogx(OCx2x1_F,OCx2x1); title('C: x_2 x_1');grid on;
% subplot(4,4,7); semilogx(OCx2y1_F,OCx2y1); title('C: x_2 y_1');grid on;
% subplot(4,4,4); semilogx(OCx1y2_F,OCx1y2); title('C: x_1 y_2');grid on;
% subplot(4,4,3); semilogx(OCx1y1_F,OCx1y1); title('C: x_1 y_1');grid on;
% subplot(4,4,8); semilogx(OCx2y2_F,OCx2y2); title('C: x_2 y_2');grid on;


for j=1:nFreq
    Y11(j,1) = (Px1y1(j,1)/Px1x1(j,1))*(1-(Px1x2(j,1)*Px2y1(j,1))/(Px2x2(j,1)*Px1y1(j,1)))/(1-OCx1x2(j,1));
    Y12(j,1) = (Px2y1(j,1)/Px2x2(j,1))*(1-(Px2x1(j,1)*Px1y1(j,1))/(Px1x1(j,1)*Px2y1(j,1)))/(1-OCx1x2(j,1));
    Y21(j,1) = (Px1y2(j,1)/Px1x1(j,1))*(1-(Px1x2(j,1)*Px2y2(j,1))/(Px2x2(j,1)*Px1y2(j,1)))/(1-OCx1x2(j,1));
    Y22(j,1) = (Px2y2(j,1)/Px2x2(j,1))*(1-(Px2x1(j,1)*Px1y2(j,1))/(Px1x1(j,1)*Px2y2(j,1)))/(1-OCx1x2(j,1));
    
    % Partial Coherence
    A.PC11(j,1) = abs(Px1y1(j,1)*Px2x2(j,1)-Px2y1(j,1)*Px1x2(j,1))^2/(Px2x2(j,1)*Px2x2(j,1)*Px1x1(j,1)*Py1y1(j,1))/(1-OCx2x1(j,1))/(1-OCx2y1(j,1));
    A.PC21(j,1) = abs(Px1y2(j,1)*Px2x2(j,1)-Px2y2(j,1)*Px1x2(j,1))^2/(Px2x2(j,1)*Px2x2(j,1)*Px1x1(j,1)*Py2y2(j,1))/(1-OCx2x1(j,1))/(1-OCx2y2(j,1));
    A.PC12(j,1) = abs(Px2y1(j,1)*Px1x1(j,1)-Px1y1(j,1)*Px2x1(j,1))^2/(Px1x1(j,1)*Px1x1(j,1)*Px2x2(j,1)*Py1y1(j,1))/(1-OCx1x2(j,1))/(1-OCx1y1(j,1));
    A.PC22(j,1) = abs(Px2y2(j,1)*Px1x1(j,1)-Px1y2(j,1)*Px2x1(j,1))^2/(Px1x1(j,1)*Px1x1(j,1)*Px2x2(j,1)*Py2y2(j,1))/(1-OCx1x2(j,1))/(1-OCx1y2(j,1));
    
    % Anklebot+Ankle Impedance
    K_matrix = inv([Y11(j,1) Y12(j,1); Y21(j,1) Y22(j,1)]);
    
    Z11(j,1) = K_matrix(1,1);
    Z12(j,1) = K_matrix(1,2);
    Z21(j,1) = K_matrix(2,1);
    Z22(j,1) = K_matrix(2,2);
    
end

A.Z11 = Z11;
A.Z12 = Z12;
A.Z21 = Z21;
A.Z22 = Z22;

%% Mag & Phase calculation for Impedance plot
for j=1:nFreq
    A.Z11_mag(j,1) = abs(Z11(j,1));
    A.Z22_mag(j,1) = abs(Z22(j,1));
    A.Z12_mag(j,1) = abs(Z12(j,1));
    A.Z21_mag(j,1) = abs(Z21(j,1));
    A.Z11_phi(j,1) = 180/pi*unwrap2(angle(Z11(j,1)),unwrapThreshold,'up');
    A.Z22_phi(j,1) = 180/pi*unwrap2(angle(Z22(j,1)),unwrapThreshold,'up');
    A.Z12_phi(j,1) = 180/pi*unwrap2(angle(Z12(j,1)),unwrapThreshold,'up');
    A.Z21_phi(j,1) = 180/pi*unwrap2(angle(Z21(j,1)),unwrapThreshold,'up');
end

end


function [] =  plotStructt(A,namesString)

%% Impedance plot (Diagonal)
xLowerLim = 0.15;
xUpperLim = 50.0;
yLowerLim = 0.0;
yUpperLim = 180.0;

X = figure;
XX = figure;
XXX = figure;
% colorr = {'-b','-r','-k','--b','--r','--k'};
% colorr = {'-b','-r','-k'};
colorr = {'-b','-b','-b','-b','-b','-r','-r','-r','-r','-r'};


FS = 12;

% figure(X);
% for i = 1:length(A)
%     
%     % Magnitude plot of ankle impedance
%     ax1 = subplot(2,2,1);
%     set(gca,'fontWeight','bold','fontSize',FS);
%     loglog(A(i).TF_freq,A(i).Z11_mag(:,1),colorr{i},'LineWidth',2); hold on; grid on; box on;
%     %axis([xLowerLim xUpperLim yLowerLim yUpperLim]);
%     xlabel('frequency(Hz)','fontWeight','bold','fontSize',FS);
%     ylabel('magnitude (abs)','fontWeight','bold','fontSize',FS);
%     title('Z11','fontWeight','bold','fontSize',FS);
%     
%     ax2 = subplot(2,2,2);
%     set(gca,'fontWeight','bold','fontSize',FS);
%     loglog(A(i).TF_freq,A(i).Z12_mag(:,1),colorr{i},'LineWidth',2); hold on; grid on; box on;
%     %axis([xLowerLim xUpperLim yLowerLim yUpperLim]);
%     xlabel('frequency (Hz)','fontWeight','bold','fontSize',FS);
%     ylabel('magnitude (abs)','fontWeight','bold','fontSize',FS);
%     title('Z12','fontWeight','bold','fontSize',FS);
%     
%     ax3 = subplot(2,2,3);
%     set(gca,'fontWeight','bold','fontSize',FS);
%     loglog(A(i).TF_freq,A(i).Z21_mag(:,1),colorr{i},'LineWidth',2); hold on; grid on; box on;
%     %axis([xLowerLim xUpperLim yLowerLim yUpperLim]);
%     xlabel('frequency (Hz)','fontWeight','bold','fontSize',FS);
%     ylabel('magnitude (abs)','fontWeight','bold','fontSize',FS);
%     title('Z21','fontWeight','bold','fontSize',FS);
%     
%     ax4 = subplot(2,2,4);
%     set(gca,'fontWeight','bold','fontSize',FS);
%     loglog(A(i).TF_freq,A(i).Z22_mag(:,1),colorr{i},'LineWidth',2); hold on; grid on; box on;
%     %axis([xLowerLim xUpperLim yLowerLim yUpperLim]);
%     xlabel('frequency (Hz)','fontWeight','bold','fontSize',FS);
%     ylabel('magnitude (abs)','fontWeight','bold','fontSize',FS);
%     title('Z22','fontWeight','bold','fontSize',FS);
%     
%     linkaxes([ax1,ax2,ax3,ax4],'x');
%     
% end
% legend(namesString);
% 
% figure(XX);
% for i = 1:length(A)
%     
%     ax1 = subplot(2,2,1);
%     set(gca,'fontWeight','bold','fontSize',FS);
%     semilogx(A(i).TF_freq,A(i).Z11_phi(:,1),colorr{i},'LineWidth',2); hold on; grid on; box on;
%     axis([xLowerLim xUpperLim 0 180]);
%     xlabel('frequency (Hz)','fontWeight','bold','fontSize',14); ylabel('phase (deg)','fontWeight','bold','fontSize',14);
%     title('Z11','fontWeight','bold','fontSize',FS);
%     
%     ax2 = subplot(2,2,2);
%     set(gca,'fontWeight','bold','fontSize',FS);
%     semilogx(A(i).TF_freq,A(i).Z12_phi(:,1),colorr{i},'LineWidth',2); hold on; grid on; box on;
%     axis([xLowerLim xUpperLim 0 180]);
%     xlabel('frequency (Hz)','fontWeight','bold','fontSize',14); ylabel('phase (deg)','fontWeight','bold','fontSize',14);
%     title('Z12','fontWeight','bold','fontSize',FS);
%     
%     ax3 = subplot(2,2,3);
%     set(gca,'fontWeight','bold','fontSize',FS);
%     semilogx(A(i).TF_freq,A(i).Z21_phi(:,1),colorr{i},'LineWidth',2); hold on; grid on; box on;
%     axis([xLowerLim xUpperLim 0 180]);
%     xlabel('frequency (Hz)','fontWeight','bold','fontSize',14); ylabel('phase (deg)','fontWeight','bold','fontSize',14);
%     title('Z21','fontWeight','bold','fontSize',FS);
% 
%     ax4 = subplot(2,2,4);
%     set(gca,'fontWeight','bold','fontSize',FS);
%     semilogx(A(i).TF_freq,A(i).Z22_phi(:,1),colorr{i},'LineWidth',2); hold on; grid on; box on;
%     axis([xLowerLim xUpperLim 0 180]);
%     xlabel('frequency (Hz)','fontWeight','bold','fontSize',14); ylabel('phase (deg)','fontWeight','bold','fontSize',14);
%     title('Z22','fontWeight','bold','fontSize',FS);
% 
%     linkaxes([ax1,ax2,ax3,ax4],'x');
%     
% end
% legend(namesString);

figure(X);
for i = 1:length(A)
    
    % Magnitude plot of ankle impedance
    ax1 = subplot(2,2,1);
    set(gca,'fontWeight','bold','fontSize',FS);
    p1(i) = loglog(A(i).TF_freq,A(i).Z11_mag(:,1),colorr{i},'LineWidth',2); hold on; grid on; box on;
    %axis([xLowerLim xUpperLim yLowerLim yUpperLim]);
    xlabel('frequency(Hz)','fontWeight','bold','fontSize',FS);
    ylabel('magnitude (abs)','fontWeight','bold','fontSize',FS);
    title('Z11','fontWeight','bold','fontSize',FS);
    
    ax2 = subplot(2,2,2);
    set(gca,'fontWeight','bold','fontSize',FS);
    loglog(A(i).TF_freq,A(i).Z22_mag(:,1),colorr{i},'LineWidth',2); hold on; grid on; box on;
    %axis([xLowerLim xUpperLim yLowerLim yUpperLim]);
    xlabel('frequency (Hz)','fontWeight','bold','fontSize',FS);
    ylabel('magnitude (abs)','fontWeight','bold','fontSize',FS);
    title('Z22','fontWeight','bold','fontSize',FS);
    
    ax3 = subplot(2,2,3);
    set(gca,'fontWeight','bold','fontSize',FS);
    semilogx(A(i).TF_freq,A(i).Z11_phi(:,1),colorr{i},'LineWidth',2); hold on; grid on; box on;
    axis([xLowerLim xUpperLim 0 180]);
    xlabel('frequency (Hz)','fontWeight','bold','fontSize',14); ylabel('phase (deg)','fontWeight','bold','fontSize',14);
    
    ax4 = subplot(2,2,4);
    set(gca,'fontWeight','bold','fontSize',FS);
    semilogx(A(i).TF_freq,A(i).Z22_phi(:,1),colorr{i},'LineWidth',2); hold on; grid on; box on;
    axis([xLowerLim xUpperLim 0 180]);
    xlabel('frequency (Hz)','fontWeight','bold','fontSize',14); ylabel('phase (deg)','fontWeight','bold','fontSize',14);
    
    linkaxes([ax1,ax2,ax3,ax4],'x');
    
end
legend([p1(1) p1(6)],{'Robot','Robot+Spring'});

%% Sanity check Expected Dynamics
% opts = bodeoptions('cstprefs');
% s = tf('s');
% 
% % [theta_r, theta_dr, J_r, J_dr, h_r, M_r, PlotPosVec_X, PlotPosVec_Y] = InMotionPassiveForce(0, -0.4699, 0, 0)
% 
% 
% % M_x = inv(J_r*inv(M_r)*J_r');
% 
% addedColorr = {':b',':r'};
% figure(X);
% bVec = [20,20];
% kVec = [500,1000];
% for i = 1:length(bVec)
%     b = bVec(i);
%     k = kVec(i);
%     
%     m = 10;
%     [MAG1,PHASE1,W1] = bode(m*s^2 + b*s + k,opts);
%     
%     m = 1;
%     [MAG2,PHASE2,W2] = bode(m*s^2 + b*s + k,opts);
%     W1 = W1/(2*pi); % Convert to Hz
%     W2 = W2/(2*pi); % Convert to Hz
%     MAG1 = squeeze(MAG1);
%     MAG2 = squeeze(MAG2);
% 
%     PHASE1 = squeeze(PHASE1);
%     PHASE2 = squeeze(PHASE2);
% 
%     subplot(2,2,1);loglog(W1,MAG1,addedColorr{i},'linewidth',2.5); grid on; %xlim([W(2) W(end)]); ylim([10^-6 10^-1]);
%     subplot(2,2,2);loglog(W2,MAG2,addedColorr{i},'linewidth',2.5); grid on; %xlim([W(2) W(end)]); ylim([10^-6 10^-1]);
% 
%     subplot(2,2,3); semilogx(W1,PHASE1,addedColorr{i},'linewidth',2.5); grid on; hold on;
%     subplot(2,2,4); semilogx(W2,PHASE2,addedColorr{i},'linewidth',2.5); grid on; hold on;
% end
% legend(namesString);

disp('test');

%% Partial Coherence Plot

figure(XXX);
for i = 1:length(A)
    ax1 = subplot(2,2,1);
    set(gca,'fontWeight','bold','fontSize',12);
    semilogx(A(i).TF_freq,A(i).PC11(:,1),colorr{i},'LineWidth',2); hold on; grid on; box on;
    axis([xLowerLim xUpperLim 0 1]);
    xlabel('frequence (Hz)','fontWeight','bold','fontSize',14);
    ylabel('\gamma^2','fontWeight','bold','fontSize',14);
    title('Y11','fontWeight','bold','fontSize',16);
    
    ax2 = subplot(2,2,2);
    set(gca,'fontWeight','bold','fontSize',12);
    semilogx(A(i).TF_freq,A(i).PC12(:,1),colorr{i},'LineWidth',2); hold on; grid on; box on;
    axis([xLowerLim xUpperLim 0 1]);
    xlabel('frequence (Hz)','fontWeight','bold','fontSize',14);
    ylabel('\gamma^2','fontWeight','bold','fontSize',14);
    title('Y12','fontWeight','bold','fontSize',16);
    
    ax3 = subplot(2,2,3);
    set(gca,'fontWeight','bold','fontSize',12);
    semilogx(A(i).TF_freq,A(i).PC21(:,1),colorr{i},'LineWidth',2); hold on; grid on; box on;
    axis([xLowerLim xUpperLim 0 1]);
    xlabel('frequence (Hz)','fontWeight','bold','fontSize',14);
    ylabel('\gamma^2','fontWeight','bold','fontSize',14);
    title('Y21','fontWeight','bold','fontSize',16);
    
    ax4 = subplot(2,2,4);
    set(gca,'fontWeight','bold','fontSize',12);
    semilogx(A(i).TF_freq,A(i).PC22(:,1),colorr{i},'LineWidth',2); hold on; grid on; box on;
    axis([xLowerLim xUpperLim 0 1]);
    xlabel('frequence (Hz)','fontWeight','bold','fontSize',14);
    ylabel('\gamma^2','fontWeight','bold','fontSize',14);
    title('Y22','fontWeight','bold','fontSize',16);
    
    linkaxes([ax1,ax2,ax3,ax4],'x');
end
% legend(namesString);


% %% Plot diffrence
% figure;
% % Magnitude plot of ankle impedance
% ax1 = subplot(2,2,1);
% set(gca,'fontWeight','bold','fontSize',12);
% loglog(A(1).TF_freq,A(2).Z11_mag(:,1)-A(1).Z11_mag(:,1),colorr{i},'LineWidth',2); hold on; grid on; box on;
% %axis([xLowerLim xUpperLim yLowerLim yUpperLim]);
% xlabel('frequency(Hz)','fontWeight','bold','fontSize',14);
% ylabel('magnitude (abs)','fontWeight','bold','fontSize',14);
% title('Z11','fontWeight','bold','fontSize',16);
% 
% ax2 = subplot(2,2,2);
% set(gca,'fontWeight','bold','fontSize',12);
% loglog(A(i).TF_freq,A(2).Z22_mag(:,1)-A(1).Z22_mag(:,1),colorr{i},'LineWidth',2); hold on; grid on; box on;
% %axis([xLowerLim xUpperLim yLowerLim yUpperLim]);
% xlabel('frequency (Hz)','fontWeight','bold','fontSize',14);
% ylabel('magnitude (abs)','fontWeight','bold','fontSize',14);
% title('Z22','fontWeight','bold','fontSize',16);
% 
% ax3 = subplot(2,2,3);
% set(gca,'fontWeight','bold','fontSize',12);
% semilogx(A(1).TF_freq,A(2).Z11_phi(:,1)-A(1).Z11_phi(:,1),colorr{i},'LineWidth',2); hold on; grid on; box on;
% axis([xLowerLim xUpperLim 0 180]);
% xlabel('frequency (Hz)','fontWeight','bold','fontSize',14); ylabel('phase (deg)','fontWeight','bold','fontSize',14);
% 
% ax4 = subplot(2,2,4);
% set(gca,'fontWeight','bold','fontSize',12);
% semilogx(A(1).TF_freq,A(2).Z22_phi(:,1)-A(1).Z22_phi(:,1),colorr{i},'LineWidth',2); hold on; grid on; box on;
% axis([xLowerLim xUpperLim 0 180]);
% xlabel('frequency (Hz)','fontWeight','bold','fontSize',14); ylabel('phase (deg)','fontWeight','bold','fontSize',14);
% 
% linkaxes([ax1,ax2,ax3,ax4],'x');

% figure; 

dexStiff = find( 0.2 < A(i).TF_freq & A(i).TF_freq < 1.3 );
 K_r = (A(1).Z22_mag(dexStiff,1) +...
           A(2).Z22_mag(dexStiff,1) +...
           A(3).Z22_mag(dexStiff,1) +...
           A(4).Z22_mag(dexStiff,1) +...
           A(5).Z22_mag(dexStiff,1))./5;
K_rh = (A(6).Z22_mag(dexStiff,1) +...
          A(7).Z22_mag(dexStiff,1) +...
          A(8).Z22_mag(dexStiff,1) +...
          A(9).Z22_mag(dexStiff,1) +...
          A(10).Z22_mag(dexStiff,1))./5;
      
K_hat = K_rh-K_r;

            
% figure;plot(A(1).TF_freq(dexStiff),K_r);
% figure;plot(A(1).TF_freq(dexStiff),K_rh);
% figure;plot(A(1).TF_freq(dexStiff),K_hat);
      

figure;
ax1 = errorbar(1.4,mean(K_hat),std(K_hat),'linewidth',2,'color','b'); hold on;
ax2 = errorbar(1.45,315.2,8.8*2,'linewidth',2,'color','r');
plot(A(i).TF_freq(dexStiff), K_hat,'ob','linewidth',2);
plot([0.2,1.5],[mean(K_hat),mean(K_hat)],'-b','linewidth',2);
plot([0.2,1.5],[315.2,315.2],'-r','linewidth',2);

xlabel('Frequency (Hz)');
ylabel('Stiffness (N/m)');
set(gca,'fontsize',18); ylim([0 400]); xlim([0.2 1.5]);
legend('Stiffness Estimate','Manufacturer Stiffness \pm tolerance','location','south');

% 
% % axes(ax2)
% figure; 
% % Sanity check
% opts = bodeoptions('cstprefs');
% opts.MagUnits = 'abs';
% opts.MagScale = 'log';
% s = tf('s');
% 
% m = 2.5;
% b = 7;
% k = 31;
% [MAG,PHASE,W] = bode(m*s^2+b*s+k);
% MAG = squeeze(MAG);
% PHASE = squeeze(PHASE);
% subplot(2,1,1);loglog(W,MAG,'-','linewidth',2.5); grid on; %xlim([W(2) W(end)]); ylim([10^-6 10^-1]);
% ylabel('Amplitude (abs)'); set(gca,'fontsize',18);
% %             yticks([1e-5,1e-3,1e-1]);
% 
% subplot(2,1,2); semilogx(W,PHASE,'linewidth',2.5); grid on;
% ylabel('Phase (deg)'); xlabel('Frequency ()'); set(gca,'fontsize',18);
    

end

function [A] = leeIDArm(fileNamee,Z_bot)

% SS Dynamic Ankle Impedance Identification (Relaxed)
% Impedance Identification in principal axis directions (DP and IE)

%% Variables
nfft=12000;
nFreq = nfft/2+1;
Hz = 2000;
runningTime = 5*55;

unwrapThreshold = -50*pi/180;

% Admittance of Anklebot + Ankle
Y11_s = zeros(nFreq,1);
Y22_s = zeros(nFreq,1);
Y12_s = zeros(nFreq,1);
Y21_s = zeros(nFreq,1);

% Impedance of Anklebot + Ankle
Z11_s = zeros(nFreq,1);     Z11_s_mag = zeros(nFreq,1);     Z11_s_phi = zeros(nFreq,1);
Z22_s = zeros(nFreq,1);     Z22_s_mag = zeros(nFreq,1);     Z12_s_phi = zeros(nFreq,1);
Z12_s = zeros(nFreq,1);     Z12_s_mag = zeros(nFreq,1);     Z21_s_phi = zeros(nFreq,1);
Z21_s = zeros(nFreq,1);     Z21_s_mag = zeros(nFreq,1);     Z22_s_phi = zeros(nFreq,1);

% Impedance of Ankle
Z11_l = zeros(nFreq,1);     Z11_l_mag = zeros(nFreq,1);     Z11_l_phi = zeros(nFreq,1);
Z22_l = zeros(nFreq,1);     Z22_l_mag = zeros(nFreq,1);     Z12_l_phi = zeros(nFreq,1);
Z12_l = zeros(nFreq,1);     Z12_l_mag = zeros(nFreq,1);     Z21_l_phi = zeros(nFreq,1);
Z21_l = zeros(nFreq,1);     Z21_l_mag = zeros(nFreq,1);     Z22_l_phi = zeros(nFreq,1);

PC11_s = zeros(nFreq,1);
PC22_s = zeros(nFreq,1);
PC12_s = zeros(nFreq,1);
PC21_s = zeros(nFreq,1);

% Anklebot Data
% data_abot = load('/Users/jhermus/Documents/School/MIT/Research/Limb_Impedance/PreliminaryData/Stocastic/SampleData.asc', 'r');
% data_abot_length = length(data_abot);
% cutOff_abot = data_abot_length-runningTime*Hz;

% joint info
% d_DP=data_abot(cutOff_abot:end-1,4);
% d_IE=data_abot(cutOff_abot:end-1,3);
% t_DP=data_abot(cutOff_abot+1:end,8);
% t_IE=data_abot(cutOff_abot+1:end,7);


% robot info
% d_DP=data_abot(cutOff_abot:end-1,10);
% d_IE=data_abot(cutOff_abot:end-1,13);
% t_DP=data_abot(cutOff_abot+1:end,11);
% t_IE=data_abot(cutOff_abot+1:end,14);

% Import data and convert to X1, X2, Y1, Y2
% load('C:\Users\Newman Lab\Desktop\InMotion2\Matlab\Matlabdata\C_K2500_B40_P6_FPGA-17-Jan-2020-17-01.mat');
% load('C:\Users\Newman Lab\Desktop\InMotion2\Matlab\Matlabdata\C_K2500_B20_P6_FPGA-17-Jan-2020-17-01.mat');
% load('C:\Users\Newman Lab\Desktop\InMotion2\Matlab\Matlabdata\C_K2500_B1_P6_FPGA-17-Jan-2020-17-01.mat');

load(['/Users/jhermus/Documents/School/MIT/Research/InMotion2/Matlab/Matlabdata/',fileNamee,'.mat']);

% Sanity check
figure(1);
plot(U_X_N); hold on;
% Select points
% [dex_X,dex_Y] = ginput(1);
dex = find(diff(Pret_X_N)>1);
dex_X = dex(5*Hz):(dex(5*Hz)+Hz*runningTime);
% dex_X = [round(dex_X):(round(dex_X)+Hz*runningTime)];
plot([dex_X(1), dex_X(end)],[0,0],'ok','linewidth',2.5);
title((dex_X(end)-dex_X(1))/(Hz*60));

X1=Pret_X_N(dex_X);
X2=Pret_Y_N(dex_X);
Y1=X_m(dex_X);
Y2=Y_m(dex_X);

%% Filter measurements
% cf = 500; % cutoff freqnency
% [b,a] = butter(3,cf/(Hz/2)); % make filter
% Y1=filtfilt(b,a,X_m(dex_X));
% Y2=filtfilt(b,a,Y_m(dex_X));

% Check xcorr preturbations
% [c,lags] = xcorr(X1,X1,220000,'coeff'); title('xcorr(X1,X1)');
% figure; subplot(3,1,1); stem(lags,c); hold on;
% [c,lags] = xcorr(X2,X2,220000,'coeff'); title('xcorr(X2,X2)');
% subplot(3,1,2); stem(lags,c);
% [c,lags] = xcorr(X1,X2,220000,'coeff'); title('xcorr(X1,X2)');
% subplot(3,1,3); stem(lags,c);

Y1 = detrend(Y1);
Y2 = detrend(Y2);

TF_freq = Hz/2*linspace(0,1,nfft/2+1);
A.TF_freq = TF_freq;

% Cross power spectral density
[Px1x1,Fx1x1] = cpsd(X1,X1,hamming(nfft/2),nfft/4,nfft,Hz);
[Px1x2,Fx1x2] = cpsd(X2,X1,hamming(nfft/2),nfft/4,nfft,Hz);
[Px1y1,Fx1y1] = cpsd(Y1,X1,hamming(nfft/2),nfft/4,nfft,Hz);
[Px1y2,Fx1y2] = cpsd(Y2,X1,hamming(nfft/2),nfft/4,nfft,Hz);

[Px2x1,Fx2x1] = cpsd(X1,X2,hamming(nfft/2),nfft/4,nfft,Hz);
[Px2x2,Fx2x2] = cpsd(X2,X2,hamming(nfft/2),nfft/4,nfft,Hz);
[Px2y1,Fx2y1] = cpsd(Y1,X2,hamming(nfft/2),nfft/4,nfft,Hz);
[Px2y2,Fx2y2] = cpsd(Y2,X2,hamming(nfft/2),nfft/4,nfft,Hz);

[Py1x1,Fy1x1] = cpsd(X1,Y1,hamming(nfft/2),nfft/4,nfft,Hz);
[Py1x2,Fy1x2] = cpsd(X2,Y1,hamming(nfft/2),nfft/4,nfft,Hz);
[Py1y1,Fy1y1] = cpsd(Y1,Y1,hamming(nfft/2),nfft/4,nfft,Hz);
[Py1y2,Fy1y2] = cpsd(Y2,Y1,hamming(nfft/2),nfft/4,nfft,Hz);

[Py2x1,Fy2x1] = cpsd(X1,Y2,hamming(nfft/2),nfft/4,nfft,Hz);
[Py2x2,Fy2x2] = cpsd(X2,Y2,hamming(nfft/2),nfft/4,nfft,Hz);
[Py2y1,Fy2y1] = cpsd(Y1,Y2,hamming(nfft/2),nfft/4,nfft,Hz);
[Py2y2,Fy2y2] = cpsd(Y2,Y2,hamming(nfft/2),nfft/4,nfft,Hz);

%% Sanity Check

% Sanity check xx
% figure;
% subplot(2,2,1); loglog(Fx1x1,abs(Px1x1)); title('x_1 x_1'); grid on;
% subplot(2,2,2); loglog(Fx1x2,abs(Px1x2)); title('x_1 x_2'); grid on;
% subplot(2,2,3); loglog(Fx2x1,abs(Px2x1)); title('x_2 x_1'); grid on;
% subplot(2,2,4); loglog(Fx2x2,abs(Px2x2)); title('x_2 x_2'); grid on;
%
% % Sanity check yy
% figure;
% subplot(2,2,1); loglog(Fy1y1,abs(Py1y1)); title('y_1 y_1'); grid on;
% subplot(2,2,2); loglog(Fy1y2,abs(Py1y2)); title('y_1 y_2'); grid on;
% subplot(2,2,3); loglog(Fy2y1,abs(Py2y1)); title('y_2 y_1'); grid on;
% subplot(2,2,4); loglog(Fy2y2,abs(Py2y2)); title('y_2 y_2'); grid on;
%
% % Sanity check xy
% figure;
% subplot(2,2,1); loglog(Fx1y1,abs(Px1y1)); title('x_1 y_1'); grid on;
% subplot(2,2,2); loglog(Fx1y2,abs(Px1y2)); title('x_1 y_2'); grid on;
% subplot(2,2,3); loglog(Fx2y1,abs(Px2y1)); title('x_2 y_1'); grid on;
% subplot(2,2,4); loglog(Fx2y2,abs(Px2y2)); title('x_2 y_2'); grid on;
%
% % Sanity check yx
% figure;
% subplot(2,2,1); loglog(Fy1x1,abs(Py1x1)); title('y_1 x_1'); grid on;
% subplot(2,2,2); loglog(Fy1x2,abs(Py1x2)); title('y_1 x_2'); grid on;
% subplot(2,2,3); loglog(Fy2x1,abs(Py2x1)); title('y_2 x_1'); grid on;
% subplot(2,2,4); loglog(Fy2x2,abs(Py2x2)); title('y_2 x_2'); grid on;

%% Ordinary Coherence
[OCx1x2,OCx1x2_F] = mscohere(X2,X1,hamming(nfft/2),nfft/4,nfft,Hz);
[OCx2x1,OCx2x1_F] = mscohere(X1,X2,hamming(nfft/2),nfft/4,nfft,Hz);
[OCx2y1,OCx2y1_F] = mscohere(Y1,X2,hamming(nfft/2),nfft/4,nfft,Hz);
[OCx1y2,OCx1y2_F] = mscohere(Y2,X1,hamming(nfft/2),nfft/4,nfft,Hz);
[OCx1y1,OCx1y1_F] = mscohere(Y1,X1,hamming(nfft/2),nfft/4,nfft,Hz);
[OCx2y2,OCx2y2_F] = mscohere(Y2,X2,hamming(nfft/2),nfft/4,nfft,Hz);

% figure;
% subplot(4,4,2); semilogx(OCx1x2_F,OCx1x2); title('C: x_1 x_2');grid on;
% subplot(4,4,5); semilogx(OCx2x1_F,OCx2x1); title('C: x_2 x_1');grid on;
% subplot(4,4,7); semilogx(OCx2y1_F,OCx2y1); title('C: x_2 y_1');grid on;
% subplot(4,4,4); semilogx(OCx1y2_F,OCx1y2); title('C: x_1 y_2');grid on;
% subplot(4,4,3); semilogx(OCx1y1_F,OCx1y1); title('C: x_1 y_1');grid on;
% subplot(4,4,8); semilogx(OCx2y2_F,OCx2y2); title('C: x_2 y_2');grid on;


for j=1:nFreq
    Y11(j,1) = (Px1y1(j,1)/Px1x1(j,1))*(1-(Px1x2(j,1)*Px2y1(j,1))/(Px2x2(j,1)*Px1y1(j,1)))/(1-OCx1x2(j,1));
    Y12(j,1) = (Px2y1(j,1)/Px2x2(j,1))*(1-(Px2x1(j,1)*Px1y1(j,1))/(Px1x1(j,1)*Px2y1(j,1)))/(1-OCx1x2(j,1));
    Y21(j,1) = (Px1y2(j,1)/Px1x1(j,1))*(1-(Px1x2(j,1)*Px2y2(j,1))/(Px2x2(j,1)*Px1y2(j,1)))/(1-OCx1x2(j,1));
    Y22(j,1) = (Px2y2(j,1)/Px2x2(j,1))*(1-(Px2x1(j,1)*Px1y2(j,1))/(Px1x1(j,1)*Px2y2(j,1)))/(1-OCx1x2(j,1));
    
    % Partial Coherence
    A.PC11(j,1) = abs(Px1y1(j,1)*Px2x2(j,1)-Px2y1(j,1)*Px1x2(j,1))^2/(Px2x2(j,1)*Px2x2(j,1)*Px1x1(j,1)*Py1y1(j,1))/(1-OCx2x1(j,1))/(1-OCx2y1(j,1));
    A.PC21(j,1) = abs(Px1y2(j,1)*Px2x2(j,1)-Px2y2(j,1)*Px1x2(j,1))^2/(Px2x2(j,1)*Px2x2(j,1)*Px1x1(j,1)*Py2y2(j,1))/(1-OCx2x1(j,1))/(1-OCx2y2(j,1));
    A.PC12(j,1) = abs(Px2y1(j,1)*Px1x1(j,1)-Px1y1(j,1)*Px2x1(j,1))^2/(Px1x1(j,1)*Px1x1(j,1)*Px2x2(j,1)*Py1y1(j,1))/(1-OCx1x2(j,1))/(1-OCx1y1(j,1));
    A.PC22(j,1) = abs(Px2y2(j,1)*Px1x1(j,1)-Px1y2(j,1)*Px2x1(j,1))^2/(Px1x1(j,1)*Px1x1(j,1)*Px2x2(j,1)*Py2y2(j,1))/(1-OCx1x2(j,1))/(1-OCx1y2(j,1));
    
    % Anklebot+Ankle Impedance
    K_matrix = inv([Y11(j,1) Y12(j,1); Y21(j,1) Y22(j,1)]);
    
    Z11(j,1) = K_matrix(1,1) - Z_bot.Z11(j,1);
    Z12(j,1) = K_matrix(1,2) - Z_bot.Z12(j,1);
    Z21(j,1) = K_matrix(2,1) - Z_bot.Z21(j,1);
    Z22(j,1) = K_matrix(2,2) - Z_bot.Z22(j,1);
    
end

A.Z11 = Z11;
A.Z12 = Z12;
A.Z21 = Z21;
A.Z22 = Z22;

%% Mag & Phase calculation for Impedance plot
for j=1:nFreq
    A.Z11_mag(j,1) = abs(Z11(j,1));
    A.Z22_mag(j,1) = abs(Z22(j,1));
    A.Z12_mag(j,1) = abs(Z12(j,1));
    A.Z21_mag(j,1) = abs(Z21(j,1));
    A.Z11_phi(j,1) = 180/pi*unwrap2(angle(Z11(j,1)),unwrapThreshold,'up');
    A.Z22_phi(j,1) = 180/pi*unwrap2(angle(Z22(j,1)),unwrapThreshold,'up');
    A.Z12_phi(j,1) = 180/pi*unwrap2(angle(Z12(j,1)),unwrapThreshold,'up');
    A.Z21_phi(j,1) = 180/pi*unwrap2(angle(Z21(j,1)),unwrapThreshold,'up');
    
end

end


