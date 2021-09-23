classdef importWAM < handle 
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        sfrq
        t
        q
        q_dot
        q0
        tau
        X
        X_dot
        F_pret
        
        robotPos
        robotPos_end
        robot
        
        W_Tranform
    end
    
    methods
        function this = importWAM()
            % UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            this.q0 = [-pi/2,0,0,pi/2];
            this.W_Tranform =[0 1 0 0; 0 0 1 0; 1 0 0 0; 0 0 0 1];
                            %[1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
            filee = 'dataFile_notTracked/KingKongWAM02883.csv';
            this.importWAMData(filee);
%             check_stiffness(this);
%             import_WAM_model(this);
            get_robotJointPlot(this);
            
            figure;
            ax1 = subplot(2,1,1); plot(this.X(:,2));
            ax2 = subplot(2,1,2); plot(this.F_pret);
            linkaxes([ax1,ax2],'x');
            
            % Post Check Point Constraint
            % Use move joint postion
            x = [0,0.5,0.0]';
            q = [-pi/2,-pi/4,0,pi/2];
            pos = this.get_robotSinglePos(q);
            figure;
            plot3(x(1),x(2),x(3),'or','markersize',10);hold on;
            plot3(pos(:,1),pos(:,2),pos(:,3),'.-b','linewidth',2.5,'markersize',15);
            plot3([0,0.1],[0,0],[0,0],'-r','linewidth',2);
            plot3([0,0],[0,0.1],[0,0],'-r','linewidth',2);
            plot3([0,0],[0,0],[0,0.1],'-r','linewidth',2);
            xlabel('x'); ylabel('y'); zlabel('z'); grid on; axis equal;
            xlim([-1 1]); ylim([-1 1]); zlim([-1 1]); hold off;
            
            % Check Constraint Plan
            x = [0,0;-0.25,0.25;0.7,0.7];
            q = [-pi/2,-pi/4,0,pi/2];
            % q0 = [-pi/2, pi/4, 0, pi/2]
            % K =  [k, 0, k, 0]
            pos = this.get_robotSinglePos(q);
            figure;
            plot3(x(1,:),x(2,:),x(3,:),':k','linewidth',2.5);hold on;
            plot3(pos(:,1),pos(:,2),pos(:,3),'.-b','linewidth',2.5,'markersize',15);
            plot3([0,0.1],[0,0],[0,0],'-r','linewidth',2);
            plot3([0,0],[0,0.1],[0,0],'-r','linewidth',2);
            plot3([0,0],[0,0],[0,0.1],'-r','linewidth',2);
            xlabel('x'); ylabel('y'); zlabel('z'); grid on; axis equal;
            xlim([-1 1]); ylim([-1 1]); zlim([-1 1]); hold off;
            
            figure;
            subplot(2,1,1); plot(this.F_pret(:,1));
            subplot(2,1,2); histogram(this.F_pret(:,1));
            
            for i = 1:length(this.t)
                X_joint(i,1) = this.robotPos{i}(5,1);
            	X_joint(i,2) = this.robotPos{i}(5,2);
                X_joint(i,3) = this.robotPos{i}(5,3);
            end
            
            % postion error
             figure; 
             subplot(3,1,1); plot(this.t,(this.X(:,1)-X_joint(:,1)));
             subplot(3,1,2); plot(this.t,(this.X(:,2)-X_joint(:,2)));
             subplot(3,1,3); plot(this.t,(this.X(:,3)-X_joint(:,3)));

%             get_ToolPathPlot(this);
%             get_toolPathMovie(this);

            % get x0 pos
%             pos = get_robotSinglePos(this,this.q0);
            
        end
        
        function importWAMData(this,filee)
           
%             data = importdata('20200909aft00.csv'); % Make sanity check fig for joint impedance

            data = importdata(filee);
              
            this.t = data(:,1);
            this.q = data(:,2:5);
            this.q_dot = data(:,6:9);
            this.X = data(:,10:12);
            this.X_dot = data(:,13:15);
            this.tau = data(:,16:19);
            this.F_pret= data(:,20:22);
%             figure; plot(1./diff(this.t));
            this.sfrq = 500; % Check this
            
            figure;
            subplot(3,1,1); plot(this.t,this.X(:,1)); hold on;
            subplot(3,1,2); plot(this.t,this.X(:,2)); hold on;
            subplot(3,1,3); plot(this.t,this.X(:,3)); hold on;
            
%             % Check import plot
            figure;
            subplot(2,3,1);
            plot(this.t, this.q);
            title('jointPositions');
            subplot(2,3,2);
            plot(this.t,this.q_dot);
            title('jointVelocities');
            subplot(2,3,3);
            plot(this.t, this.tau);
            title('JointTorqueController');
            
            subplot(2,3,4);
            plot(this.t,this.X);
            title('toolPositions');
            subplot(2,3,5);
            plot(this.t,this.X_dot);
            title('toolVelocities');
            subplot(2,3,6);
            plot(this.t,this.F_pret);
            title('ToolForceController');
            
            figure; 
            title('Raw Postion Data No Transformation');
            plot3(this.X(:,1),this.X(:,2),this.X(:,3)); hold on;
            plot3([0,0.1],[0,0],[0,0],'-r','linewidth',2);
            plot3([0,0],[0,0.1],[0,0],'-r','linewidth',2);
            plot3([0,0],[0,0],[0,0.1],'-r','linewidth',2);
            xlabel('x'); ylabel('y'); zlabel('z'); grid on; axis equal;
            xlim([-1 1]); ylim([-1 1]); zlim([-1 1]);
            
        end
        
        function trans = get_transformMatrix(this,q)
            % Generate tranformation matrix
            q = [q,0];
            a = [0,0,0.045,-0.045,0];
            alpha = [-pi/2,pi/2,-pi/2,pi/2,0];
            d = [0,0,0.4400,0,0.36];
            % d = [0,0,0.55,0,0.35];
            
            for i = 1:5
               trans{i}.T = [cos(q(i)), -sin(q(i))*cos(alpha(i)), sin(q(i))*sin(alpha(i)),  a(i)*cos(q(i));...
                       sin(q(i)),       cos(q(i))*cos(alpha(i)),  -cos(q(i))*sin(alpha(i)), a(i)*sin(q(i));...
                       0,               sin(alpha(i)),            cos(alpha(i)),            d(i);...
                       0,               0,                        0,                        1];
            end

        end
        
        function get_robotJointPlot(this)

                      
            for i = 1:length(this.q)
               trans = get_transformMatrix(this,this.q(i,:));
               tmp1 = this.W_Tranform*trans{1}.T; 
               tmp2 = this.W_Tranform*trans{1}.T*trans{2}.T;
               tmp3 = this.W_Tranform*trans{1}.T*trans{2}.T*trans{3}.T;
               tmp4 = this.W_Tranform*trans{1}.T*trans{2}.T*trans{3}.T*trans{4}.T;
               tmp5 = this.W_Tranform*trans{1}.T*trans{2}.T*trans{3}.T*trans{4}.T*trans{5}.T;
                      
               robotPos{i} = [tmp1(1:3,4)';tmp2(1:3,4)';tmp3(1:3,4)';tmp4(1:3,4)';tmp5(1:3,4)'];
               robotPos_end(i,:) = tmp5(1:3,4);
            end
            
            this.robotPos = robotPos;
            this.robotPos_end = robotPos_end;      
        end
        
        function pos = get_robotSinglePos(this,q)
                      
            trans = get_transformMatrix(this,q);
            tmp1 = this.W_Tranform*trans{1}.T;
            tmp2 = this.W_Tranform*trans{1}.T*trans{2}.T;
            tmp3 = this.W_Tranform*trans{1}.T*trans{2}.T*trans{3}.T;
            tmp4 = this.W_Tranform*trans{1}.T*trans{2}.T*trans{3}.T*trans{4}.T;
            tmp5 = this.W_Tranform*trans{1}.T*trans{2}.T*trans{3}.T*trans{4}.T*trans{5}.T;
            
            pos = [tmp1(1:3,4)';tmp2(1:3,4)';tmp3(1:3,4)';tmp4(1:3,4)';tmp5(1:3,4)'];
               
        end
        
        function import_WAM_model(this)
            
            this.robot = importrobot('WAM_dataFiles/wam_4dof.urdf');
            this.robot.DataFormat = 'column';
        end
        
        function check_stiffness(this)
            % Check joint stiffness
            figure;
            subplot(2,1,1); plot(this.t, this.tau./(this.q0-this.q),'o'); ylim([0 20]);
            legend('J_0','J_1','J_2','J_3'); ax1 = gca; ylabel('K (N/m)'); xlabel('Time (s)');
            set(gca,'fontsize',12);
            
            subplot(2,1,2);plot(this.t, this.tau,'o'); ax2 = gca;
            linkaxes([ax1, ax2],'x'); ylabel('\tau (Nm)'); xlabel('Time (s)');
            set(gca,'fontsize',12);
        end
        
        function get_ToolPathPlot(this)
            figure;
            plot3(this.X(:,1),this.X(:,2),this.X(:,3),'b','linewidth',2.5);hold on;
            plot3(this.robotPos_end(:,1),this.robotPos_end(:,2),this.robotPos_end(:,3),'-r','linewidth',2.5);
            plot3([0,0.25],[0,0],[0,0],'-r','linewidth',2.5);
            plot3([0,0],[0,0.25],[0,0],'-r','linewidth',2.5);
            plot3([0,0],[0,0],[0,0.25],'-r','linewidth',2.5);
            xlabel('x');ylabel('y'); zlabel('z'); grid on; axis equal;
            xlim([-1 1]); ylim([-1 1]); zlim([-1 1]);
                        
        end
        
        function get_toolPathMovie(this)
            videoFileName = '/Users/jhermus/Downloads/testVideo4DNew';
            figure;
            v = VideoWriter(videoFileName, 'MPEG-4');
            skipSamples = 200;
            v.FrameRate = this.sfrq/skipSamples;
            open(v);
            
            for i = 1:skipSamples:length(this.t)
                
%                 show(this.robot,this.q(i,:)'); hold on;
%                 view(-90,90);
                plot3(this.X(1:i,1),this.X(1:i,2),this.X(1:i,3),'-');hold on;
                plot3(this.X(i,1),this.X(i,2),this.X(i,3),'o','markersize',10);
                plot3(this.robotPos{i}(:,1),this.robotPos{i}(:,2),this.robotPos{i}(:,3),'.-b','linewidth',2.5,'markersize',25);
                plot3([0,0.25],[0,0],[0,0],'-r','linewidth',2.5);
                plot3([0,0],[0,0.25],[0,0],'-r','linewidth',2.5);
                plot3([0,0],[0,0],[0,0.25],'-r','linewidth',2.5);
                xlabel('x'); ylabel('y'); zlabel('z'); grid on; axis equal;
                xlim([-1 1]); ylim([-1 1]); zlim([-1 1]); hold off;
                
                frame = getframe(gcf);
                writeVideo(v,frame);
            end
            close(v);
            
            
        end
    end
end


% clear all
% close all
% clc
% 
% %% Test Transformation matrix
% 
% p0 = [0,0,0,1]';
% T10 = [rotx(pi),[0,0,0.5]';0,0,0,1];
% T21 = [rotx(pi/2),[0,0,0.2]';0,0,0,1];
% T32 = [rotx(pi/2),[0,0,0.2]';0,0,0,1];
% 
% p1 = T10*p0;
% p2 = T21*p1;
% p3 = T32*p2;
% 
% arm = [p0,p1,p2,p3];
% 
% figure; 
% plot3(arm(1,:),arm(2,:),arm(3,:),'.-','linewidth',2.5,'markersize',25); hold on;
% plot3([0,0.25],[0,0],[0,0],'-r','linewidth',2.5);
% plot3([0,0],[0,0.25],[0,0],'-r','linewidth',2.5);
% plot3([0,0],[0,0],[0,0.25],'-r','linewidth',2.5);
% xlabel('x'); ylabel('y'); zlabel('z'); grid on; axis equal;
% xlim([-1 1]); ylim([-1 1]); zlim([-1 1]); hold off;
