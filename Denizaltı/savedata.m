classdef savedata < handle

    properties
        position_n_history = [];
        position_e_history = [];
        position_d_history = [];

        speed_history = [];
        theta_history = [];
        phi_history = [];

        u_v_history = [];
        u_theta_history = [];
        u_phi_history = [];

        reference_n_history = [];
        reference_e_history = [];
        reference_d_history = [];
        
        %%%%%%%% kalmansal %%%%%%%%%%%

        x_true_history = [];
        x_hat_history = [];
        z_position_history = [];

        error_history = [];
        P_history = [];
        p_diag_history = [];

        time_history = [];
        reference = [];

        
    end


    methods 

        function obj = savedata(reference)
            obj.reference = reference;
        end


        function record(obj,sub,u,target,kalman,z_position)
            
            obj.time_history(end+1) = sub.time;


            %%%%%%%%%%%%% position %%%%%%%%%%%%%%%%%

            obj.position_n_history(end+1) = sub.state(1);
            obj.position_e_history(end+1) = sub.state(2);
            obj.position_d_history(end+1) = sub.state(3);


            %%%%%%%%%%%%% v-theta-phi %%%%%%%%%%%%%%

            obj.speed_history(end+1) = sub.state(4);
            obj.theta_history(end+1) = sub.state(5);
            obj.phi_history(end+1) = sub.state(6);


            %%%%%%%%%%%%% input %%%%%%%%%%%%%%%%%%%%

            obj.u_v_history(end+1) = u(1);
            obj.u_theta_history(end+1) = u(2);
            obj.u_phi_history(end+1) = u(3);


            %%%%%%%%%%%%% reference %%%%%%%%%%%%%%%%

            obj.reference_n_history(end+1) = target(1);
            obj.reference_e_history(end+1) = target(2);
            obj.reference_d_history(end+1) = target(3);

            %%%%%%%%%%%%% Kalman Ground Truth %%%%%%%%%%%%%

            v = sub.state(4);
            theta = sub.state(5);
            phi = sub.state(6);
            
            vN = v*cos(phi)*cos(theta);
            vE = v*cos(phi)*sin(theta);
            vD = v*sin(phi);
            
            x_true = [sub.state(1:3);
                      vN;
                      vE;
                      vD];
            
            obj.x_true_history(:,end+1) = x_true;
            
            %%%%%%%%%%%%% Kalman Estimate %%%%%%%%%%%%%%%%%
            
            obj.x_hat_history(:,end+1) = kalman.x_hat;
            
            %%%%%%%%%%%%% Position Measurement %%%%%%%%%%%%
            
            obj.z_position_history(:,end+1) = z_position;

            %%%%%%%%%%%%% P and error %%%%%%%%%%%%%%%%%%%%%%
            obj.error_history(:,end+1) = x_true - kalman.x_hat;
            obj.p_diag_history(:,end+1) = diag(kalman.P);

            


        end
        function P_kiyas(obj)
            sigma = sqrt(obj.p_diag_history);
            state_names = ["N","E","D","vN","vE","vD"];

            figure(6)

            for i= 1:6
                subplot(2,3,i)

                plot(obj.time_history,obj.error_history(i,:))
                hold on
                plot(obj.time_history,3*sigma(i,:),"--")
                plot(obj.time_history,-3*sigma(i,:),"--")
                yline(0)
                hold off
        
                xlabel("time(s)")
                ylabel("error")
                title(state_names(i))
                legend("Error","+3\sigma","-3\sigma")
                grid on
            end
            figure(7)
            plot(obj.time_history,obj.p_diag_history)
        end

                    


        function show(obj)

            %%%%%%%%%%%%% position %%%%%%%%%%%%%%%%%

            figure(1)

            subplot(2,3,1)
            plot(obj.time_history,obj.position_n_history)
            hold on
            plot(obj.time_history,obj.reference_n_history,"--")
            hold off
            xlabel("time(s)")
            ylabel("position n(m)")


            subplot(2,3,2)
            plot(obj.time_history,obj.position_e_history)
            hold on
            plot(obj.time_history,obj.reference_e_history,"--")
            hold off
            xlabel("time(s)")
            ylabel("position e(m)")


            subplot(2,3,3)
            plot(obj.time_history,obj.position_d_history)
            hold on
            plot(obj.time_history,obj.reference_d_history,"--")
            hold off
            xlabel("time(s)")
            ylabel("position d(m)")


            %%%%%%%%%%%%% v-theta-phi %%%%%%%%%%%%%%

            subplot(2,3,4)
            plot(obj.time_history,obj.speed_history)
            xlabel("time(s)")
            ylabel("speed(m/s)")


            subplot(2,3,5)
            plot(obj.time_history, mod(obj.theta_history*180/pi, 360));
            xlabel("time(s)")
            ylabel("theta(rad)")


            subplot(2,3,6)
            plot(obj.time_history, mod(obj.phi_history*180/pi, 360));
            xlabel("time(s)")
            ylabel("phi(rad)")


            %%%%%%%%%%%%% input %%%%%%%%%%%%%%%%%%%%%

            figure(2)

            subplot(1,3,1)
            plot(obj.time_history,obj.u_v_history)
            xlabel("time(s)")
            ylabel("u v(m/s^2)")


            subplot(1,3,2)
            plot(obj.time_history,mod(obj.theta_history*180/pi, 360));
            xlabel("time(s)")
            ylabel("u theta(rad/s)")


            subplot(1,3,3)
            plot(obj.time_history, mod(obj.phi_history*180/pi, 360));
            xlabel("time(s)")
            ylabel("u phi(rad/s)")

        end


        function threedshow(obj)

            figure(3)

            plot3(obj.position_n_history, ...
                  obj.position_e_history, ...
                  obj.position_d_history)

            hold on

            plot3(obj.reference(1,:), ...
                  obj.reference(2,:), ...
                  obj.reference(3,:),"--o")

            hold off

            xlabel("North (m)")
            ylabel("East (m)")
            zlabel("Down (m)")

            title("Submarine Trajectory")

            grid on
            axis equal

        end
        function kalmanshow(obj)

            figure(4)
        
            %%%%%%%%%%%%% Position %%%%%%%%%%%%%
        
            subplot(2,3,1)
            plot(obj.time_history,obj.x_true_history(1,:))
            hold on
            plot(obj.time_history,obj.z_position_history(1,:))
            plot(obj.time_history,obj.x_hat_history(1,:))
            hold off
            xlabel("time(s)")
            ylabel("North(m)")
            legend("True","Measured","Estimated")
            grid on
        
            subplot(2,3,2)
            plot(obj.time_history,obj.x_true_history(2,:))
            hold on
            plot(obj.time_history,obj.z_position_history(2,:))
            plot(obj.time_history,obj.x_hat_history(2,:))
            hold off
            xlabel("time(s)")
            ylabel("East(m)")
            legend("True","Measured","Estimated")
            grid on
        
            subplot(2,3,3)
            plot(obj.time_history,obj.x_true_history(3,:))
            hold on
            plot(obj.time_history,obj.z_position_history(3,:))
            plot(obj.time_history,obj.x_hat_history(3,:))
            hold off
            xlabel("time(s)")
            ylabel("Down(m)")
            legend("True","Measured","Estimated")
            grid on
        
            %%%%%%%%%%%%% Velocity %%%%%%%%%%%%%
        
            subplot(2,3,4)
            plot(obj.time_history,obj.x_true_history(4,:))
            hold on
            plot(obj.time_history,obj.x_hat_history(4,:))
            hold off
            xlabel("time(s)")
            ylabel("vN(m/s)")
            legend("True","Estimated")
            grid on
        
            subplot(2,3,5)
            plot(obj.time_history,obj.x_true_history(5,:))
            hold on
            plot(obj.time_history,obj.x_hat_history(5,:))
            hold off
            xlabel("time(s)")
            ylabel("vE(m/s)")
            legend("True","Estimated")
            grid on
        
            subplot(2,3,6)
            plot(obj.time_history,obj.x_true_history(6,:))
            hold on
            plot(obj.time_history,obj.x_hat_history(6,:))
            hold off
            xlabel("time(s)")
            ylabel("vD(m/s)")
            legend("True","Estimated")
            grid on

            figure(5)
            plot3(obj.x_hat_history(1,:), ...
                  obj.x_hat_history(2,:), ...
                  obj.x_hat_history(3,:))
            hold on 
            plot3(obj.x_true_history(1,:), ...
                  obj.x_true_history(2,:), ...
                  obj.x_true_history(3,:),"--")

            hold off
            xlabel("North (m)")
            ylabel("East (m)")
            zlabel("Down (m)")

            title("Submarine Kalman vs True Trajectory")
            legend("x_hat","X_true")
            grid on
            axis equal


            

        
        end
        function animate_with_axes_kalman(obj, simSpeed, axLen)
            if nargin<2, simSpeed = 1; end
            if nargin<3, axLen = 5; end
            if isempty(obj.time_history) || isempty(obj.x_hat_history), return; end
        
            % time and ensure row vector
            t = obj.time_history(:)';
            N = numel(t);
        
            % x_hat_history: assume size = (stateRows x samples)
            X = obj.x_hat_history;
            [nStates, nSamples] = size(X);
        
            % If samples mismatch time, interpolate along time
            if nSamples ~= N
                tx = linspace(t(1), t(end), nSamples);
                X = interp1(tx, X', t, 'linear', 'extrap')';
            end
        
            % positions from first 3 rows
            xn = X(1,:);
            xe = X(2,:);
            xd = X(3,:);
        
            % velocities assumed last 3 rows
            if nStates >= 6
                vN = X(end-2,:);
                vE = X(end-1,:);
                vD = X(end,:);
                yaw = atan2(vE, vN);
                pitch = atan2(vD, sqrt(vN.^2 + vE.^2));
            else
                % fallback to stored angles (interpolated to t if needed)
                if numel(obj.theta_history) == N
                    yaw = obj.theta_history(:)';
                else
                    yaw = interp1(obj.time_history, obj.theta_history, t, 'previous', 'extrap');
                end
                if numel(obj.phi_history) == N
                    pitch = obj.phi_history(:)';
                else
                    pitch = interp1(obj.time_history, obj.phi_history, t, 'previous', 'extrap');
                end
            end
        
            % figure setup
            figure(10); clf;
            plot3(obj.position_n_history, obj.position_e_history, obj.position_d_history, ':', 'Color',[0.7 0.7 0.7]); hold on;
            hLine = plot3(xn, xe, xd, '-b');
            hPoint = plot3(xn(1), xe(1), xd(1), 'ro', 'MarkerFaceColor','r');
            if ~isempty(obj.reference)
                try plot3(obj.reference(1,:), obj.reference(2,:), obj.reference(3,:), '--k'); end
            end
            xlabel('North (m)'); ylabel('East (m)'); zlabel('Down (m)');
            axis equal; grid on; view(3);
        
            % initial quivers
            hN = quiver3(xn(1), xe(1), xd(1), axLen, 0, 0, 'r', 'LineWidth',2, 'MaxHeadSize',0.5);
            hE = quiver3(xn(1), xe(1), xd(1), 0, axLen, 0, 'g', 'LineWidth',2, 'MaxHeadSize',0.5);
            hD = quiver3(xn(1), xe(1), xd(1), 0, 0, axLen, 'b', 'LineWidth',2, 'MaxHeadSize',0.5);
        
            % playback loop (wall-clock sync)
            t0 = t(1);
            startWall = tic;
            for k = 1:N
                targetWall = (t(k) - t0) / simSpeed;
                waitTime = targetWall - toc(startWall);
                if waitTime > 0, pause(waitTime); end
        
                % rotation from yaw and pitch (adjust convention if needed)
                cy = cos(yaw(k)); sy = sin(yaw(k));
                cp = cos(pitch(k)); sp = sin(pitch(k));
                R = [ cy*cp, -sy,  cy*sp;
                      sy*cp,  cy,  sy*sp;
                     -sp,     0,   cp   ];
        
                n_vec = (R * [1;0;0]) * axLen;
                e_vec = (R * [0;1;0]) * axLen;
                d_vec = (R * [0;0;1]) * axLen;
        
                set(hPoint, 'XData', xn(k), 'YData', xe(k), 'ZData', xd(k));
                set(hN, 'XData', xn(k), 'YData', xe(k), 'ZData', xd(k), 'UData', n_vec(1), 'VData', n_vec(2), 'WData', n_vec(3));
                set(hE, 'XData', xn(k), 'YData', xe(k), 'ZData', xd(k), 'UData', e_vec(1), 'VData', e_vec(2), 'WData', e_vec(3));
                set(hD, 'XData', xn(k), 'YData', xe(k), 'ZData', xd(k), 'UData', d_vec(1), 'VData', d_vec(2), 'WData', d_vec(3));
        
                drawnow limitrate;
            end
        end

    end

end