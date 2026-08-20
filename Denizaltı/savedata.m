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

        x_true_history = [];
        x_hat_history = [];
        z_position_history = [];

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
    end

end