classdef kalman_filter < handle
    properties
        x_hat
        P
        Q
        R_imu
        R_position
        R_velocity
        std_dist = 0.1;
        Q_c
        Q_d
        
    end
    methods
        function obj = kalman_filter(x_init,P_init,R_imu,R_position,R_velocity)
            obj.x_hat = x_init;
            obj.P = P_init;
            obj.R_imu = R_imu;
            obj.R_position = R_position;
            obj.R_velocity = R_velocity;
        end

        function prediction(obj,a_imu,dt)

            F = [1 0 0 dt 0 0;
                 0 1 0 0 dt 0;
                 0 0 1 0 0 dt;
                 0 0 0 1 0 0;
                 0 0 0 0 1 0;
                 0 0 0 0 0 1];
        
            B = [0.5*dt^2 0 0;
                 0 0.5*dt^2 0;
                 0 0 0.5*dt^2;
                 dt 0 0;
                 0 dt 0;
                 0 0 dt];
        
            q = obj.std_dist^2;
        
            obj.Q_c = q*eye(3);
        
            obj.Q_d = [(dt^3/3)*obj.Q_c   (dt^2/2)*obj.Q_c;
                       (dt^2/2)*obj.Q_c   dt*obj.Q_c];
        
            obj.x_hat = F*obj.x_hat + B*a_imu;
        
            obj.P = F*obj.P*F' ...
                  + B*obj.R_imu*B' ...
                  + obj.Q_d;
        
        end

        function correction (obj,z_position,z_velocity)
            if  isnan(z_position) ~= true
                H = [1 0 0 0 0 0 ;
                 0 1 0 0 0 0 ;
                 0 0 1 0 0 0 ];
            
                
                y = z_position - H*obj.x_hat;
                S = H*obj.P*H' + obj.R_position;
                K = (obj.P*H')/S;
                obj.x_hat = obj.x_hat + K*y;
                obj.P = (eye(6,6) - K*H)*obj.P;
            end
            if  isnan(z_velocity) ~= true
                H = [0 0 0 1 0 0 ;
                    0 0 0 0 1 0 ;
                    0 0 0 0 0 1 ];
            
                
                y = z_velocity - H*obj.x_hat;
                S = H*obj.P*H' + obj.R_position;
                K = (obj.P*H')/S;
                obj.x_hat = obj.x_hat + K*y;
                obj.P = (eye(6,6) - K*H)*obj.P;
            end
        end

            
    end
end
