classdef kalman_filter < handle
    properties
        x_hat
        P
        Q
        R_imu
        R_position
        std_dist = 0.02;
    end
    methods
        function obj = kalman_filter(x_init,P_init,R_imu,R_position)
            obj.x_hat = x_init;
            obj.P = P_init;
            obj.R_imu = R_imu;
            obj.R_position = R_position;
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
            R_dist = obj.std_dist^2 * eye(3);            
            obj.Q = B*R_dist*B';
            obj.x_hat = F*obj.x_hat + B*a_imu;
            obj.P = F*obj.P*F' +B*obj.R_imu*B'+obj.Q;

        end

        function correction (obj,z)
            H = [1 0 0 0 0 0 ;
                 0 1 0 0 0 0 ;
                 0 0 1 0 0 0 ];

            y = z - H*obj.x_hat;
            S = H*obj.P*H' + obj.R_position;
            K = (obj.P*H')/S;
            obj.x_hat = obj.x_hat + K*y;
            obj.P = (eye(6,6) - K*H)*obj.P;
        end

    end
end
