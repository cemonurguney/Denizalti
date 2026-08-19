classdef sensor_model < handle

    properties
        noise_mean
        noise_std = [1;
             1;
             0.5;
             0.2;
             0.02;
             0.02];
        R
        measurement
        measurement_history = [];
        imu_noise_mean
        imu_noise_std
        acceleration_true
        imu_measurement
    end

    methods
        function obj = sensor_model(noise_mean,noise_std)

            obj.noise_mean = noise_mean;
            obj.noise_std = noise_std;

            obj.R = diag(obj.noise_std.^2);
        
        end

        function z = measure(obj,sub)

            noise = obj.noise_mean + ...
                        obj.noise_std .* randn(6,1);

            obj.measurement = (sub.state) + noise;
            obj.measurement_history(:,end+1) = obj.measurement;
            
            z = obj.measurement;
        end

        function imu_init(obj,imu_noise_mean,imu_noise_std)

            obj.imu_noise_mean = imu_noise_mean;
            obj.imu_noise_std = imu_noise_std;

        end
        function a_imu = imu_measure(obj,sub,u) 
            %%%% burada gerçek ivme aldığımızı kabul ediyoruz ancak
            %%%% gerçekte imu body alır, body to ned ve gravity ile fc
            %%%% correction normalde gereklidir.
            v = sub.state(4);
            theta = sub.state(5);
            phi = sub.state(6);

            uv = u(1);
            utheta =u(2);
            uphi = u(3);

            %%%%% gercek ivme%%%%%%

            a_n = uv*cos(phi)*cos(theta)...
                -v*sin(phi)*cos(theta)*uphi...
                -v*cos(phi)*sin(theta)*utheta;

            a_e = uv*cos(phi)*sin(theta)...
                -v*sin(phi)*sin(theta)*uphi...
                +v*cos(phi)*cos(theta)*utheta;

            a_d = sin(phi)*uv + v*cos(phi)*uphi;


            obj.acceleration_true = [a_n;
                                     a_e;
                                     a_d];

            %%%%%%% imu  %%%%%%%

            noise = obj.imu_noise_mean ...
                + obj.imu_noise_std .* randn(3,1);
            obj.imu_measurement = obj.acceleration_true + noise;
            a_imu = obj.imu_measurement;
        end
    end

end