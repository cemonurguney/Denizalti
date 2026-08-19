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

    end

end