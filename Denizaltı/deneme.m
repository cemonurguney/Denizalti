clc;clear;

%%%%%%%% Gerçek State %%%%%%%%%

sub = submarine_model(20,10,5,3,0.5,0.1);

%%%%%%%% Noise Parametreleri %%%%%%%%%

noise_mean = [0;
              0;
              0;
              0;
              0;
              0];

noise_std = [1;
             1;
             0.5;
             0.2;
             0.02;
             0.02];

%%%%%%%% Sensor %%%%%%%%%

sensor = sensor_model(noise_mean,noise_std);

%%%%%%%% Ölçüm Sayısı %%%%%%%%%

N = 10000;

%%%%%%%% Ölçümler %%%%%%%%%

for i = 1:N

    sensor.measure(sub);

end

%%%%%%%% Mean Elle %%%%%%%%%

measurement_mean_manual = ...
    sum(sensor.measurement_history,2) / N;

%%%%%%%% Mean MATLAB %%%%%%%%%

measurement_mean_matlab = ...
    mean(sensor.measurement_history,2);

%%%%%%%% Variance Beklenen %%%%%%%%%

variance_expected = noise_std.^2;

%%%%%%%% Variance Elle %%%%%%%%%

difference = sensor.measurement_history ...
             - measurement_mean_manual;

difference_square = difference.^2;

variance_manual = ...
    sum(difference_square,2)/(N-1);

%%%%%%%% Variance MATLAB %%%%%%%%%

variance_matlab = ...
    var(sensor.measurement_history,0,2);

%%%%%%%% Sonuçlar %%%%%%%%%

disp("Gerçek state")
disp(sub.state)

disp("Elle mean")
disp(measurement_mean_manual)

disp("MATLAB mean")
disp(measurement_mean_matlab)

disp("Beklenen variance")
disp(variance_expected)

disp("Elle variance")
disp(variance_manual)

disp("MATLAB variance")
disp(variance_matlab)
%addfdfdfssdff