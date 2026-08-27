clc;clear;

%%%%%%%% Initials %%%%%%%%%

sub = submarine_model(0,0,0,0,0,0);
dt = 0.01;
T = 1000;
k = 1;

%%%%%%%% Reference %%%%%%%%%

ref = reference_motion();
ref.edit("kup",20);
reference = ref.reference;


%%%%%%%% PID %%%%%%%%%

input_parameter = input_params(sub,reference,k);
input_parameter.pid_init(sub);

%%%%%%%% Save Data %%%%%%%%%

savedat = savedata(reference);

%%%%%%%% Sensor %%%%%%%%%

noise_mean = [0;
              0;
              0];

noise_std = [0.03;
             0.03;
             0.05];

sensor = sensor_model(noise_mean,noise_std);
sensor.imu_init(noise_mean,noise_std);
P = [0.25 0 0 0 0 0;
     0 0.25 0 0 0 0;
     0 0 0.04 0 0 0;
     0 0 0 0.25 0 0;
     0 0 0 0 0.25 0;
     0 0 0 0 0 0.25];
z_position = sensor.position_measure(sub,100); 
x_init(1:3,1) = z_position;
x_init(4:6,1) = 0;
kalman = kalman_filter(x_init,P,sensor.R_imu,sensor.R_position);

for t = 0:dt:T-dt

    %%%%% Pozisyon hatası ve referansların güncellenmesi %%%%%
    input_parameter.ref_update(sub,reference,k);

    %%%%% PID ve kontrol girişleri %%%%%
    u = input_parameter.pid_update(sub,dt);
    
    %%%% gerçek ivmeyi imuya dönüştürme %%%%%%
    a_imu = sensor.imu_measure(sub,u);
    %%%%%%%%% predict x_hat%%%%%%%%%%%% 100 hz imu
    kalman.prediction(a_imu,dt);

    %%%%% Submarine %%%%%
    sub.update(u,dt);
    %%%%% pozisyon ölçümü %%%%%%
    z_position = sensor.position_measure(sub,dt);
    %%%%%%%%%%correction %%%%%%%
    %  %%%% 1hz gps update
    kalman.correction(z_position);
    %%%%% Save Data %%%%%

    savedat.record(sub,u,reference(:,k),kalman,z_position);

    %%%%% Waypoint kontrolü %%%%%
    if norm(input_parameter.position_error) < 0.5

        k = k+1;

        if k > size(reference,2)
            break
        end

    end

end

%%%%%%%% Plot %%%%%%%%%

savedat.show()
savedat.threedshow()
savedat.kalmanshow()
savedat.P_kiyas()
%savedat.animate_with_axes_kalman(7,5)

%%%%%%%% Simulation Time %%%%%%%%%

sure = savedat.time_history(end);
fprintf("Simülasyon tamamlandı. Süre = %.2f sn\n",sure)