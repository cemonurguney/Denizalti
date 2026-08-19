clc;clear;

%%%%%%%% Initials %%%%%%%%%

sub = submarine_model(0,0,0,0,315*180/pi,0);

dt = 0.01;
T = 1000;
k = 1;

%%%%%%%% Reference %%%%%%%%%

ref = reference_motion();
ref.edit("kare",20);
reference = ref.reference;


%%%%%%%% PID %%%%%%%%%

input_parameter = input_params(sub,reference,k);
input_parameter.pid_init(sub);

%%%%%%%% Save Data %%%%%%%%%

savedat = savedata(reference);

%%%%%%%% Simulation %%%%%%%%%

for t = 0:dt:T-dt

    %%%%% Pozisyon hatası ve referansların güncellenmesi %%%%%
    input_parameter.ref_update(sub,reference,k);

    %%%%% PID ve kontrol girişleri %%%%%
    u = input_parameter.pid_update(sub,dt);

    %%%%% Submarine %%%%%
    sub.update(u,dt);

    %%%%% Save Data %%%%%
    savedat.record(sub,u,reference(:,k));

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

%%%%%%%% Simulation Time %%%%%%%%%

sure = savedat.time_history(end);
fprintf("Simülasyon tamamlandı. Süre = %.2f sn\n",sure)