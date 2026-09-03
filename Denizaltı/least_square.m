classdef least_square < handle
    %Least Squares ve Kalman Filter yöntemleri aynı sensör verileri
    %kullanılarak karşılaştırılmıştır. LS yönteminde sonlu bir zaman
    %penceresi içerisindeki GPS ve DVL ölçümleri, sistem modeli ve IMU
    %verileri kullanılarak durum kestirimi gerçekleştirilmiştir. Kalman
    %Filter ise ölçüm ve proses belirsizliklerini kovaryans matrisleri
    %üzerinden dikkate alarak ardışık kestirim yapmaktadır. Elde edilen
    %sonuçlarda LS yöntemi konum durumlarında yaklaşık 0.82–2.15 m, hız
    %durumlarında ise 0.09–0.23 m/s RMSE üretirken, Kalman Filter aynı
    %koşullarda konum için yaklaşık 0.13–0.44 m ve hız için 0.027–0.077 m/s
    %RMSE değerlerine ulaşmıştır. Bu sonuçlar, Kalman Filter'ın sensör
    %gürültülerini ve model belirsizliğini istatistiksel olarak hesaba
    %katması nedeniyle LS yöntemine göre daha başarılı kestirim sağladığını
    %göstermektedir. Bununla birlikte LS yöntemi daha basit bir yapıyla
    %durum kestirimi gerçekleştirebilmiş ve Kalman Filter için
    %karşılaştırmalı bir referans yöntem olarak kullanılmıştır.
    properties
        x_hat
        buffer_imu = zeros(5000,3);
        imu_count=0;
        buffer_position
        buffer_position_step
        buffer_velocity;
        buffer_velocity_step
        F
        B

    end
    methods
        function obj = least_square(initialState)
            obj.x_hat = initialState;
        end
        function prediction(obj,a_imu,dt)
            obj.F = [eye(3),eye(3).*dt;
                zeros(3) , eye(3)];
            obj.B = [eye(3).* (0.5*dt^2);
                eye(3).*dt];

            obj.x_hat = obj.F*obj.x_hat + obj.B*a_imu;
        end
        function add_imu(obj,a_imu)
            obj.imu_count = obj.imu_count + 1 ;
            if obj.imu_count <= 5000
                obj.buffer_imu(obj.imu_count,:) = a_imu';
            else
                obj.buffer_imu(1:end-1, :) = obj.buffer_imu(2:end, :);
                obj.buffer_imu(end, :) = a_imu';
            end
        end
        function add_position(obj,z_position)

            if ~any(isnan(z_position))
        
                obj.buffer_position(end+1,:) = z_position';
                obj.buffer_position_step(end+1,1) = obj.imu_count;
            end
        
        end
        function add_velocity(obj,z_velocity)

            if ~any(isnan(z_velocity))
        
                obj.buffer_velocity(end+1,:) = z_velocity';
                obj.buffer_velocity_step(end+1,1) = obj.imu_count;
        
            end
        
        end
        function solve(obj)

            A = [];
            Y = [];
        
            Phi = eye(6);
            c = zeros(6,1);
        
        
            %%%%% Window start %%%%%
        
            if obj.imu_count > 5000
                start_step = obj.imu_count - 5000;
            else
                start_step = 0;
            end
        
        
            %%%%% Eski DVL measurementlarini sil %%%%%
        
            while ~isempty(obj.buffer_velocity_step) && ...
                    obj.buffer_velocity_step(1,:) < start_step
        
                obj.buffer_velocity(1,:) = [];
                obj.buffer_velocity_step(1,:) = [];
        
            end
        
        
            %%%%% Eski GPS measurementlarini sil %%%%%
        
            while ~isempty(obj.buffer_position_step) && ...
                    obj.buffer_position_step(1,:) < start_step
        
                obj.buffer_position(1,:) = [];
                obj.buffer_position_step(1,:) = [];
        
            end
        
        
            %%%%% Window start aninda GPS var mi? %%%%%
        
            index_position = find( ...
                obj.buffer_position_step == start_step);
        
            if ~isempty(index_position)
        
                z_position = ...
                    obj.buffer_position(index_position,:)';
        
                H_position = [eye(3) zeros(3)];
        
                Y_position = z_position - H_position*c;
                A_position = H_position*Phi;
        
                A = [A;
                     A_position];
        
                Y = [Y;
                     Y_position];
        
            end
        
        
            %%%%% Window start aninda DVL var mi? %%%%%
        
            index_velocity = find( ...
                obj.buffer_velocity_step == start_step);
        
            if ~isempty(index_velocity)
        
                z_velocity = ...
                    obj.buffer_velocity(index_velocity,:)';
        
                H_velocity = [zeros(3) eye(3)];
        
                Y_velocity = z_velocity - H_velocity*c;
                A_velocity = H_velocity*Phi;
        
                A = [A;
                     A_velocity];
        
                Y = [Y;
                     Y_velocity];
        
            end
        
        
            %%%%% IMU buffer boyunca ilerle %%%%%
        
            N_imu = min(obj.imu_count,5000);
        
            for i = 1:N_imu
        
                step_i = start_step + i;
        
                a = obj.buffer_imu(i,:)';
        
                Phi = obj.F*Phi;
                c = obj.F*c + obj.B*a;
        
                index_position = find(obj.buffer_position_step == step_i);
                index_velocity = find(obj.buffer_velocity_step == step_i);
                if ~isempty(index_position)
                    z_position = obj.buffer_position(index_position,:)';
                    
                    H_position = [eye(3) zeros(3)];
                    Y_position = z_position - H_position*c;
                    A_position = H_position*Phi; A = [A; A_position];
                    Y = [Y; Y_position]; 
                end 
                if ~isempty(index_velocity)
                    z_velocity = obj.buffer_velocity(index_velocity,:)';
                    H_velocity = [zeros(3) eye(3)];
                    Y_velocity = z_velocity - H_velocity*c;
                    A_velocity = H_velocity*Phi;
                    A = [A; A_velocity];
                    Y = [Y; Y_velocity];
        
                end
        
            end
            if size(A,1) < 6
                return
            end
            if rank(A) < 6
                return
            end
            
            x_start_hat = A\Y;
            
            obj.x_hat = Phi*x_start_hat + c;
        end
    end
end
           