classdef ca_3d < handle

    properties
        state %pn pe pd, vn ve vd
        time = 0;
    end

    methods
        function obj = ca_3d(position_n,position_e,position_d, ...
                            velocity_n,velocity_e,velocity_d)
            obj.state = [position_n;position_e;position_d;...
                        velocity_n;velocity_e;velocity_d];
            
        end

        function update(obj,u, dt)% input girecek (ivme)
            
            A = [1 0 0 dt 0 0; %pn
                0 1 0 0 dt 0; %pe
                0 0 1 0 0 dt; %pd
                0 0 0 1 0 0; %vn
                0 0 0 0 1 0; %ve
                0 0 0 0 0 1]; %vd

            B = [0.5*dt^2 0 0; % an to pn
                0 0.5*dt^2 0; % ae to pe
                0 0 0.5*dt^2; % ad to pd
                dt 0 0; % an to vn
                0 dt 0; % ae to ve
                0 0 dt]; % ad to vd
            obj.state = A* obj.state + B*u;
            
            obj.time = obj.time + dt;
        end
    end
end