classdef ca_2d_ne < handle

    properties
        state %state 1 konum, state 2 hız, ivme input gibi giricek
        time = 0;
    end

    methods
        function obj = ca_2d_ne(position_n,position_e,velocity_n,velocity_e)
            obj.state = [position_n;position_e;velocity_n;velocity_e];
            
        end

        function update(obj,u, dt)% input girecek (ivme)
            A = [1 0 dt 0; %pn
                0 1 0 dt; %pe
                0 0 1 0;
                0 0 0 1];
            B = [0.5*dt^2 0;
                0 0.5*dt^2;
                dt 0;
                0 dt];
            obj.state = A* obj.state + B*u;
            
            obj.time = obj.time + dt;
        end
    end
end