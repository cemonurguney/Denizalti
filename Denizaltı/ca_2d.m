classdef ca_2d < handle

    properties
        state %state 1 konum, state 2 hız, ivme input gibi giricek
        time = 0;
    end

    methods
        function obj = ca_2d(position,velocity)
            obj.state = [position;velocity];
            
        end

        function update(obj,u, dt)% input girecek (ivme)
            A = [1 dt;
                0 1];
            B = [0.5*dt^2; dt];
            obj.state = A* obj.state + B*u;
            
            obj.time = obj.time + dt;
        end
    end
end