classdef submarine_model < handle

    properties
        state %[N,E,D,V,THETA,PHİ
        time =0;
    end

    methods 
        function obj = submarine_model(N,E,D,v,theta,phi)

            obj.state = [N;E;D;v;theta;phi];
        end

        function update(obj,u,dt)

            N = obj.state(1);
            E = obj.state(2);
            D = obj.state(3);
            v = obj.state(4);
            theta = obj.state(5);
            phi = obj.state(6);
        
            uv = u(1);
            utheta = u(2);
            uphi = u(3);
        
            N = N + v*cos(phi)*cos(theta)*dt;
            E = E + v*cos(phi)*sin(theta)*dt;
            D = D + v*sin(phi)*dt;
        
            v = v + uv*dt;
            theta = theta + utheta*dt;
            phi = phi + uphi*dt;
            
            
            obj.state = [N;E;D;v;theta;phi];
        
            obj.time = obj.time + dt;
        
        end
    end
end

