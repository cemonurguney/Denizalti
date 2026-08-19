classdef PIDcontroller < handle
    properties
        Kp
        Ki
        Kd
    end

    properties(SetAccess=private)
        previous_error 
        integral_error = 0;
    end

    methods
        function obj = PIDcontroller(Kp,Ki,Kd,initial_error)
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;
            obj.previous_error = initial_error;
        end

        function u = update(obj, error, dt)
            
            derivative_error= (error - obj.previous_error)/dt;
            obj.integral_error = obj.integral_error + error*dt;
            u = obj.Kp * error ...
                + obj.Kd * derivative_error...
                + obj.Ki * obj.integral_error;

            obj.previous_error = error ;
            
        end
    end
end

