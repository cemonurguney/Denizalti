classdef input_params < handle

    properties
        position_error
        theta_ref
        horizontal
        phi_ref 
        v_ref = 3;

        pid_v
        pid_theta
        pid_phi

        u
    end

    methods

        function obj = input_params(sub,reference,k)

            obj.ref_update(sub,reference,k);

        end


        function ref_update(obj,sub,reference,k)

            obj.position_error = reference(:,k) - sub.state(1:3);

            obj.theta_ref = atan2(obj.position_error(2), ...
                                  obj.position_error(1));

            obj.horizontal = sqrt(obj.position_error(2)^2 + ...
                                  obj.position_error(1)^2);

            obj.phi_ref = atan2(obj.position_error(3), ...
                                obj.horizontal);

        end


        function error_theta = theta_error(obj,sub)

            error_theta = atan2( ...
                sin(obj.theta_ref-sub.state(5)), ...
                cos(obj.theta_ref-sub.state(5)));

        end


        function pid_init(obj,sub)

            obj.pid_v = PIDcontroller(1.0,0.05,0.10, ...
                obj.v_ref-sub.state(4));


            error_theta = obj.theta_error(sub);

            obj.pid_theta = PIDcontroller(1.5,0.02,0.20, ...
                error_theta);


            obj.pid_phi = PIDcontroller(1.5,0.02,0.20, ...
                obj.phi_ref-sub.state(6));

        end


        function u = pid_update(obj,sub,dt)

            error_v = obj.v_ref - sub.state(4);

            error_theta = obj.theta_error(sub);

            error_phi = obj.phi_ref - sub.state(6);


            uv = obj.pid_v.update(error_v,dt);

            utheta = obj.pid_theta.update(error_theta,dt);

            uphi = obj.pid_phi.update(error_phi,dt);


            u = [uv;
                 utheta;
                 uphi];

            obj.u = u;

        end

    end

end