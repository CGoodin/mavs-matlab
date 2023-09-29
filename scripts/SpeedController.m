classdef SpeedController
    properties
        kp = 1.0
        ki = 0.0
        kd = 0.0
        setpoint = 0.0
        prev_err = 0.0
        integral = 0.0
        output = 0.0
        crossed_setpoint = false
    end

    methods
        function obj = SpeedController(kp, ki, kd)
            obj.kp = kp;
            obj.ki = ki;
            obj.kd = kd;
            obj.crossed_setpoint = false;
        end
        function obj = Update(obj, val)
            err = obj.setpoint - val;
            % prevent integrator windup
            if (err*obj.prev_err<0.0 && ~obj.crossed_setpoint) 
                obj.integral = 0.0;
                obj.crossed_setpoint = true;
            end
            obj.integral = obj.integral + err;
            derivative = (err - obj.prev_err);
            obj.prev_err = err;
            obj.output = obj.kp*err + obj.ki*obj.integral + obj.kd*derivative;
        end
    end
end