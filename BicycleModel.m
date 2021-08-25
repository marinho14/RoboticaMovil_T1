classdef BicycleModel 
    %   BicycleModel kinematic model
    %   Calculates forward and inverse kinematics for a differntial drive
    %   robot

    properties
        % Wheel radius in meters [m]
        WheelRadius  = 1.0;
        % Distance from wheel to wheel in meters [m]
        Wheelbase    = 1.0;
    end
    
    methods
        function obj = BicycleModel(wheelRadius, wheelbase)
            %DiffDrive Class Constructor
            % Inputs: 
            %   wheelRadius: wheel radius [m]
            %   trackWidth: 
            % Assign parameters
            obj.WheelRadius = wheelRadius;
            obj.Wheelbase   = wheelbase;
        end
        
        function [r_xi] = calcForwardKinematics(obj, wr, delta) 
            %CALCFORWARDKINEMATICS Calculates forward kinematics
            % Inputs:
            %    wr   : rear wheel speed [rad/s]
            %    delta: Angle in radians [rad/s]
            % Outputs:
            %    r_xi : [vx; 0 ; wz] robot velocity vector in robot base frame                        
            %       vx: robot speed in robot frame [m/s]
            %       wz: robot angular vel in robot frame [rad/s]            
            
            vx = obj.WheelRadius * wr;
            wz = wr*obj.WheelRadius* tan(delta) / obj.Wheelbase;
            r_xi = [vx;0;wz];
        end
        
        function [wr, delta] = calcInverseKinematics(obj, r_xi)
            %CALCINVERSEKINEMATICS Calculates forward kinematics
            % Inputs:
            %    r_xi : [vx; 0 ; wz] robot velocity vector in robot base frame             
            %       vx: robot linear speed in robot frame  [m/s]
            %       wz: robot angular speed in robot frame  [rad/s]          
            % Outputs:
            %       wr: right wheel speed [rad/s]
            %       delta: Angle in radians [rad/s]
            
            
            vx = r_xi(1);
            wz = r_xi(3);

            wr = vx/obj.WheelRadius;
            delta = atan2(wz*obj.Wheelbase,vx);          
            
        end   
    end
end