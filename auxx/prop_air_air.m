classdef prop_air_air < handle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    properties (SetAccess = private, GetAccess = private)
        
        % propellers superior e inferior
        prop_up, prop_dw;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % system parameters
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        l;  % braco do motor
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % environment parameters
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        env; % ambiente
        rho_air = 1.293;    % [kg/m^3]   air density
        rho_wat = 1.0e3;    % [kg/m^3]   water density
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % construtor
        function this = prop_air_air(l, env)
            
            this.l = l; % braco do motor
            
            this.env = env; % enviroment
            
            % cria as duas propellers
            this.prop_up = prop_air(env);
            this.prop_dw = prop_air(env);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % atualiza estado do motor
        function update(this, wdes, dt, env)
            
            % enviroment
            this.env = env;
            
            % update propellers
            if isAir(this.env)
                this.prop_up.update(wdes, dt, env);
                this.prop_dw.update(wdes, dt, env); %0
            else
                this.prop_up.update(-wdes, dt, env); %0
                this.prop_dw.update(-wdes, dt, env);
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % retorna velocidade de rotacao
        function w = getSpeed(this)            
            if isAir(this.env)
                w = this.prop_up.getSpeed();
            else
                w = -this.prop_dw.getSpeed();
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % forca gerada pelo motor
        function F = getForce(this)
            if isAir(this.env)
                F = this.prop_up.getForce();
            else
                F = -this.prop_dw.getForce();
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % momento gerado pelo motor - [Nm/rpm^2] moment gain;
        function M = getMoment(this)
            M = this.l*this.getForce();
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % calcula coeficiente em funcao da velocidade
        function kf = getKf(this)
            if isAir(this.env)
                kf = this.prop_up.getKf();
            else
                kf = this.prop_dw.getKf();
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function draw(this, R, p, pup, pdw, type)
            this.prop_up.draw(R, p, pup);
            this.prop_dw.draw(R, p, pdw); 
        end
    end
end