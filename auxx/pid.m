classdef pid < handle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    properties (SetAccess = private, GetAccess = private)
        % ganhos de controle
		Kp = 1;
		Ti = inf;
		Td = 0;
		
		% referencia
		rn = 0.0
		
		% erros
		ePn1  = 0.0
		eDfn1 = 0.0
		eDfn2 = 0.0
		
		% saida de controle
		umin = -inf;
		umax = inf;
		un = 0.0;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % construtor
        function this = pid(Kp, Ti, Td, ulimits)
            this.Kp = Kp;
            this.Ti = Ti;
            this.Td = Td;
            
            this.reset();
            
            this.umin = ulimits(1);
            this.umax = ulimits(2);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function reset(this)
            % erros
            this.ePn1  = 0.0;
            this.eDfn1 = 0.0;
            this.eDfn2 = 0.0;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % atualiza estado do motor
        function u = getU(this, rn, yn, dt)
            
            % reference
            this.rn = rn;
            
            % parametros default do algoritmo
            BETA  = 1.0;
            ALPHA = 0.1;
            GAMMA = 0.0;

            % tempo de amostragem
            Ts = dt;

            % Proportional error with reference weighing
            ePn = (BETA*this.rn) - yn;

            % Calculates error
            en = this.rn - yn;

            % Calculates filter time
            Tf = ALPHA*this.Td;
            if (Tf ~= 0.0) && (Ts ~= -Tf)
                % Calculates derivate error
                eDn = (GAMMA*this.rn) - yn;
                % Filters the derivate error
                eDfn = (this.eDfn1/((Ts/Tf) + 1)) + (eDn*(Ts/Tf)/((Ts/Tf) + 1));
            else
                eDfn = 0.0;
            end

            % delta de atuacao Proporcional
            delta_un = this.Kp*(ePn - this.ePn1);

            % termo integral
            if (this.Ti ~= 0.0)
                delta_un = delta_un + this.Kp*(Ts/this.Ti)*en;
            end

            % termo derivativo
            if (Ts ~= 0.0)
                delta_un = delta_un + this.Kp*(this.Td/Ts)*(eDfn - 2*this.eDfn1 + this.eDfn2);
            end

            % incrementa saida
            this.un = this.un + delta_un;

            % integrator anti-windup logic
            if this.un > this.umax
                this.un = this.umax;
            end
            if this.un < this.umin
                this.un = this.umin;
            end

            % update indexed values
            this.ePn1 = ePn;
            this.eDfn2 = this.eDfn1;
            this.eDfn1 = eDfn;
            
            % retorna a nova saida			
            u = this.un;
        end
    end
end