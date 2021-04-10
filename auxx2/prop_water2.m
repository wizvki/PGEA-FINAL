classdef prop_water2 < handle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    properties (SetAccess = private, GetAccess = private)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % system parameters
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        w;          % velocidade do motor
        kg = 30;    % motor gain
        
        r = 0.055/2; % propellers radius
        
        height = 0.27/2;
        
        % saturacoes de velocidade
        satW_air = saturation([0 4000]);
        satW_wat = saturation([0 4000]);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % environment parameters
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        env; % ambiente
        z;
        rho_air = 1.293;    % [kg/m^3]   air density
        rho_wat = 1.0e3;    % [kg/m^3]   water density
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % construtor
        function this = prop_water2(env, z)
            
            this.env = env; % enviroment
            
            % seta velocidade inicial do motor (comeca parado)
            this.w = 0;
            this.z = z;
            %[x, y, z] = feval(@(a)a{:}, num2cell(this.x([1 2 3])));
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % atualiza estado do motor
        function update(this, wdes, dt, env)
            
            % enviroment
            this.env = env;
            
            % valor desejado de velocidade depende do meio
            if isAir(this.env)
                wdes = this.satW_air.evaluate(wdes);
            else
                wdes = this.satW_wat.evaluate(wdes);
            end
            
            % equacao dinamica do motor
            dw = this.kg*(wdes - this.getSpeed());
            
            % integra o modelo do motor
            this.w = this.getSpeed() + dw*dt;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % forca gerada pelo motor
        function F = getForce(this)
            F = this.getKf()*this.getRho()*(this.getSpeed().^2);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % retorna velocidade de rotacao
        function w = getSpeed(this)
            w = this.w;
            if isnan(w)
                w = 0;
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % calcula coeficiente em funcao da velocidade
        function kf = getKf(this)
            
            % verifica o ambiente
            if isAir(this.env)
                p = [2.3281e-11  -1.4307e-07    0.0010654];
            else
                p = [2.3281e-11  -1.4307e-07    0.0010654];
            end
            cT = polyval(p, this.getSpeed());
            % calcula ganho da helice
            A = pi*(this.r.^2);
            kf = cT*A*(this.r.^2);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % pega densidade do ambiente
        function rho = getRho(this)
            if isAir(this.env)
                if (this.z  >= this.height)    
                    rho = this.rho_air;
                else
                    rho = (-(this.rho_wat - this.rho_air) / (2 * this.height))*(this.z - this.height) + this.rho_air;
                end
            else
                if (this.z <= -this.height)
                    rho = this.rho_wat;
                else
                    rho = (-(this.rho_wat - this.rho_air) / (2 * this.height))*(this.z - this.height) + this.rho_air;
                end
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function draw(this, R, p, pwh, type)
            
            n = 20; % resolution
            
            % cor da helice
            color = [0 1.0 0.2];
            
            % draw propeller
            if abs(this.getSpeed()) > 1
                alpha = .5; % rodando
            else
                alpha = .01; % parado
            end
            
            % desenha a helice
            if isempty(type)
                c = circle(this.r, n) + pwh;
            else
                c = roty(pi/2)*circle(this.r, n) + pwh;
            end
            [x, y, z] = feval(@(a)a{:}, num2cell(p));
            c = R*c + [x*ones(1,n); y*ones(1,n); z*ones(1,n)];
            %
            patch(c(1,:), c(2,:), c(3,:), color, 'FaceAlpha', alpha);
        end
    end
end