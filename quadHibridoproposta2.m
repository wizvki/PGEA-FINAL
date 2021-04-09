
classdef quadHibridoproposta2 < handle
    %aq1 % descrição do arquivo %propriedades privado
    properties (SetAccess = private, GetAccess = private)
        %aq2% descricao quando bota na janela de comando 'help quadHibridoproposta'
        % system parameters
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % ROSA, et al. A Comparative Study on Sigma Point Kalman Filters for 
        % Trajectory Estimation of Hybrid Aerial-Aquatic Vehicles. IROS`18.
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%ate aq
        m = (.9)*1.42887;        % [kg] uav mass
        % momentos de inercia
        I = [   144648.98   2041.46     -7870.33
                2041.46     288179.61   -1157.20
                -7870.33    -1157.20    154104.84]*1e-7;
        %
        vol = (1.1)*1.42887e-03;  % Volume [m^3]
        l = 0.27;   % [m] wing span
        rc = 0.27/4;   % raio do centro
        contraroting_dist = .1;     % [m] altura
        
        Cp = diag([2.5; 2.5; 9.99])*1e-2;    % coeficiente de arrasto de translacao
        Cr = diag([1.25; 1.25; 1.25*2])*1e-2;    % coeficiente de arrasto de rotacao
        %%%%%%%%%%%calculado depois
%         Cp = diag([1.723; 1.723; 2.568])*1e-2;    % coeficiente de arrasto de translacao
%         Cr = diag([0.9477; 0.9477; 1.8954])*1e-2;    % coeficiente de arrasto de rotacao
        
        maxAngAir = deg2rad(30); % [rad]
        maxAngWat = deg2rad(65); % [rad]
        satAngAir;      % saturacao de angulo de referencia
        satAngWat;      % saturacao de angulo de referencia
        
        satRot = saturation(deg2rad(179)*[-1; 1]); % satura rotacao pra evitar singularidade em R()
        
        mot = {}; % motores
        % direcionameto dos motores inferiores
        alfa = deg2rad([0; 0; 0; 0]);
        beta = deg2rad([0; 90; 0; 90]);
        
        % controladores
        pidZ;
        pidAtt;
        
        cor; % cor de plot
        
        waypoint_dist = 1;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % environment parameters
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        env;    % variavel de ambiente
        rho_air = 1.293;    % [kg/m^3]   air density
        rho_wat = 1.0e3;    % [kg/m^3]   water density
        g = [0; 0; 9.78];   % gravidade [m/s^2]
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % tempo
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        dt = (1/500);   % 100Hz de frequencia interna
        t = 0;          % relogio
    end
    %propriedades publico%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    properties (SetAccess = public)
        x;      % vetor de estados
        ref;    % referencias de posicao e angulo
        u;      % vetor de entradas
        F = zeros(4,1); % forças
        M = zeros(4,1); % momentos
        forca1 = zeros(3,1);
        forca2 = zeros(3,1);
        forca3 = zeros(3,1);
        forca4 = zeros(3,1);
        forca5 = zeros(3,1);
        at=zeros(3,1); %aceleracao translalcao
        
        hyst;   % historico das variaveis
    end
    
    methods (Access = private)
        function [dwF, dv, angref] = controllerPos2(this, ref)
        % controlador de posicao
            
            % referencias de posicao
            [xref, yref, zref, psiref] = feval(@(a)a{:}, num2cell(ref));
            
            % medicoes
            [x, y, z, psi, vx, vy] = feval(@(a)a{:}, num2cell(this.x([1 2 3 6 7 8])));
            
            % calcula e satura sinais de referencia angular (***so roll e pitch***)
            angref = zeros(3,1);
            
            % calcula o angulo de direcao do controle
            dx = xref - x;
            dy = yref - y;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % controle no ar
            if isAir(this.env)
                
                % ganhos de posicao
                kp =  0.05;
                kd =  0.11;
                
                % acao de controle
                R = rotz(psi);
                R = R(1:2,1:2);
                dp = R*[dy; dx];
                dv = -R*[vy; vx];
                angref = kp*[-1; 1].*dp + kd*[-1; 1].*dv;
                angref = this.satAngAir.evaluate(angref);
                
                % referencia de yaw
                angref(3) = psiref;
                %
                dv = 0;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % controle na agua
            else
                % erro de posicao
                d = norm([dx, dy], 2);

                % comando de velocidade
                if d > this.waypoint_dist
                    
                    % erro angular: sempre entre [0...2pi]
                    th = atan2(dy,dx);
                    th = mod(th, 2*pi);
                    
                    % alpha entre [-pi..pi]
                    alpha = th - psi;
                    while alpha > pi
                        alpha = alpha - 2*pi;
                    end
                    while alpha < -pi
                        alpha = alpha + 2*pi;
                    end
                    
                    kv = -300;
                    ka = 1e6;
                    
                    %if abs(rad2deg(alpha)) > 35
                    %    dv = 0;
                    %else 
                        dv = kv*d + ka*abs(alpha)^2;
                    %end
                    % satura
                    dv = max(dv, -4000);
                    dv = min(dv, 0);
                else
                    dv = 0;
                    th = psi;
                end
                
                % referencia aponta para waypoint
                angref(3) = th;
            end
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % acao de controle de altitude
            if isAir(this.env)
                dwF = this.pidZ.air.getU(zref, z, this.dt);
            else
                dwF = this.pidZ.wat.getU(zref, z, this.dt);
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function dw = controllerAtt2(this, rref)
        % controlador de atitude
            
            r = this.x(4:6); % angulos medidos
            dw = zeros(3,1); % vetor de atuacao
            
            % ganhos de orientacao conforme o meio
            if isAir(this.env)
                for i = 1:3
                    dw(i) = this.pidAtt.air{i}.getU(rref(i), r(i), this.dt);
                end
            else
                % alpha entre [-pi..pi]
                alpha = rref(3) - r(3);
                while alpha > pi
                    alpha = alpha - 2*pi;
                end
                while alpha < -pi
                    alpha = alpha + 2*pi;
                end
                dw(2) = this.pidAtt.wat{1}.getU(rref(2), r(2), this.dt);
                dw(3) = this.pidAtt.wat{2}.getU(0, -alpha, this.dt);
            end
        end
        
         %% calcula atuacao nas helices
         function actuator2(this, dwF, dv, dw, z)
            %% rotacao de equilibrio
            % calculado aq ->
            % <matlab:matlab.desktop.editor.openDocument(which('quadHibridoproposta.m')).goToFunction('equilibrium') 
            % equilibrium(this)>
            wh = this.equilibrium(); 
            
            %% calcula as velocidades desejadas
            if isAir(this.env)
                dv = 0;  
                % * linha = motor;
                % * coluna = quais motores são acionados quando
                %realizam o movimento correspondente para helice aerea
                %
                %       z  xy roll pitch  yaw
                D = [   1   0    0   -1    1;
                        1   0    1    0   -1;
                        1   0    0    1    1;
                        1   0   -1    0   -1  ];
            else
                % * linha = motor;
                % * coluna = quais motores são acionados quando
                %realizam o movimento correspondente para helice aquatica
                %       z  xy roll pitch  yaw
                D = [   1   0   0   -1    0;
                        0   1   0    0    -1;
                        1   0   0    1    0;
                        0   1   0    0    1];
            end
            
            %% acao de controle
            % * w = velocidade desejada 
            % * wh = velocidade para rotação de equilíbrio calculado por aq -> 
            % <matlab:matlab.desktop.editor.openDocument(which('quadHibridoproposta.m')).goToFunction('equilibrium')
            % equilibrium(this)>
            % * dwF = acao de controle de altitude calculado 
            % <matlab:matlab.desktop.editor.openDocument(which('quadHibridoproposta.m')).goToFunction('controllerPos')
            % controllerPos(this, ref)>
            % * dv = velocidade linear calculado aq -> 
            % <matlab:matlab.desktop.editor.openDocument(which('quadHibridoproposta.m')).goToFunction('controllerPos')
            % controllerPos(this, ref)>
            % * dw = velocidade angular; calculado aq -> 
            % <matlab:matlab.desktop.editor.openDocument(which('quadHibridoproposta.m')).goToFunction('controllerAtt')
            % controllerAtt(this, rref)>
            w = [(wh+dwF); dv; dw];
                
            %% velocidades desejadas para os motores
            % aplica as velocidades desejadas para os motores
            % correspondentes ao movimento
            %wdes = D*w;
            wdes = [0; 0; 0; 0];
            
            %% atualiza o estado dos motores
            % a função update(this, wdes, dt, env) com entradas 
            % (velocidade desejada nos motores correspondentes, 
            % frequencia interna e tipo de ambiente) está na função aqui -> 
            % <matlab:matlab.desktop.editor.openDocument(which('prop_air_water.m')).goToFunction('update')
            % update(this, wdes, dt, env)>
            %
            % retorna o estado dos motores em relaçaõ ao ambiente (como
            % por exemplo, se esta no ar aciona os motores aéreas e
            % não aciona os motores aquáticos e vice versa nesse caso)
            %
            % Acesse o codigo da função aqui -> 
            % <matlab:edit(fullfile('prop_air_water.m')) prop_air_water.m> 
            %
            % Veja  <matlab:doc('prop_air_water') aqui> a janela de HELP do
            % codigo.
            for i = 1:4
                this.mot{i}.update(wdes(i), this.dt, this.env, z);
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% rotacao dos motores no ponto de equilibrio (tipo hover)
        function wh = equilibrium(this)
            
            % medicoes de angulos
            [phi, the] = feval(@(a)a{:}, num2cell(this.x(4:5)));
            
            %% efeito da inclinacao
            % os angulos são referentes a diagonal da matriz de rotacao
            % angular (eu acho n lembro) aq->
            % <matlab:matlab.desktop.editor.openDocument(which('quadHibridoproposta.m')).goToFunction('B')
            % B(this)>
            if isAir(this.env)
                alpha = cos(phi)*cos(the);
            else
                alpha = cos(the);
            end
            % evita divisao por zero!
            if alpha < 0.1
                alpha = 0.1;
            end
            
            %% gravidade
            % $$ g = |m - \rho  vol| \cdot ||g|| $$ 
            % 
            %gravidade = módulo (massa - densidade do ambiente * volume) * norma matricial da gravidade 
            grav_term = abs(this.m - this.getRho2()*this.vol)*norm(this.g, 2);
            
            %% ganho do motor
            % é calculada (ou só definida) coeficiente referente ao motor (ou helice?) aqui: 
            %
            % * se esta acionado os motores aereos ->
            % <matlab:matlab.desktop.editor.openDocument(which('prop_air.m')).goToFunction('getKf')
            % getKf(this)>
            % * se esta acionado os motores aquaticos ->
            % <matlab:matlab.desktop.editor.openDocument(which('prop_water.m')).goToFunction('getKf')
            % getKf(this)>
            % 
            % Acesse o codigo da função aqui -> 
            % <matlab:edit(fullfile('prop_air.m')) prop_air.m> ou 
            % <matlab:edit(fullfile('prop_water.m')) prop_water.m> 
            %
            % Veja  <matlab:doc('prop_air') prop_air.m> ou <matlab:doc('prop_water')
            % prop_water.m> a janela de HELP do codigo.
            kf = this.mot{1}.getKf(); 
            
            %% velocidade para rotação de equilíbrio
            %
            % * para ar:  $wh = \frac{\sqrt{\frac{g}{4 \rho kf}}}{\alpha}$  
            % * para agua:  $wh = \frac{\sqrt{\frac{g}{2 \rho kf}}}{\alpha}$
            %
            % velocidade = sqrt((gravidade)/(4*densidade do
            % ambiente*coeficiente do motor))/angulo do efeito de
            % inclinação 
            if isAir(this.env)
                wh =  sqrt(abs(grav_term)/(4*this.getRho2()*kf))/alpha;
            else
                wh = -sqrt(abs(grav_term)/(2*this.getRho2()*kf))/alpha;
            end
        end
        
        %% calcula aceleracao baseado no modelo
        function [at, aa] = getAccel2(this, v, q, r)
            
            %% variaveis do ambiente
            % é calculada (ou só definida) densidade do ambiente de acordo
            % com o meio que se encontra aqui:
            % * se esta acionado os motores aereos ->
            % <matlab:matlab.desktop.editor.openDocument(which('prop_air.m')).goToFunction('getRho')
            % getRho(this)>
            % * se esta acionado os motores aquaticos ->
            % <matlab:matlab.desktop.editor.openDocument(which('prop_water.m')).goToFunction('getRho')
            % getRho(this)>
            % 
            rho = this.getRho2();
            
            % massa adicionais --> "George Green 1833"
            added_mass = ((4/6)*pi*(this.rc^3))*rho;
            % inercia adicionais
            added_inertia = (2/5)*(added_mass)*(this.rc^2);
            
            %% Posicao
            if isAir(this.env)
                % helices superiores nao sao direcionadas
                f(:,1) = this.R()*[0; 0; sum(this.F)];         
            else
                f(:,1) = this.R()*[-sum(this.F([2 4])); 0; sum(this.F([1 3]))];         
            end
            
            %% gravidade
            % $$f_2 = - m g$$
            f(:,2) = -this.m*this.g; 
            
            %% arquimedes (empuxo)
            % $$f_3 = \rho vol g$$
            f(:,3) = rho*this.vol*this.g;          
            
            %% arrasto
            % $$ f_4 = - \frac{1}{2}\rho C_p v |v|$$
            f(:,4) = -(1/2)*rho*this.Cp*v.*abs(v); 
            
            %% coriolis
            % $$f_5 = - m v \times q$$
            f(:,5) = -cross(q, this.m*v);           
            
            %% aceleracao translacional
            % $$a_t = \frac{\sum_{i=1}^5 f_i}{m + m_a}$$
            at = sum(f,2)/(this.m + added_mass);
            this.forca1 = f(:,1);
            this.forca2 = f(:,2);
            this.forca3 = f(:,3);
            this.forca4 = f(:,4);
            this.forca5 = f(:,5);
            %% Atitude
            % $$P = mg$$
            %
            % $$E = \rho V g$$
            peso = norm(this.m*this.g, 2);
            empuxo = norm(rho*this.vol*this.g, 2);
            
            % distancia entre cg e empuxo
            dist = 0.02; 
            
            %% motores
            % verifica o ambiente que o veícuulo está para determinar
            % quais motores são considerados em cada eixo do sistema
            if isAir(this.env)
                %se está no ambiente aéreo 
                m(:,1) = [  this.l*(this.F(2)-this.F(4))
                            this.l*(this.F(3)-this.F(1))
                            this.M(1)-this.M(2)+this.M(3)-this.M(4) ];  
            else
                %senão, está no ambiente aquático
                m(:,1) = [  0
                            this.M(3)-this.M(1)
                            -this.M(2)+this.M(4) ];  
            end
            
            %% coriolis
            % $$F_{coriolis} = - \omega \times I\omega$$
            m(:,2) = -cross(q, this.I*q); 
            
            %% arrasto
            % $$ F_{arrasto} = -\frac{1}{2}\rho C_r \omega |\omega| $$
            m(:,3) = -(1/2)*rho*this.Cr*q.*abs(q); 
            
            %% momento restaurador (passivo)
            if ~isAir(this.env)
                m(:,4) = -[ dist*sin(r(1))*(peso+empuxo)
                            dist*sin(r(2))*(peso+empuxo)
                            0];                                         
            end
            
            %% calcula a aceleracao angular
            % $$a_angular = \frac{I + I_adicional}{\sum F}$$
            %
            aa = (this.I + added_inertia)\sum(m,2);
        end
        
        %% modelo dinamico do sistema
        function dynamics2(this)
            
            % atualiza o estado dos motores
            for i = 1:4
                this.F(i) = this.mot{i}.getForce();
                this.M(i) = this.mot{i}.getMoment();
            end
            
            %% variaveis de estado
            % p: posição x, y, z
            % r: orientação phi, theta, psi (ou pitch roll yaw)
            % v: velocidade linear v_x, v_y, v_z
            % q: velocidade angular \omega_x, \omega_y, \omega_z
            p = this.x(1:3);
            r = this.x(4:6);
            v = this.x(7:9);
            q = this.x(10:12);
            
            %% Runge-Kutta
            % integração da equação de aceleração obtida em
            % <matlab:matlab.desktop.editor.openDocument(which('quadHibridoproposta.m')).goToFunction('getAccel')
            % getAccel(this, v, q, r)>
            % utilizando Meétodo de Runge-Kutta de quarta ordem
            %p1 = p
            v1 = v;
            q1 = q;
            r1 = r;
            [at1, aa1] = this.getAccel2( v1, q1, r1);
            
            v2 = v + 0.5*at1*this.dt;
            q2 = q + 0.5*aa1*this.dt;
            r2 = this.satAngles(r + 0.5*q1*this.dt);
            [at2, aa2] = this.getAccel2(v2, q2, r2);
            
            v3 = v + 0.5*at2*this.dt;
            q3 = q + 0.5*aa2*this.dt;
            r3 = this.satAngles(r + 0.5*q2*this.dt);
            [at3, aa3] = this.getAccel2(v3, q3, r3);

            v4 = v + at3*this.dt;
            q4 = q + aa3*this.dt;
            r4 = this.satAngles(r + 0.5*q3*this.dt);
            [at4, aa4] = this.getAccel2(v4, q4, r4);
            
            %% aceleração translacional
            this.at=(at1 + 2*at2 + 2*at3 + at4)/6;
            
            %% velocidade translacional
            v = v + (at1 + 2*at2 + 2*at3 + at4)*(this.dt/6.0);
            %% posicao
            p = p + (v1 + 2*v2 + 2*v3 + v4)*(this.dt/6.0);
            %% velocidade rotacional
            q = q + (aa1 + 2*aa2 + 2*aa3 + aa4)*(this.dt/6.0);
            %% angulos
            r = r + (this.B()) \ ((q1 + 2*q2 + 2*q3 + q4)*(this.dt/6.0));
            %% satura angulos para evitar singularidades em this.R()
            r = this.satAngles(r);
            
            %faixa de velocidade para zerar (faz isso por causa da falta de precisao)
%             for i =1:3
%                 if (v(i)>=-0.001)&&(v(i)<=0.001)
%                     v(i)=0;
%                 end
%             end
            
            %% update estados
            this.x(1:3)   = p;
            this.x(4:6)   = r;
            this.x(7:9)   = v;
            this.x(10:12) = q;
            
            %% incrementa relogio interno
            this.t = this.t + this.dt;
            
            %% atualiza ambiente
            this.setEnvironment();
            
            %% update saidas
            for i = 1:4
                this.u(i) = this.mot{i}.getSpeed();
            end
        end
        %% atualiza a variavel de ambiente
        function setEnvironment(this)
            %se a variavel z>0 é ar
            %senão é água
            if this.x(3) >= 0
                this.env = 'air';
            else
                this.env = 'water';
            end
        end
        %% matriz de rotacao
        function Rzyx = R(this)
            %% Matriz rotacional em torno do eixo X
            % R_x = [1 0 0 \\
            %        0 cos(phi) -sin(phi)\\
            %        0 sin(phi) cos(phi)] 
            Rx = rotx(this.x(4));
            %% Matriz rotacional em torno do eixo X
            % R_y = [cos(theta)  0  sin(theta)\\
            %                 0  1          0 \\
            %        -sin(theta) 0  cos(theta)] 
            Ry = roty(this.x(5));
            %% Matriz rotacional em torno do eixo X
            % R_z = [cos(psi) -sin(psi)  0 \\
            %        sin(psi)  cos(psi)  0\\
            %              0          0  1] 
            Rz = rotz(this.x(6));
            %% Matriz rotacional ZYX
            % R 
            Rzyx = Rz*Ry*Rx;
        end
        %% satura angulos para evitar singularidades
        function ang = satAngles(this, ang)
            ang(1:2) = this.satRot.evaluate(ang(1:2));
            ang(3) = rem(ang(3), 2*pi);
        end
        
        %% Matriz rotacional para velocidade angular
        function Bot = B(this)
            cphi = cos(this.x(4));
            sphi = sin(this.x(4));
            cthe = cos(this.x(5));
            sthe = sin(this.x(5));
            % R_z = [cos(theta)  0  -cos(phi)sin(theta) \\
            %               0    1             sin(phi) \\
            %        sin(theta)  0  cos(phi)cos(theta)] 
            r1 = cthe;  r2 = 0; r3 = -cphi*sthe;
            r4 = 0;     r5 = 1; r6 = sphi;
            r7 = sthe;  r8 = 0; r9 = cphi*cthe;
            
            Bot = [r1 r2 r3; r4 r5 r6; r7 r8 r9];
        end
    end
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods (Access = public)
        % construtor
        function this = quadHibridoproposta2(x, cor)
            
            % saturacoes de angulos
            this.satAngAir = saturation(this.maxAngAir*[-1; 1]);
            this.satAngWat = saturation(this.maxAngWat*[-1; 1]);
            
            % condicoes iniciais
            this.x = x;
            this.ref = this.x(1:6);
            
            % define o ambiente inicial
            this.setEnvironment();
            
            % inicializa os motores
            for m = 1:4
                this.mot{m} = prop_air_water2(this.l, this.env, x(3));
                this.u(m) = this.mot{m}.getSpeed();
            end
            
            % historicos
            this.hyst.x = this.x(:);
            this.hyst.u = this.u(:);
            this.hyst.F = this.F(:);
            this.hyst.forca1 = this.forca1(:);
            this.hyst.forca2 = this.forca2(:);
            this.hyst.forca3 = this.forca3(:);
            this.hyst.forca4 = this.forca4(:);
            this.hyst.forca5 = this.forca5(:);
            this.hyst.ref = this.ref(:);
            this.hyst.t = this.t;
            this.hyst.at = this.at(:);
            
            this.cor = cor;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % PIDS (air) -- OK
            satz = 1000;
            satang = 3000;
            % z
            this.pidZ.air = pid(0, 0, 0, satz*[-1, 1]);
            % roll and pitch
            for i = 1:2
                this.pidAtt.air{i} = pid(0, 0, 0, satang*[-1, 1]);
            end
            % yaw
            this.pidAtt.air{3} = pid(0, 0, 0, satang*[-1, 1]);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % PIDS (water)
            satz = 1000;
            satang = 3000;
            % z
            this.pidZ.wat = pid(0, 0, 0, satz*[-1, 1]);
            % pitch e yaw
            for i = 1:2
                this.pidAtt.wat{i} = pid(0, 0, 0, satang*[-1, 1]);
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % update do modelo
        function update(this, ref, x)
            
            % controle de posicao
            [dwF, dv, rref] = this.controllerPos2(ref);
            
            % controle de atitude
            dw = this.controllerAtt2(rref);
            
            % calcula a atuacao
            this.actuator2(dwF, dv, dw, x(3));
            
            % dinamica
            this.dynamics2();
            
            % historicos
            posref = ref(1:3);
            this.ref = [posref; rref];
            this.saveTraj();
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % salva trajetoria
        function saveTraj(this)
            this.hyst.x(:,end+1)    = this.x(:);
            this.hyst.u(:,end+1)    = this.u(:);
            this.hyst.F(:,end+1)    = this.F(:);
            this.hyst.forca1(:,end+1)= this.forca1(:);
            this.hyst.forca2(:,end+1)= this.forca2(:);
            this.hyst.forca3(:,end+1)= this.forca3(:);
            this.hyst.forca4(:,end+1)= this.forca4(:);
            this.hyst.forca5(:,end+1)= this.forca5(:);
            this.hyst.ref(:,end+1)  = this.ref(:);
            this.hyst.t(end+1)      = this.t;
            this.hyst.at(:,end+1)   = this.at(:);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % verifica se o robo chegou
        function c = reach(this)
            
            c = false;
            
            [x, y, z, psi] = feval(@(a)a{:}, num2cell(this.x([1 2 3 6])));
            [xr, yr, zr, psir] = feval(@(a)a{:}, num2cell(this.ref));
            
            % calcula distancia para waypoint corrent
            d = norm([xr yr zr]-[x y z], 2);
            if d <= this.waypoint_dist
                c = true;
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % retorna o tempo de simulacao
        function t = time(this)
            t = this.t;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % pega densidade do ambiente
        function rho = getRho2(this)
            if isAir(this.env)
                if (this.x(3)  >= 0.135)    
                    rho = this.rho_air;
                else
                    rho = -4993.535*this.x(3) + 500.6465;
                end
            else
                if (this.x(3) <= -0.135)
                    rho = this.rho_wat;
                else
                    rho = -4993.535*this.x(3) + 500.6465 ;
                end
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % desenha o quadrotor
        function draw(this)
            
            color = 'k'; % body color
            p = this.x(1:3);
            R = this.R(); % Monta matrizes de rotacao
            
            % shafts
            s{1} = this.l*[  1;  0; 0];
            s{2} = this.l*[  0; -1; 0];
            s{3} = this.l*[ -1;  0; 0];
            s{4} = this.l*[  0;  1; 0];

            % Aplica transformacoes: Body
            for i = 1:4
                b{i} = R*s{i} + p(:);
            end
            
            % draw shafts
            plot3([b{1}(1) b{3}(1)], [b{1}(2) b{3}(2)], [b{1}(3) b{3}(3)], color, 'linewidth', 2); 
            hold on;
            plot3([b{2}(1) b{4}(1)], [b{2}(2) b{4}(2)], [b{2}(3) b{4}(3)], color, 'linewidth', 2);

            % desenha propulsor
            for i = 1:4
                sp{i} = s{i} + [0; 0; (this.contraroting_dist/2)];
                sp{i+4} = s{i} - (this.contraroting_dist/2)*[   cos(this.alfa(i))*sin(this.beta(i)); 
                                                                sin(this.alfa(i))*sin(this.beta(i));  
                                                                cos(this.beta(i))];
                % define o tipo do propulsor
                if mod(i, 2)
                    type = [];
                else
                    type = ['proposta'];
                end
                this.mot{i}.draw(R, p, sp{i}, sp{i+4}, type);
                %
                sp{i} = R*sp{i} + p(:);
                sp{i+4} = R*sp{i+4} + p(:);
                %
                plot3([sp{i}(1) b{i}(1)], [sp{i}(2) b{i}(2)], [sp{i}(3) b{i}(3)], ...
                    color, 'linewidth', 2);
                plot3([b{i}(1) sp{i+4}(1)], [b{i}(2) sp{i+4}(2)], [b{i}(3) sp{i+4}(3)], ...
                    color, 'linewidth', 2);
                
            end
            
            % draw body
            this.body();
            
            % plota trajetoria
            ns = 10;
            z = this.hyst.x(3,:);
            id1 = find(z >= 0);
            id2 = find(z < 0);
            % trajetoria aerea
            plot3(this.hyst.x(1,id1), this.hyst.x(2,id1), this.hyst.x(3,id1), ...
                '-', 'Color', .7*[1 0 0], 'linewidth', 2);
            % trajetoria aquatica
            plot3(this.hyst.x(1,id2), this.hyst.x(2,id2), this.hyst.x(3,id2), ...
                '-', 'Color', .7*[1 0 1], 'linewidth', 2);
            
            xlabel('x[m]')
            ylabel('y[m]')
            zlabel('z[m]')
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function body(this)
            
            % configuração da esfera
            [unitSphereX, unitSphereY, unitSphereZ] = sphere(8);

            % localização do plot das esferas
            sphereX = this.x(1) + unitSphereX*this.rc;
            sphereY = this.x(2) + unitSphereY*this.rc;
            sphereZ = this.x(3) + 1*unitSphereZ*this.rc;
            h = surface(sphereX, sphereY, sphereZ);

            % cor preta
            colormap(.5*[.8 .8 1]);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % plota resultados
        function plot(this, plots, tf)
            
            % resultados
            t = this.hyst.t;
            ref = this.hyst.ref;
            p = this.hyst.x(1:3,:);
            r = this.hyst.x(4:6,:);
            v = this.hyst.x(7:9,:);
            q = this.hyst.x(10:12,:);
            w = this.hyst.u;
            F = this.hyst.F;
            forca1 = this.hyst.forca1;
            forca2 = this.hyst.forca2;
            forca3 = this.hyst.forca3;
            forca4 = this.hyst.forca4;
            forca5 = this.hyst.forca5;
            at = this.hyst.at;
            %fim=1.5;
            fim=tf;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if (plots(1) == 1)
                figure(30)
                labels = {'x', 'y', 'z'};
                for i = 1:3
                    subplot(3,2,2*i-1)
                    plot(t, p(i,:), 'Color', this.cor, 'linewidth', 1); hold on;
                    %plot(t, ref(i,:), '--', 'Color', this.cor, 'linewidth', 1); hold on;
                    ylabel(['$$' labels{i} '$$ [m.]'], 'Interpreter','latex')
                    xlim([t(1) fim])
                    box off;
                    xlabel('time [sec.]')
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%
                labels = {'\phi', '\theta', '\psi'};
                sp = [2 1 3];
                for i = 1:3
                    subplot(3,2,2*sp(i))
                    plot(t, rad2deg(r(i,:)), 'Color', this.cor, 'linewidth', 1); hold on;
                    %plot(t, rad2deg(ref(3+i,:)), '--', 'Color', this.cor, 'linewidth', 1); hold on;
                    ylabel(['$$' labels{i} '$$ [deg.]'], 'Interpreter','latex')
                    xlim([t(1) fim])
                    box off;
                    xlabel('time [sec.]')
                end
            end

            % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if (plots(2)==1)
                figure(31);
                labels = {'v_x', 'v_y', 'v_z'};
                for i = 1:3
                    figure(31),set(gca,'FontSize',18);
                    subplot(3,2,2*i-1)
                    plot(t, v(i,:), 'Color', this.cor, 'linewidth', 1); hold on;
                    ylabel(['$$' labels{i} '$$ [m./sec.]'], 'Interpreter','latex')
                    xlim([t(1) fim])
                    box off;
                    xlabel('time [sec.]')
                end
                % %%%%%%%%%%%%%%%%%%%%%%%%%%%%
                labels = {'p', 'q', 'r'};
                for i = 1:3
                    figure(31),set(gca,'FontSize',18);
                    subplot(3,2,2*i)
                    plot(t, rad2deg(q(i,:)), 'Color', this.cor, 'linewidth', 1); hold on;
                    ylabel(['$$' labels{i} '$$ [deg./sec.]'], 'Interpreter','latex')
                    xlim([t(1) fim])
                    box off;
                    xlabel('time [sec.]')
                end
            end

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if (plots(3)== 1)
                figure(32)
                for i = 1:4
                    subplot(4,1,i)
                    plot(t, w(i,:), 'Color', this.cor, 'linewidth', 1); 
                    hold on;
                    ylabel('$$\vec{\Omega}$$ [rpm]', 'Interpreter','latex')
                    xlim([t(1) fim])
                    box off;
                    xlabel('time [sec.]')
                end
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if (plots(4)== 1)
                figure(33)
                for i = 1:4
                    subplot(4,1,i)
                    plot(t, F(i,:), 'Color', this.cor, 'linewidth', 1); 
                    hold on;
                    ylabel('$$\vec{F}$$ [N]', 'Interpreter','latex')
                    xlim([t(1) fim])
                    box off;
                    xlabel('time [sec.]')
                end
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            labels = {'f_x', 'f_y', 'f_z'};
            if (plots(5)== 1)
                figure(34)
                for i = 1:3
                    subplot(3,1,i)
                    plot(t, forca1(i,:), 'Color', 'b', 'linewidth', 1); 
                    hold on;
                    plot(t, forca2(i,:), 'Color', 'g', 'linewidth', 1); 
                    hold on;
                    plot(t, forca3(i,:), 'Color', 'r', 'linewidth', 1); 
                    hold on;
                    plot(t, forca4(i,:), 'Color', 'c', 'linewidth', 1); 
                    hold on;
                    plot(t, forca5(i,:), 'Color', 'k', 'linewidth', 1); 
                    ylabel(['$$' labels{i} '$$ [N]'], 'Interpreter','latex')
                    xlim([t(1) t(end)])
                    xlim([t(1) fim])
                    box off;
                    xlabel('time [sec.]')
                end
                legend('motor','peso','empuxo','arrasto','coriolis');
            end
            if (plots(6)== 1)
                figure(35)
                subplot(3,1,1)
                plot(t, p(3,:), 'Color', this.cor, 'linewidth', 1); hold on;
                ylabel(['$$z$$ [m.]'], 'Interpreter','latex')
                xlim([t(1) fim])
                box off;
                xlabel('time [sec.]')
                subplot(3,1,2)
                plot(t, v(3,:), 'Color', this.cor, 'linewidth', 1); hold on;
                ylabel(['$$v_z$$ [m./sec.]'], 'Interpreter','latex')
                xlim([t(1) fim])
                box off;
                xlabel('time [sec.]')
                subplot(3,1,3)
                plot(t, forca1(3,:), 'Color', 'b', 'linewidth', 1); 
                hold on;
                plot(t, forca2(3,:), 'Color', 'g', 'linewidth', 1); 
                hold on;
                plot(t, forca3(3,:), 'Color', 'r', 'linewidth', 1); 
                hold on;
                plot(t, forca4(3,:), 'Color', 'c', 'linewidth', 1); 
                hold on;
                plot(t, forca5(3,:), 'Color', 'k', 'linewidth', 1); 
                ylabel(['$$f_z$$ [N]'], 'Interpreter','latex')
                xlim([t(1) t(end)])
                xlim([t(1) fim])
                box off;
                xlabel('time [sec.]')
                legend('motor','peso','empuxo','arrasto','coriolis');
            end
            if (plots(7)== 1)
                figure(36),set(gca,'FontSize',18);
                subplot(2,1,2)
                yyaxis left
                plot(t, forca1(3,:), '-', 'Color', 'b', 'linewidth', 1); 
                hold on;
                yyaxis left
                plot(t, forca2(3,:), '-', 'Color', 'g', 'linewidth', 1); 
                hold on;
                yyaxis left
                plot(t, forca3(3,:), '-', 'Color', 'c', 'linewidth', 1); 
                hold on;
                yyaxis right
                plot(t, forca4(3,:),'-', 'Color', 'r', 'linewidth', 1); 
                hold on;
                ylabel(['$$arrasto f_z$$ [N]'], 'Interpreter','latex')
                yyaxis left
                plot(t, forca5(3,:), '-', 'Color', 'k', 'linewidth', 1);
                hold off;
                ylabel(['$$f_z$$ [N]'], 'Interpreter','latex')
                xlim([t(1) t(end)])
                xlim([t(1) fim])
                box off;
                xlabel('time [sec.]')
                legend('motor','peso','empuxo','coriolis','arrasto');
                title("Com transição");
            end
        end
    end
end