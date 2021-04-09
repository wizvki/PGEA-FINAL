clear all;
close all;
clc;

addpath('auxx/');
addpath('auxx2/');
%format long
%digitsOld = digits(50);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% trajetoria 1
% traj{1} = [0; 0; 5; deg2rad(0)];
% traj{end+1} = [5; 0; 5; deg2rad(0)];
% traj{end+1} = [5; 5; 5; deg2rad(0)];
% traj{end+1} = [-5; 5; 5; deg2rad(0)];
% traj{end+1} = [-5; -5; 5; deg2rad(0)];
% traj{end+1} = [-5; -5; 0; deg2rad(0)];
% traj{end+1} = [5; 0; -5; deg2rad(0)];
% traj{end+1} = [10; -5; -5; deg2rad(0)];
% traj{end+1} = [10; 10; -5; deg2rad(0)];
% traj{end+1} = [-10; 10; -5; deg2rad(0)];
% traj{end+1} = [-10; -10; -5; deg2rad(0)];
% traj{end+1} = [-10; -10; -1; deg2rad(0)];
% 
% traj{1} = [0; 0; 5; deg2rad(0)];
% traj{end+1} = [5; 0; 5; deg2rad(0)];
% traj{end+1} = [5; 5; 5; deg2rad(0)];
% traj{end+1} = [-5; 5; 5; deg2rad(0)];
% traj{end+1} = [-5; -5; 5; deg2rad(0)];
% traj{end+1} = [-5; -5; 0; deg2rad(0)];
traj{1} = [-5; -5; 0.3; deg2rad(0)];
% traj{end+1} = [10; -5; -5; deg2rad(0)];
% traj{end+1} = [10; 10; -5; deg2rad(0)];
% traj{end+1} = [-10; 10; -5; deg2rad(0)];
% traj{end+1} = [-10; -10; -5; deg2rad(0)];
% traj{end+1} = [-10; -10; -1; deg2rad(0)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% condicoes iniciais
p1 = [-5; -5; 2];              % posicao [m]
v1 = [0; 0; 0];              % velocidade [m/s]
r1 = deg2rad([0; 0; 0]);     % atitude [rad]
q1 = deg2rad([0; 0; 0]);     % rotacao [rad/s]
x1 = [p1; r1; v1; q1];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% condicoes iniciais
p2 = [-4; -5; 2];              % posicao [m]
v2 = [0; 0; 0];              % velocidade [m/s]
r2 = deg2rad([0; 0; 0]);     % atitude [rad]
q2 = deg2rad([0; 0; 0]);     % rotacao [rad/s]
x2 = [p2; r2; v2; q2];

p = [p1 p2];

% Hydrone V1
%uav{1} = quadHibrido(x, 'aw', [1 0 1]);
% Hydrone V2
uav{1} = quadHibridoproposta(x1, 'b');
uav{2} = quadHibridoproposta2(x2, 'r');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% main loop
tf = 15.0;
count = 0;
wps = ones(size(uav));
chegou = false(size(uav));

%figure(10), clf, set(gcf, 'Position', [0 0 1200 1200]);
%figure(10), clf, set(gcf, 'Position', [-0.6000  361.0000  560.0000  420.0000]);
%figure(10), clf, set(gcf, 'Position', 1.0e+03 *[0.3922    0.0386    1.1456    0.7368]);%posicao lg 
% figure(10), clf, set(gcf, 'Position',  1.0e+03 *[1.8586   0.0654    0.9200    0.9680]);%posicao hp canto superior esquerda

figure(10), clf, set(gcf, 'Position',  1.0e+03 *[1.8586   -0.1854    0.8544    0.9680]);%posicao hp esquerda

set(figure(10),'name','figure_name','numbertitle','on') % Setting the name of the figure
clf(figure(10)) % Erase the contents of the figure

plots = [0 1 0 0 0 0];%define quais plots quer visualizar
if (plots(1)==1)%posicao%orientacao
    %figure(30), clf, set(gcf, 'Position', [562.6000  360.2000  560.0000  420.0000]);
    figure(30), clf, set(gcf, 'Position', 1.0e+03 *[2.7154   -0.1854    0.9608    0.9680]);%direita
end
if (plots(2)==1)%velocidade linear %angular
    %figure(31), clf, set(gcf, 'Position', [ 562.6000  360.2000  560.0000  420.0000]);
    figure(31), clf, set(gcf, 'Position', 1.0e+03 *[2.7154   -0.1854    0.9608    0.9680]);%direita
end
if (plots(3)==1)%velocidade motor
    figure(32), clf, set(gcf, 'Position', 1.0e+03 *[2.7154   -0.1854    0.9608    0.9680]);
    %figure(32), clf, set(gcf, 'Position', 1.0e+03 *[2.7154   -0.1854    0.9608    0.9680]);%direita
end
if (plots(4)==1)%forcas motores
    %figure(33), clf, set(gcf, 'Position', 1.0e+03 *[2.7154   -0.1854    0.9608    0.9680]);
    figure(33), clf, set(gcf, 'Position', 1.0e+03 *[2.7154   -0.1854    0.9608    0.9680]);%direita
end
if (plots(5)==1)%forca motor, peso, empuxo, arrasto, coriolis 
    %figure(34), clf, set(gcf, 'Position', [ 2250.6    243.4    561.6    538.4]);
    figure(34), clf, set(gcf, 'Position', 1.0e+03 *[2.7154   -0.1854    0.9608    0.9680]);%direita
end
if (plots(6)==1)%z; v_z; f_z  
    %figure(35), clf, set(gcf, 'Position', [ 2250.6    243.4    561.6    538.4]);
    figure(35), clf, set(gcf, 'Position', 1.0e+03 *[2.7154   -0.1854    0.9608    0.9680]);%direita
end
while (uav{1}.time() < tf)% && (min(wps) <= length(traj))    
    
    % atualiza modelo
    for u = 1:length(uav)
        try
            ref = traj{wps(u)};
        catch
            ref = traj{end};
        end
        uav{u}.update(ref, p1);
    
        % atualiza waypoint
        if uav{u}.reach()
            wps(u) = wps(u) + 1;
        end
    end
    
    desenha = true;
    
    % desenha
    if (mod(uav{1}.time, 1/2) < 1/500) && (desenha)
        
        figure(10), clf;
        % desenha drones
        for u = 1:length(uav)
            uav{u}.draw();
        end
        % desenha agua
        [c idx] = max(wps);
%         drawWater(  [min(uav{idx}.hyst.x(1,:))-1 max(uav{idx}.hyst.x(1,:))+1], ...
%                     [min(uav{idx}.hyst.x(2,:))-1 max(uav{idx}.hyst.x(2,:))+1], ...
%                     [min(uav{idx}.hyst.x(3,:))-1 max(uav{idx}.hyst.x(3,:))+1]);
        drawWater(  [-6 -3], ...
                    [-4 -6], ...
                    [-1 0]);
        % desenha trajetoria
        wayps = [traj{:}]';
        plot3(wayps(:,1), wayps(:,2), wayps(:,3), 'k--', 'linewidth', 2)
        %
        %title(['t = ' num2str(uav.time())])
        axis equal;
        axis tight;
        set(gca,'View',[-35,20]); % set the azimuth and elevation of the plot
        grid on;
        hold off;
        drawnow;
        for u = 1:length(uav)
            uav{u}.plot(plots, tf);      
        end
    end
    
end                    
                    
% imprime em pdf
% fonte = 16;
% trataImpressao(gcf, ['pdfs/teste1'], fonte);

% % mais alguns segundos
% tf = uav{1}.time() + 10; 
% while (uav{1}.time() < tf)
%     
%     % atualiza modelo
%     for u = 1:length(uav)
%         uav{u}.update(traj{end}, p);
%     end
% end
% 
% plota graficos
% for u = 1:length(uav)
%     uav{u}.plot();
% end