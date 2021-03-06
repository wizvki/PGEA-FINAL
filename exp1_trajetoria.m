clear all;
close all;
clc;

addpath('auxx/');
addpath('auxx2/');
addpath('auxx3/');
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
traj{1} = [5; 5; 10; deg2rad(0)];
% traj{end+1} = [10; -5; -5; deg2rad(0)];
% traj{end+1} = [10; 10; -5; deg2rad(0)];
% traj{end+1} = [-10; 10; -5; deg2rad(0)];
% traj{end+1} = [-10; -10; -5; deg2rad(0)];
% traj{end+1} = [-10; -10; -1; deg2rad(0)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% condicoes iniciais
p1 = [5; 5; -1];              % posicao [m]
v1 = [0; 0; 0];              % velocidade [m/s]
r1 = deg2rad([30; 0; 0]);     % atitude [rad]
q1 = deg2rad([0; 0; 0]);     % rotacao [rad/s]
x1 = [p1; r1; v1; q1];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% condicoes iniciais
p2 = [5; 5; -1];              % posicao [m]
v2 = [0; 0; 0];              % velocidade [m/s]
r2 = deg2rad([30; 0; 0]);     % atitude [rad]
q2 = deg2rad([0; 0; 0]);     % rotacao [rad/s]
x2 = [p2; r2; v2; q2];

p = [p1 p2];

%%%%%%%forca arrasto plot unico
%figure(33), set(gcf, 'Position', [304.2000  252.2000  842.4000  424.0000])

%%%%%%%z vz az plot 3x1
%figure(38), set(gcf, 'Position', [343.4000  -52.6000  842.4000  792.0000])

%%%%%% peso empuxo plot 2x1
%figure(21), set(gcf, 'Position', [389.8000   83.4000  842.4000  656.0000])


%compara??o com e sem modifica??o
%uav{1} = quadHibridoproposta(x1, 'b');
%uav{2} = quadHibridoproposta1(x2, 'r');

%compara??o com e sem transi??o
uav{1} = quadHibridoproposta1(x1, 'b');
uav{2} = quadHibridoproposta(x2, 'r');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% main loop
tf = 5.0;
count = 0;
wps = ones(size(uav));
chegou = false(size(uav));

%%%%%teste funcao pause
para=1;

%figure(10), clf, set(gcf, 'Position', [0 0 1200 1200]);
%figure(10), clf, set(gcf, 'Position', [-0.6000  361.0000  560.0000  420.0000]);
%figure(10), clf, set(gcf, 'Position', 1.0e+03 *[0.3922    0.0386    1.1456    0.7368]);%posicao lg 
% figure(10), clf, set(gcf, 'Position',  1.0e+03 *[1.8586   0.0654    0.9200    0.9680]);%posicao hp canto superior esquerda

%figure(10), clf, set(gcf, 'Position',  1.0e+03 *[1.9098   -0.1854    0.8032    0.9680]);%posicao hp esquerda
set(figure(10),'name','Simula??o','numbertitle','on') % Setting the name of the figure
clf(figure(10)) % Erase the contents of the figure

%define quais plots quer visualizar na simulacao
plots =     [0 0 0 0 0 0 0 0 0 0 0 0 0];
%define quais plots quer visualizar no final
plotsfim =  [1 1 0 0 0 0 1 1 1 1 1 0 0];

if (plots(1)==1)%posicao %orientacao %ambos
    set(figure(30),'name','Posi??o e Orienta??o','numbertitle','on') % Setting the name of the figure
    figure(30), clf, set(gcf, 'Position', 1.0e+03 *[2.7154   -0.1854    0.9608    0.9680]);%direita
    figure(10), clf, set(gcf, 'Position',  1.0e+03 *[1.9098   -0.1854    0.8032    0.9680]);%posicao hp esquerda
end
if (plots(2)==1)%velocidade linear %angular %ambos
    figure(10), clf, set(gcf, 'Position',  1.0e+03 *[1.9098   -0.1854    0.8032    0.9680]);%posicao hp esquerda
    figure(31), clf, set(gcf, 'Position', 1.0e+03 *[2.7154   -0.1854    0.9608    0.9680]),;%direita
    set(figure(31),'name','Velocidades Linear e Angular','numbertitle','on') % Setting the name of the figure
end
if (plots(3)==1)%velocidade motor %ambos
    figure(10), clf, set(gcf, 'Position',  1.0e+03 *[1.9098   -0.1854    0.8032    0.9680]);%posicao hp esquerda
    figure(32), clf, set(gcf, 'Position', 1.0e+03 *[2.7154   -0.1854    0.9608    0.9680]),;%direita
    set(figure(32),'name','Velocidades dos Motores','numbertitle','on') % Setting the name of the figure
end
if (plots(4)==1)%forcas motores %ambos
    figure(10), clf, set(gcf, 'Position',  1.0e+03 *[1.9098   -0.1854    0.8032    0.9680]);%posicao hp esquerda
    figure(33), clf, set(gcf, 'Position', 1.0e+03 *[2.7154   -0.1854    0.9608    0.9680]),;%direita
    set(figure(33),'name','For?as nos motores','numbertitle','on') % Setting the name of the figure
end
if (plots(5)==1)%forca motor, peso, empuxo, arrasto, coriolis AMBOS
    figure(34), clf, set(gcf, 'Position', 1.0e+03 *[ 2.6226  -0.1862   1.0584   0.968]);%direita
    figure(10), clf, set(gcf, 'Position',  1.0e+03 *[1.9154  -0.1862    0.7048   0.968]);%posicao hp esquerda
    set(figure(34),'name','For?as','numbertitle','on') % Setting the name of the figure
end
if (plots(6)==1)%z; v_z; f_z arrasto 
    figure(35), clf, set(gcf, 'Position', 1.0e+03 *[ 2.6226  -0.1862   1.0584   0.968]);%direita
    figure(10), clf, set(gcf, 'Position',  1.0e+03 *[1.9154  -0.1862    0.7048   0.968]);%posicao hp esquerda
    set(figure(35),'name','Posi??o, Velocidade e For?a de Arrasto em z','numbertitle','on') % Setting the name of the figure
end
if (plots(7)==1)% f_z  %ambos 
    figure(36), clf, set(gcf, 'Position', 1.0e+03 *[2.7154   -0.1854    0.8856    0.9680]);%direita
    figure(10), clf, set(gcf, 'Position',  1.0e+03 *[1.9098   -0.1854    0.8032    0.9680]);%posicao hp esquerda
    set(figure(36),'name','fz','numbertitle','on') % Setting the name of the figure
end
if (plots(8)==1)% z; vz; a_z; %ambos
    figure(10), clf, set(gcf, 'Position',  1.0e+03 *[1.9098   -0.1854    0.8032    0.9680]);%posicao hp esquerda
    figure(38), clf, set(gcf, 'Position', 1.0e+03 *[2.7154   -0.1854    0.9608    0.9680]);%direita
end
if (plots(9)==1)%forca motor, peso, empuxo, arrasto, coriolis quad1
    %figure(39), clf, set(gcf, 'Position', [ -12.6000  362.6000  560.0000  420.0000]);
    figure(39), clf, set(gcf, 'Position', 1.0e+03 *[2.7154   -0.1854    0.9608    0.9680]);%direita
    figure(10), clf, set(gcf, 'Position',  1.0e+03 *[1.9098   -0.1854    0.8032    0.9680]);%posicao hp esquerda
end
if (plots(10)==1)%forca motor, peso, empuxo, arrasto, coriolis quad2
    figure(40), clf, set(gcf, 'Position', 1.0e+03 *[2.7154   -0.1854    0.9608    0.9680]);%direita
    figure(10), clf, set(gcf, 'Position',  1.0e+03 *[1.9098   -0.1854    0.8032    0.9680]);%posicao hp esquerda
end
if (plots(11)==1)%forca motor, peso, empuxo, arrasto, coriolis quad2
    figure(41), clf, set(gcf, 'Position', 1.0e+03 *[2.7154   -0.1854    0.9608    0.9680]);%direita
    figure(10), clf, set(gcf, 'Position',  1.0e+03 *[1.9098   -0.1854    0.8032    0.9680]);%posicao hp esquerda
end

tempo=5;
dt = 1/100;

while (uav{1}.time() < tf)% && (min(wps) <= length(traj))    
    
    %figure(10), set(gca,'FontSize',16);
        
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
    %fora_do_if = uav{1}.time
    %fora_do_if = tempo
    % desenha
    if (tempo >= 5) %&& (desenha)
    if (mod(uav{1}.time, 1/100) <= .001) %&& (desenha)
        entrei_no_if = uav{1}.time
        entrei_no_if = tempo
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
        drawWater(  [4 6], ...
                    [6 4], ...
                    [-1 0]);
        % desenha trajetoria
        %wayps = [traj{:}]';
        %plot3(wayps(:,1), wayps(:,2), wayps(:,3), 'k--', 'linewidth', 2), set(gca,'FontSize',16);
        axis equal;
        axis tight;
        xlabel(['$$x [m]$$'], 'Interpreter','latex')
        ylabel(['$$y [m]$$'], 'Interpreter','latex')
        zlabel(['$$z [m]$$'], 'Interpreter','latex')
        
        set(gca,'View',[-35,20],'FontSize',16); % set the azimuth and elevation of the plot
        grid on;
        hold off;
        drawnow;
        for u = 1:length(uav)
            uav{u}.plot(plots, tf);      
        end
        if (para == 1)
            w = waitforbuttonpress;%pause(5);
            para = 0;
        end
        tempo = 0;
    end
    %tempo = tempo + dt;
    tempo = tempo + 1;
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
for u = 1:length(uav)
    uav{u}.plot(plotsfim, tf);
end

%figure(33), set(gcf, 'Position', [304.2000  252.2000  842.4000  424.0000])
% figure(30), set(gcf, 'Position', [343.4000  -52.6000  842.4000  792.0000])
% figure(31), set(gcf, 'Position', [343.4000  -52.6000  842.4000  792.0000])
% figure(34), set(gcf, 'Position', [343.4000  -52.6000  842.4000  792.0000])
 figure(41), set(gcf, 'Position', [343.4000  -52.6000  842.4000  792.0000])
% % figure(21), set(gcf, 'Position', [389.8000   83.4000  842.4000  656.0000])
figure(36), set(gcf, 'Position', [389.8000   83.4000  842.4000  656.0000])