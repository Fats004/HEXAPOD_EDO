clc;
clear;
close all;
clear classes;
%%
set(0,'DefaultFigureRenderer','painters')
%% Longitudes
L1 = 0.052; % COXA
L2 = 0.083; % FEMUR
L3 = 0.136390; % TIBIA
%% Robot
s = 'Rz(q1) Ty(L1) Rx(q2) Ty(L2) Rx(q3) Tz(L3)'; % Secuencia CD
dh = DHFactor(s);
c = dh.command('leg');
leg = eval(c); % Definición de una pata
%% Cuerpo
W = 0.150;
L = 0.200;
R = 0.085;
%% Patas
legs(6) = SerialLink(leg, ...
'name', 'leg6', ...
'base', SE3(L/2, W/2, 0)*SE3.Rz(-pi/4));
legs(5) = SerialLink(leg, ...
'name', 'leg5', ...
'base', SE3(0, R, 0));
legs(4) = SerialLink(leg, ...
'name', 'leg4', ...
'base', SE3(-L/2, W/2, 0)*SE3.Rz(pi/4));
legs(3) = SerialLink(leg, ...
'name', 'leg3', ...
'base', SE3(-L/2, -W/2, 0)*SE3.Rz(pi-pi/4));
legs(2) = SerialLink(leg, ...
'name', 'leg2', ...
'base', SE3(0, -R, 0)*SE3.Rz(pi));
legs(1) = SerialLink(leg, ...
'name', 'leg1', ...
'base', SE3(L/2, -W/2, 0)*SE3.Rz(pi+pi/4));
%% Figura
figRobot = figure;
title('Postura HOME')
hold on;
grid on;
view(3);
axis equal;
% axis([-0.4 0.4 -0.4 0.4 -0.3 0.3]);
axis([-0.4 0.4 -0.4 0.4 -0.3 0.3]);
%% no home
% q1 = [-pi/4 0 0];
% q2 = [0      0 0];
% q3 = [pi/4   0 0];
% q4 = [3*pi/4 0 0];
% q5 = [pi     0 0];
% q6 = [-3*pi/4 0 0];
%% Configuración
%configuracion HOME
q1 = [-pi/4 0.4 -0.3];
q2 = [0      0.4 -0.3];
q3 = [pi/4   0.4 -0.3];
q4 = [3*pi/4 0.4 -0.3];
q5 = [pi     0.4 -0.3];
q6 = [-3*pi/4 0.4 -0.3];
%% Plot SOLO UNA VEZ
legs(6).plot(q1, ...
'nobase', ...
'noshadow', ...
'notiles', ...
'delay', 0);
legs(5).plot(q2, ...
'nobase', ...
'noshadow', ...
'notiles', ...
'delay', 0);
legs(4).plot(q3, ...
'nobase', ...
'noshadow', ...
'notiles', ...
'delay', 0);
legs(3).plot(q4, ...
'nobase', ...
'noshadow', ...
'notiles', ...
'delay', 0);
legs(2).plot(q5, ...
'nobase', ...
'noshadow', ...
'notiles', ...
'delay', 0);
legs(1).plot(q6, ...
'nobase', ...
'noshadow', ...
'notiles', ...
'delay', 0);
%% Dibujar cuerpo
patch([L/2 L/2 -L/2 -L/2], ...
      [-W/2 W/2 W/2 -W/2], ...
      [0 0 0 0], ...
'r', ...
'FaceAlpha',0.5);
drawnow;
%% Cinematica Inversa - Ciclo de Marcha (marcha trípode)
% La trayectoria del pie se define EN EL MARCO DE UNA PATA. El MISMO ciclo se
% reutiliza para las 6 patas, con desfases.
% --- Parámetros de la zancada -------------------------------------------
stride = 0.04;     % medio paso (m)  -> zancada total = 2*stride
lift   = 0.05;     % cuánto se levanta el pie en la fase que se levanta (m)
% El pie APOYADO coincide con el pie de la posicion HOME.
qHome = [0 0.4 -0.3];
pHome = transl( leg.fkine(qHome) );    % [x y z] del pie en HOME (marco de la pata)
yy = pHome(2);                          % alcance lateral (hacia afuera del cuerpo)
zd = pHome(3);                          % z del pie APOYADO (mismo que el HOME)
zu = zd - sign(zd)*lift;                % z del pie LEVANTADO (más cerca de z=0)
xf =  stride;  xb = -stride;            % límites adelante / atrás
% --- Rectángulo que recorre cada pie --------------------------------------
% 1) empuja apoyado  2) levanta  3) regresa por el aire  4) apoya
segments = [ xf  yy  zd      % adelante, apoyado
             xb  yy  zd      % atrás, apoyado  (fase de empuje)
             xb  yy  zu      % atrás, levantado
             xf  yy  zu ];   % adelante, levantado  (regresa al inicio)
%            ->apoya  ->empuje  ->levanta  ->regreso
tseg_one = [  0.25     1.0       0.25       0.5 ]'; %tiempos
% Generamos varios ciclos y nos quedamos con UNO del centro.
dt = 0.01;  tacc = 0.1;  nrep = 3;
segrep  = repmat(segments, nrep, 1);
tsegrep = repmat(tseg_one, nrep, 1);
x = mstraj(segrep, [], tsegrep, segments(1,:), dt, tacc);
nspc   = round(sum(tseg_one)/dt);     % muestras por ciclo
xcycle = x(nspc+1 : 2*nspc, :);
% --- Cinemática inversa --------------------------------------------------
qcycle = leg.ikine( transl(xcycle), qHome, 'mask', [1 1 1 0 0 0] );
qcycle(:,1) = qcycle(:,1) - mean(qcycle(:,1));
%% Parámetros de la marcha
N   = size(qcycle,1);
off = round(N/2);
% Apuntar las patas hacia afuera de la base.
hipHome = [ -3*pi/4, pi, 3*pi/4, pi/4, 0, -pi/4 ];   % patas 1..6
% Signo de orientacion de las patas
sgn = [ +1, +1, +1, -1, -1, -1 ];
%% Figura: Trayectoria del extremo de la extremidad
% Datos en mm. La altura se mide desde el pie apoyado (suelo = 0)
xmm = 1000*xcycle(:,1);
hmm = 1000*abs(xcycle(:,3) - zd);
V   = 1000*[xf 0; xb 0; xb lift; xf lift];   % vértices P1..P4
apoyo = hmm < 0.5;   % muestras con el pie en contacto con el suelo
figT = figure('Color','w','Units','centimeters','Position',[2 2 16 9]);
hold on; grid on; box on;
% Rectángulo ideal y trayectoria generada con mstraj
plot([V(:,1); V(1,1)], [V(:,2); V(1,2)], '--', 'Color',[0.6 0.6 0.6], 'LineWidth',1);
plot(xmm, hmm, '-', 'Color',[0.2 0.2 0.2], 'LineWidth',0.8, 'HandleVisibility','off');
plot(xmm(apoyo),  hmm(apoyo),  '.', 'Color',[0.00 0.45 0.74], 'MarkerSize',9);
plot(xmm(~apoyo), hmm(~apoyo), '.', 'Color',[0.85 0.33 0.10], 'MarkerSize',9);
% Vértices
plot(V(:,1), V(:,2), 'ko', 'MarkerFaceColor','k', 'MarkerSize',5, 'HandleVisibility','off');
etq = {'P_1','P_2','P_3','P_4'};
dx  = [ 6 -6 -6  6];
dy  = [-5 -5  5  5];
for i = 1:4
    text(V(i,1)+dx(i), V(i,2)+dy(i), etq{i}, 'HorizontalAlignment','center', 'FontSize',10);
end
% Flechas del sentido de recorrido
qv = {'k', 'LineWidth',1.2, 'MaxHeadSize',2, 'HandleVisibility','off'};
quiver(  8,          0,        -16,   0, 0, qv{:});   % empuje
quiver(1000*xb,  1000*lift/2-8,  0,  16, 0, qv{:});   % levantamiento
quiver( -8,     1000*lift,      16,   0, 0, qv{:});   % retorno
quiver(1000*xf,  1000*lift/2+8,  0, -16, 0, qv{:});   % apoyo
% Nombre de cada fase
text(0,             -9,             'Empuje (1.00 s)',        'HorizontalAlignment','center', 'FontSize',9);
text(0,             1000*lift+7,    'Retorno (0.50 s)',       'HorizontalAlignment','center', 'FontSize',9);
text(1000*xb-4,     1000*lift/2,    'Levantamiento (0.25 s)', 'HorizontalAlignment','right',  'FontSize',9);
text(1000*xf+4,     1000*lift/2,    'Apoyo (0.25 s)',         'HorizontalAlignment','left',   'FontSize',9);
xlabel('Desplazamiento en X (mm)');
ylabel('Altura del pie (mm)');
legend({'Trayectoria ideal','Pie apoyado','Pie en el aire'}, ...
'Location','southoutside', 'Orientation','horizontal');
xlim([-120 120]); ylim([-18 70]);
daspect([1 1 1]);
set(gca, 'FontSize',10);
title('Trayectoria Base por Extremidad')
%% Figura: Fases del ciclo de marcha
% Una instantánea por cada fase del ciclo (tomando como referencia las patas 1, 3 y 5)
kFase   = [13 75 138 175];   % muestras: apoyo, empuje, levantamiento, retorno
nomFase = {'Apoyo','Empuje','Levantamiento','Retorno'};
figF = figure;
for f = 1:4
    subplot(1,4,f);
    hold on;
    grid on;
    view(3);
    for i = 1:6
        if mod(i,2) == 1
            o = 0;      % patas 1, 3, 5
        else
            o = off;    % patas 2, 4, 6
        end
        pata = SerialLink(legs(i), 'name', sprintf('leg%d_f%d', i, f));
        pata.plot( gait(qcycle,kFase(f),o,hipHome(i),sgn(i)), 'nobase','noshadow','notiles','noname','delay',0 );
    end
    patch([L/2 L/2 -L/2 -L/2], [-W/2 W/2 W/2 -W/2], [0 0 0 0], 'r', 'FaceAlpha', 0.5);
    axis equal;
    axis([-0.4 0.4 -0.4 0.4 -0.3 0.3]);
    title(sprintf('%s', nomFase{f}));
end
%% Animación
figure(figRobot);   % regresa a la figura del robot
% Se genera el cuerpo
patch([L/2 L/2 -L/2 -L/2], [-W/2 W/2 W/2 -W/2], [0 0 0 0], 'r', 'FaceAlpha', 0.5);
% Velocidad de la animación: 1 = lento (todos), 2-4 = más rapido
kstep = 3;
k = 1;
while true
    legs(1).plot( gait(qcycle,k,0,  hipHome(1),sgn(1)), 'nobase','noshadow','notiles','delay',0 );
    legs(2).plot( gait(qcycle,k,off,hipHome(2),sgn(2)), 'nobase','noshadow','notiles','delay',0 );
    legs(3).plot( gait(qcycle,k,0,  hipHome(3),sgn(3)), 'nobase','noshadow','notiles','delay',0 );
    legs(4).plot( gait(qcycle,k,off,hipHome(4),sgn(4)), 'nobase','noshadow','notiles','delay',0 );
    legs(5).plot( gait(qcycle,k,0,  hipHome(5),sgn(5)), 'nobase','noshadow','notiles','delay',0 );
    legs(6).plot( gait(qcycle,k,off,hipHome(6),sgn(6)), 'nobase','noshadow','notiles','delay',0 );
    drawnow;
    k = mod(k - 1 + kstep, N) + 1;   % avanza kstep y da la vuelta
end
%% Funcion gait
function q = gait(cycle, k, offset, hip0, sgn)
    k = mod(k + offset - 1, size(cycle,1)) + 1;  % avanza el índice y da la vuelta
    q = cycle(k, :);
    q(1) = hip0 + sgn * q(1);
end