clc;
clear;
close all;
clear classes;
%% 

set(0,'DefaultFigureRenderer','painters')

%% Longitudes

L1 = 0.062732; % COXA
L2 = 0.083; % FEMUR
L3 = 0.134390; % TIBIA

%% Robot

s = 'Rz(q1) Ty(L1) Rx(q2) Ty(L2) Rx(q3) Tz(L3)'; % Secuencia CD
dh = DHFactor(s); 

c = dh.command('leg');
leg = eval(c); % Definición de una pata

%% Cuerpo

W = 0.150354;
L = 0.200354;
R = 0.085402;

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

hold on;
grid on;
view(3);

axis equal;
axis([-0.4 0.4 -0.4 0.4 -0.3 0.3]);

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

%% Cinematica Inversa - Ciclo de Marcha (GIRO SOBRE SU PROPIO EJE)

% La trayectoria del pie se define EN EL MARCO DE UNA PATA. El MISMO ciclo se
% reutiliza para las 6 patas, con desfases.

% --- Parámetros de la zancada -------------------------------------------
stride = 0.05;     % medio barrido (m) -> barrido total = 2*stride
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

% --- Restricciones articulares -------------------------------------------
% Rango util de los AX-12A: +-150 deg respecto a la posicion neutra.
QMAX = deg2rad(150);
sweep = max(abs(qcycle(:,1)));
fprintf('Barrido de coxa: %.2f deg\n', rad2deg(sweep));
if sweep > QMAX
    fprintf('Excede el rango de la coxa, reduzca stride.\n');
end

%% Parámetros de la marcha

N   = size(qcycle,1);
off = round(N/2);

% Apuntar las patas hacia afuera de la base.
hipHome = [ -3*pi/4, pi, 3*pi/4, pi/4, 0, -pi/4 ];   % patas 1..6

% ---------------------------------------------------------------------
% AQUI ESTA LA UNICA DIFERENCIA CON LA MARCHA DE AVANCE:
%
%   avance -> sgn = [ +1, +1, +1, -1, -1, -1 ]
%             las patas de un lado y del otro empujan en sentidos
%             opuestos en su marco local, y el cuerpo se traslada.
%
%   giro   -> sgn = [ +1, +1, +1, +1, +1, +1 ]
%             las 6 empujan en el MISMO sentido angular, las fuerzas se
%             cancelan en traslacion y solo queda par -> el hexapodo gira
%             sobre su propio eje.
%
% DIR = +1 gira en un sentido, DIR = -1 en el contrario.
% ---------------------------------------------------------------------
DIR = +1;
sgn = DIR * [ +1, +1, +1, +1, +1, +1 ];

%% Figura: Fases del ciclo de giro
% Una instantánea por cada fase del ciclo (tomando como referencia las patas 1, 3 y 5)
kFase   = [13 75 138 175];   % muestras: apoyo, empuje, levantamiento, retorno
nomFase = {'Apoyo','Empuje','Levantamiento','Retorno'};

figF = figure('Units','centimeters','Position',[2 2 36 10]);
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
        pata = SerialLink(legs(i), 'name', sprintf('leg%d_g%d', i, f));
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