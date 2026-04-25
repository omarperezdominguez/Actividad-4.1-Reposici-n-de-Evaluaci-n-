clear
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf = 10;              % Tiempo para llegar a x = 10
ts = 0.001;           % Paso de integración
t = 0: ts: tf;       
N = length(t);       

%%%%%%%%%%%%%%%%%%%%%%%%%%% CONFIGURACIÓN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vx_ref = 1.0;        

%%%%%%%%%%%%%%%%%%%%%%%% CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1 = zeros(1,N+1);  
y1 = zeros(1,N+1);  
phi = zeros(1,N+1); 

x1(1) = 0;              
y1(1) = 0;              

phi(1) = atan(2); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%% PUNTO DE CONTROL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hx = zeros(1,N+1);  
hy = zeros(1,N+1);  
hx(1) = x1(1);
hy(1) = y1(1);

%%%%%%%%%%%%%%%%%%%%%% VELOCIDADES DE REFERENCIA %%%%%%%%%%%%%%%%%%%%%%%%%%
u = zeros(1,N); 
w = zeros(1,N); 

for k = 1:N
    xk = vx_ref * t(k);
    
    % --- FUNCIÓN DE LA GRÁFICA: f(x) = 2 * sin(x) * e^(-0.3x) ---
    ex = exp(-0.3 * xk);  
    sx = sin(xk);
    cx = cos(xk);

    % Primera derivada (dy/dx) -> Pendiente
    dy_dx = ex * (2 * cx - 0.6 * sx); 
    
    % Segunda derivada (d2y/dx2) -> Curvatura
    ddy_dxx = ex * (-1.2 * cx - 1.82 * sx);
    
    % Velocidad lineal resultante (u)
    u(k) = vx_ref * sqrt(1 + dy_dx^2);
    
    % Velocidad angular (w) para el modelo diferencial
    w(k) = (ddy_dxx * vx_ref) / (1 + dy_dx^2); 
end

%%%%%%%%%%%%%%%%%%%%%%%%% BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k=1:N 
    % Integración de la orientación
    phi(k+1) = phi(k) + w(k)*ts;
    
    xp1 = u(k)*cos(phi(k+1)); 
    yp1 = u(k)*sin(phi(k+1));
    
    x1(k+1) = x1(k) + xp1*ts; 
    y1(k+1) = y1(k) + yp1*ts; 
    
    hx(k+1) = x1(k+1); 
    hy(k+1) = y1(k+1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACION 3D %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
scene = figure;
set(scene,'Color','white');
axis equal; grid on; box on;
xlabel('x(m)'); ylabel('y(m)');
view([0 90]); 

axis([-0.5 11 -1 1.5]); 

scale = 0.3; 
try
    MobileRobot_5; 
    H1 = MobilePlot_4(x1(1),y1(1),phi(1),scale); hold on;
catch
    H1 = plot(x1(1),y1(1),'bo', 'MarkerSize', 8); hold on;
end

H2 = plot(hx(1),hy(1),'r','lineWidth',2);

% Animación
for k=1:30:N 
    if ishandle(H1) delete(H1); end
    if ishandle(H2) delete(H2); end
    
    try
        H1 = MobilePlot_4(x1(k),y1(k),phi(k),scale);
    catch
        H1 = plot(x1(k),y1(k),'bo');
    end
    H2 = plot(hx(1:k),hy(1:k),'r','lineWidth',2.5);
    
    drawnow;
end