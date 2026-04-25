clear
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf = 10;              % Tiempo para completar el recorrido
ts = 0.001;           % Paso de integración
t = 0: ts: tf;       
N = length(t);       

%%%%%%%%%%%%%%%%%%%%%%%%%%% CONFIGURACIÓN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vx_ref = 1.0;         
base_val = 2 * exp(1);
L = log(base_val);    

%%%%%%%%%%%%%%%%%%%%%%%% CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1 = zeros(1,N+1);  
y1 = zeros(1,N+1);  
phi = zeros(1,N+1); 

x1(1) = 0;              
y1(1) = 0;              
phi(1) = atan(0); 

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
    
    % --- CÁLCULO DE DERIVADAS POR TRAMO ---
    if xk < 3
        % TRAMO 1: f1(x) = 0.5 * x^2
        dy_dx = xk;
        ddy_dxx = 1;
        
    elseif xk < 7
        % TRAMO 2: f2(x) = -(x-5)^2 + 8
        dy_dx = -2 * (xk - 5);
        ddy_dxx = -2;
        
    else
        % TRAMO 3: f3(x) = (2e)^(x-7) * cos(x-7)
        z = xk - 7;
        exp_z = base_val^z;
        % Derivada 1: u'v + uv'
        dy_dx = exp_z * (L * cos(z) - sin(z));
        % Derivada 2: u''v + 2u'v' + uv''
        ddy_dxx = exp_z * ((L^2 - 1) * cos(z) - 2 * L * sin(z));
    end
    
    % Generación de velocidades cinemáticas
    u(k) = vx_ref * sqrt(1 + dy_dx^2);
    w(k) = (ddy_dxx * vx_ref) / (1 + dy_dx^2); 
end

%%%%%%%%%%%%%%%%%%%%%%%%% BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k=1:N 
    % --- RESET DE ESTADOS EN FRONTERAS (ROBUSTEZ) ---
    if abs(x1(k) - 3) < 0.005
        y1(k) = 4;        
        phi(k) = atan(4); 
    elseif abs(x1(k) - 7) < 0.005
        y1(k) = 1;        
        phi(k) = atan(L); 
    end

    % Integración del modelo cinemático
    phi(k+1) = phi(k) + w(k)*ts;
    xp1 = u(k)*cos(phi(k+1)); 
    yp1 = u(k)*sin(phi(k+1));
    
    x1(k+1) = x1(k) + xp1*ts; 
    y1(k+1) = y1(k) + yp1*ts; 
    
    hx(k+1) = x1(k+1); 
    hy(k+1) = y1(k+1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
scene = figure;
set(scene,'Color','white');
axis equal; grid on; box on;
xlabel('x(m)'); ylabel('y(m)');
view([0 90]); 

% Ajuste de límites para visualizar la caída del tramo 3
axis([-0.5 11 -10 10]); 
set(gca, 'Color', [0.1 0.1 0.1]); 

scale = 0.3; 
try
    MobileRobot_5; 
    H1 = MobilePlot_4(x1(1),y1(1),phi(1),scale); hold on;
catch
    H1 = plot(x1(1),y1(1),'yo', 'MarkerFaceColor', 'y'); hold on;
end

H2 = plot(hx(1),hy(1),'r','lineWidth',2.5);

for k=1:40:N 
    if ishandle(H1) delete(H1); end
    if ishandle(H2) delete(H2); end
    
    try
        H1 = MobilePlot_4(x1(k),y1(k),phi(k),scale);
    catch
        H1 = plot(x1(k),y1(k),'yo', 'MarkerSize', 10, 'MarkerFaceColor', 'y');
    end
    H2 = plot(hx(1:k),hy(1:k),'r','lineWidth',2.5);
    
    drawnow;
end