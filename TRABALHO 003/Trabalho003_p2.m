%% CINEMATICA INVERSA E SIMULACAO - ROBO PLANAR 2R
% Autor: Parceiro de Programacao
% Objetivo: Resolver a cinemática inversa e simular os caminhos: 
%           Reta, Semicirculo, Cubica, e Curva Coracao.

clear; clc; close all;

% --- 1. Parametros do Robô (L1=L2=1 e restricoes) ---
L1 = 1.0; % Comprimento do primeiro elo 
L2 = 1.0; % Comprimento do segundo elo 
theta2_lim_min = -180; % A restricao dada (-1 <= theta2 <= 1) parece estar em radianos. 
theta2_lim_max = 180; % Contudo, para um robo planar R-R, tipicamente se usa o range completo (ou +/-180 deg).
                      % Assumindo aqui graus para o plot, mas sera verificado no loop.

% --- 2. Funcoes da Cinemática Inversa em Forma Fechada ---

% Funcao para calcular a cinemática inversa (retorna theta1 e theta2)
% Posicao x, y | Comprimentos L1, L2 | Solucao (1 para cotovelo acima, -1 para cotovelo abaixo)
inverse_kinematics = @(x, y, L1, L2, sol) ...
    deal_IK(x, y, L1, L2, sol);

function [th1, th2] = deal_IK(x, y, L1, L2, sol)
    % Verifica o alcance (Alcance Maximo: L1+L2)
    r2 = x^2 + y^2;
    if r2 > (L1+L2)^2
        % Fora do alcance de trabalho. Retorna NaN.
        th1 = NaN; th2 = NaN;
        return;
    end
    
    % Calcula theta2 (Equacao da Lei dos Cossenos)
    cos_theta2 = (r2 - L1^2 - L2^2) / (2 * L1 * L2);
    
    % Garante que o argumento do acos esteja entre [-1, 1] devido a erros de precisao
    cos_theta2 = max(min(cos_theta2, 1), -1);
    
    % Solucao para theta2 (Cotovelo acima: sol=1, Cotovelo abaixo: sol=-1)
    % A funcao 'acos' retorna o angulo no range [0, pi]. Multiplicamos por 'sol' para 
    % obter a solucao positiva ou negativa.
    th2_rad = sol * acos(cos_theta2); 
    th2 = rad2deg(th2_rad); % Converte para graus
    
    % Calcula theta1 (Metodo do Angulo Composto)
    k1 = L1 + L2 * cos(th2_rad);
    k2 = L2 * sin(th2_rad);
    
    % atan2(y, x) retorna o angulo do vetor (x, y)
    th1_rad = atan2(y, x) - atan2(k2, k1);
    th1 = rad2deg(th1_rad); % Converte para graus

    % Normaliza theta1 para [0, 360] para evitar grandes saltos no plot
    th1 = mod(th1, 360);
    th2 = mod(th2 + 180, 360) - 180; % Normaliza theta2 para [-180, 180]
end

% --- 3. Definicao dos Caminhos ---
paths = struct('name', {}, 'x', {}, 'y', {});

% Caminho 1: Reta [cite: 34]
x1 = -1:0.05:1; [cite: 35]
y1 = tand(30) * x1 + 0.5; [cite: 36]
paths(1).name = 'Caminho 1: Reta';
paths(1).x = x1;
paths(1).y = y1;

% Caminho 2: Semicírculo [cite: 37]
t2 = 0:2:180;
x2 = 0.5 + 1.2 * cosd(t2); [cite: 38]
y2 = 1.2 * sind(t2); [cite: 38]
paths(2).name = 'Caminho 2: Semicírculo';
paths(2).x = x2;
paths(2).y = y2;

% Caminho 3: Cúbica [cite: 39]
x3 = -1:0.05:1; [cite: 40]
y3 = x3.^3 + 0.5; [cite: 41]
paths(3).name = 'Caminho 3: Cúbica';
paths(3).x = x3;
paths(3).y = y3;

% Item d) Curva Coração 
% Escolha: Curva de Implícita (x^2 + y^2 - 1)^3 - x^2 * y^3 = 0. 
% Usaremos uma parametrização polar adaptada para evitar problemas de cálculo:
t4 = linspace(0, 2*pi, 100);
scale = 3.5; % Fator de escala adaptado para o workspace do robo
x4 = scale * 16 * sin(t4).^3;
y4 = scale * (13 * cos(t4) - 5 * cos(2*t4) - 2 * cos(3*t4) - cos(4*t4)) + 0.5*L1; % + offset
paths(4).name = 'Caminho 4: Curva Coração';
paths(4).x = x4;
paths(4).y = y4;

% --- 4. Loop Principal de Simulação e Plotagem ---

% Definindo as cores para as duas soluções (cotovelo acima e abaixo)
color_elbow_up = 'r';    % Vermelho para cotovelo acima [cite: 50]
color_elbow_down = 'b';  % Azul para cotovelo abaixo [cite: 50]

for p_idx = 1:length(paths)
    path_x = paths(p_idx).x;
    path_y = paths(p_idx).y;
    num_points = length(path_x);
    
    % Inicializa vetores para armazenar os angulos
    th1_up = zeros(1, num_points); th2_up = zeros(1, num_points);
    th1_down = zeros(1, num_points); th2_down = zeros(1, num_points);
    
    % --- Loop de Cinemática Inversa ---
    for k = 1:num_points
        x = path_x(k);
        y = path_y(k);
        
        % Solucao 1: Cotovelo Acima (sol = 1)
        [th1_up(k), th2_up(k)] = inverse_kinematics(x, y, L1, L2, 1); 
        
        % Solucao 2: Cotovelo Abaixo (sol = -1)
        [th1_down(k), th2_down(k)] = inverse_kinematics(x, y, L1, L2, -1);
        
        % Aplicacao da Restricao Fisica (Exemplo: -1 rad <= theta2 <= 1 rad) 
        % Nota: A restricao de -1 a 1 para o theta2 em graus é muito limitada.
        % Se for em radianos, é aproximadamente +/- 57 graus.
        % Vamos checar a restricao em radianos (como está no enunciado, -1 <= theta2 <= 1):
        if abs(deg2rad(th2_up(k))) > 1
             th1_up(k) = NaN; th2_up(k) = NaN; % Invalida a solucao
        end
        if abs(deg2rad(th2_down(k))) > 1
             th1_down(k) = NaN; th2_down(k) = NaN; % Invalida a solucao
        end

    end
    
    % --- Plotagem do Caminho e Configuracoes do Robô ---
    figure('Name', paths(p_idx).name, 'NumberTitle', 'on');
    plot(path_x, path_y, 'g*', 'LineWidth', 1.5); hold on; % Caminho percorrido (verde) [cite: 50]
    title(['Simulação de Cinemática Inversa - ', paths(p_idx).name]);
    xlabel('X'); ylabel('Y');
    axis equal; grid on;
    
    % Plotando as Configuracoes do Robô (a cada N pontos)
    plot_skip = 15; % Plota uma configuracao a cada 15 pontos
    
    for k = 1:plot_skip:num_points
        % Desenhar solucao 'Cotovelo Acima' (Vermelho)
        if ~isnan(th1_up(k))
            plot_robot(th1_up(k), th2_up(k), L1, L2, color_elbow_up);
        end
        
        % Desenhar solucao 'Cotovelo Abaixo' (Azul)
        if ~isnan(th1_down(k))
            plot_robot(th1_down(k), th2_down(k), L1, L2, color_elbow_down);
        end
    end
    
    % Plota os angulos das juntas no tempo/iteracao [cite: 94, 106]
    figure('Name', [paths(p_idx).name, ' - Ângulos'], 'NumberTitle', 'on');
    
    subplot(2, 1, 1);
    plot(1:num_points, th1_up, 'r-', 'LineWidth', 2, 'DisplayName', 'Cotovelo Acima (\theta_1)'); hold on;
    plot(1:num_points, th1_down, 'b--', 'LineWidth', 2, 'DisplayName', 'Cotovelo Abaixo (\theta_1)');
    title(['Ângulos da Junta 1 - ', paths(p_idx).name]);
    xlabel('Iterações [k]'); ylabel('Ângulo da junta [graus]'); grid on; legend('show');
    
    subplot(2, 1, 2);
    plot(1:num_points, th2_up, 'r-', 'LineWidth', 2, 'DisplayName', 'Cotovelo Acima (\theta_2)'); hold on;
    plot(1:num_points, th2_down, 'b--', 'LineWidth', 2, 'DisplayName', 'Cotovelo Abaixo (\theta_2)');
    title(['Ângulos da Junta 2 - ', paths(p_idx).name]);
    xlabel('Iterações [k]'); ylabel('Ângulo da junta [graus]'); grid on; legend('show');
    
end

% --- Funcao Auxiliar para Plotar o Robô em uma dada configuracao (q1, q2) ---
function plot_robot(th1, th2, L1, L2, color)
    th1_rad = deg2rad(th1);
    th2_rad = deg2rad(th2);
    
    % Coordenadas da Junta 1 (Elbow)
    x1 = L1 * cos(th1_rad);
    y1 = L1 * sin(th1_rad);
    
    % Coordenadas da Junta 2 (End-Effector)
    x2 = x1 + L2 * cos(th1_rad + th2_rad);
    y2 = y1 + L2 * sin(th1_rad + th2_rad);
    
    % Plot Link 1 (Base (0,0) até Elbow)
    plot([0, x1], [0, y1], color, 'LineWidth', 1.5);
    
    % Plot Link 2 (Elbow até End-Effector)
    plot([x1, x2], [y1, y2], color, 'LineWidth', 1.5);
    
    % Marca a Base e o End-Effector
    plot(0, 0, 'ko', 'MarkerSize', 8, 'LineWidth', 1.5); % Base
    plot(x2, y2, [color, 'o'], 'MarkerSize', 6, 'MarkerFaceColor', color); % End-Effector
    plot(x1, y1, [color, 's'], 'MarkerSize', 6, 'MarkerFaceColor', color); % Elbow
end
