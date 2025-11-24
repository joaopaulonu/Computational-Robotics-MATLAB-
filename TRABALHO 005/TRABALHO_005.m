%% TRABALHO 5 - ROBÓTICA COMPUTACIONAL
% Cinemática Inversa por Gradiente Descendente
% Aluno: [SEU NOME]
% Data: [DATA]

clear; clc; close all;

%% PARÂMETROS GLOBAIS
n = 0.05;          % Taxa de aprendizado
xi = 1e-4;         % Critério de convergência
max_iter = 1000;   % Máximo de iterações

%% CONFIGURAÇÕES DOS ROBÔS
L_abb = [3, 3, 3];  % Comprimentos dos links ABB
theta_init_abb = [0, 0, 0];

%% FUNÇÕES AUXILIARES

% Matriz de transformação homogênea DH
function T = dh_matrix(theta, d, a, alpha)
    T = [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha)  a*cos(theta);
         sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha)  a*sin(theta);
         0           sin(alpha)             cos(alpha)             d;
         0           0                       0                      1];
end

% Cinemática direta para ABB 3DoF
function T = fk_abb(theta, L)
    T01 = dh_matrix(theta(1), L(1), 0, -pi/2);
    T12 = dh_matrix(theta(2), 0, L(2), 0);
    T23 = dh_matrix(theta(3), 0, L(3), 0);
    T = T01 * T12 * T23;
end

% Função para plotar o robô ABB 3DoF
function h = plot_abb_robot(theta, L, color, line_width)
    T01 = dh_matrix(theta(1), L(1), 0, -pi/2);
    T02 = T01 * dh_matrix(theta(2), 0, L(2), 0);
    T03 = T02 * dh_matrix(theta(3), 0, L(3), 0);
    
    p0 = [0; 0; 0];
    p1 = T01(1:3, 4);
    p2 = T02(1:3, 4);
    p3 = T03(1:3, 4);
    
    h = plot3([p0(1), p1(1), p2(1), p3(1)], ...
              [p0(2), p1(2), p2(2), p3(2)], ...
              [p0(3), p1(3), p2(3), p3(3)], ...
              'o-', 'Color', color, 'LineWidth', line_width, ...
              'MarkerSize', 6, 'MarkerFaceColor', color);
end

% Cálculo do erro entre transformações
function error = calculate_error(T_target, T_current)
    error_pos = T_target(1:3,4) - T_current(1:3,4);
    error_rot = 0.5 * (cross(T_current(1:3,1), T_target(1:3,1)) + ...
                  cross(T_current(1:3,2), T_target(1:3,2)) + ...
                  cross(T_current(1:3,3), T_target(1:3,3)));
    error = [error_pos; error_rot];
end

%% CINEMÁTICA INVERSA - GRADIENTE DESCENDENTE

function [theta_opt, error_history] = gradient_descent_abb(T_target, theta_init, L, n, xi, max_iter)
    theta = theta_init(:)';
    error_history = zeros(1, max_iter);
    
    for iter = 1:max_iter
        T_current = fk_abb(theta, L);
        error = calculate_error(T_target, T_current);
        error_norm = norm(error);
        error_history(iter) = error_norm;
        
        if error_norm < xi
            error_history = error_history(1:iter);
            break;
        end
        
        % Jacobiano numérico
        J = zeros(6,3);
        delta = 1e-6;
        for i = 1:3
            theta_plus = theta;
            theta_plus(i) = theta_plus(i) + delta;
            T_plus = fk_abb(theta_plus, L);
            
            J(1:3,i) = (T_plus(1:3,4) - T_current(1:3,4)) / delta;
            J(4:6,i) = 0.5 * (cross(T_current(1:3,1), T_plus(1:3,1)) + ...
                          cross(T_current(1:3,2), T_plus(1:3,2)) + ...
                          cross(T_current(1:3,3), T_plus(1:3,3))) / delta;
        end
        
        % Atualização com regularização
        lambda = 0.01;
        theta = theta + n * (J' / (J*J' + lambda*eye(6)) * error)';
        theta = atan2(sin(theta), cos(theta));
    end
    
    if iter == max_iter && error_norm >= xi
        fprintf('Máximo de iterações: %d, Erro: %e\n', iter, error_norm);
    else
        fprintf('Convergência: %d iterações, Erro: %e\n', iter, error_norm);
    end
    
    theta_opt = theta;
end

%% EXECUÇÃO PRINCIPAL - ABB 3DoF

fprintf('=== CINEMÁTICA INVERSA - ABB 3DoF ===\n');

% Configuração da trajetória
theta_current = theta_init_abb;
results_abb = zeros(7, 6);
trajectory_points = zeros(7, 3);
TH1_g = 0; TH2_g = 0; TH3_g = 0;

% Executa os 7 pontos da trajetória
for i = 1:7
    if i == 1
        TH1_goal = 0; TH2_goal = 0; TH3_goal = 0;
    else
        TH1_goal = TH1_g + deg2rad(60);
        TH2_goal = TH2_g + deg2rad(-5);
        TH3_goal = TH3_g;
    end
    
    theta_goal = [TH1_goal, TH2_goal, TH3_goal];
    T_goal = fk_abb(theta_goal, L_abb);
    
    [theta_opt, error_hist] = gradient_descent_abb(T_goal, theta_current, L_abb, n, xi, max_iter);
    
    results_abb(i, 1:3) = theta_goal;
    results_abb(i, 4:6) = theta_opt;
    
    T_current_pos = fk_abb(theta_opt, L_abb);
    trajectory_points(i,:) = T_current_pos(1:3,4)';
    
    theta_current = theta_opt;
    TH1_g = TH1_goal; TH2_g = TH2_goal; TH3_g = TH3_goal;
    
    fprintf('Ponto %d - Erro: %.6f\n', i, norm(T_goal(1:3,4) - T_current_pos(1:3,4)));
end

%% GRÁFICO 1: ANIMAÇÃO DO MOVIMENTO DO ROBÔ
fprintf('\n=== GERANDO GRÁFICOS ===\n');

figure('Position', [100, 100, 1200, 800]);

% Subplot 1 - Animação 3D
subplot(2,2,[1,3]);
hold on; grid on; axis equal;
xlim([-10, 10]); ylim([-10, 10]); zlim([-2, 10]);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Animação do Movimento - Robô ABB 3DoF');
view(45, 30);

% Plota trajetória
plot3(trajectory_points(:,1), trajectory_points(:,2), trajectory_points(:,3), ...
      'r--', 'LineWidth', 2, 'DisplayName', 'Trajetória do Efetuador');

% Animação dos pontos
robot_plot = [];
efetuador_plot = [];

for i = 1:7
    theta_anim = results_abb(i, 4:6);
    
    % Remove plots anteriores
    if ~isempty(robot_plot), delete(robot_plot); end
    if ~isempty(efetuador_plot), delete(efetuador_plot); end
    
    % Plota robô atual
    robot_plot = plot_abb_robot(theta_anim, L_abb, 'b', 3);
    
    % Plota efetuador
    T_efetuador = fk_abb(theta_anim, L_abb);
    efetuador_plot = plot3(T_efetuador(1,4), T_efetuador(2,4), T_efetuador(3,4), ...
                          'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    
    title(sprintf('Animação do Movimento - Robô ABB 3DoF\nPonto %d/7', i));
    legend('Trajetória', 'Robô', 'Efetuador', 'Location', 'best');
    drawnow;
    pause(1.0);
end

%% GRÁFICO 2: EVOLUÇÃO DO ERRO
subplot(2,2,2);

% Simula a evolução do erro para um ponto específico
theta_test = deg2rad([35, 296, 250]);
T_test = fk_abb(theta_test, L_abb);
[~, error_history] = gradient_descent_abb(T_test, theta_init_abb, L_abb, 0.05, xi, 200);

plot(1:length(error_history), error_history, 'b-', 'LineWidth', 2);
grid on;
xlabel('Iteração');
ylabel('Norma do Erro');
title('Evolução do Erro - Cinemática Inversa');
set(gca, 'YScale', 'log');

% Destaca ponto de convergência
conv_idx = find(error_history < xi, 1);
if ~isempty(conv_idx)
    hold on;
    plot(conv_idx, error_history(conv_idx), 'ro', 'MarkerSize', 8, ...
         'MarkerFaceColor', 'r');
    text(conv_idx, error_history(conv_idx)*5, ...
         sprintf('Convergência\n%d iterações', conv_idx), ...
         'HorizontalAlignment', 'center', 'FontSize', 8);
end

legend('Erro Total', 'Convergência', 'Location', 'northeast');

%% GRÁFICO 3: DESEMPENHO VS TAXA DE APRENDIZADO
subplot(2,2,4);

% Testa diferentes taxas de aprendizado
taxas = [0.01, 0.05, 0.1, 0.15, 0.2];
iteracoes = zeros(size(taxas));
erros_finais = zeros(size(taxas));

for idx = 1:length(taxas)
    [~, err_hist] = gradient_descent_abb(T_test, theta_init_abb, L_abb, taxas(idx), xi, 300);
    iteracoes(idx) = length(err_hist);
    erros_finais(idx) = err_hist(end);
end

% Gráfico de barras para iterações
yyaxis left;
bar(taxas, iteracoes, 0.6, 'FaceAlpha', 0.7, 'FaceColor', [0.2 0.6 0.8]);
ylabel('Iterações até Convergência');
ylim([0, max(iteracoes)*1.2]);

% Linha para erros finais
yyaxis right;
plot(taxas, erros_finais, 's-', 'LineWidth', 2, 'MarkerSize', 8, ...
     'Color', [0.8 0.2 0.2], 'MarkerFaceColor', [0.8 0.2 0.2]);
ylabel('Erro Final');
set(gca, 'YScale', 'log');

xlabel('Taxa de Aprendizado (η)');
title('Desempenho vs Taxa de Aprendizado');
grid on;

legend('Iterações', 'Erro Final', 'Location', 'northwest');

