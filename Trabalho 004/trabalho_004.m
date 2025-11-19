%% Robo Schunk LWA3 7DoF (MATLAB)
% Cinemática Inversa Diferencial (Damped Least Squares - DLS)

clear; clc;
close all;

%% 1. Setup Simbólico e Matrizes D-H
% Variáveis simbólicas D-H
syms d th a alpha real

% Matrizes de Transformação D-H
Tx = [1 0 0 a; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Tz = [1 0 0 0; 0 1 0 0; 0 0 1 d; 0 0 0 1];
Rx = [1 0 0 0; 0 cos(alpha) -sin(alpha) 0; 0 sin(alpha) cos(alpha) 0; 0 0 0 1];
Rz = [cos(th) -sin(th) 0 0; sin(th) cos(th) 0 0; 0 0 1 0; 0 0 0 1];

% Matriz D-H genérica
A = Tz * Rz * Tx * Rx;

%% 2. Parâmetros do Robô e Cinemática Direta Simbólica
% Variáveis simbólicas das juntas e comprimentos
syms th1 th2 th3 th4 th5 th6 th7 real 
syms d1 d2 d3 d4 d5 d6 d7 real 

% Matrizes de Transformação Homogênea (T_{i-1}^{i})
I = eye(4);
A0 = subs(A, [d th a alpha], [0 th1 0 0]);
A1 = subs(A, [d th a alpha], [d1 0 0 pi/2]);
A2 = subs(A, [d th a alpha], [0 th2 -d2 -pi/2]);
A3 = subs(A, [d th a alpha], [d3 th3 0 pi/2]);
A4 = subs(A, [d th a alpha], [0 th4 -d4 -pi/2]);
A5 = subs(A, [d th a alpha], [d5 th5 0 pi/2]);
A6 = subs(A, [d th a alpha], [0 th6 -d6 -pi/2]);
A7 = subs(A, [d th a alpha], [d7 th7 0 pi/2]);

% Transformações cumulativas (T_{0}^{i})
F0 = I; F1 = A0 * A1; F2 = F1 * A2; F3 = F2 * A3;
F4 = F3 * A4; F5 = F4 * A5; F6 = F5 * A6; F7 = F6 * A7; % End-effector

% Posição simbólica do End-effector (coluna 4)
V_J7 = F7(1:3,4);

% Vetores de variáveis simbólicas
DH_VARS = [d1 d2 d3 d4 d5 d6 d7 th1 th2 th3 th4 th5 th6 th7];
POS_TRANSFORMS = {F0, F1, F2, F3, F4, F5, F6, F7};

%% 3. Setup de Cinemática Inversa e Jacobiano
% Comprimentos dos links (valores numéricos)
D1 = 0.5; D2 = 0.5; D3 = 0.5; D4 = 0.5; D5 = 0.5; D6 = 0.5; D7 = 0.5;
D_VALS = [D1 D2 D3 D4 D5 D6 D7];

% Jacobiano Posicional Simbólico (3x7)
J_end = jacobian(V_J7, [th1 th2 th3 th4 th5 th6 th7]);

% Caminho Desejado (Hélice/Parábola)
N_range = -2:0.05:2;
Caminho.x = cos(2*pi*(N_range/4));
Caminho.y = sin(2*pi*(N_range/4));
Caminho.z = (N_range.^2)/2;

% Posição base (origem)
base.x = 0; base.y = 0; base.z = 0;

% Juntas iniciais e hiperparâmetros
Theta = zeros(7, 1);
n = 0.08;   % Taxa de aprendizado
ksi = 1e-4; % Limiar de erro (EQM)
epsi = 1e-4; % Fator de regularização DLS (estabilidade)
Pose = [];    % Armazena a trajetória do robô
PoseTH = [];  % Armazena o histórico das juntas

% Limites de Junta (para evitar instabilidade simbólica)
THETA_MAX = 2 * pi; 
THETA_MIN = -2 * pi;

%% 4. Inicialização da Visualização 3D
figure('Name', 'Simulação do Robô Schunk LWA3 - Cinemática Inversa', 'NumberTitle', 'off');
hold on;
grid on;
axis equal; % Proporções corretas
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Robô Schunk LWA3 - Seguimento de Trajetória');
view(3); % Vista 3D inicial

% Plot do caminho desejado (completo, uma vez)
plot3(Caminho.x, Caminho.y, Caminho.z, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Caminho Desejado');

% Inicialização dos objetos gráficos para o robô
% Handles para atualizar sem redesenhar tudo
h_robot_links = plot3(NaN, NaN, NaN, 'r', 'LineWidth', 4, 'DisplayName', 'Links do Robô');
h_robot_joints = plot3(NaN, NaN, NaN, 'o', 'MarkerFaceColor', [0.5 0.5 0.5], 'MarkerSize', 8, 'DisplayName', 'Juntas do Robô');
h_effector_path = plot3(NaN, NaN, NaN, 'm-', 'LineWidth', 2, 'DisplayName', 'Trajetória do End-Effector');
h_current_target = plot3(NaN, NaN, NaN, 'g*', 'MarkerSize', 10, 'DisplayName', 'Alvo Atual');
h_effector_pos = plot3(NaN, NaN, NaN, 'b.', 'MarkerSize', 15, 'DisplayName', 'Posição End-Effector');

legend('show', 'Location', 'best'); % Mostra legenda

%% 5. Loop de Seguimento de Caminho (Gradiente Descendente DLS)
for i = 1:length(N_range)
    % Ponto alvo atual
    G = [Caminho.x(i); Caminho.y(i); Caminho.z(i)];
    
    % Valores numéricos para substituição
    THETA_VALS = Theta';
    DH_VALS_CURRENT = [D_VALS, THETA_VALS];

    % Avalia F7 (Cinemática Direta)
    P_J7_Transform = double(subs(F7, DH_VARS, DH_VALS_CURRENT));
    P_J7 = P_J7_Transform(1:3, 4);
    EQM = mean((G - P_J7).^2); % Erro quadrático médio

    % Loop de CI: converge para G
    iter_count = 0; % Contador de iterações do loop interno
    MAX_ITER = 500; % Limite para evitar loops infinitos
    while (EQM > ksi && iter_count < MAX_ITER)
        iter_count = iter_count + 1;

        % Jacobiano Numérico
        J = double(subs(J_end, DH_VARS, DH_VALS_CURRENT));

        % Gradiente Cartesiano (Erro)
        GradEQM = -(G - P_J7);

        % DLS: J'J + epsi*I (Tratamento de singularidade)
        A_dls = J' * J + epsi * eye(size(J, 2)); 
        
        % Gradiente no Espaço de Juntas: (A_dls)^-1 * J' * GradEQM
        GradTH = (A_dls \ J') * GradEQM; 

        % Atualização das Juntas
        Theta = Theta - n * GradTH;
        
        % Limitação dos Ângulos das Juntas (Clamping)
        Theta = max(Theta, THETA_MIN);
        Theta = min(Theta, THETA_MAX);

        % Armazenando histórico
        PoseTH = [PoseTH, Theta];

        % Recalcula Posição e Erro
        THETA_VALS = Theta';
        DH_VALS_CURRENT = [D_VALS, THETA_VALS];
        
        P_J7_Transform = double(subs(F7, DH_VARS, DH_VALS_CURRENT));
        P_J7 = P_J7_Transform(1:3, 4);
        EQM = mean((G - P_J7).^2);
    end

    % Armazena a pose final do robô para este ponto
    P_all = [];
    for k = 1:numel(POS_TRANSFORMS)
        T_k = double(subs(POS_TRANSFORMS{k}, DH_VARS, DH_VALS_CURRENT));
        P_all = [P_all, T_k(1:3, 4)'];
    end
    Pose = [Pose; P_all];

    %% 6. Atualização da Visualização 3D
    P_current = reshape(Pose(end, :), 3, 8); % Posições das Juntas (J0 a J7)

    % Atualiza links do robô
    set(h_robot_links, 'XData', P_current(1, :), 'YData', P_current(2, :), 'ZData', P_current(3, :));
    
    % Atualiza juntas do robô
    set(h_robot_joints, 'XData', P_current(1, :), 'YData', P_current(2, :), 'ZData', P_current(3, :));
    
    % Atualiza trajetória do end-effector
    set(h_effector_path, 'XData', Pose(:, 22), 'YData', Pose(:, 23), 'ZData', Pose(:, 24));
    
    % Atualiza alvo atual
    set(h_current_target, 'XData', G(1), 'YData', G(2), 'ZData', G(3));
    
    % Atualiza posição atual do end-effector
    set(h_effector_pos, 'XData', P_J7(1), 'YData', P_J7(2), 'ZData', P_J7(3));

    drawnow limitrate; % Atualiza o gráfico de forma eficiente
end

hold off; % Finaliza a edição do gráfico

%% 7. Plot Opcional de Convergência das Juntas
% Plota a variação dos 7 ângulos de junta ao longo das iterações

figure('Name', 'Convergência das Juntas');
plot(PoseTH');
title('Convergência das Juntas (Theta)');
xlabel('Iterações');
ylabel('Ângulo (rad)');
grid on;
