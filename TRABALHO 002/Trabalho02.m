%% Script MATLAB para Cinemática Direta do Robô Articulado (RRR)
% Autor: Parceiro de Programacao
% Objetivo: Obter a matriz de transformacao homogenea T_3^0 usando DH simbolicamente.

% --- 1. Definicao de Variaveis Simbolicas ---
% Variaveis de junta (Rotacionais: theta)
syms theta1 theta2 theta3 real 
% Parametros constantes do link (Comprimentos 'a' e Offsets 'd')
syms a2 a3 d1 real 
% Parametros de torsao (alpha)
alpha = [-pi/2, 0, 0]; % [-90 deg, 0 deg, 0 deg] em radianos

% Agrupando os parametros de DH para facilitar
a = [0, a2, a3];
d = [d1, 0, 0];
theta = [theta1, theta2, theta3]; % As variaveis de junta
num_juntas = 3;

% --- 2. Funcao da Matriz de Transformacao A_i (DH padrao) ---
% Define uma funcao anonima para a matriz de transformacao homogenea A_i
% Recebe os parametros DH: a_i, alpha_i, d_i, theta_i
A = @(a_i, alpha_i, d_i, theta_i) [
    cos(theta_i), -sin(theta_i)*cos(alpha_i), sin(theta_i)*sin(alpha_i), a_i*cos(theta_i);
    sin(theta_i), cos(theta_i)*cos(alpha_i), -cos(theta_i)*sin(alpha_i), a_i*sin(theta_i);
    0, sin(alpha_i), cos(alpha_i), d_i;
    0, 0, 0, 1
];

% --- 3. Calculo da Matriz de Cinemática Direta (T_3^0) ---
T_0_to_1 = A(a(1), alpha(1), d(1), theta(1)); % Matriz A1 (T_1^0)
T_1_to_2 = A(a(2), alpha(2), d(2), theta(2)); % Matriz A2 (T_2^1)
T_2_to_3 = A(a(3), alpha(3), d(3), theta(3)); % Matriz A3 (T_3^2)

% Matriz de Transformacao Homogenea total: T_3^0 = A1 * A2 * A3
T_0_to_3 = simplify(T_0_to_1 * T_1_to_2 * T_2_to_3);

% --- 4. Exibicao dos Resultados ---
fprintf('=== Equacionamento da Cinemática Direta - Robô Articulado (RRR) ===\n\n');
disp('Matriz de Transformação Homogênea T_3^0:');
disp(T_0_to_3);

% Extraindo a posicao (Px, Py, Pz) do efetuador final
Px = T_0_to_3(1, 4);
Py = T_0_to_3(2, 4);
Pz = T_0_to_3(3, 4);

fprintf('\n=== Vetor Posição (x, y, z) do Efetuador Final ===\n');
disp(['Px = ', char(simplify(Px))]);
disp(['Py = ', char(simplify(Py))]);
disp(['Pz = ', char(simplify(Pz))]);
