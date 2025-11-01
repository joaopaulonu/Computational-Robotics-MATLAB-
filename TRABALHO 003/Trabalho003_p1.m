%% CINEMATICA DIRETA SIMBOLICA - ROBO PLANAR 2R
% Autor: Parceiro de Programacao
% Objetivo: Obter a matriz de transformacao homogenea T_2^0 a partir dos
%           parametros Denavit-Hartenberg fornecidos.

clear; clc;
% --- 1. Definicao de Variaveis Simbolicas ---
% Variaveis de junta
syms theta1 theta2 real 
% Comprimentos de elo
syms L1 L2 real 

% --- 2. Tabela DH ---
% Parametros DH: [a, alpha, d, theta]
a = [L1, L2];
alpha = [0, 0];
d = [0, 0];
theta = [theta1, theta2];

% --- 3. Funcao da Matriz de Transformacao A_i (DH padrao) ---
% A = @(a_i, alpha_i, d_i, theta_i) [ R3x3 | Px,y,z; 000 | 1 ]
A = @(a_i, alpha_i, d_i, theta_i) [
    cos(theta_i), -sin(theta_i)*cos(alpha_i), sin(theta_i)*sin(alpha_i), a_i*cos(theta_i);
    sin(theta_i), cos(theta_i)*cos(alpha_i), -cos(theta_i)*sin(alpha_i), a_i*sin(theta_i);
    0, sin(alpha_i), cos(alpha_i), d_i;
    0, 0, 0, 1
];

% --- 4. Calculo das Matrizes A_i e T_2^0 ---

% Matriz de Transformacao A1 (T_1^0)
A1 = simplify(A(a(1), alpha(1), d(1), theta(1))); 
disp('Matriz de Transformação Homogênea A1 (T_1^0):');
disp(A1);

% Matriz de Transformacao A2 (T_2^1)
A2 = simplify(A(a(2), alpha(2), d(2), theta(2))); 
disp('Matriz de Transformação Homogênea A2 (T_2^1):');
disp(A2);

% Matriz de Transformacao Homogenea total: T_2^0 = A1 * A2
T_0_to_2 = simplify(A1 * A2);
disp('Matriz de Transformação Homogênea Total T_2^0 (Cinemática Direta):');
disp(T_0_to_2);
