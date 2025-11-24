%% Configuração Inicial e Definição de Variáveis

% Limpa o workspace, fecha figuras e limpa a janela de comando.
clear all
close all
clc

% Define as variáveis simbólicas para as dimensões e ângulos das juntas.
syms th dx dy dz L1 L2 L3 L4 L5 L6 L7 real

% Variáveis de junta para cada grau de liberdade.
syms th1 th2 th3 th4 th5 th6 th7 real

% Matrizes genéricas de translação e rotação para a cinemática.
T = [1 0 0 dx
0 1 0 dy
0 0 1 dz
0 0 0 1];
Rx = [1 0 0 0
0 cos(th) -sin(th) 0
0 sin(th) cos(th) 0
0 0 0 1];
Rz = [cos(th) -sin(th) 0 0
sin(th) cos(th) 0 0
0 0 1 0
0 0 0 1];

%% Encadeamentode Frames e Cinemática Direta
% Frame {0} (base do robô), a matriz identidade.
F0 = eye(4);
% Encadeamento de transformações para cada junta.
Tr_0_1 = double(subs(T , [dx dy dz] , [0 0 0]));
TH_0_1 = Tr_0_1 * subs(Rz, th, th1);
F1 = (F0 * TH_0_1);
Tr_1_2 = subs(T , [dx dy dz] , [0 0 L1]) * subs(Rx , th, deg2rad(+90));
TH_1_2 = Tr_1_2 * subs(Rz, th, th2);
F2 = (F1 * TH_1_2);
Tr_2_3 = subs(T , [dx dy dz] , [0 L2 0])* subs(Rx , th, deg2rad(-90));
TH_2_3 = Tr_2_3 * subs(Rz, th, th3);
F3 = (F2 * TH_2_3);
Tr_3_4 = subs(T , [dx dy dz] , [0 0 L3])* subs(Rx , th, deg2rad(+90));
TH_3_4 = Tr_3_4 * subs(Rz, th, th4);
F4 = (F3 * TH_3_4);
Tr_4_5 = subs(T , [dx dy dz] , [0 L4 0])* subs(Rx , th, deg2rad(-90));
TH_4_5 = Tr_4_5 * subs(Rz, th, th5);
F5 = (F4 * TH_4_5);
Tr_5_6 = subs(T , [dx dy dz] , [0 0 L5])* subs(Rx , th, deg2rad(+90));
TH_5_6 = Tr_5_6 * subs(Rz, th, th6);
F6 = (F5 * TH_5_6);
Tr_6_7 = subs(T , [dx dy dz] , [0 L6 0])* subs(Rx , th, deg2rad(-90));
TH_6_7 = Tr_6_7 * subs(Rz, th, th7);
F7 = (F6 * TH_6_7);
% Frame {8} (punho do robô).
Tr_7_8 = subs(T , [dx dy dz] , [0 0 L7]);
F8 = (F7 * Tr_7_8);

%% Atribuição de valores e cálculo numérico
% Valores de comprimento dos elos em metros.
l1 = 0.18;
l2 = 0.169;
l3 = 0.159;
l4=0.14825;l5=0.12825;l6=0.12585;
l7=0.04585;
% Posição das juntas em graus, convertida para radianos.
t1 = deg2rad(0);
t2 = deg2rad(0);
t3 = deg2rad(0);
t4 = deg2rad(0);
t5 = deg2rad(0);
t6 = deg2rad(0);
t7 = deg2rad(45);
% Substitui as variáveis simbólicas por valores numéricos.
F1 = double(subs(F1 , th1 , t1));
F2 = double(subs(F2 , [L1 th1 th2], [l1 t1 t2]));
F3 = double(subs(F3 , [L2 L1 th1 th2 th3], [l2 l1 t1 t2 t3]));
F4 = double(subs(F4 , [L3 L2 L1 th1 th2 th3 th4], [l3 l2 l1 t1 t2 t3 t4]));
F5 = double(subs(F5 , [L4 L3 L2 L1 th1 th2 th3 th4 th5], [l4 l3 l2 l1 t1 t2 t3 t4 t5]));
F6 = double(subs(F6 , [L5 L4 L3 L2 L1 th1 th2 th3 th4 th5 th6], [l5 l4 l3 l2 l1 t1 t2 t3 t4 t5 t6]));
F7 = double(subs(F7 , [L6 L5 L4 L3 L2 L1 th1 th2 th3 th4 th5 th6 th7], [l6 l5 l4 l3 l2 l1 t1 t2 t3 t4 t5 t6 t7]));
F8 = double(subs(F8 , [L7 L6 L5 L4 L3 L2 L1 th1 th2 th3 th4 th5 th6 th7], [l7 l6 l5 l4 l3 l2 l1 t1 t2 t3 t4 t5 t6 t7]));
% Fatores de escala e marcação para o plot.
esc = 0.05;
mark=10;

%% Visualização Gráfica do Robô
% Configura a figura para plotagem 3D.
plot3(F0(1,4) , F0(2,4) , F0(3,4) , 'om', 'linewidth', 2 , 'markersize', mark);
axis equal;
grid on;
xlabel('x'); ylabel('y'); zlabel('z');
hold on;
% Plota o frame {0}.
plot3([F0(1,4) F0(1,4)+esc*F0(1,1)] , [F0(2,4) F0(2,4)+esc*F0(2,1)] , ...
[F0(3,4) F0(3,4)+esc*F0(3,1)] , 'b', 'linewidth', 2)
text(F0(1,4)+esc*F0(1,1) , F0(2,4)+esc*F0(2,1) , F0(3,4)+esc*F0(3,1) , 'x_{\{0\}}')
plot3([F0(1,4) F0(1,4)+esc*F0(1,2)] , [F0(2,4) F0(2,4)+esc*F0(2,2)] , [F0(3,4) F0(3,4)+esc*F0(3,2)] , 'g', 'linewidth', 2)
text(F0(1,4)+esc*F0(1,2) , F0(2,4)+esc*F0(2,2) , F0(3,4)+esc*F0(3,2) , 'y_{\{0\}}')
plot3([F0(1,4) F0(1,4)+esc*F0(1,3)] , [F0(2,4) F0(2,4)+esc*F0(2,3)] , [F0(3,4) F0(3,4)+esc*F0(3,3)] , 'r', 'linewidth', 2)
text(F0(1,4)+esc*F0(1,3) , F0(2,4)+esc*F0(2,3) , F0(3,4)+esc*F0(3,3) , 'z_{\{0\}}')
plot3([F0(1,4) F1(1,4)] , [F0(2,4) F1(2,4)] , [F0(3,4) F1(3,4)] , 'k')
% Plota o frame {1}.
plot3(F1(1,4) , F1(2,4) , F1(3,4) , 'om', 'linewidth', 2 , 'markersize', mark);
text(F1(1,4) , F1(2,4) , F1(3,4)-0.2 , '\{1\}')
plot3([F1(1,4) F1(1,4)+esc*F1(1,1)] , [F1(2,4) F1(2,4)+esc*F1(2,1)] , [F1(3,4) F1(3,4)+esc*F1(3,1)] , 'b', 'linewidth', 2)
text(F1(1,4)+esc*F1(1,1) , F1(2,4)+esc*F1(2,1) , F1(3,4)+esc*F1(3,1) , 'x_{\{1\}}')
plot3([F1(1,4) F1(1,4)+esc*F1(1,2)] , [F1(2,4) F1(2,4)+esc*F1(2,2)] , [F1(3,4) F1(3,4)+esc*F1(3,2)] , 'g', 'linewidth', 2)
text(F1(1,4)+esc*F1(1,2) , F1(2,4)+esc*F1(2,2) , F1(3,4)+esc*F1(3,2) , 'y_{\{1\}}')
plot3([F1(1,4) F1(1,4)+esc*F1(1,3)] , [F1(2,4) F1(2,4)+esc*F1(2,3)] , [F1(3,4) F1(3,4)+esc*F1(3,3)] , 'r', 'linewidth', 2)
text(F1(1,4)+esc*F1(1,3) , F1(2,4)+esc*F1(2,3) , F1(3,4)+esc*F1(3,3) , 'z_{\{1\}}')
plot3([F1(1,4) F2(1,4)] , [F1(2,4) F2(2,4)] , [F1(3,4) F2(3,4)] , 'k')
% Plota o frame {2}.
plot3(F2(1,4) , F2(2,4) , F2(3,4) , 'om', 'linewidth', 2 , 'markersize', mark);
text(F2(1,4) , F2(2,4) , F2(3,4)-0.2 , '\{2\}')
plot3([F2(1,4) F2(1,4)+esc*F2(1,1)] , [F2(2,4) F2(2,4)+esc*F2(2,1)] , [F2(3,4) F2(3,4)+esc*F2(3,1)] , 'b', 'linewidth', 2)
text(F2(1,4)+esc*F2(1,1) , F2(2,4)+esc*F2(2,1) , F2(3,4)+esc*F2(3,1) , 'x_{\{2\}}')
plot3([F2(1,4) F2(1,4)+esc*F2(1,2)] , [F2(2,4) F2(2,4)+esc*F2(2,2)] , [F2(3,4) F2(3,4)+esc*F2(3,2)] , 'g', 'linewidth', 2)
text(F2(1,4)+esc*F2(1,2) , F2(2,4)+esc*F2(2,2) , F2(3,4)+esc*F2(3,2) , 'y_{\{2\}}')
plot3([F2(1,4) F2(1,4)+esc*F2(1,3)] , [F2(2,4) F2(2,4)+esc*F2(2,3)] , [F2(3,4) F2(3,4)+esc*F2(3,3)] , 'r', 'linewidth', 2)
text(F2(1,4)+esc*F2(1,3) , F2(2,4)+esc*F2(2,3) , F2(3,4)+esc*F2(3,3) , 'z_{\{2\}}')
plot3([F2(1,4) F3(1,4)] , [F2(2,4) F3(2,4)] , [F2(3,4) F3(3,4)] , 'k')
% Plota o frame {3}.
plot3(F3(1,4), F3(2,4), F3(3,4), 'om','linewidth',2,'markersize',mark);
text(F3(1,4), F3(2,4), F3(3,4)-0.2, '\{3\}')
plot3([F3(1,4) F3(1,4)+esc*F3(1,1)], [F3(2,4) F3(2,4)+esc*F3(2,1)], [F3(3,4) F3(3,4)+esc*F3(3,1)], 'b','linewidth',2)
text(F3(1,4)+esc*F3(1,1), F3(2,4)+esc*F3(2,1), F3(3,4)+esc*F3(3,1), 'x_{\{3\}}')
plot3([F3(1,4) F3(1,4)+esc*F3(1,2)], [F3(2,4) F3(2,4)+esc*F3(2,2)], [F3(3,4) F3(3,4)+esc*F3(3,2)], 'g','linewidth',2)
text(F3(1,4)+esc*F3(1,2), F3(2,4)+esc*F3(2,2), F3(3,4)+esc*F3(3,2), 'y_{\{3\}}')
plot3([F3(1,4) F3(1,4)+esc*F3(1,3)], [F3(2,4) F3(2,4)+esc*F3(2,3)], [F3(3,4) F3(3,4)+esc*F3(3,3)], 'r','linewidth',2)
text(F3(1,4)+esc*F3(1,3), F3(2,4)+esc*F3(2,3), F3(3,4)+esc*F3(3,3), 'z_{\{3\}}')
plot3([F3(1,4) F4(1,4)], [F3(2,4) F4(2,4)], [F3(3,4) F4(3,4)], 'k')
% Plota o frame {4}.
plot3(F4(1,4), F4(2,4), F4(3,4), 'om','linewidth',2,'markersize',mark);
text(F4(1,4), F4(2,4), F4(3,4)-0.2, '\{4\}')
plot3([F4(1,4) F4(1,4)+esc*F4(1,1)], [F4(2,4) F4(2,4)+esc*F4(2,1)], [F4(3,4) F4(3,4)+esc*F4(3,1)], 'b','linewidth',2)
text(F4(1,4)+esc*F4(1,1), F4(2,4)+esc*F4(2,1), F4(3,4)+esc*F4(3,1), 'x_{\{4\}}')
plot3([F4(1,4) F4(1,4)+esc*F4(1,2)], [F4(2,4) F4(2,4)+esc*F4(2,2)], [F4(3,4) F4(3,4)+esc*F4(3,2)], 'g','linewidth',2)
text(F4(1,4)+esc*F4(1,2), F4(2,4)+esc*F4(2,2), F4(3,4)+esc*F4(3,2), 'y_{\{4\}}')
plot3([F4(1,4) F4(1,4)+esc*F4(1,3)], [F4(2,4) F4(2,4)+esc*F4(2,3)], [F4(3,4) F4(3,4)+esc*F4(3,3)], 'r','linewidth',2)
text(F4(1,4)+esc*F4(1,3), F4(2,4)+esc*F4(2,3), F4(3,4)+esc*F4(3,3), 'z_{\{4\}}')
plot3([F4(1,4) F5(1,4)], [F4(2,4) F5(2,4)], [F4(3,4) F5(3,4)], 'k')
% Plota o frame {5}.
plot3(F5(1,4), F5(2,4), F5(3,4), 'om','linewidth',2,'markersize',mark);
text(F5(1,4), F5(2,4), F5(3,4)-0.2, '\{5\}')
plot3([F5(1,4) F5(1,4)+esc*F5(1,1)], [F5(2,4) F5(2,4)+esc*F5(2,1)], [F5(3,4) F5(3,4)+esc*F5(3,1)], 'b','linewidth',2)
text(F5(1,4)+esc*F5(1,1), F5(2,4)+esc*F5(2,1), F5(3,4)+esc*F5(3,1), 'x_{\{5\}}')
plot3([F5(1,4) F5(1,4)+esc*F5(1,2)], [F5(2,4) F5(2,4)+esc*F5(2,2)], [F5(3,4) F5(3,4)+esc*F5(3,2)], 'g','linewidth',2)
