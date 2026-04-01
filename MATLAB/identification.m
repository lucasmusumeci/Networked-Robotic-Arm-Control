clear; clc; close all;

%% 1. Import des données
data = readtable(['joint5_step.csv']);

% Adapter si besoin
t = data.t_ms/1000;
u = data.qdot_cmd;
y = data.qdot_simu;

%% 2. Détection automatique du step

u_final = mean(u(end-10:end));
y_final = mean(y(end-10:end));

K = y_final / u_final;

%% 3. Estimation de tau (63%)

y_63 = 0.632 * y_final;

% Trouver l'indice du passage à 63%
idx = find(y >= y_63, 1);

tau = t(idx);

fprintf('Gain K = %.4f\n', K);
fprintf('Constante de temps tau = %.4f s\n', tau);

%% 4. Modèle simulé (1er ordre)

y_model = K * u_final * (1 - exp(-t / tau));

%% 5. Comparaison

figure;
plot(t, y, 'b', 'LineWidth', 1.5); hold on;
plot(t, y_model, 'r--', 'LineWidth', 1.5);
grid on;

legend('Mesure', 'Modèle 1er ordre');
xlabel('Temps (ms)');
ylabel('Vitesse (rad/s)');
title('Identification sans toolbox');