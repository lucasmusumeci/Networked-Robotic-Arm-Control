clear; clc; close all;

%% =========================================================
%% 1. Import des données
%% =========================================================

data = readtable('joint0_step.csv');

t = data.t_ms / 1000;   % Conversion ms -> s
u = data.qdot_cmd;
y = data.qdot_simu;

dt = mean(diff(t));

%% =========================================================
%% 2. Détection du début du step et du retard Trc
%%    Méthode : on détecte quand l'entrée change, puis
%%    quand la sortie commence réellement à réagir
%% =========================================================

% --- Détection du début du step sur l'entrée ---
seuil_u = mean(u(1:10)) + 0.05 * (max(u) - mean(u(1:10)));
idx_step = find(u >= seuil_u, 1);

if isempty(idx_step)
    error('Impossible de détecter le début du step sur l''entrée.');
end
t_step = t(idx_step);
fprintf('=== Identification 1er ordre avec retard ===\n');
fprintf('Début du step détecté à t = %.4f s\n', t_step);

% --- Détection de la réaction de la sortie ---
% La sortie commence à réagir quand elle dépasse son niveau initial de 2%
y_init  = mean(y(1:idx_step));
seuil_y = y_init + 0.02 * (max(y) - y_init);
idx_reaction = find(y(idx_step:end) >= seuil_y, 1) + idx_step - 1;

if isempty(idx_reaction)
    error('Impossible de détecter la réaction de la sortie.');
end
t_reaction = t(idx_reaction);

% --- Retard estimé ---
Trc_estime = t_reaction - t_step;
fprintf('Réaction de la sortie détectée à t = %.4f s\n', t_reaction);
fprintf('Retard estimé : Trc = %.4f s\n\n', Trc_estime);

%% =========================================================
%% 3. Identification de K et tau (sur signal décalé)
%%    On travaille uniquement à partir de t_reaction
%%    pour que le modèle 1er ordre soit bien calé
%% =========================================================

% Extraction de la fenêtre utile (après le retard)
idx_debut = idx_reaction;
t_id   = t(idx_debut:end) - t_reaction;   % Remise à zéro du temps
y_id   = y(idx_debut:end);
u_id   = u(idx_debut:end);

% Gain statique K
u_final = mean(u(end-10:end));
y_final = mean(y(end-10:end));
y_init  = mean(y(1:idx_step));             % Valeur initiale avant le step

K = (y_final - y_init) / u_final;
fprintf('Gain statique K = %.4f\n', K);

% Constante de temps tau (méthode des 63%)
% La sortie doit atteindre y_init + 63.2% * (y_final - y_init)
y_63  = y_init + 0.632 * (y_final - y_init);
idx63 = find(y_id >= y_63, 1);

if isempty(idx63)
    error('La sortie n''atteint pas 63%% de sa valeur finale.');
end

tau = t_id(idx63);
fprintf('Constante de temps tau = %.4f s\n', tau);
fprintf('Retard Trc = %.4f s\n\n', Trc_estime);

%% =========================================================
%% 4. Reconstruction du modèle complet avec retard
%%    y_model(t) = 0                            si t < t_step + Trc
%%    y_model(t) = y_init + K*u*(1-exp(-t/tau)) si t >= t_step + Trc
%% =========================================================

y_model = y_init * ones(size(t));   % Valeur initiale partout

% Indice à partir duquel le modèle démarre (après step + retard)
t_debut_modele = t_step + Trc_estime;
idx_modele = find(t >= t_debut_modele, 1);

t_modele = t(idx_modele:end) - t_debut_modele;   % Temps local depuis 0
y_model(idx_modele:end) = y_init + K * u_final * (1 - exp(-t_modele / tau));

%% =========================================================
%% 6. Comparaison mesure vs modèle
%% =========================================================

figure('Name', 'Identification 1er ordre avec retard', 'NumberTitle', 'off');
hold on;

plot(t, y,       'b-',  'LineWidth', 1.5, 'DisplayName', 'Mesure');
plot(t, y_model, 'r--', 'LineWidth', 2.0, 'DisplayName', 'Modèle 1er ordre + retard');

% Ligne verticale : début du step
xline(t_step,    'k:',  'LineWidth', 1.5, ...
    'Label', 'Début step', 'FontSize', 10, 'LabelVerticalAlignment', 'bottom');

% Ligne verticale : début de la réaction (step + Trc)
xline(t_debut_modele, 'm--', 'LineWidth', 1.5, ...
    'Label', sprintf('Trc = %.3f s', Trc_estime), ...
    'FontSize', 10, 'LabelVerticalAlignment', 'bottom');

% Annotation tau (63%)
t_tau_abs = t_debut_modele + tau;
y_tau_val = y_init + K * u_final * (1 - exp(-1));
plot(t_tau_abs, y_tau_val, 'gs', 'MarkerSize', 10, 'LineWidth', 2, ...
    'DisplayName', sprintf('\\tau = %.3f s (63%%)', tau));

grid on;
legend('Location', 'southeast', 'FontSize', 11);
xlabel('Temps (s)', 'FontSize', 12);
ylabel('Vitesse (rad/s)', 'FontSize', 12);
title(sprintf('Identification 1^{er} ordre avec retard\nK=%.4f | \\tau=%.4f s | Trc=%.4f s | RMSE=%.4f', ...
    K, tau, Trc_estime), 'FontSize', 13);
hold off;

%% =========================================================
%% 7. Récapitulatif console
%% =========================================================

fprintf('========================================\n');
fprintf('  PARAMÈTRES IDENTIFIÉS\n');
fprintf('========================================\n');
fprintf('  Gain statique      K   = %.4f\n',   K);
fprintf('  Constante de temps tau = %.4f s\n', tau);
fprintf('  Retard constant    Trc = %.4f s\n', Trc_estime);
fprintf('========================================\n');