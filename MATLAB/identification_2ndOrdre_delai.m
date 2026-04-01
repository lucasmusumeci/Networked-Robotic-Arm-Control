%% Identification d'un système du second ordre avec retard constant
%% à partir d'un fichier CSV (colonnes : temps, entree, sortie)
%% Université de Montpellier - Automatique et Réseaux

clear; clc; close all;

%% =========================================================
%% 1. CHARGEMENT DU FICHIER CSV
%% =========================================================

nom_fichier = 'joint0_step.csv';  % <-- Modifier selon votre fichier

opts = detectImportOptions(nom_fichier);
opts.VariableNamesLine = 1;
donnees = readtable(nom_fichier, opts);

% Extraction des colonnes
t      = donnees.temps;
u      = donnees.entree;
y      = donnees.sortie;

% Vérification
fprintf('=== Chargement des données ===\n');
fprintf('Nombre de points : %d\n', length(t));
fprintf('Durée totale     : %.3f s\n', t(end) - t(1));
fprintf('Pas d''échantillonnage moyen : %.4f s\n\n', mean(diff(t)));

%% =========================================================
%% 2. VISUALISATION DES DONNÉES BRUTES
%% =========================================================

figure('Name', 'Données brutes', 'NumberTitle', 'off');

subplot(2,1,1);
plot(t, u, 'b-', 'LineWidth', 1.5);
xlabel('Temps (s)', 'FontSize', 12);
ylabel('Entrée u(t)', 'FontSize', 12);
title('Signal d''entrée', 'FontSize', 13);
grid on;

subplot(2,1,2);
plot(t, y, 'r-', 'LineWidth', 1.5);
xlabel('Temps (s)', 'FontSize', 12);
ylabel('Sortie y(t)', 'FontSize', 12);
title('Signal de sortie', 'FontSize', 13);
grid on;

sgtitle('Données brutes du système', 'FontSize', 14, 'FontWeight', 'bold');

%% =========================================================
%% 3. ESTIMATION DU RETARD Trc
%%    Méthode : corrélation croisée entre entrée et sortie
%% =========================================================

fprintf('=== Estimation du retard Trc ===\n');

% Normalisation des signaux avant corrélation
u_norm = (u - mean(u)) / std(u);
y_norm = (y - mean(y)) / std(y);

[correlation, lags] = xcorr(y_norm, u_norm);
dt = mean(diff(t));
lags_temps = lags * dt;

% Retard = position du maximum de corrélation
[~, idx_max] = max(correlation);
Trc_estime = lags_temps(idx_max);

% On ne garde que des valeurs positives et raisonnables
if Trc_estime < 0
    warning('Retard estimé négatif (%.4f s), forcé à 0.', Trc_estime);
    Trc_estime = 0;
end

fprintf('Retard estimé par corrélation croisée : Trc = %.4f s\n\n', Trc_estime);

% Visualisation de la corrélation croisée
figure('Name', 'Estimation du retard', 'NumberTitle', 'off');
plot(lags_temps, correlation, 'k-', 'LineWidth', 1.5);
xline(Trc_estime, 'r--', 'LineWidth', 2, ...
    'Label', sprintf('Trc = %.3f s', Trc_estime), 'FontSize', 11);
xlabel('Décalage temporel (s)', 'FontSize', 12);
ylabel('Corrélation croisée', 'FontSize', 12);
title('Corrélation croisée entrée/sortie - Estimation du retard', 'FontSize', 13);
grid on;

%% =========================================================
%% 4. COMPENSATION DU RETARD
%%    On décale la sortie pour aligner avec l'entrée
%% =========================================================

n_retard = round(Trc_estime / dt);   % Nombre d'échantillons de retard

% Recalage : on supprime les n_retard premiers points de la sortie
% et les n_retard derniers de l'entrée pour garder des vecteurs alignés
t_comp  = t(1:end - n_retard);
u_comp  = u(1:end - n_retard);
y_comp  = y(n_retard + 1:end);

%% =========================================================
%% 5. IDENTIFICATION DU GAIN STATIQUE G
%%    G = valeur finale sortie / valeur finale entrée
%%    (valable si le signal d'entrée est un échelon)
%% =========================================================

fprintf('=== Identification du gain statique G ===\n');

% Moyenne sur les 10 derniers % des données pour estimer les valeurs finales
n_fin = max(1, round(0.10 * length(t_comp)));
y_final = mean(y_comp(end - n_fin:end));
u_final = mean(u_comp(end - n_fin:end));
y_init  = mean(y_comp(1:n_fin));
u_init  = mean(u_comp(1:n_fin));

delta_y = y_final - y_init;
delta_u = u_final - u_init;

if abs(delta_u) < 1e-10
    error('Variation de l''entrée trop faible pour estimer G.');
end

G_estime = delta_y / delta_u;
fprintf('Gain statique estimé : G = %.4f\n\n', G_estime);

%% =========================================================
%% 6. IDENTIFICATION DE ωn ET ζ PAR OPTIMISATION (lsqcurvefit)
%%    Fonction de transfert du 2nd ordre :
%%    H(p) = G * ωn² / (p² + 2ζωn*p + ωn²)
%% =========================================================

fprintf('=== Identification de ωn et ζ (lsqcurvefit) ===\n');

% Simulation de la réponse du système du 2nd ordre (sans retard)
% via la fonction de transfert discrétisée
function y_sim = simule_second_ordre(params, t_sim, u_sim, G, dt)
    wn  = params(1);
    zeta = params(2);

    % Fonction de transfert continue
    num = [G * wn^2];
    den = [1, 2*zeta*wn, wn^2];
    sys = tf(num, den);

    % Simulation avec lsim
    y_sim = lsim(sys, u_sim, t_sim);
end

% Valeurs initiales de wn et zeta
wn0   = 2.0;    % rad/s  - à ajuster selon votre système
zeta0 = 0.7;    % sans unité (0 < zeta < 1 pour sous-amorti)

params0 = [wn0, zeta0];

% Bornes : wn > 0, 0 < zeta < 2
lb = [0.01, 0.01];
ub = [100,  2.0 ];

% Fonction objectif : erreur entre sortie mesurée et simulée
objectif = @(params, t_sim) simule_second_ordre(params, t_sim, u_comp, G_estime, dt);

options_lsq = optimoptions('lsqcurvefit', ...
    'Display',       'iter', ...
    'MaxIterations', 500, ...
    'FunctionTolerance', 1e-8);

[params_opt, resnorm, residus] = lsqcurvefit(objectif, params0, t_comp, y_comp, lb, ub, options_lsq);

wn_estime   = params_opt(1);
zeta_estime = params_opt(2);

fprintf('\nRésultats de l''identification :\n');
fprintf('  Pulsation propre  ωn   = %.4f rad/s\n', wn_estime);
fprintf('  Coefficient d''amortissement ζ = %.4f\n', zeta_estime);
fprintf('  Résidu quadratique (resnorm)  = %.6f\n\n', resnorm);

%% =========================================================
%% 7. CALCUL DES GAINS Kp ET Kd DU CONTRÔLEUR
%%    Correspondance avec la FT du 2nd ordre en BF :
%%    H_BF(p) = KG / (p² + Kd*p + Kp + KG)
%%    => ωn² = Kp + KG  et  2ζωn = Kd
%% =========================================================

fprintf('=== Calcul des gains du contrôleur Kp et Kd ===\n');

KG = G_estime;  % On assimile le gain à KG (produit contrôleur * procédé)

Kd_estime = 2 * zeta_estime * wn_estime;
Kp_estime = wn_estime^2 - KG;

fprintf('  Kd = 2·ζ·ωn         = %.4f\n', Kd_estime);
fprintf('  Kp = ωn² - KG       = %.4f\n', Kp_estime);

if Kp_estime < 0
    warning('Kp estimé négatif (%.4f). Vérifiez G, ωn et ζ.', Kp_estime);
end

%% =========================================================
%% 8. VALIDATION : COMPARAISON MESURE VS MODÈLE IDENTIFIÉ
%% =========================================================

% Simulation avec les paramètres identifiés (avec retard)
num_id = [G_estime * wn_estime^2];
den_id = [1, 2*zeta_estime*wn_estime, wn_estime^2];
sys_id = tf(num_id, den_id);

y_id = lsim(sys_id, u_comp, t_comp);

% Réintroduction du retard sur la sortie simulée
y_id_retarde = [y_id(1) * ones(n_retard, 1); y_id(1:end)];
y_id_retarde = y_id_retarde(1:length(t));

% Calcul de l'erreur quadratique moyenne (RMSE)
RMSE = sqrt(mean((y - y_id_retarde).^2));
fprintf('\nRMSE (validation) = %.4f\n', RMSE);

% Figure de validation
figure('Name', 'Validation du modèle identifié', 'NumberTitle', 'off');
hold on;
plot(t, y,             'r-',  'LineWidth', 1.5, 'DisplayName', 'Sortie mesurée');
plot(t, y_id_retarde,  'b--', 'LineWidth', 2.0, 'DisplayName', 'Modèle identifié');
xline(Trc_estime, 'k:', 'LineWidth', 1.5, ...
    'Label', sprintf('Trc = %.3f s', Trc_estime), 'FontSize', 10);
xlabel('Temps (s)', 'FontSize', 12);
ylabel('Sortie y(t)', 'FontSize', 12);
title(sprintf('Validation — Modèle 2^{nd} ordre + retard\n\\omega_n=%.3f rad/s, \\zeta=%.3f, Trc=%.3f s, G=%.3f', ...
    wn_estime, zeta_estime, Trc_estime, G_estime), 'FontSize', 13);
legend('Location', 'best', 'FontSize', 11);
grid on;
hold off;

%% =========================================================
%% 9. RÉCAPITULATIF FINAL
%% =========================================================

fprintf('\n========================================\n');
fprintf('  RÉCAPITULATIF DES PARAMÈTRES IDENTIFIÉS\n');
fprintf('========================================\n');
fprintf('  Retard constant    Trc = %.4f s\n',      Trc_estime);
fprintf('  Gain statique        G = %.4f\n',         G_estime);
fprintf('  Pulsation propre    ωn = %.4f rad/s\n',  wn_estime);
fprintf('  Amortissement        ζ = %.4f\n',         zeta_estime);
fprintf('  Gain contrôleur     Kp = %.4f\n',         Kp_estime);
fprintf('  Gain dérivé         Kd = %.4f\n',         Kd_estime);
fprintf('========================================\n');