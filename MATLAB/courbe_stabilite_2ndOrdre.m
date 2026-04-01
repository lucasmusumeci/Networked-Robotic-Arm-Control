%% Analyse de la stabilité - Système du second ordre avec retard constant
% Frontière de stabilité : calcul de Trc_max selon l'équation (20)
% Cours : Automatique et Réseaux - Université de Montpellier

clear; clc; close all;

%% Paramètres du système
Kp = 2;     % Gain proportionnel
Kd = 1;     % Gain dérivé
KG = 3;     % Gain en boucle ouverte (produit K*G)

%% Calcul du discriminant Delta (équation 16)
% aX² + bX + c = 0 avec X = cos(w*Trc)
% a = 1, b = Kd²/KG, c = Kp*Kd²/KG² - 1
a_coef = 1;
b_coef = Kd^2 / KG;
c_coef = (Kp * Kd^2) / KG^2 - 1;

Delta = b_coef^2 - 4 * a_coef * c_coef;

fprintf('=== Analyse de la stabilité - Système du 2nd ordre ===\n');
fprintf('Paramètres : Kp = %.2f | Kd = %.2f | KG = %.2f\n', Kp, Kd, KG);
fprintf('Discriminant Delta = %.4f\n\n', Delta);

if Delta < 0
    error('Delta < 0 : pas de solution réelle pour cos(wTrc). Le système est instable pour tout retard.');
end

%% Calcul des deux solutions de cos(wTrc) (équation 18)
cos_theta1 = 0.5 * (-b_coef + sqrt(Delta));
cos_theta2 = 0.5 * (-b_coef - sqrt(Delta));

fprintf('cos(θ₁) = %.4f\n', cos_theta1);
fprintf('cos(θ₂) = %.4f\n\n', cos_theta2);

%% Calcul de Trc pour chaque solution (équation 20)
% Trc = (Kd / KG) * arccos(cos_theta) / sqrt(1 - cos_theta²)

Trc_solutions = [];

for k = 1:2
    if k == 1
        cos_theta = cos_theta1;
        label = '1';
    else
        cos_theta = cos_theta2;
        label = '2';
    end

    % Vérification que cos_theta est dans [-1, 1]
    if abs(cos_theta) > 1
        fprintf('Solution %s : cos(θ) = %.4f hors de [-1,1], rejetée.\n', label, cos_theta);
        continue;
    end

    theta = acos(cos_theta);                    % θ = arccos(cos_theta) en radians
    sin_theta = sqrt(1 - cos_theta^2);

    % Vérification que sin_theta est non nul
    if sin_theta < 1e-10
        fprintf('Solution %s : sin(θ) ≈ 0, Trc non défini, rejetée.\n', label);
        continue;
    end

    Trc = (Kd / KG) * theta / sin_theta;

    % On ne conserve que les solutions réelles et strictement positives
    if isreal(Trc) && Trc > 0
        fprintf('Solution %s : θ = %.4f rad | Trc_%s = %.4f s\n', label, theta, label, Trc);
        Trc_solutions(end+1) = Trc;
    else
        fprintf('Solution %s : Trc = %.4f rejetée (négative ou complexe).\n', label, Trc);
    end
end

%% Sélection du Trc_max (on prend la plus grande valeur admissible)
if isempty(Trc_solutions)
    error('Aucune solution admissible pour Trc. Le système est instable pour tout retard.');
end

Trc_max = max(Trc_solutions);
fprintf('\n=> Trc_max = %.4f s (frontière de stabilité)\n', Trc_max);
fprintf('   Système STABLE si Trc < %.4f s\n\n', Trc_max);

%% Tracé de la frontière de stabilité en fonction de KG
% On fait varier KG et on recalcule Trc_max pour chaque valeur

KG_vec = linspace(0.1, 10, 500);
Trc_max_vec = NaN(size(KG_vec));

for i = 1:length(KG_vec)
    KGi = KG_vec(i);

    b_i = Kd^2 / KGi;
    c_i = (Kp * Kd^2) / KGi^2 - 1;
    Delta_i = b_i^2 - 4 * c_i;

    if Delta_i < 0
        continue;
    end

    candidats = [];
    for signe = [+1, -1]
        cos_th = 0.5 * (-b_i + signe * sqrt(Delta_i));

        if abs(cos_th) > 1
            continue;
        end

        sin_th = sqrt(1 - cos_th^2);
        if sin_th < 1e-10
            continue;
        end

        th = acos(cos_th);
        Trc_i = (Kd / KGi) * th / sin_th;

        if isreal(Trc_i) && Trc_i > 0
            candidats(end+1) = Trc_i;
        end
    end

    if ~isempty(candidats)
        Trc_max_vec(i) = max(candidats);
    end
end

%% Figure
figure('Name', 'Frontière de stabilité - Système du 2nd ordre', 'NumberTitle', 'off');
hold on;

% Zone stable (sous la courbe)
KG_valid = KG_vec(~isnan(Trc_max_vec));
Trc_valid = Trc_max_vec(~isnan(Trc_max_vec));
Trc_top = max(Trc_valid, [], 'omitnan') * 1.2;

if ~isempty(KG_valid)
    fill([KG_valid, fliplr(KG_valid)], ...
         [Trc_valid, zeros(1, length(KG_valid))], ...
         [0.8 0.9 1.0], 'EdgeColor', 'none', 'FaceAlpha', 0.5);

    % Zone instable (au-dessus de la courbe)
    fill([KG_valid, fliplr(KG_valid)], ...
         [repmat(Trc_top, 1, length(KG_valid)), fliplr(Trc_valid)], ...
         [1.0 0.85 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
end

% Courbe de la frontière
plot(KG_vec, Trc_max_vec, 'b-', 'LineWidth', 2);

% Point de fonctionnement actuel
plot(KG, Trc_max, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
text(KG + 0.15, Trc_max, sprintf('  Point nominal\n  KG=%.1f, Trc_{max}=%.2fs', KG, Trc_max), ...
    'FontSize', 10, 'Color', [0.8 0.1 0.1]);

% Annotations des zones
x_mid = mean(KG_valid);
text(x_mid, Trc_top * 0.75, 'INSTABLE', ...
    'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.8 0.1 0.1], ...
    'HorizontalAlignment', 'center');
text(x_mid, Trc_top * 0.15, 'STABLE', ...
    'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.1 0.3 0.8], ...
    'HorizontalAlignment', 'center');

%% Mise en forme
xlabel('K \cdot G (gain en boucle ouverte)', 'FontSize', 13);
ylabel('T_{rc}^{max} (s)', 'FontSize', 13);
title(sprintf('Frontière de stabilité - Système du 2^{nd} ordre\n(K_p = %.1f, K_d = %.1f)', Kp, Kd), ...
    'FontSize', 14);
legend('Zone stable', 'Zone instable', 'Frontière T_{rc}^{max}', 'Point nominal', ...
    'Location', 'northeast');
grid on;
xlim([0, max(KG_vec)]);
ylim([0, Trc_top]);
hold off;