%% === Analyse stabilité - 1er Ordre ===
%  Trc_max = tau * arccos(-1/(Kc*G)) / sqrt((Kc*G)^2 - 1)
clear; clc; close all;

%% === Paramètres ===
tau = 0.002;  % Constante de temps (s)
G    = 1;     % Gain statique

%% === Intervalle de Kc (doit être > 1/G pour que KcG > 1) ===
Kc_min = 1.01 / G;
Kc_max = 10  / G;
Kc = linspace(Kc_min, Kc_max, 1000);

%% === Calcul KcG et limite de stabilité ===
KcG     = Kc * G;
Trc_max = (tau .* acos(-1 ./ KcG)) ./ sqrt(KcG.^2 - 1);

%% === Plot ===
figure;
hold on;

Trc_top = max(Trc_max) * 1.2;

% Région stable
fill([Kc, fliplr(Kc)], [Trc_max, zeros(1, length(Kc))], ...
    [0.8 0.9 1.0], 'EdgeColor', 'none', 'FaceAlpha', 0.5);

% Région instable
fill([Kc, fliplr(Kc)], [repmat(Trc_top, 1, length(Kc)), fliplr(Trc_max)], ...
    [1.0 0.85 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.5);

% Courbe de stabilité
plot(Kc, Trc_max, 'b-', 'LineWidth', 2);

% Ligne verticale à Kc = 1/G (équivalent à KcG = 1, toujours stable à gauche)
xline(1/G, '--k', 'LineWidth', 1.5, ...
    'Label', sprintf('K_c < 1/G = %.2f (toujours stable)', 1/G));

xlabel('K_c', 'FontSize', 13);
ylabel('T_{rc}^{max} (s)', 'FontSize', 13);
title(sprintf('Limite de stabilité | \\tau = %.3fs et G = %.2f', tau, G), ...
    'FontSize', 14);
legend('Région stable', 'Région instable', 'Limite de stabilité', ...
    'Location', 'northeast');

Kc_mid = (Kc_min + Kc_max) / 2;
text(Kc_mid, Trc_top * 0.8, 'INSTABLE', 'FontSize', 13, 'Color', [0.8 0.1 0.1], ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'center');
text(Kc_mid, Trc_top * 0.2, 'STABLE',   'FontSize', 13, 'Color', [0.1 0.3 0.8], ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'center');

grid on;
xlim([1/G, Kc_max]);
ylim([0, Trc_top]);
hold off;
