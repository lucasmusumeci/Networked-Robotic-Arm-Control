%% Analyse stabilité - 1er Ordre
% Stability boundary curve: Trc_max = tau * arccos(-1/(Kc*G)) / sqrt((Kc*G)^2 - 1)

clear; clc; close all;

%% Paramètres
tau = 0.004;        % Time constant of the first-order system (seconds)

%% Intervalle de KcG (doit être > 1 car c'est forcement stable sinon)
KcG = linspace(1.01, 10, 1000);

%% Calcule limite stabilité
Trc_max = (tau .* acos(-1 ./ KcG)) ./ sqrt(KcG.^2 - 1);

%% Plot
figure;
hold on;

% Colorie région stable
fill([KcG, fliplr(KcG)], [Trc_max, zeros(1, length(KcG))], ...
    [0.8 0.9 1.0], 'EdgeColor', 'none', 'FaceAlpha', 0.5);

% Colorie région instable
Trc_top = max(Trc_max) * 1.2;
fill([KcG, fliplr(KcG)], [repmat(Trc_top, 1, length(KcG)), fliplr(Trc_max)], ...
    [1.0 0.85 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.5);

% Affiche courbe stabilité
plot(KcG, Trc_max, 'b-', 'LineWidth', 2);

% Ligne verticale à KcG = 1 ( toujours stable à gauche )
xline(1, '--k', 'LineWidth', 1.5, 'Label', 'KcG < 1 (toujours stable)');


xlabel('K_c G', 'FontSize', 13);
ylabel('T_{rc}^{max} (s)', 'FontSize', 13);
title('Frontière de stabilité - Système 1er Ordre avec delai constant', 'FontSize', 14);

legend('Région stable', 'Région instable', 'Stability boundary', ...
       'Location', 'northeast');

text(5, max(Trc_max)*0.6, 'INSTABLE', 'FontSize', 13, 'Color', [0.8 0.1 0.1], ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'center');
text(5, max(Trc_max)*0.2, 'STABLE', 'FontSize', 13, 'Color', [0.1 0.3 0.8], ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'center');

grid on;
xlim([1, max(KcG)]);
ylim([0, Trc_top]);
hold off;