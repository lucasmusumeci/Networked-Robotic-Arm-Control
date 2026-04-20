clc; clear; close all;

%% Importation des données
opts = detectImportOptions('adaptive_control_log.csv');
data = readtable('adaptive_control_log.csv', opts);

% Extraction du temps
t = data.t_ms / 1000; % (s)

%% Définition des consignes
% Consignes définies dans le code C++ (en degrés, puis converties en radians)
qf_deg = [120.0, 60.0, -60.0, 120.0, 60.0, 120.0];
qf_rad = qf_deg * pi / 180.0;

%% Création de la figure 
figure('Name', 'Réponse en position (Commande Adaptative)', 'Position', [100, 100, 1200, 700]);

for i = 1:6
    subplot(3, 2, i); % définit le layout des figures
    
    col_pos = sprintf('q_simu_%d', i);
    col_cmd = sprintf('cmd_%d', i);
    
    % --- Axe de gauche : Position ---
    yyaxis left
    plot(t, data.(col_pos), '-', 'LineWidth', 1.5);
    hold on;
    yline(qf_rad(i), '--', 'LineWidth', 1.5);
    ylabel('Position (rad)');
    % Force les couleurs pour plus de clarté
    ax = gca; ax.YColor = [0 0.4470 0.7410]; % Bleu MATLAB par défaut
    
    % --- Axe de droite : Commande en Vitesse ---
    yyaxis right
    plot(t, data.(col_cmd), '-', 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1); % Orange
    ylabel('Vitesse cmd (rad/s)');
    ax.YColor = [0.8500 0.3250 0.0980];
    
    % --- Nom des axes et légende ---
    title(sprintf('Joint%d', i));
    xlabel('Temps (s)');
    grid on;
    
    if i == 1
        legend('Pos. Mesurée', 'Pos. Cible', 'Cmd Vitesse', 'Location', 'southeast');
    end
    hold off;
end