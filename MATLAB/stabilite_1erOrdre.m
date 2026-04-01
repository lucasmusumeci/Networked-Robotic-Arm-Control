clear; clc; close all;

%% Paramètres système (à adapter)
G = 1.0000;          % gain du moteur
tau = 0.0008;      % constante de temps (s)

%% Plage de gains correcteur
Kc_values = linspace(0.1, 5, 200);

Tr_max_values = zeros(size(Kc_values));

%% Calcul du retard max pour chaque Kc

for i = 1:length(Kc_values)
    
    Kc = Kc_values(i);
    KG = Kc * G;
    
    if KG <= 1
        % Toujours stable (théoriquement)
        Tr_max_values(i) = Inf;
    else
        % Formule du cours
        Tr_max_values(i) = tau * acos(-1/KG) / sqrt(KG^2 - 1);
    end
end

%% Tracé

figure;
plot(Kc_values, Tr_max_values, 'LineWidth', 2);
grid on;

xlabel('Gain correcteur Kc');
ylabel('Retard maximal T_r (s)');
title('Limite de stabilité en fonction du gain');

ylim([0, max(Tr_max_values(~isinf(Tr_max_values)))*1.2]);

%% Zone stable (visualisation)

hold on;

% Exemple de retard réel
Tr_real = 0.500;

yline(Tr_real, 'r--', 'LineWidth', 2);

legend('T_r max', 'Retard réel');