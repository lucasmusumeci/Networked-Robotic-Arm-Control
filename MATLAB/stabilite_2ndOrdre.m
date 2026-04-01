clear; clc; close all;

%% ---------------------------------------------------------------
%  Paramètres identifiés (à remplacer par vos valeurs de analyse_step.py)
%  K*G : gain statique du moteur (qdot_simu_ss / qdot_cmd)
%  Trc : retard total = retard actionneur + retard réseau UDP (0.5 s)
%% ---------------------------------------------------------------
KG      = 1.0;    % K*G identifié
Trc_reel = 0.5;   % retard réel total (s)

%% ---------------------------------------------------------------
%  Fonction Trc_max selon les équations du cours (éq. 3 → 7)
%  Entrées : Kp, Kd (gains du correcteur PD), KG (gain moteur)
%  Sortie  : Trc_max (s), ou NaN si aucune solution valide
%% ---------------------------------------------------------------
function trc_max = compute_Trcmax(Kp, Kd, KG)
    % Coefficients du trinôme en X = cos(omega*Trc)  (éq. 3)
    b = Kd^2 / KG;
    c = Kp * Kd^2 / KG^2 - 1;

    delta = b^2 - 4*c;   % discriminant (éq. 3)

    if delta < 0
        % Pas de racine réelle → pas de frontière de stabilité finie
        % → stable pour tout retard (retard max = +inf)
        trc_max = Inf;
        return;
    end

    candidates = [];
    for sign_val = [+1, -1]
        % cos(omega*Trc)  (éq. 5)
        cos_val = 0.5 * (-b + sign_val * sqrt(delta));

        % cos doit être dans [-1, 1]
        if cos_val < -1 || cos_val > 1
            continue;
        end

        % theta = omega*Trc  (éq. 5)
        theta = acos(cos_val);

        % sin(theta) au dénominateur de éq. 6
        sin_val = sqrt(1 - cos_val^2);
        if sin_val < 1e-10
            continue;   % évite la division par zéro
        end

        % Trc_max  (éq. 7)
        trc_candidate = (Kd / (KG * sin_val)) * theta;

        % Garder seulement les solutions réelles strictement positives
        if isreal(trc_candidate) && trc_candidate > 0
            candidates(end+1) = trc_candidate; %#ok<AGROW>
        end
    end

    if isempty(candidates)
        trc_max = NaN;   % aucune solution valide dans le domaine
    else
        trc_max = min(candidates);   % on prend la plus petite (cours : "solution la plus petite")
    end
end

%% ---------------------------------------------------------------
%  1. Courbe Trc_max = f(Kd) pour plusieurs valeurs de Kp
%     → choisir Kd tel que Trc_max > Trc_reel
%% ---------------------------------------------------------------
Kd_values = linspace(0.01, 20, 400);
Kp_list   = [1, 2, 4, 8];

figure('Name', 'Trc_max vs Kd (pour differents Kp)');
hold on; grid on;
colors = lines(length(Kp_list));

for k = 1:length(Kp_list)
    Kp = Kp_list(k);
    trc_vals = zeros(size(Kd_values));
    for i = 1:length(Kd_values)
        t = compute_Trcmax(Kp, Kd_values(i), KG);
        trc_vals(i) = min(t, 5);   % plafonner à 5 s pour la lisibilité
    end
    plot(Kd_values, trc_vals, 'LineWidth', 2, 'Color', colors(k,:), ...
         'DisplayName', sprintf('Kp = %.0f', Kp));
end

yline(Trc_reel, 'r--', 'LineWidth', 2, 'DisplayName', ...
      sprintf('Retard réel = %.2f s', Trc_reel));
xlabel('Gain dérivé Kd');
ylabel('Retard maximal T_{rc,max} (s)');
title('Limite de stabilité : T_{rc,max} vs K_d');
legend('Location', 'best');
ylim([0, 5]);

%% ---------------------------------------------------------------
%  2. Carte de stabilité dans le plan (Kp, Kd)
%     Zone verte : Trc_max > Trc_reel  → stable avec le retard réel
%     Zone rouge  : Trc_max <= Trc_reel → instable
%% ---------------------------------------------------------------
N = 150;
Kp_range = linspace(0.01, 20, N);
Kd_range = linspace(0.01, 20, N);
[KP, KD] = meshgrid(Kp_range, Kd_range);
STABLE   = zeros(N, N);

for i = 1:N
    for j = 1:N
        t = compute_Trcmax(KP(i,j), KD(i,j), KG);
        if isinf(t) || (~isnan(t) && t > Trc_reel)
            STABLE(i,j) = 1;
        end
    end
end

figure('Name', 'Carte de stabilite (Kp, Kd)');
imagesc(Kp_range, Kd_range, STABLE);
set(gca, 'YDir', 'normal');
colormap([0.85 0.2 0.2; 0.2 0.75 0.45]);   % rouge=instable, vert=stable
colorbar('Ticks', [0.25, 0.75], 'TickLabels', {'Instable', 'Stable'});
xlabel('Gain proportionnel Kp');
ylabel('Gain dérivé Kd');
title(sprintf('Zone de stabilité  (KG = %.2f,  T_{rc} = %.2f s)', KG, Trc_reel));
grid on;

%% ---------------------------------------------------------------
%  3. Pour un (Kp, Kd) choisi : afficher Trc_max et la marge
%% ---------------------------------------------------------------
Kp_choisi = 4.0;
Kd_choisi = 3.0;

trc_max_choisi = compute_Trcmax(Kp_choisi, Kd_choisi, KG);

fprintf('\n=== Gains choisis ===\n');
fprintf('  Kp      = %.4f\n', Kp_choisi);
fprintf('  Kd      = %.4f\n', Kd_choisi);
fprintf('  KG      = %.4f\n', KG);
fprintf('  Trc_max = %.4f s\n', trc_max_choisi);
fprintf('  Trc_reel= %.4f s\n', Trc_reel);
if ~isnan(trc_max_choisi) && trc_max_choisi > Trc_reel
    marge = (trc_max_choisi - Trc_reel) / trc_max_choisi * 100;
    fprintf('  --> STABLE  (marge = %.1f %%)\n', marge);
else
    fprintf('  --> INSTABLE\n');
end

%% ---------------------------------------------------------------
%  4. Ajouter le point choisi sur la carte de stabilité
%% ---------------------------------------------------------------
figure(2);
hold on;
plot(Kp_choisi, Kd_choisi, 'w*', 'MarkerSize', 12, 'LineWidth', 2, ...
     'DisplayName', sprintf('(Kp=%.1f, Kd=%.1f)', Kp_choisi, Kd_choisi));
legend('Location', 'best');