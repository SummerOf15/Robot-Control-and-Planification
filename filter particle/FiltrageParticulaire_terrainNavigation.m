%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%           TP Filtre Particulaire
%           Nicolas Merlinge (ONERA, TP ENSTA ROB312)
%           Version 2.0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear % effacer toutes les variables
close all % fermer toutes les fenêtres
clc % effacer la ligne de commande
rng(123457) % imposer la graine de génération de nombres pseudo-aléatoire pour la répétabilité


% Paramètres initiaux et de simulation
erreurInitiale = sqrt(diag([5000, 5000, 100, 20*pi/180].^2))*randn(4,1);
X_reel = [230000, 90000, 1000, 150*pi/180]' + erreurInitiale; % état vrai initial (inconnu du filtre)    
d = size(X_reel,1); % dimension de l'état
dt = 1; % pas de temps
R = 20.^2; % matrice de covariance du bruit de mesure réelle (inconnu du filtre)
dm = size(R,1); % dimension du vecteur de mesures

% Chargement des données
load carte.mat
params.pasx_reel = pasx_reel;
params.pasy_reel = pasy_reel;
params.nrow_h = size(h_MNT,1);
params.dxh_MNT = diff(h_MNT,1,2)/pasx_reel;
params.dyh_MNT = diff(h_MNT)/pasy_reel;
params.h_MNT = h_MNT;
params.x_MNT = x_MNT;
params.y_MNT = y_MNT;

% Initialisation des variables de stockage des données
tk=1;
t_sim(tk) = 0;
Y_sim(:,tk) = 0;
X_reel_sim(:,tk) = X_reel;

%% Boucle de simulation physique de la trajectoire
T = 80; % durée (s)
for tk = 2:(T/dt)
    % commande (définit la trajectoire)
    V = 300; % Vitesse (connue)
    omega = -0.01; % vitesse angulaire (rad/s) (connue)
    
    % simulation de l'état vrai (attention, inconnu du filtre)
    t = dt*(tk-1); % temps courant
    X_reel = [X_reel(1) + V*dt*cos(X_reel(4)); X_reel(2) + V*dt*sin(X_reel(4)); X_reel(3); X_reel(4) + dt*omega]; % propagation de l'état réel (à compléter)
    X_reel(4) = mod(X_reel(4), 2*pi); % modulo 2pi sur le cap

    X_reel_sim(:,tk) = X_reel;
    t_sim(tk) = t;
end

%% Boucle de simulation physique des mesures
T = 80; % durée (s)
for tk = 2:(T/dt)
    % Récupération de l'état
    X_reel = X_reel_sim(:,tk);
    
    % génération de la mesure réelle    
    Y = X_reel(3,:) - lectureCarte(X_reel, params) + sqrt(R)*randn(dm,1);
    
    Y_sim(:,tk) = Y;
end

%% Boucle du filtre particulaire

% Paramètres du filtre
N = 3000; % nombre de particules
P_hat = diag([5000, 5000, 100, 20*pi/180].^2); % matrice de covariance initiale
X_hat = [230000, 90000, 1000, 150*pi/180]'; % estimé initial (x, y, z, theta)
Qf = diag([3, 3, 0.6, 0.001*180/pi].^2); % matrice de covariance de bruit de dynamique
Rf = 20.^2; % covariance du bruit de mesure du filtre
threshold_resampling = 1; % seuil de ré-échantillonnage (theta_eff)

% Initialisation des variables de stockage des données
tk=1;
P_sim(:,:,tk) = P_hat;
Pdiag_sim(:,tk) = diag(P_hat);
X_hat_sim(:,tk) = X_hat;

% Initialisation du filtre
Xp = sqrt(P_hat)*randn(4,N)+X_hat; % Tirage des particules autours de X_hat initial (à compléter)
wp = ones(1,N)/N; % poids initiaux (à compléter)

T = 80; % durée (s)
for tk = 2:(T/dt)
    % commande (définit la trajectoire)
    V = 300; % Vitesse (connue)
    omega = -0.01; % vitesse angulaire (rad/s) (connue)
    
    % Récupération de la mesure réelle
    Y = Y_sim(:,tk);
    
    % prediction (à compléter: variables Xp, X_hat et P_hat)
    Xp = [Xp(1,:) + V*dt*cos(Xp(4,:)); Xp(2,:) + V*dt*sin(Xp(4,:)); Xp(3,:); Xp(4,:) + dt*omega]+sqrt(Qf)*randn(d,N); % particules prédites
    X_hat = Xp*wp'; % état estimé prédit
    X_hat(4,:) = mod(X_hat(4,:), 2*pi); % modulo 2pi sur le cap
    P_hat =wp.*(Xp-X_hat)*(Xp-X_hat)'; % matrice de covariance prédite
    
%     is_measurementValid = false;
   
    % validité de la mesure réelle (à compléter pour la gestion des fréquences et des trous de mesures)
    is_measurementValid = true;

        % correction (à compléter)
    if is_measurementValid
        % définition de la mesure prédite (à compléter)
        for i=1:N
            Y_hat = Xp(3,i) - lectureCarte(Xp(:,i), params);

            % correction des poids des particules (à compléter)
            inno = (Y-Y_hat);
            likelihood = exp(-inno'*inno/2/Rf);
            wp(i) = wp(i)*likelihood; % correction des poids
        end
        wp = wp./sum(wp); % normalisation des poids (la somme doit être égale à 1)
    end
    
    % Ré-échantillonnage (critère de seuil à compléter, puis coder un
    % autre algorithme de ré-échantillonnage de votre choix pour la
    % dernière question, en substitution du fichier select.p)
    criterionResampling = 1/sum(wp.^2); % à compléter
    if criterionResampling < N*threshold_resampling
%         Xp = Xp(:,select(wp)); % sélection des nouvelles particules selon l'algorithme de ré-échantillonnage multinomial
        Xp = Xp(:,resampleSystematic(wp));
        wp = ones(1,N)/N; % ré-initialisation des poids (à compléter)
    end
    
    % enregistrement des variables (pour plot)
    P_sim(:,:,tk) = P_hat;
    Pdiag_sim(:,tk) = diag(P_hat);
    X_hat_sim(:,tk) = X_hat;
    
    % plot instantané (ne pas hésiter à passer <is_temporalPlot> à false pour gagner du temps d'éxécution)
    is_temporalPlot = true;
    if is_temporalPlot
        figure(2)
        clf
        hold on
        imagesc(params.x_MNT*params.pasx_reel/1000, params.y_MNT*params.pasx_reel/1000, h_MNT)
        xlabel('km'); ylabel('km'); 
        title(['Erreur position: ', num2str(norm(X_hat(1:3) - X_reel(1:3))), ' m'])
        grid
        colorbar
        hold on
        plot(X_reel_sim(1,1:tk)./1000, X_reel_sim(2,1:tk)./1000,'.k')
        plot(X_hat_sim(1,1:tk)./1000, X_hat_sim(2,1:tk)/1000,'.r')
        scatter(Xp(1,:)./1000, Xp(2,:)./1000, '.y')
        scatter(X_reel_sim(1,tk)./1000, X_reel_sim(2,tk)./1000, '.k')
        scatter(X_hat_sim(1,tk)./1000, X_hat_sim(2,tk)./1000, '.r')
        grid on
        ylim([50, 150])
        xlim([170, 250])
        legend('Position vraie', 'position estimée','particules')
%         figure(3)
%         hist(wp)
        drawnow

    end
end

% Plot des résultats
figure(1)
labels = {'x (m)','y (m)','z (m)','\theta (rad)'};
for i = 1:d
    subplot(4,1,i)
    hold on
    fill([t_sim flip(t_sim,2)],[X_hat_sim(i,:) - 3*sqrt(Pdiag_sim(i,:)), X_hat_sim(i,end:-1:1) + 3*sqrt(Pdiag_sim(i,end:-1:1))], [7 7 7]/8);
    plot(t_sim, X_reel_sim(i,:), 'b')
    plot(t_sim, X_hat_sim(i,:), 'r')
    grid on
    xlabel('time (s)')
    ylabel(labels(i))
end
legend('uncertainty (3\sigma)', 'actual state', 'estimated state')


