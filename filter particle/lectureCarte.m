function h = lectureCarte(X, params)
    % Fonction qui renvoit la hauteur du terrain (par rapport au niveau de la
    % mer) � partir de l'�tat X et des param�tres de carte param (structure)
    % Entr�es:
    %           X:  doit obligatoirement contenir la position x en premi�re
    %               composante et la position y en seconde composante
    %           params: doit contenir les champs pasx_reel, pasy_reel,
    %                   nrow_h et h_MNT
    lam = X(1,:)/params.pasx_reel;
    ix = floor(lam);
    lam = lam - ix;
    mu = 1 + X(2,:)/params.pasy_reel;
    iy = floor(mu);
    mu = mu - iy;
    iy = params.nrow_h*ix+iy;
    h = params.h_MNT(iy);


