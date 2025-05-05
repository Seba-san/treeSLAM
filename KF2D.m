classdef KF2D
    
    properties
        % El estado x = [ x_robot; y_robot; x_land1; y_land1; x_land2; y_land2; ... ]
        x    % estado
        p    % covarianza del estado
        
        % Ruido de proceso para el robot (2D).
        % Puede ser una matriz 2x2 o un escalar que se multiplicará por la identidad 2x2
        Q_r  = (0.3)^2 * eye(2);
        
        % Ruido de medición de landmarks (2D). Podría ser 2x2, o se puede usar
        % un escalar para cada medición y expandirlo en la actualización.
        R_z  = (0.2)^2 * eye(2);
        
        % Estas propiedades manejan la lógica de matching y de administración:
        pPred           % covarianza de predicción
        H               % matriz de medición construida para la actualización
        zMatching       % índices de observaciones asociadas a landmarks
        lMatching       % índices de landmarks asociados a observaciones
        minDist         % distancia mínima para asociar un landmark existente
        minDistNewLandmark
        confidenceLevel = 0.95;  % nivel de confianza para el test de chi-cuadrado
        nNew            % número de landmarks nuevos en el ciclo
        alpha           = 0.10;  % parámetro para descartar outliers
        variance=[];
    end
    
    methods
        %% Constructor
        function obj = KF2D()
            % Estado inicial con robot en (0,0)
            obj.x = [0; 0];  % (x_r, y_r)
            
            % Covarianza inicial del robot (puedes ajustarla)
            obj.p = 0.05^2 * eye(2);
            
            % Distancias mínimas (puedes ajustarlas)
            obj.minDist             = 1.0;  % para asociar un landmark existente
            obj.minDistNewLandmark  = 3.0;  % para decidir que un landmark es completamente nuevo
            
            disp('KF2D iniciado. Se estima (x,y) del robot y (x,y) de cada landmark.')
        end
        
        %% ------------------------------------------------
        %% ADMINISTRACIÓN DE LANDMARKS (crea H y decide si hay landmarks nuevos)
        function obj = landmarksAdministration(obj, Z)
            % Z es una matriz Nx2 con las mediciones en 2D: Z(i,:) = [z_x, z_y].
            % Por ejemplo, z_i = (x_Li - x_r, y_Li - y_r) medido.
            
            % Número actual de landmarks (cada landmark son 2 estados).
            % El estado total es [x_r, y_r, x_l1, y_l1, x_l2, y_l2, ...].
            % => #landmarks = (length(obj.x) - 2)/2
            nl = (size(obj.x,1) - 2) / 2;  
            
            % Predicción de las mediciones (zPred) para cada landmark existente
            % zPred_i = [ (x_l_i - x_r); (y_l_i - y_r) ]
            zPred = [];
            if nl > 0
                x_robot = obj.x(1);
                y_robot = obj.x(2);
                for i = 1:nl
                    iLand_x = 2 + 2*(i-1) + 1;  % índice de x_l_i
                    iLand_y = 2 + 2*(i-1) + 2;  % índice de y_l_i
                    zPred_i = [obj.x(iLand_x) - x_robot; ...
                               obj.x(iLand_y) - y_robot];
                    zPred   = [zPred; zPred_i'];
                end
                % zPred será una matriz nl x 2. Cada fila es [zPred_x, zPred_y].
            end
            
            % Realizar matching entre zPred y Z
            [iPred, j, ambiguous] = obj.matching(zPred, Z);
            
            nObserved = size(Z,1);
            nMatched  = size(j,1);
            nAmb      = length(ambiguous);
            nNew      = nObserved - nMatched - nAmb;
            obj.nNew  = nNew;  % se guardará para uso posterior
            
            % -----------------------------------
            % A) Crear nuevos landmarks si hay mediciones "no matcheadas"
            if nMatched + nAmb < nObserved
                warning('Nuevos landmarks detectados');
                % Índices de todas las mediciones
                allIdx = 1:nObserved;
                % Quitar las que ya hicieron matching
                ll = setdiff(allIdx, j');
                % Quitar las ambiguas
                ll = setdiff(ll, ambiguous);
                
                % Añadir nuevos landmarks con esas mediciones
                obj = obj.addNewLandmarks(ll, Z);
            else
                ll = [];
            end
            
            % -----------------------------------
            % B) Construir matriz de medición H.
            % El número total de mediciones "usables" = matched + las que fueron asignadas a new.
            nUsables = (nMatched + nNew) * 2;  % cada medición es 2D
            nStates  = length(obj.x);
            obj.H    = zeros(nUsables, nStates);
            
            % Indices "globales" de las observaciones que sí se usan:
            obj.zMatching = [j; ll'];  % mediciones que iremos a actualizar
            % Landmark indices: 
            %   iPred(i) + 1  (en 1D) se convertía en iPred(i)+1;
            %   Aquí cada landmark tiene 2 índices: x_l_i, y_l_i.
            %   Los landmarks están a partir de la posición 3 en el vector (indices 3 y 4 para land1, 5 y 6 para land2, etc.)
            
            lIdxMatch = [];  % para guardar índices de columna (en el estado) de los landmarks matcheados
            % 1) Landmarks ya existentes (iPred -> landmark iPred)
            for k = 1:length(iPred)
                iLand = iPred(k);  % i-ésimo landmark
                % Las columnas correspondientes al landmark iLand en el estado:
                colX = 2 + 2*(iLand-1) + 1;  % x del iLand
                colY = 2 + 2*(iLand-1) + 2;  % y del iLand
                lIdxMatch = [lIdxMatch; colX colY];
            end
            
            % 2) Landmarks nuevos (ll -> se acaban de añadir al final del estado)
            %   si teníamos nl landmarks, se añaden nNew => los nuevos están 
            %   en los índices: 2 + 2*nl + 1 : 2 + 2*nl + 2, etc.
            for k = 1:nNew
                iGlobalLand = nl + k;  % "número" de landmark en el conteo total
                colX = 2 + 2*(iGlobalLand-1) + 1;
                colY = 2 + 2*(iGlobalLand-1) + 2;
                lIdxMatch = [lIdxMatch; colX colY];
            end
            
            obj.lMatching = lIdxMatch;
            
            % Construir H para cada medición "usable"
            % (recuerda que cada medición en 2D genera 2 filas en H)
            row = 1;
            for mm = 1:size(lIdxMatch,1)
                % Landmark columns
                cx = lIdxMatch(mm,1);
                cy = lIdxMatch(mm,2);
                
                % Filas para la medición en x
                obj.H(row,   1) = -1;      % d/d(x_r)
                obj.H(row,   cx) =  1;     % d/d(x_l)
                % Filas para la medición en y
                obj.H(row+1, 2) = -1;      % d/d(y_r)
                obj.H(row+1, cy) =  1;     % d/d(y_l)
                
                row = row + 2;
            end
            
            % -----------------------------------
            % C) Calcular pPred
            % Aquí puedes añadir tu modelo de movimiento del robot.
            % Si, por simplicidad, asumes que A = I (sin cambio) más ruido Q_r en el
            % bloque del robot, se haría algo así:
            obj.pPred = obj.p;
            % Incrementar la submatriz [1:2,1:2] (correspondiente al robot) por Q_r
            obj.pPred(1:2,1:2) = obj.pPred(1:2,1:2) + obj.Q_r;
        end
        
        %% ------------------------------------------------
        %% MATCHING
        function [iPred,j,ambi] = matching(obj, zPred, Z)
% matching para SLAM 2D pero con la misma lógica que la versión 1D.
%
% ENTRADAS:
%   obj   - objeto KF2D (contiene minDist, minDistNewLandmark, R_z, etc.)
%   zPred - [nl x 2] predicción de medición para cada landmark (x_l - x_r, y_l - y_r)
%   Z     - [nm x 2] observaciones reales (por ejemplo, distancias 2D)
%
% SALIDAS:
%   iPred - índices de landmarks matcheados
%   j     - índices de observaciones asociadas
%   ambi  - índice (en las observaciones) de mediciones ambiguas

    nm = size(Z,1);  % número de mediciones
    if isempty(zPred)
        nl = 0;      % no hay landmarks predichos
    else
        nl = size(zPred,1);
    end
    
    iPred = [];
    j     = [];
    ambi  = [];
    
    % Si no hay landmarks modelados aún, salimos
%     if nl == 0
%         return
%     end
    
    %======================================================================
    % (1) Calcular la distancia entre cada zPred (landmark) y cada Z (obs)
    %======================================================================
    Dist = zeros(nl, nm);
    for i = 1:nl
        for k = 1:nm
            diff_ = zPred(i,1:2) - Z(k,1:2);
            Dist(i,k) = norm(diff_);  % distancia euclidiana
        end
    end
    
    %======================================================================
    % (2) Criterio de matching: distMatching
    %======================================================================
    % En 1D se usaba: minDist, (sqrt(obj.p(1,1)) + sqrt(obj.Q_r)) * 3, etc.
    % Aquí haremos algo análogo en 2D:
    %   sqrt(obj.p(1,1) + obj.p(2,2)) ~ "incertidumbre del robot"
    %   sqrt(det(obj.Q_r)) ~ "ruido proceso" (aprox)
    % Asegúrate de que obj.Q_r sea 2x2 (o algo equivalente).
    
    distMatching = min( obj.minDist , ...
        ( sqrt( obj.p(1,1) + obj.p(2,2) ) + sqrt(det(obj.Q_r)) ) ) * 3;
    
    %======================================================================
    % (3) Buscar todos los pares (i, k) con Dist(i,k) < distMatching
    %======================================================================
    [tempI, tempJ] = find(Dist < distMatching);
    iPred = tempI;  % índice del landmark
    j     = tempJ;  % índice de la observación
    
    %======================================================================
    % (4) Detectar ambigüedades
    %======================================================================
    % Igual que en 1D, si un landmark se asocia con múltiples observaciones
    % o una observación se asocia a múltiples landmarks, se marcan como
    % ambiguas. Se quitan de iPred, j y se guardan en 'ambi'.
    
    if ~isempty(iPred)
        % Ordenamos para buscar duplicados consecutivos
        [iPredSorted, iIdx] = sort(iPred);
        [jSorted,   jIdx]   = sort(j);
        
        if sum(diff(iPredSorted)==0) > 0 || sum(diff(jSorted)==0) > 0
            warning('ambiguity detected (2D matching)');
            k = [];
            % Recorremos todas las asociaciones
            for idx = 1:size(j,1)-1
                if (j(idx) == j(idx+1)) || (iPred(idx) == iPred(idx+1))
                    k = [k idx idx+1];
                    if ~isempty(ambi)
                        % Evita duplicar índices en 'ambi'
                        if ambi(end) ~= j(idx)
                            ambi = [ambi j(idx)];
                        end
                    else
                        ambi = [ambi j(idx)];
                    end
                end
            end
            % Eliminamos las parejas ambiguas
            iPred(k) = [];
            j(k)     = [];
        end
    end
    
    %======================================================================
    % (5) Check new landmarks
    %======================================================================
    % newL son las observaciones que NO se han matcheado con ningún landmark
    allObs = 1:nm;
    newL   = setdiff(allObs, j);
    
    % (5a) Calcular distancia entre las observaciones "candidatas" newL
    % y todas las demás observaciones. 
    if ~isempty(newL)
        d__ = zeros(nm, numel(newL));
        for cc = 1:numel(newL)
            idxObs = newL(cc);  % índice de la observación "candidata"
            for k = 1:nm
                diff_ = Z(idxObs,1:2) - Z(k,1:2);
                d__(k, cc) = norm(diff_);
            end
        end
        
        % (5b) La lógica del 1D era:
        % if sum( d__(:,i) < obj.minDistNewLandmark & d__(:,i) > 0 )>0
        %     => esa observación es ambigua
        % Además, comprobaba obj.R_z(newL(i))>0.99
        for cc = 1:numel(newL)
            idxObs = newL(cc);
            % Distancias de esta nueva obs a todas las obs
            distVec = d__(:, cc);
            if sum(distVec < obj.minDistNewLandmark & distVec>0) > 0 ... % distVec>0 elimina la comparacion consigo misma
                    || (max(obj.variance(idxObs,1:2)) > 0.99)% .99 ya que si es 1 viene de una unica observacion
                % Marcarla como ambigua (si no está ya)
                if isempty(find(ambi == idxObs, 1))
                    ambi = [ambi idxObs];
                end
            end
        end
        
        % Quitar las ambiguas de newL
        if ~isempty(ambi)
            newL = setdiff(newL, ambi);
        end
        
        %==================================================================
        % (5c) Ultimo check: ver si esa obs candidata newL está "cerca"
        % de alguna predicción (Dist) por debajo de minDistNewLandmark
        % para marcarla como ambigua
        %==================================================================
        if ~isempty(Dist) && ~isempty(newL)
            for cc = 1:numel(newL)
                obsIdx = newL(cc);
                % Miramos Dist(:, obsIdx)
                if sum(Dist(:,obsIdx) < obj.minDistNewLandmark) > 0
                    if isempty(find(ambi == obsIdx, 1))
                        ambi = [ambi obsIdx];
                    end
                end
            end
        end
    end
end

        
        %% ------------------------------------------------
        %% AÑADIR LANDMARKS NUEVOS
        function obj = addNewLandmarks(obj, newIdx, Z)
            % newIdx: índices de las mediciones que se consideran nuevos landmarks
            % Z: matriz de mediciones Nx2
            nNewL = numel(newIdx);
            if nNewL == 0
                return
            end
            
            % Expandir la matriz de covarianza
            nOld = size(obj.p,1);
            nNewStates = 2*nNewL;  % cada landmark nuevo aporta 2 estados
            Pnew = zeros(nOld + nNewStates);
            Pnew(1:nOld, 1:nOld) = obj.p;
            
            % Inicializar la covarianza para los nuevos landmarks
            % Cada landmark nuevo tiene una covarianza: Cov_robot + Cov_medición
            % Asumimos que obj.R_z es una matriz 2x2 para cada landmark
            newCov = zeros(nNewStates, nNewStates);
            for k = 1:nNewL
                % Índices en la nueva matriz para este landmark
                idx = (k-1)*2 + 1 : k*2;
                % Asumiendo que obj.R_z es una matriz de covarianza 2x2 por landmark
                R_z_land = diag(obj.variance(newIdx(k),1:2));
                R_z_land(1,2)=obj.variance(newIdx(k),3);
                R_z_land(2,1)=R_z_land(1,2);
                % Combinar con la covarianza del robot
                newCov(idx, idx) = obj.p(1:2, 1:2) + R_z_land;
            end
            % Asignar la covarianza de los nuevos landmarks
            Pnew(nOld+1:end, nOld+1:end) = newCov;
            % Pnew(n+1:end, 1:n)=obj.p(:,1)'; % Para 1D se hacia asi
%             Pnew(1:n, n+1:end)=obj.p(:,1);
            obj.p = Pnew;
            
            % Expandir el estado
            xOld = obj.x;
            xNew = zeros(nOld + nNewStates, 1);
            xNew(1:nOld) = xOld;
            
            % Asumimos que la medición Z(k,:) = [ z_x, z_y ] = [ x_l - x_r, y_l - y_r].
            % => estimación inicial del landmark i: (x_r, y_r) + Z(k,:).
            x_r = obj.x(1);
            y_r = obj.x(2);
            idx = nOld+1;
            for ii = 1:nNewL
                kObs = newIdx(ii);
                z_x = Z(kObs,1);
                z_y = Z(kObs,2);
                xNew(idx)   = x_r + z_x;  % x del landmark
                xNew(idx+1) = y_r + z_y;  % y del landmark
                idx = idx + 2;
            end
            
            obj.x = xNew;
        end
        
        %% ------------------------------------------------
        %% ACTUALIZACIÓN KALMAN
        function obj = update(obj, Z)
            % Z es la matriz Nx2 con las mediciones reales en este instante.
            % Se asume que ya se llamó a landmarksAdministration(Z) antes,
            % para construir H, y definir obj.zMatching y obj.pPred.
            
            if isempty(obj.zMatching)
                % Si no hay mediciones que "actualizar", solo propagamos pPred.
                obj.p = obj.pPred;
                return
            end
            
            % Predecir z con la H construida
            zPred = obj.H * obj.x;
            % Ojo: zPred es un vector largo 2*(#mediciones_usadas).
            % Z(obj.zMatching,:) es la parte de Z que sí está asociada.
            
            % Construir el vector zReal de mediciones "usables" en el mismo orden.
            zReal = [];
            for i = 1:length(obj.zMatching)
                idxMeas = obj.zMatching(i);
                zReal = [zReal; Z(idxMeas,1); Z(idxMeas,2)];
            end
            
            % Innovación
            y = zReal - zPred;
            
            % Matriz de innovación S = H PPred H^T + R
            % Pero R aquí debe ser bloque-diagonal (2x2 por cada medición).
            used  = obj.zMatching;   % índices de observaciones matcheadas
            nUsed = length(used);    % cuántas observaciones se usan            
            blocks = cell(nUsed,1);
            for i = 1:nUsed
                idxObs = used(i);
                 R = diag( obj.variance(idxObs, 1:2) ); 
                  R(1,2)=obj.variance(idxObs,3);
                R(2,1)=R(1,2);
                  blocks{i}=R;
            end
            
            % 2) Combinar con blkdiag
            Rbig = blkdiag(blocks{:});  % dimensión = (2*nMeas) x (2*nMeas)
            
            % 3) Finalmente
            S = obj.H * obj.pPred * obj.H' + Rbig;
                        
            % Distancia de Mahalanobis
            d2Mahal = y'*(S \ y);
            n       = length(y); % Grados de libertad
            chi2Threshold = chi2inv(obj.confidenceLevel, n);
            
            if d2Mahal > chi2Threshold
                warning('KF2D: Actualizacion descartada (chi2)');
                obj.p = obj.pPred;
                obj.zMatching = [];
                obj.lMatching = [];
                return
            end
            
            % Ganancia de Kalman
            K = obj.pPred * obj.H' / S;
            
            % Actualizamos estado
            obj.x = obj.x + K * y;
            
            % Actualizamos covarianza (forzando simetría)
            I = eye(size(obj.pPred));
            Pupd = (I - K*obj.H)*obj.pPred;
            obj.p = 0.5*(Pupd + Pupd');
        end
        
    end
end
