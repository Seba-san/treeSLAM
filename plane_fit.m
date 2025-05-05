% Plane fit
% P=downsamplePointCloud(P(:,1:3), 0.25);
% Optimización para encontrar los mejores parámetros
% tic
% optimalParams = fminsearch(@(p) fitPlaneCost(p, P_(:,1), P_(:,2), P_(:,3)), initialParams, options);
% [rotatedPoints,rotationMatrix]=rotatePlane(optimalParams,P);
zTh=0.3;%0.3m
threshold=0.2;
[best_plane, inliers] = ransac_plane_fit([P_(:,1), P_(:,2), P_(:,3)], 100, threshold);
%{
% ############ INBICIO MODIFICACION
iteraciones_ransac=100;
iteraciones_bootsrap=500;
[mean_plane_params, cov_params, sigmas_vec] = bootstrap_plane_fit(P_, iteraciones_bootsrap, iteraciones_ransac, 0.2);
 if ~any(isnan(mean_plane_params))
%      optimalParams = mean_plane_params; % Usar el plano medio como resultado
     sigma_zs = sigmas_vec(1)
     sigma_pitch = sigmas_vec(2)
     sigma_roll = sigmas_vec(3)
sigmaPlaneGoogle=(sigma_zs^2*sigma_pitch^2*sigma_roll^2)^(1/6);
else
      error('Ajuste de plano falló.');
 end
% ############ FIN modificacion 2
%}
%{
% ############ INBICIO MODIFICACION
P_inliers = P_(inliers,:); % Asegúrate de que P_ son los puntos originales y 'inliersBest' los índices correctos
[best_plane, centroid_inliers, V_svd, S_diag_svd, N_inliers] = least_squares_fit_detailed(P_inliers);

if ~any(isnan(best_plane))
    optimalParams = best_plane; % [nx, ny, nz, d]'
    normal_opt = optimalParams(1:3);
    % Asegurar normalización y dirección (si es necesario, aunque SVD ya lo da normalizado)
    normal_opt = normal_opt / norm(normal_opt);
    if normal_opt(3) < 0 % Asumimos que el plano del suelo tiene normal apuntando hacia Z positivo
       normal_opt = -normal_opt;
       optimalParams(1:4) = [-normal_opt; -optimalParams(4)]; % Corregir d también
    end

    % Estimar incertidumbres
    [sigma_zs, sigma_pitch, sigma_roll] = estimate_plane_uncertainty(V_svd, S_diag_svd, N_inliers, normal_opt);
    sigmaPlaneGoogle=(sigma_zs^2*sigma_pitch^2*sigma_roll^2)^(1/6);
%      fprintf('  Sigma plane   : %.4f m\n',  sigmaPlaneGoogle);
    %{
    fprintf('Incertidumbres estimadas:\n');
    fprintf('  Sigma Zs    : %.4f m\n', sigma_zs);
    fprintf('  Sigma Pitch : %.4f rad (%.2f deg)\n', sigma_pitch, rad2deg(sigma_pitch));
    fprintf('  Sigma Roll  : %.4f rad (%.2f deg)\n', sigma_roll, rad2deg(sigma_roll));
    %}
    % Ahora puedes usar estas sigmas en tu sistema SLAM o análisis
else
      error('Ajuste de plano falló.');
end

% ############ FIN MODIFICACION
%}
[best_plane] = least_squares_fit(P_(inliers,:));
best_plane(4)=-best_plane(4);
optimalParams=best_plane;
[rotatedPoints,rotationMatrix]=rotatePlane(best_plane,P);
[rotatedPoints_,rotationMatrix_]=rotatePlane(best_plane,P_);

n=optimalParams(1:3)/(sum(optimalParams(1:3).^2)).^0.5;
k = [0, 0, 1]; % normal objetivo
if dot(n, k)<0
    n=-n; % ajusta la orientacion de la proyeccion
end
% pitch=asin(-n(1));pitchDerees=pitch*180/pi
% roll=atan2(n(2),n(3)); rollDegrees=roll*180/pi

% toc

% offset_z=min(rotatedPoints(:,3));
% offset_z=prctile(rotatedPoints(:,3),25);
offset_z=mode(round(rotatedPoints(:,3),2)); % redondea xq sino no se repite ningun valor
% point=[0 0 -best_plane(4)/best_plane(3)];
% pointRotated=point*rotationMatrix;
% offset_z=pointRotated(3);

%{
%########################################################## Visualizacion
figure(1);
idx= ((rotatedPoints(:,3)-offset_z)>zTh);%&((rotatedPoints(:,3)-offset_z)<0.7);
%scatter3(rotatedPoints(:,1), rotatedPoints(:,2), rotatedPoints(:,3)-offset_z,3,P(:,4), 'filled');
scatter3(rotatedPoints(:,1), rotatedPoints(:,2), rotatedPoints(:,3)-offset_z,3, 'filled');
axis equal
hold on
scatter3(rotatedPoints(idx,1), rotatedPoints(idx,2), rotatedPoints(idx,3)-offset_z,3,'r' ,'filled');
hold off
figure(2)
pointsPC=pointCloud(rotatedPoints);
pcshow(pointsPC,'MarkerSize',20)
pause(0.1)
% continue
%}

pp=round(rotatedPoints_(:,3),6);
err=pp-mode(pp);
% sigmaPlane=(sum(err(idx).^2)/sum(idx))^0.5; % asumo simetria
p1=prctile(err,10);
p3=prctile(err,30);
p7=prctile(err,70);
p5=prctile(err,50);

sigmaPlane1=(p5-p1)/1.28155;
sigmaPlane2=(p7-p3)/1.048; % hay ciertas hipotesis: el valor mas frecuente cae sobre el plano....
sigmaPlane3=(p3-p1)/0.75715;
sigmaPlane4=(p5-p3)/0.52440;
sigmaInliers=(sum((pp(inliers)-mean(pp(inliers))).^2)/sum(inliers))^0.5; % el maximo 2*0.1/12^0.5
% sigmaInliers=std(pp(inliers)); % el maximo 2 0.1/12^0.5
sigmas=[sigmaPlane1, sigmaPlane2,sigmaPlane3,sigmaPlane4]+1e-4;% para evitar que sean cero
sigmaPlane=min(sigmas);
idx= ((rotatedPoints(:,3)-offset_z)>zTh);
sigmaPlane=std(rotatedPoints(~idx,3));
 

% fprintf('  Sigma plane google: %.4f , sigma seba %.4f \n',  sigmaPlaneGoogle,sigmaPlane);

% sigma_plane=(sum(errors(idx_).^2)/size(errors(idx_),1))^0.5;%sum(errors)/size(errors,1)
if  sigmaPlane>threshold*1.5
    warning('Problemas en el ajuste del plano, skip frame')
    sigmaPlane
    figure(2)
    [x, y] = meshgrid(min(P_(:,1)):range(P_(:,1))/10:max(P_(:,1)), min(P_(:,2)):range(P_(:,2))/10:max(P_(:,2)));
    a_opt=optimalParams(1);b_opt=optimalParams(2);c_opt=optimalParams(3);d_opt=optimalParams(4);
    z = (d_opt - a_opt*x - b_opt*y) / c_opt;
    scatter3(P_(:,1), P_(:,2), P_(:,3),3,'b', 'filled');
    hold on;
    mesh(x, y, z);
    idx=abs(err)>2*sigmaPlane;
    scatter3(P_(idx,1), P_(idx,2), P_(idx,3),3,'r', 'filled');
    scatter3(P_(inliers,1), P_(inliers,2), P_(inliers,3),3,'k', 'filled');
    hold off;
    legend('puntos crudos','plano','puntos del plano','inliers')
    title(strcat('Frame:',' ',num2str(i)))
    i
    axis equal
    figure(1);
    idx= ((rotatedPoints(:,3)-offset_z)>zTh);%&((rotatedPoints(:,3)-offset_z)<0.7);
%     scatter3(rotatedPoints(:,1), rotatedPoints(:,2), rotatedPoints(:,3)-offset_z,3,P(:,4), 'filled');
    scatter3(rotatedPoints(:,1), rotatedPoints(:,2), rotatedPoints(:,3)-offset_z,3, 'filled');
    axis equal
    hold on
    scatter3(rotatedPoints(idx,1), rotatedPoints(idx,2), rotatedPoints(idx,3)-offset_z,3,'r' ,'filled');
    hold off
    %     continue
end


idxPlane= ((rotatedPoints(:,3)-offset_z)>zTh);
% idx2= ((rotatedPoints(:,3)-offset_z)<1.3);
% idx=idx & idx2;
pp=[rotatedPoints(idxPlane,1), rotatedPoints(idxPlane,2)];pp_extendido=[rotatedPoints(idxPlane,1), rotatedPoints(idxPlane,2),rotatedPoints(idxPlane,3)-offset_z];
%%
function [best_plane, inliersBest] = ransac_plane_fit(points, num_iterations, distance_threshold)
    num_points = size(points, 1);
    best_inlier_count = 0;
    best_plane = [];
    inliersBest=[];

    for i = 1:num_iterations
        % Selección aleatoria de tres puntos
        indices = randperm(num_points, 3);
        sample = points(indices, :);

        % Calcular el plano a partir de los tres puntos
        v1 = sample(2, :) - sample(1, :);
        v2 = sample(3, :) - sample(1, :);
        normal = cross(v1, v2);
        point = sample(1, :);
        d = -dot(normal, point);

        % Calcular la distancia de todos los puntos al plano
        distances = abs(points * normal' + d) / norm(normal);

        % Contar inliers
        inliers = distances < distance_threshold;
        inlier_count = sum(inliers);

        % Actualizar el mejor modelo
        if inlier_count > best_inlier_count
            best_inlier_count = inlier_count;
            best_plane = [normal, d];
            inliersBest=inliers;
        end
    end

    %fprintf('Mejor plano encontrado:\n %f x + %f y + %f z + %f = 0 con %d inliers\n', ...
      %  best_plane(1), best_plane(2), best_plane(3), best_plane(4), best_inlier_count);
end
function [plane_model] = least_squares_fit(points)
%     Asumimos que points es una matriz Nx3 de los inliers

    % Normalizar los puntos restando el centroide
    centroid = mean(points);
    normalized_points = points - centroid;

    % Aplicar SVD a los puntos normalizados
    [U, S, V] = svd(normalized_points, 'econ');

    % El vector normal al plano es la última columna de V
    normal = V(:,end);

    % El término d se calcula como el producto escalar del normal y el centroide
    d = -dot(normal, centroid');

    % El modelo del plano es entonces [a; b; c; d]
    plane_model = [normal; d];
%     fprintf('Modelo de plano ajustado: %f x + %f y + %f z + %f = 0\n', plane_model(1), plane_model(2), plane_model(3), plane_model(4));
end
function [rotatedPoints,rotationMatrix]=rotatePlane(optimalParams,P)
% Extrae los parámetros óptimos
a_opt = optimalParams(1);
b_opt = optimalParams(2);
c_opt = optimalParams(3);
d_opt = optimalParams(4);

% rotacion del plano
normal=[a_opt b_opt c_opt];normal=normal./norm(normal);
k = [0, 0, 1]; % normal objetivo
% Calcular el ángulo de rotación y el eje
if dot(normal, k)<0
    normal=-normal; % ajusta la orientacion de la proyeccion
end
    
angle = acos(dot(normal, k));
axis_ = cross(normal, k);axis_=axis_./norm(axis_);

% Crear la matriz de rotación usando la fórmula de Rodrigues
K = [0, -axis_(3), axis_(2); axis_(3), 0, -axis_(1); -axis_(2), axis_(1), 0];
rotationMatrix = eye(3) + sin(angle) * K + (1 - cos(angle)) * (K * K);

% transladar lo puntos para que la ordenada al origen sea cero
n=[a_opt b_opt c_opt];
P_(:,1:3)=P(:,1:3)-d_opt*n/norm(n)^2;
centroide=mean(P);
P_=P(:,1:3)-centroide(1:3);
P_=P(:,1:3);centroide=zeros(1,3);
% Rotar los puntos
rotatedPoints = (rotationMatrix * P_(:,1:3)')'+centroide(1:3);
end

function [plane_model, centroid, V, S_diag, N_in] = least_squares_fit_detailed(points)
    N_in = size(points, 1);
    if N_in < 3
        % No se puede ajustar un plano
        plane_model = [NaN; NaN; NaN; NaN];
        centroid = [NaN, NaN, NaN];
        V = nan(3);
        S_diag = nan(3,1);
        return;
    end

    centroid = mean(points);
    normalized_points = points - centroid;

    [~, S, V] = svd(normalized_points, 'econ');
    S_diag = diag(S); % Singular values s1, s2, s3

    % El vector normal al plano es la última columna de V
    normal = V(:,end);

    % Asegurar consistencia (opcional, depende de la convención deseada)
    % if normal(3) < 0
    %     normal = -normal;
    % end

    d = -dot(normal, centroid');
    plane_model = [normal; d];
end

% --- 2. Función para estimar incertidumbres ---
function [sigma_zs, sigma_pitch, sigma_roll] = estimate_plane_uncertainty(V, S_diag, N_in, normal_opt)
    % V: Matriz de vectores propios (columnas) de SVD(normalized_points)
    % S_diag: Vector de valores singulares (s1, s2, s3)
    % N_in: Número de inliers
    % normal_opt: Vector normal estimado [nx, ny, nz]'

    sigma_zs = NaN;
    sigma_pitch = NaN;
    sigma_roll = NaN;
    min_inliers_for_cov = 5; % Umbral mínimo para confiar en la covarianza

    if N_in < min_inliers_for_cov || any(isnan(S_diag)) || any(isnan(normal_opt))
        warning('Insuficientes inliers o datos inválidos para estimación de incertidumbre.');
        return;
    end

    lambda = (S_diag.^2) / N_in; % Estimación de eigenvalues de M = Cov(inliers)
    lambda1 = lambda(1);
    lambda2 = lambda(2);
    lambda3 = lambda(3); % Corresponde a la dirección normal

    % Asegurar eigenvalues positivos y distintos (añadir epsilon)
    epsilon_eig = 1e-9; % Pequeño valor para evitar division por cero
    lambda3 = max(lambda3, epsilon_eig);
    lambda1 = max(lambda1, lambda3 + epsilon_eig);
    lambda2 = max(lambda2, lambda3 + epsilon_eig);

    % Estimación de sigma_zs^2
    % Varianza de la distancia perpendicular al plano
    sigma_d_sq = lambda3; % Varianza de la distancia a lo largo de la normal
    % Propagar a la altura zs (aproximado: dividir por nz^2)
    nz_sq = normal_opt(3)^2;
    sigma_zs_sq = sigma_d_sq / max(nz_sq, epsilon_eig); % Evitar división por cero si nz es muy pequeño
    sigma_zs = sqrt(sigma_zs_sq);


    % Estimación de Covarianza de la Normal (Sigma_n) - Aproximación Kanatani-like
    v1 = V(:,1);
    v2 = V(:,2);
    % Factor común (varianza residual estimada)
    res_var_factor = lambda3; % Usamos lambda3 directamente como proxy de varianza residual por punto

    cov_n_approx = res_var_factor * ( (1 / (lambda1 - lambda3)) * (v1 * v1') + ...
                                       (1 / (lambda2 - lambda3)) * (v2 * v2') );

    % Estimación de sigma_pitch^2 y sigma_roll^2 (Aproximación de Pequeños Ángulos)
    % Asume que la covarianza de (nx, ny) se mapea a (pitch, roll)
    % ¡¡CUIDADO!! Esto depende de la convención de ejes y ángulos.
    % Si R de Rodrigues alinea la normal con Z, entonces Sigma_n representa
    % la incertidumbre de la normal *antes* de la rotación.
    % Lo que necesitamos es la incertidumbre de los *ángulos* que definen R.

    % Enfoque alternativo y quizás más directo para pequeños ángulos:
    % La varianza angular alrededor del eje v2 está relacionada con lambda1 y lambda3.
    % La varianza angular alrededor del eje v1 está relacionada con lambda2 y lambda3.
    % sigma_angle_v2^2 ~ lambda3 / lambda1
    % sigma_angle_v1^2 ~ lambda3 / lambda2
    % Necesitamos mapear esto a pitch/roll. Si v1 ~ eje Y global y v2 ~ eje X global (aprox)
    % entonces sigma_roll^2 ~ lambda3 / lambda1 y sigma_pitch^2 ~ lambda3 / lambda2

    % Vamos a usar esta aproximación MÁS SIMPLE y potencialmente más robusta:
    var_pitch_approx = lambda3 / max(lambda2, epsilon_eig); % Varianza angular sobre eje v2 (aprox X -> Pitch)
    var_roll_approx  = lambda3 / max(lambda1, epsilon_eig); % Varianza angular sobre eje v1 (aprox Y -> Roll)

    % Las varianzas deben ser positivas
    sigma_pitch = sqrt(max(var_pitch_approx, epsilon_eig));
    sigma_roll  = sqrt(max(var_roll_approx,  epsilon_eig));

    % [OPCIONAL] Convertir a grados si se desea para display/interpretación
    % sigma_pitch_deg = rad2deg(sigma_pitch);
    % sigma_roll_deg  = rad2deg(sigma_roll);

end


function [mean_plane, cov_plane_params, sigmas_est] = bootstrap_plane_fit(P, num_bootstrap, num_ransac_iter, distance_threshold)

    N_pts = size(P, 1);
    stored_normals = [];
    stored_ds = [];
    valid_fits = 0;

    for b = 1:num_bootstrap
        % 1. Resample with replacement
        indices_boot = randi(N_pts, N_pts, 1);
        P_boot = P(indices_boot, :);

        % 2. RANSAC
        [best_plane_ransac, inliers_ransac] = ransac_plane_fit(P_boot, num_ransac_iter, distance_threshold); % Tu RANSAC

        if isempty(best_plane_ransac) || sum(inliers_ransac) < 5 % Chequeo mínimo
            continue; % Saltar si RANSAC falla
        end

        % 3. SVD Fit on RANSAC inliers
        P_inliers_boot = P_boot(inliers_ransac,:);
        [plane_model_svd, ~, ~, ~, ~] = least_squares_fit_detailed(P_inliers_boot); % Tu SVD detallado

        if any(isnan(plane_model_svd))
            continue; % Saltar si SVD falla
        end

        % 4. Store results (ensure consistent normal direction)
        normal_b = plane_model_svd(1:3);
        d_b = plane_model_svd(4);
        if normal_b(3) < 0
            normal_b = -normal_b;
            d_b = -d_b;
        end

        stored_normals = [stored_normals, normal_b];
        stored_ds = [stored_ds, d_b];
        valid_fits = valid_fits + 1;
    end

    if valid_fits < 10 % Necesitamos suficientes muestras para estimar covarianza
        warning('Bootstrap no generó suficientes ajustes válidos.');
        mean_plane = [NaN; NaN; NaN; NaN];
        cov_plane_params = nan(4);
        sigmas_est = [NaN, NaN, NaN];
        return;
    end

    % 5. Calculate Mean Plane
    mean_n = mean(stored_normals, 2);
    mean_n = mean_n / norm(mean_n); % Normalize mean normal
    mean_d = mean(stored_ds);
    mean_plane = [mean_n; mean_d];

    % 6. Calculate Covariance Matrix (4x4)
    params_matrix = [stored_normals; stored_ds]; % 4 x valid_fits
    cov_plane_params = cov(params_matrix'); % Transponer para que cov calcule bien

    % 7. Propagate to Pitch/Roll/Zs Uncertainty
    %   a. Covarianza de la normal (empírica)
    cov_n_empirical = cov_plane_params(1:3, 1:3);
    %   b. Varianza de d (empírica)
    var_d_empirical = cov_plane_params(4, 4);
    %   c. Varianza de Zs (propagando d y nz)
    %      Jacobiano de zs = -d/nz respecto a [nx, ny, nz, d] en mean_plane
    nz = mean_n(3);
    d = mean_d;
    J_zs = [d*mean_n(1)/(nz^2), d*mean_n(2)/(nz^2), d*mean_n(3)/(nz^2) - 1/nz , -1/nz]; % Revisar derivada nz
    var_zs_propagated = J_zs * cov_plane_params * J_zs';
    sigma_zs = sqrt(max(var_zs_propagated, 1e-9));

    %   d. Covarianza de Pitch/Roll (propagando cov_n_empirical)
    %      Jacobiano de (pitch, roll) = angles_from_normal(nx, ny, nz)
    [J_angles] = jacobian_normal_to_angles(mean_n); % Necesitas implementar esta función
    cov_angles = J_angles * cov_n_empirical * J_angles';
    sigma_pitch = sqrt(max(cov_angles(1,1), 1e-9));
    sigma_roll  = sqrt(max(cov_angles(2,2), 1e-9));

    sigmas_est = [sigma_zs, sigma_pitch, sigma_roll];

end

function [J] = jacobian_normal_to_angles(n)
    % Calcula el Jacobiano 2x3 de [pitch; roll] respecto a [nx; ny; nz]
    % Usando pitch = asin(nx), roll = atan2(ny, nz) (¡CUIDADO CON SINGULARIDAD SI nx=1!)
    nx = n(1); ny = n(2); nz = n(3);
    eps = 1e-6; % Para evitar división por cero y sqrt de negativo

    dp_dnx = 1 / sqrt(max(1 - nx^2, eps));
    dp_dny = 0;
    dp_dnz = 0;

    dr_dnx = 0;
    dr_dny = nz / max(ny^2 + nz^2, eps);
    dr_dnz = -ny / max(ny^2 + nz^2, eps);

    J = [dp_dnx, dp_dny, dp_dnz;
         dr_dnx, dr_dny, dr_dnz];
end
