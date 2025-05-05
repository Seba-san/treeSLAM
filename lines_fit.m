% Lines fit

% esto es a modo de prueba, obtiene el ajuste con y sin una rotacion de 45
% grados y se fija cual de la menos incertidumbre.
% tic
[bestModel, inliers,errors] =ransac2Line2D_paralles(pp,parametrosPlantacion,500,1.0); % iterations threshold, 500,1.0
% vect=findPerpPlaneToRows(pp);

% sigmaLines=(sum(errors.^2)/size(errors,1))^0.5;
sigmaLines=std(errors);
% hay que detectar giros de 180 grados o por ahi... ya que la odometria
% cambia de signo. ver que el angulo no cambie bruscamente.

x = linspace(min(pp(:,1)), max(pp(:,1)), 100);
y1 = -(bestModel(1)*x + bestModel(3)) / bestModel(2);
y2 = -(bestModel(1)*x + bestModel(4)) / bestModel(2);

%{
figure(2);
plot(pp(:,1), pp(:,2), 'b.'); hold on;
plot(pp(inliers,1), pp(inliers,2), 'ro');

plot(x, y1, 'g-', 'LineWidth', 2);
plot(x, y2, 'g-', 'LineWidth', 2);
plot(0,0,'*k','MarkerSize',10)
axis equal;
xlim([-5 20]); ylim([-15 15]); grid on
legend('Points', 'Inliers', 'Best fit line','agent pose');
title('RANSAC Line Fitting');
hold off;
pause(0.1)
%}

% fi=atan(bestModel(1)/(-bestModel(2)));saltoFi=fi-fi0;
fi=atan2(bestModel(1),(-bestModel(2)));saltoFi=fi-fi0;
% fi=atan2(vect(2),vect(1))-pi;saltoFi=fi-fi0;
% saltoFi=-acos(cos(fi-fi0)); % para eliminar ambiguedades
% por discontinuidades
while saltoFi<-1.0
    saltoFi= saltoFi+pi/2;
end
while saltoFi>1.0
    saltoFi= saltoFi-pi/2;
end
skipFrameLines=0;
if abs(saltoFi)>0.30
    warning('Salto orientacion de filas grande')
    fi
    fi0
    a=4;
    skipFrameLines=1;
end
% disp(strcat('saltoFi: ',' ',num2str(saltoFi)))
% sigmaLines_=[sigmaLines_ saltoFi];
if  i>1
    % sLm=movmean(sigmaLines_,10);mx=max(abs(sigmaLines_-sLm(end)));
    sLm=movmean(sigmaLines_,10);mx=max(abs(sigmaLines_-sLm));            
    if abs(sigmaLines_(end)-sigmaLines)>mx*3.0+sigmaLines_(1)|| skipFrameLines%+ 1e-4 para evitar el 0; mx*1.5+1e-4        
        % %{
        x = linspace(min(pp(:,1)), max(pp(:,1)), 100);
        y1 = -(bestModel(1)*x + bestModel(3)) / bestModel(2);
        y2 = -(bestModel(1)*x + bestModel(4)) / bestModel(2);
        figure(3);
        plot(pp(:,1), pp(:,2), 'b.'); hold on;
        plot(pp(inliers,1), pp(inliers,2), 'ro');
        plot(x, y1, 'g-', 'LineWidth', 2);
        plot(x, y2, 'g-', 'LineWidth', 2);
        plot(0,0,'*k','MarkerSize',10)
        axis equal;
        xlim([-5 20]); ylim([-15 15]); grid on
        legend('Points', 'Inliers', 'Best fit line','agent pose');
        title('RANSAC Line Fitting');
        hold off;
        pause(0.1)
        %}
        skipFrameLines=1;
    else
        % For debugging propose
        %{ 
        x = linspace(min(pp(:,1)), max(pp(:,1)), 100);
        y1 = -(bestModel(1)*x + bestModel(3)) / bestModel(2);
        y2 = -(bestModel(1)*x + bestModel(4)) / bestModel(2);
        figure(3);
        plot(pp(:,1), pp(:,2), 'b.'); hold on;
        plot(pp(inliers,1), pp(inliers,2), 'ro');
        plot(x, y1, 'g-', 'LineWidth', 2);
        plot(x, y2, 'g-', 'LineWidth', 2);
        plot(0,0,'*k','MarkerSize',10)
        axis equal;
        xlim([-5 20]); ylim([-15 15]); grid on
        legend('Points', 'Inliers', 'Best fit line','agent pose');
        title('RANSAC Line Fitting');
        hold off;
        pause(0.1)
        %}
        sigmaLines_=[sigmaLines_ sigmaLines];
    end
else
    sigmaLines_=[sigmaLines_ sigmaLines];
end

fi0=fi;
fiAcumulado=fiAcumulado+saltoFi;

if fi*atan2(sin(fiAcumulado),cos(fiAcumulado))<0 % Se aplica atan2(y,x) para mapearlo de -pi, pi
    R_=-eye(2);
else
    R_=eye(2);
end
R=R_*[cos(fi) sin(fi);-sin(fi) cos(fi)]';
% R3D=[[cos(fi) sin(fi);-sin(fi) cos(fi)] [0 0]';[0 0 1]];
pp_=pp*R;
pp_full=[[rotatedPoints(:,1) rotatedPoints(:,2)]*R rotatedPoints(:,3)];

% figure(11)
% plot(pp_(:,1),pp_(:,2),'.b')

% #### OJO ACA!!! esto depende del grafico, hay que ver cuales son los
% calculos minimos para sacar el grafico
pp_mod=[x(1) y1(1);x(1) y2(1)];pp_mod_=pp_mod*R;

match_points=pp_(inliers,:);alturas=pp_extendido(inliers,3);


% TEST
% diezmado
% num_points = size(match_points, 1);
% indices_to_remove = randperm(num_points, round(num_points*0.8));
% match_points(indices_to_remove, :) = []; 
% data=pp(inliers,:);
idx=match_points(:,2)>mean(match_points(:,2));% Se queda con el punto del medio;-parametrosPlantacion(2)*0.7/2;
data=match_points(idx,:);




data_mean = median(data);%mean(data);
centered_data = data - data_mean;
data=match_points(~idx,:);
data_mean = median(data);%mean(data);
centered_data = [centered_data ;data - data_mean];

% centered_data = data;
% Calcular la matriz de covarianza
cov_matrix = cov(centered_data);

% Obtener los eigenvectores y eigenvalores
[eig_vectors, eig_values] = eig(cov_matrix);

% El eigenvector con el mayor eigenvalor indica la dirección principal
[~, max_idx] = max(diag(eig_values));
principal_vector = eig_vectors(:, max_idx);

% Calcular el ángulo del eigenvector principal
dominant_angle = atan2d(principal_vector(2), -principal_vector(1));

theta = -deg2rad(dominant_angle); % Convertir a radianes
rotation_matrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
R=R*rotation_matrix;
% Aplicar la rotación a los datos

angleCorrection=[angleCorrection dominant_angle];
%{
if abs(dominant_angle)>1 && false
    a=1;
    data_mean=mean(match_points);
% aligned_data = (rotation_matrix *(match_points-data_mean)')' + data_mean;
aligned_data = match_points*rotation_matrix;
figure(3)
plot(match_points(:,1),match_points(:,2),'.b'); hold on
plot(aligned_data(:,1),aligned_data(:,2),'.r');hold off
axis equal
figure(4)
pp__=pp*R;
plot(pp_(:,1),pp_(:,2),'.b');hold on
plot(pp__(:,1),pp__(:,2),'.r');hold off
axis equal
end
%}
% FIN TEST

pp_=pp*R;
pp_full=[[rotatedPoints(:,1) rotatedPoints(:,2)]*R rotatedPoints(:,3)];

% figure(11)
% plot(pp_(:,1),pp_(:,2),'.b')

% #### OJO ACA!!! esto depende del grafico, hay que ver cuales son los
% calculos minimos para sacar el grafico
pp_mod=[x(1) y1(1);x(1) y2(1)];pp_mod_=pp_mod*R;

match_points=pp_(inliers,:);alturas=pp_extendido(inliers,3);
match_pointsDiezmado=diezmado3D([match_points alturas],0.1);%0.05
% match_pointsDiezmado=[match_points alturas];
alturas=match_pointsDiezmado(:,3);
match_points_=match_pointsDiezmado(:,1:2);
if min(alturas)<0
    alturas=alturas-min(alturas)+0.1;
end

%%

function vect = findPerpPlaneToRows(P2D)
% P: matriz Nx3 con las coordenadas (x, y, z) de N puntos
% (x_i, y_i, z_i) = P(i,1), P(i,2), P(i,3).
%
% OBJETIVO:
%   Hallar un plano vertical A*x + B*y + D = 0
%   que sea perpendicular a la dirección principal de las filas de árboles
%   en el plano XY.

    % 1) Extraer sólo las coordenadas en el plano XY
%     P2D = P(:,1:2);   % Nx2

    % 2) Hacer PCA en 2D para encontrar la dirección principal de las hileras
    %    COEFF(:,1) será el vector unitario correspondiente a la mayor varianza.
    [COEFF, ~, ~] = pca(P2D);

    % 3) La dirección de las hileras es la primera componente principal
    dirPrincipal = COEFF(:,1);     % dirPrincipal es un vector 2x1
    dirPrincipal = dirPrincipal / norm(dirPrincipal);  % Normalizar

    %   Con esto, el vector normal del plano será la misma dirección
    %   (A, B) = (dirPrincipal(1), dirPrincipal(2))
    A = dirPrincipal(1);
    B = dirPrincipal(2);

    % 4) Calcular el mejor D en el sentido de mínimos cuadrados
    %    D = - mean(A*x_i + B*y_i)
    proyecciones = A*P2D(:,1) + B*P2D(:,2);
    D = -mean(proyecciones);
    
    vect=[A,B,D];

    % El plano resultante es: A*x + B*y + D = 0
end

function [bestModel, inliers,errors] = ransac2Line2D_paralles(points,parametrosPlantacion,maxIterations, threshold)
% points: Nx2 matrix, where each row is a point [x, y]
% maxIterations: Maximum number of RANSAC iterations
% threshold: Distance threshold to consider a point an inlier
% al final implementa un minimos cuadrados para estimar bien las rectas

if nargin<2
    d=4;% distancia de paralelas
else
    d=parametrosPlantacion(2);
end
if nargin<3
    maxIterations=100;%100
    threshold=parametrosPlantacion(1)*0.8/2;%0.5;%1.0
end
% Number of points
numPoints = size(points, 1);

% Variable to store the best model and inliers
bestModel = [];
bestInliersCount = 0;
bestInliers = [];

% Distance of each point
D=(points(:,1).^2+points(:,2).^2).^0.5;
% densidad=100*exp(-0.34*D);
densidad=5.7e5*exp(-3.55*D);
thresholdModified=threshold./densidad;


% RANSAC iterations
for i = 1:maxIterations
    % Step 1: Randomly sample 2 points to define a line
    sampleIdx = randperm(numPoints, 2);
    samplePoints = points(sampleIdx, :);
    
    % Step 2: Fit a line model using the two points
    p1 = samplePoints(1, :);
    p2 = samplePoints(2, :);
    
    % Line parameters (ax + by + c = 0)
    a = p2(2) - p1(2); % y2 - y1
    b = p1(1) - p2(1); % x1 - x2
    c = p2(1)*p1(2) - p1(1)*p2(2); % x2*y1 - x1*y2
    
    % Step 3: Measure the distance of all points to the line
    distances = abs(a*points(:,1) + b*points(:,2) + c) / sqrt(a^2 + b^2);
    
    distances2= abs(a*points(:,1) + b*points(:,2) + c+d*sqrt(a^2 + b^2)) / sqrt(a^2 + b^2);
    
    
    % Step 4: Determine inliers
    inliers = find(distances < threshold);
    inliers=[inliers ; find(distances2 < threshold)];
%     inliers = find(distances < thresholdModified);
%     inliers=[inliers ; find(distances2 < thresholdModified)];
    inliersCount = length(inliers);
%     inliersCount = sum(thresholdModified(inliers));
    
    % Step 5: Update the best model if the current one has more inliers
    if inliersCount > bestInliersCount
        n=sqrt(a^2 + b^2);
        bestModel = [a/n, b/n, c/n, c/n+d];
        bestInliersCount = inliersCount;
        bestInliers = inliers;
    end
end

% Return the best model and inliers
inliers = bestInliers;
bestModel = fminsearch(@(p) fit2parallelLinesCost(p, points(inliers,1), points(inliers,2), parametrosPlantacion(2)),bestModel(1:3));
[~,errors,idx]=fit2parallelLinesCost(bestModel, points(inliers,1), points(inliers,2),parametrosPlantacion(2));
% Selecciona cual de las 2 rectas posibles es la correcta
n=(bestModel(1)^2 + bestModel(2)^2)^0.5;c=bestModel(3);lastParameter=[c+d*n c-d*n];
bestModel=[bestModel lastParameter(idx)];
end

function [cost,errors,idx]=fit2parallelLinesCost(params,x,y,d_)
a = params(1);
b = params(2);
c = params(3);
%     d = params(4);
n=sqrt(a^2+b^2);
%     a=a/n;b=b/n;c=c/n;
%     n=sqrt(a^2+b^2);
da =c+d_*n;
db =c-d_*n;

%     y1 = -(a*x + c) / b;
%     y2 = -(a*x + d) / b;
d1=abs(a*x+b*y+c)/n;
d2a=abs(a*x+b*y+da)/n;
d2b=abs(a*x+b*y+db)/n;
%     errors=min((([y1 y2]-y).^2),[],2);
errors=min([d1 d2a d2b],[],2);
cost=sum(errors);
[~,idx]=min([mean(d2a) mean(d2b)]);
end
function pointsReduced = diezmado3D(points,gridStep)
% ptCloud = pointCloud([points(:,1), points(:,2), zeros(size(points(:,1),1),1)]);
ptCloud = pointCloud([points(:,1), points(:,2), points(:,3)]);

% Diezma la nube de puntos
% gridStep = 1.0;  % Define el tamaño de la celda de la cuadrícula para el diezmado
ptCloudDownsampled = pcdownsample(ptCloud, 'gridAverage', gridStep);

% Extrae las coordenadas 2D diezmadas
X_downsampled = ptCloudDownsampled.Location(:,1);
Y_downsampled = ptCloudDownsampled.Location(:,2);
Z_downsampled = ptCloudDownsampled.Location(:,3);
pointsReduced=[X_downsampled Y_downsampled Z_downsampled];
end