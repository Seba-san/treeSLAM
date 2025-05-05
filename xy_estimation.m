saltoOdometricoGrande=0;
% figure(3);histogram(pp_(:,2),'BinWidth',0.1)
[data, edges] = histcounts(pp_(:,2), 'BinWidth',0.1);
[picos, locs] = findpeaks(data,'MinPeakProminence', 5);
% edges(locs)
mx1=max(picos);
p1=find(picos==mx1,1);
m1=min(edges)+locs(p1)*0.1;
picos(p1)=0;

mx2=max(picos);
p2=find(picos==mx2,1);
m2=min(edges)+locs(p2)*0.1;
picosTmp=picos;
while abs(m1-m2)<parametrosPlantacion(2)*0.7 || m2*m1>0 % distinto signo
    picosTmp(p2)=0;
    mx2=max(picosTmp);
    p2=find(picosTmp==mx2,1);
    m2=min(edges)+locs(p2)*0.1;
    if sum(picosTmp)==0
        error('No tree found in x_estimation')
    end
    
end
modas=[m1 m2];
centralPointsIdx=pp_full(:,2)>min(modas)+parametrosPlantacion(2)/4;
centralPointsIdx=centralPointsIdx &  pp_full(:,2)<max(modas)-parametrosPlantacion(2)/4;
centerPoints=pp_full(centralPointsIdx,:);
% Filter the middle points in the way
mm=mean(centerPoints(:,3));
std_=std(centerPoints(:,3));
th=min(mm+4*std_,max(centerPoints(:,3)));

i_=0;
enoughtPoints=0;
centerPoints_=centerPoints;
while sum(centerPoints(:,3)>th)<5 || ~enoughtPoints % evita que no se tengan puntos
    th=th-i_*0.1*std_;
    i_=i_+1;
    
    centerPoints=centerPoints_(centerPoints_(:,3)>th,:);
    %{
figure(11)
plot(pp_full(:,1),pp_full(:,2),'.b');hold on
plot(pp_full(centralPointsIdx,1),pp_full(centralPointsIdx,2),'.r');hold off
% plot(centerPoints(:,1),centerPoints(:,2),'.r');hold off
axis equal
xlim([-5 20])
    %}    
    if sum(abs(diff(centerPoints(:,1)))>1)>0 % si hay puntos del centro mas separados que 1 m en x
        %     sigmaLines
        MinPts=3; %10
        eps=1.0;
        [labels] = dbscan(centerPoints(:,1), eps, MinPts);
        ntrees=max(labels);
        if ntrees==-1
            warning("No se encontraron puntos de referencia en el camino.")
            saltoOdometricoGrande=1;
        elseif  ntrees<2 % Si no hay suficientes puntos, se filtran
            idxCluster=labels==1;
            centerPoints=centerPoints(idxCluster,:);
            [observedCenters,desvios]=getCenter(centerPoints);
            %         observedCenters=mean(centerPoints);
        else
            % More than one object was observed
            observedCenters=zeros(ntrees,3);
            desvios=zeros(ntrees,3);
            %         distancias=zeros(ntrees,3);
            for j=1:ntrees
                idxCluster=labels==j;
                [ observedCenters(j,:),desvio]=getCenter(centerPoints( idxCluster,:));
                desvios(j,:)=desvio;
            end            
        end
        enoughtPoints=1;
    elseif ~isempty(centerPoints)
        [observedCenters,desvios]=getCenter(centerPoints);
        enoughtPoints=1;
    else
        warning("No se encontraron puntos de referencia en el camino.")
        saltoOdometricoGrande=1;
    end
end    
idx_=isnan(desvios);desvios(idx_)=1.0;
% kf1d.R_z=max(desvios.^2,0.1);

% Pisa el matching y las varianzas
matching_analysis

kf2d.variance=[desvios(:,1:2).^2 desvios(:,3)];
% kf1d.R_z=max(sigmaFitX^2,0.2);
dMax=1.0;
kf2dBackUp=kf2d;

kf2d=kf2d.landmarksAdministration(observedCenters);

if ~isempty(observedCenters)
kf2d=kf2d.update(observedCenters);
else
    kf2d=kf2d.update([]);
end

%{
% KF SLAM
figure(1)
clf
% plot(kf1d.x(2:end)-kf1d.x(1),zeros(size(kf1d.x(2:end))),'ok');hold on
% plot(kf1d.x(kf1d.lMatching)-kf1d.x(1),zeros(size(kf1d.lMatching)),'or')
hold on
num_landmarks = (length(kf2d.x) - 2) / 2;
for jj = 1:num_landmarks   % Plot landmakrs
    idx_x = 2 + 2*jj - 1; % Índice de lix
    idx_y = 2 + 2*jj;     % Índice de liy    
    landmark_pos = [kf2d.x(idx_x), kf2d.x(idx_y)]-[kf2d.x(1) kf2d.x(2)];
    cov_landmark = kf2d.p(idx_x:idx_y, idx_x:idx_y);
    plotEllipse(landmark_pos, cov_landmark, 'EdgeColor', 'k', 'LineWidth', 1, 'LineStyle', '-');
end
for mm = 1:size(kf2d.lMatching,1)
    idx_x = kf2d.lMatching(mm, 1);
    idx_y = kf2d.lMatching(mm, 2);
    landmark_pos = [kf2d.x(idx_x), kf2d.x(idx_y)]-[kf2d.x(1) kf2d.x(2)];   
    cov_landmark = kf2d.p([idx_x idx_y],[idx_x idx_y]);        
    plotEllipse(landmark_pos, cov_landmark,'EdgeColor', 'r', 'LineWidth', 1, 'LineStyle', '-');
end

plot(0,0,'sk');
plot([0 2*cos(-fiAcumulado+theta)],[0 2*sin(-fiAcumulado+theta)],'-k'); % orientation
idx=pp_full(:,3)>th;
plot(pp_full(idx,1),pp_full(idx,2),'.b');
plotEllipse([0 0], kf2d.p(1:2,1:2)^0.5,'EdgeColor', 'r', 'LineWidth', 1, 'LineStyle', '-');
hold off
axis equal
if mean(pp_(:,1))>0
    xlim([-5 20]);ylim([-6 6])
else
    xlim([-20 5]);ylim([-6 6])
end
grid on
pause(0.1)
if i==200
    a=1;
end
%}
% observedCentersHistoric{i}.data=observedCenters;
% observedCentersHistoric{i}.association=kf1d.zMatching;

 %{
figure(1)
clf
plot(observedCenters(observedMatching,1),observedCenters(observedMatching,2),'*k');hold on
n_=1:1:size(observedCenters,1); observedMatching_=setdiff(n_',observedMatching);
plot(observedCenters(observedMatching_,1),observedCenters(observedMatching_,2),'*c');
plot(referencias(:,1),referencias(:,2),'or');
plot(0,0,'sk');
plot(pp_(:,1),pp_(:,2),'.b');
hold off
axis equal
% if abs(max(pp_(:,1)))>abs(min(pp_(:,1)))
if mean(pp_(:,1))>0
xlim([-5 20]);ylim([-6 6])
else
    xlim([-20 5]);ylim([-6 6])
end
if ~isempty(observedMatching_) && ~isempty(observedMatching)
legend('observed and matched points','observer not matched','references','agent','all points')
elseif ~isempty(observedMatching_) && isempty(observedMatching)
    legend('observer not matched','references','agent','all points')
elseif isempty(observedMatching_) && ~isempty(observedMatching)
     legend('observed and matched points','references','agent','all points')
end
pause(0.1)
%}

% sigmaFitX=kf1d.p(1,1)^0.5;
sigmaFitX=det(kf2d.p(1:2,1:2))^(1/4);
SigmaX=kf2d.p(1:2,1:2);
pose=[kf2d.x(1) kf2d.x(2)];
%### calcular incertidumbre de medicion

if size(poses,1)>1
    if abs(pose(1)-poses(end,1))>0.3
        pose(1)-poses(end,1)
        warning('Salto odometrico grande!!')
%         sigmaFitX=1;
%         warning('Sigmafit sobrescrito a 1')
%         saltoOdometricoGrande=1;
    end
end

%%

function kf2=copyKF(kf1,kf2,z)

kf2.h=kf1.h;
kf2.pPred=kf1.pPred;
kf2.zMatching=kf1.zMatching;
kf2.lMatching=kf1.lMatching;
kf2.nNew=kf1.nNew;
kf2.p=kf1.p;
kf2.R_z=kf1.R_z;
if ~isempty(kf2.nNew)
    xNew=z(kf2.zMatching(end-kf2.nNew+1:end))+kf2.x(1);
    xNew=[kf2.x; xNew];
    kf2.x=xNew;
    
end
end

function [odo,flag]=filterOdo(odo,parametrosPlantacion)
    odo_=odo;
    flag=0;
    saltoMx=0.3;%0.1
    dh=parametrosPlantacion(2); % delta width
    dt=parametrosPlantacion(1); % delta tree
    while abs(odo(1)/dt)>0.5
        odo(1)=odo(1)-dt*abs(odo(1))/odo(1);
    end
    while abs(odo(2)/dh)>0.5
        odo(2)=odo(2)-dh*abs(odo(2))/odo(2);
    end
%     if abs(odo(1))>saltoMx || abs(odo(2))>saltoMx
    if  abs(odo(2))>saltoMx
        warning('Salto odometrico grande') 
        flag=1;
        odo
%         odo=[0 0];
    end
%     [odo_;odo]
end
function [labels] = dbscan(X, eps, MinPts)
% DBSCAN: Density-Based Spatial Clustering of Applications with Noise
%
% Esta función implementa el algoritmo de clustering DBSCAN, el cual es capaz de identificar
% clústeres basados en la densidad de los datos. Es especialmente útil en datasets con
% formas de clúster irregulares y presencia de ruido.
%
% Parámetros de entrada:
%   X       - Matriz de datos donde cada fila es un punto y cada columna es una característica.
%   eps     - Radio de vecindad para considerar puntos como parte del mismo clúster.
%   MinPts  - Número mínimo de puntos requeridos para formar un clúster denso.
%
% Salida:
%   labels  - Vector de etiquetas de clúster para cada punto en el dataset. Los puntos
%             etiquetados como -1 son considerados ruido.
%
% Uso:
%   labels = dbscan(data, 0.5, 5);
%   Este comando ejecuta DBSCAN sobre 'data' con un eps de 0.5 y MinPts de 5.
%
% Ejemplo:
%   % Generar datos de ejemplo
%   data = [randn(50,2); randn(50,2)+3; randn(50,2)+5];
%   % Ejecutar DBSCAN
%   clusterLabels = dbscan(data, 0.5, 5);
%   % Visualizar los resultados
%   gscatter(data(:,1), data(:,2), clusterLabels);
%
% El algoritmo puede identificar clústeres de diferentes tamaños y formas, y es robusto
% frente a datos atípicos, lo que lo hace adecuado para aplicaciones como análisis de
% datos espaciales, segmentación de imágenes y detección de anomalías.
% Número de puntos
n = size(X,1);
% Etiquetas para los puntos de datos
labels = zeros(n,1);
% ID del cluster
C = 0;
% Buscar vecinos para cada punto
for i = 1:n
    if labels(i) == 0
        % Encontrar puntos en el radio eps
        neighbors = findNeighbors(X, i, eps);
        
        % Verificar si el punto es un punto central
        if numel(neighbors) < MinPts
            % Marcar como ruido
            labels(i) = -1;
        else
            % Incrementar el ID del cluster
            C = C + 1;
            % Expandir el cluster
            labels = expandCluster(X, labels, i, neighbors, C, eps, MinPts);
        end
    end
end
end
function neighbors = findNeighbors(X, idx, eps)
% Calcular la distancia de todos los puntos al punto idx
distances = sqrt(sum((X - X(idx,:)).^2, 2));
% Encuentra los puntos dentro del radio eps
neighbors = find(distances < eps);
end
function labels = expandCluster(X, labels, idx, neighbors, C, eps, MinPts)
% Asignar el cluster ID a los vecinos
labels(idx) = C;
k = 1;
while k <= length(neighbors)
    % Obtener el siguiente punto del cluster
    i = neighbors(k);
    % Si el punto fue marcado como ruido, cambiarlo a cluster
    if labels(i) == -1
        labels(i) = C;
    end
    % Si el punto no ha sido visitado, etiquetarlo como parte del cluster
    if labels(i) == 0
        labels(i) = C;
        % Encontrar más vecinos
        iNeighbors = findNeighbors(X, i, eps);
        % Si hay suficientes puntos, añadir al cluster
        if length(iNeighbors) >= MinPts
            neighbors = [neighbors; iNeighbors];
        end
    end
    k = k + 1;
end
end

function [desplazamiento_,referencias,observedMatching]=getDisplacement(observedCenters,referencias,dMax)
% En esta funcion sucede la magia del matching y mantenimiento
% referencias: Nuevas referencias
% observedMatching: de los puntos observados, cuales fueron matcheados con
% las referencias
[a,b,d_]=association(observedCenters,referencias,dMax);

if ~isempty(a)
n_=size(a,1);
desplazamiento=zeros(n_,1);
for i=1:n_ % matcheado
    desplazamiento(i)=observedCenters(a(i),1)-referencias(b(i),1);
end

% [~,idx]=max(observedCenters(a(:),1)); % el mas lejano
[~,idx]=min(observedCenters(a(:),1)); % el mas cercano
% [~,idx]=max(observedCenters(a(:),3)); % el mas alto
desplazamiento_=mean(desplazamiento(idx)); % Se queda con el desplazamiento medido por el cono mas lejano
% desplazamiento_=mean(desplazamiento(1:n_));
% desplazamiento_=median(desplazamiento);
else
    desplazamiento_=nan;
    n_=0;
end
na=size(observedCenters,1);
nb=size(referencias,1);% esto es lo nuevo, donde dice nb poner 3
lb=1:1:nb;
cb=setdiff(lb, b);
la=1:1:na;
ca=setdiff(la, a);
m=min(nb-n_,size(ca,2));
if m>0
    for i=1:m % no matcheado
        referencias(cb(i),:)=observedCenters(ca(i),:);
    end
    l=size(cb,2);
    for i=1:l-m
        referencias(cb(i+m),:)=[0 0 0];
    end
end

for i=1:n_ % matcheado
    referencias(b(i),:)=observedCenters(a(i),:);
end
observedMatching=a;
if isnan(desplazamiento_)
    warning('No association found')
    desplazamiento_=0;
end
end

function [a,b,d_]=association(A,B,minDist)
% A = [1 2; 3 4; 5 6; 1 0.5]; % n x 2 matriz para el conjunto A
% B = [1 2; 5 6]; 
n = size(A, 1);
m = size(B, 1);
d_=[];
% Calcula la distancias entre todos los puntos, los guardados y los
% observados
for i=1:n
   d=sum((B-A(i,:)).^2,2).^0.5;
   d_=[d_ d];
end

[b,a]=find(d_<minDist);
i=1;
while  sum(diff(a)==0)>0
    % Asignaciones a dos puntos de referencia
    % elimina ambiguedades
    [b,a]=find(d_<minDist-minDist*i/10);
    i=i+1;
    
end
end

function [centerEstimated,desvios]=getCenter(points)
if size(points,1)>10
dToMedian=sum((points(:,1:2)-median(points(:,1:2))).^2,2).^0.5;
p70=prctile(dToMedian,70);
idx=dToMedian<p70;
points=points(idx,:);  % Filtra los puntos que estan muy alejados (se queda con el 70% mas cercano)
end

% [~,idx]=max(points(:,3)); % toma el mas alto
% centerEstimated=points(idx,:);

centerEstimated=mean(points);
desvios=std(points(:,1:2)); % std estimation with Bessel correction

% upper confidence interval for poblational sigma estimation
 alpha = 0.05;
 n = length(points(:,1));
 df = n - 1;  % grados de libertad
chi2_lowerQuantile = chi2inv(alpha/2, df);

% Límite superior del IC para la desviación estándar
upper_limit = desvios * sqrt(df / chi2_lowerQuantile);
desvios=upper_limit;
% Método Bootstrap para intervalo de confianza de la covarianza
    numBootstraps = 1000;
    bootCov = zeros(numBootstraps,1);
    for i = 1:numBootstraps
        sampleIdx = randi(n, n, 1);
        sample = points(sampleIdx, 1:2);
         tmp= cov(sample(:,1), sample(:,2));
         bootCov(i)=tmp(1,2);
    end
%     covCI = prctile(bootCov, [2.5 97.5]); % Intervalo de confianza al 95%
    covCI=median(bootCov);
    desvios=[desvios covCI];

end

function [centerEstimated,desvios]=getCenter_old(points)
% m=mean(points);
r=mean(points,1); % coeficientes recta r: r(2)x-r(1)y=0
dr=(r(2)*points(:,1)-r(1)*points(:,2))/(r(1)^2+r(1)^1).^2 ; % distancia punto recta con signo
[~,idxA]=max(dr);
[~,idxB]=min(dr);
A=points(idxA,:); % El punto A tambien son los coeficientes de la recta A
B=points(idxB,:);
na=[A(1),A(2)]; % normal de A
nb=[B(1),B(2)]; 
ca=-sum(A.^2); %ra: na*A'+cA =0
cb=-sum(B.^2); %rb: nb*B'+cB =0
M=[r(2) -r(1);na(1) na(2)];
centerA=pinv(M)*[0;-ca]; % interseccion del normal de "a" con "r".
M=[r(2) -r(1);nb(1) nb(2)];
centerB=pinv(M)*[0;-cb];

centerEstimated=mean([centerA centerB],2)';
%{
[~,idx]=max([centerA(1) centerB(1)]);
if idx==1
    centerEstimated=centerA';
else
    centerEstimated=centerB';
end
%}
desvios=std([centerA(1) centerB(1)]);
%{
plot(points(:,1),points(:,2),'.b');hold on;axis equal
plot(A(1),A(2),'xr')
plot(B(1),B(2),'xg');
plot(centerA(1),centerA(2),'or');
plot(centerB(1),centerB(2),'og');
plot(r(1),r(2),'sr')
plot(centerEstimated(1),centerEstimated(2),'*k');
hold off
legend('puntos','A','B','centro segun A','centro segun B','media','centro estimado')
%}
% %{
[~,idx]=min(sum(points.^2,2));
centerEstimated=points(idx,:);
desvios=std(points(:,1));
%}
end

function jumpDetected=odometryJumpDetector(pose,poses,sigmaX,memoria)
jumpDetected=0;
if isempty(poses)
    return
end
m=movmean(poses(:,1),10); mx=max(abs(poses(:,1)-m));
if abs(poses(end,1)-pose(1))>(1.5*mx+sigmaX*3)*(memoria+1)
% if abs(poses(end,1)-pose(1))>2*mx+sigmaX*3
    jumpDetected=1;
else
    jumpDetected=0;
end
end
function plotEllipse(center, covMatrix, varargin)
    % plotEllipse: Plotea una elipse basada en el centro y la matriz de covarianza.
    %
    % Sintaxis:
    %   plotEllipse(center, covMatrix)
    %   plotEllipse(center, covMatrix, 'PropertyName', PropertyValue, ...)
    %
    % Parámetros:
    %   center     - Vector 1x2 [h, k] que representa el centro de la elipse.
    %   covMatrix  - Matriz 2x2 de covarianza.
    %
    % Opcionales (Name-Value):
    %   'EdgeColor' - Color del borde de la elipse (default: 'k').
    %   'LineWidth' - Ancho de línea (default: 1).
    %   'LineStyle' - Estilo de línea (default: '-').
    %
    % Ejemplo de Uso:
    %   plotEllipse([0,0], [4,1.5; 1.5, 2], 'EdgeColor', 'k', 'LineWidth', 1, 'LineStyle', '-');
    %
    % Autor: [Tu Nombre]
    % Fecha: [Fecha Actual]

    % Verificar que la matriz de covarianza sea 2x2
    if ~isequal(size(covMatrix), [2, 2])
        error('La matriz de covarianza debe ser de tamaño 2x2.');
    end

    % Valores y vectores propios
    [V, D] = eig(covMatrix);

    % Ordenar los valores propios de mayor a menor
    [~, order] = sort(diag(D), 'descend');
    D = D(order, order);
    V = V(:, order);

    % Ángulo de rotación en grados
    angle = atan2(V(2,1), V(1,1)) * (180/pi);

    % Número de sigma para el diámetro (6 sigma implica 3 sigma en cada dirección)
    sigma = 3;

    % Ejes principales de la elipse
    a = sigma * sqrt(D(1,1)); % Semi-eje mayor
    b = sigma * sqrt(D(2,2)); % Semi-eje menor

    % Puntos paramétricos para la elipse
    theta = linspace(0, 2*pi, 100);
    ellipse_x = a * cos(theta);
    ellipse_y = b * sin(theta);

    % Rotación de la elipse
    R = [cosd(angle) -sind(angle); sind(angle) cosd(angle)];
    rotated_ellipse = R * [ellipse_x; ellipse_y];

    % Coordenadas finales
    x = rotated_ellipse(1, :) + center(1);
    y = rotated_ellipse(2, :) + center(2);

    % Parámetros de estilo con valores por defecto
    p = inputParser;
    addParameter(p, 'EdgeColor', 'k', @(x) ischar(x) || isstring(x));
    addParameter(p, 'LineWidth', 1, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'LineStyle', '-', @(x) ischar(x) || isstring(x));
    parse(p, varargin{:});

    % Plotear la elipse
    plot(x, y, 'Color', p.Results.EdgeColor, 'LineWidth', p.Results.LineWidth, 'LineStyle', p.Results.LineStyle);
end

