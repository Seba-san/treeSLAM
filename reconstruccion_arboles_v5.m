cd '/home/seba/Dropbox/1_Doctorado/3_trabajo_campo/reconstruccion_arboles/matlab'
initial_configuration
sigmas__=NaN*zeros(size(pointclouds,2),6);
for i=1:size(pointclouds,2) %750
% ### Filtrado de puntos, diezmado, etc
points_filter
% ### Ajuste de plano
plane_fit
% sigmaPlane
% continue
if  sigmaPlane>threshold*1.5 
     warning('Frame skipped; high plane uncertainty')
     poses=[poses;poses(end,:)];
        continue
end
% ### Ajuste de lineas
lines_fit
if skipFrameLines
    warning('Frame skipped; high lines uncertainty')
    poses=[poses;poses(end,:)];
    continue
end
% ### Estimacion posicion frontal
% x_estimation
% x_estimation_central_v2
xy_estimation
if saltoOdometricoGrande
    warning('Salto odometrico en x grande')
    poses=[poses;poses(end,:)];
    continue
end
    
% poses=[poses;[pose offset_z (sigmaPlane*sigmaLines*sigmaFitX)^(1/3)]];
% poses=[poses;[pose offset_z (sigmaLines*SigmaX(1,1)^(0.5)*SigmaX(2,2)^(0.5))^(1/3)]];
poses=[poses;[pose offset_z (sigmaPlane*sigmaLines*SigmaX(1,1)^(0.5)*SigmaX(2,2)^(0.5))^(1/4)]];
% poses=[poses;[pose offset_z sigmaLines]];
% poses=[poses;[pose offset_z sigmaFitX]];
% end
sigmaX=[sigmaX sigmaFitX];
% sigmas__=[sigmas__;[sigmaPlane, sigmaLines,SigmaX(1,1),SigmaX(1,2),SigmaX(2,1),SigmaX(2,2)]];
sigmas__(i,:)=[sigmaPlane, sigmaLines,SigmaX(1,1),SigmaX(1,2),SigmaX(2,1),SigmaX(2,2)];
% [desplazamiento desplazamiento2 desplazamiento-desplazamiento2 odo(1)]
%{
pointsRectifiqued=rotatedPoints - [0 0 offset_z];
% fi=atan(bestModel(1)/-bestModel(2))+fi0; % ver si hay que sumarlo o restarlo al fi0
% R=[R_ zeros(2,1);zeros(1,2) 1]*[cos(fi0) sin(fi0) 0;-sin(fi0) cos(fi0) 0; 0 0 1];
R=[cos(fiAcumulado) sin(fiAcumulado) 0;-sin(fiAcumulado) cos(fiAcumulado) 0; 0 0 1]';
pointsRectifiqued=pointsRectifiqued*R;
pointsRectifiqued=pointsRectifiqued-[pose 0];
newPcl=pointCloud(pointsRectifiqued);
%{
% Real pose:
% Matrices de rotación individuales
phi = 0;    % Roll
theta = 0.3;  % Pitch
psi =0;    % Yaw
Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
% Matriz de rotación combinada
Rrpy = Rz * Ry * Rx;
Rr = quat2rotm(sOrientation(js(i),:));
realPoints=sPosition_(js(i),:)+P(:,1:3)*Rrpy'*Rr';
newPcl=pointCloud(realPoints);
%}
gridSize=0.02;
a=0;
% if abs(odo(1))>1e-6 || abs(odo(2))>1e-6 % no integra datos con errores grandes
flg=0;
if ~flg % no integra datos con errores grandes
    if ~isempty(allPoints)
        allPoints= pcmerge(allPoints, newPcl, gridSize);
        %         allPoints= pcmerge(allPoints, newPcl);
        figure(11)
        pcshow(allPoints,'MarkerSize',10);
%         pause()
    else
        allPoints= pcdownsample( newPcl, 'gridAverage', gridSize);
        %         allPoints= newPcl;
    end
else
    
    a=4;
    allPoints=[];
end
%}

% plot(poses(:,1),poses(:,2),'.b');axis equal;hold on
% plot(pose(1),pose(2),'or');hold off
% if mod(i,100)==0
if mod(i,1100)==0
    toc
figure(7)
scatter3(poses(:,1),poses(:,2),poses(:,3),10,poses(:,4),'filled');axis equal;hold on
colormap(jet);colorbar;
p5 = prctile(poses(:,4), 5);
p70 = prctile(poses(:,4), 70);
caxis([p5 p70])
scatter3(pose(1),pose(2),offset_z,'or');hold off
[sigmaPlane sigmaLines sigmaFitX fiAcumulado]
%  figure(11)
%         pcshow(allPoints,'MarkerSize',10);
pause(0.1)
tic
end
% [sigma_plane sigmaLines sigmaFitX fiAcumulado]
%{
if i==200
    disp('termino')
    pause()
end
%}
% ros_config
pcSelection
% showRecon

end
disp('termino')
if hacerVideo; close(v); end
 finishSound()
%  corrcoef(sigmas__)
%%
figure(11)
pcshow(allPoints,'MarkerSize',10);
%%

function [cost,errors]=fitPlaneCost(params,x,y,z)
    a = params(1);
    b = params(2);
    c = params(3);
    d = params(4);
    
    % Calcula el plano z calculado
    z_calculated = (d - a * x - b * y) / c; % 
    
    % Error vertical
    errors = z - z_calculated;
    idx=errors<0;
    cost=sum(errors(~idx))-sum(errors(idx)); % ver como justificar este 1000
end

function [bestModel, inliers] = ransacLine2D(points, maxIterations, threshold)
    % points: Nx2 matrix, where each row is a point [x, y]
    % maxIterations: Maximum number of RANSAC iterations
    % threshold: Distance threshold to consider a point an inlier
    if nargin<2
        maxIterations=1000;
        threshold=1.0;
    end
    % Number of points
    numPoints = size(points, 1);
    
    % Variable to store the best model and inliers
    bestModel = [];
    bestInliersCount = 0;
    bestInliers = [];
    
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
        
        % Step 4: Determine inliers
        inliers = find(distances < threshold);
        inliersCount = length(inliers);
        
        % Step 5: Update the best model if the current one has more inliers
        if inliersCount > bestInliersCount
            bestModel = [a, b, c];
            bestInliersCount = inliersCount;
            bestInliers = inliers;
        end
    end
    
    % Return the best model and inliers
    inliers = bestInliers;
    bestModel = polyfit(points(inliers,1), points(inliers,2), 1);
end

function rotatedPoints=rotationCentroid(rotationMatrix,P)
centroide=mean(P);
P_=P(:,1:3)-centroide(1:3);
% Rotar los puntos
rotatedPoints = (rotationMatrix * P_(:,1:3)')'+centroide(1:3);
end

function [d,new_]=xDisplacement(base,new)

tpc=pointCloud(base);
tpc_=pointCloud(new);
[tform,new_] = pcregistericp(tpc_,tpc);
d=tform.T(4,1);
% a=1;
end
function adjustedPoints=adjustGrid(p,match_points,objetive_points,parametrosPlantacion)
% hace una media movil sobre la estructura y los inliers, para ajustar
% minimizar la varianza local
% match_points=match_points+[p(1) p(2)];
objetive_points=objetive_points+[p(1) p(2)];
w=ones(size(match_points,1),1);
adjustedPoints=zeros(size(objetive_points));
Ninliers=zeros(size(objetive_points,1),1);
for i=1:size(objetive_points,1)
    errors=w.*sum((match_points-objetive_points(i,:)).^2,2);
    idx=errors.^0.5<parametrosPlantacion(1)/2;
    Ninliers(i)=sum(idx);
end
[~,idx_]=sort(Ninliers,'descend');
selected=logical(ones(size(match_points,1),1));
for i=1:size(objetive_points,1)
    w(~selected)=1e6;
    errors=w.*sum((match_points-objetive_points(idx_(i),:)).^2,2);
    idx=errors.^0.5<parametrosPlantacion(1)/2;
     if sum(idx)>1
        % Realiza el promedio de los puntos objetivos con el que fue
        % matcheado
    adjustedPoints(idx_(i),:)=(mean(match_points(idx,:))+objetive_points(idx_(i),:)*9)/10;
    else
        adjustedPoints(idx_(i),:)=objetive_points(idx_(i),:);
     end
    selected=~idx & selected;
end

%{


    if sum(idx)>1
        % Realiza el promedio de los puntos objetivos con el que fue
        % matcheado
    adjustedPoints(i,:)=(mean(match_points(idx,:))+objetive_points(i,:)*9)/10;
    else
        adjustedPoints(i,:)=objetive_points(i,:);
    end
    
end

for i=1:size(adjustedPoints,1)       
        d=adjustedPoints(i,:)-adjustedPoints;
        d=sum(d.^2,2).^0.5;
        idx1=abs(d)<parametrosPlantacion(1);
        idx2=abs(d)>1e-4;
        idx=idx1 & idx2;
        
        if sum(idx)>0
            d=sum(adjustedPoints(i,1)-adjustedPoints(idx,1));
        else
            d=0;
        end
      adjustedPoints(i,1)=adjustedPoints(i,1)-(abs(d)/d)*(abs(d)/2-parametrosPlantacion(1)/2);
end

adjustedPoints=adjustedPoints-[p(1) p(2)];
%}
% adjustedPoints=objetive_points-[p(1) p(2)];
adjustedPoints=adjustedPoints-[p(1) p(2)];
end





function displacement=movimientoX(observedTrees,objetive_points,desplazamiento)
% Matcheo por cercania
distances = pdist2(observedTrees, objetive_points);
% Encuentra los índices de los puntos más cercanos en objective_points para cada punto en observedTrees
[~, closestIndices] = min(distances, [], 2);
distance=observedTrees-objetive_points(closestIndices,:);
outliersIdx=(sum(distance.^2,2).^0.5)>0.5;distance(outliersIdx,:)=[];
displacement=mean(distance(:,1));
end

function [center, radius] = welzl(points)
    % Función principal para calcular el MEC usando el algoritmo de Welzl
    n = size(points, 1);
    % Mezcla los puntos aleatoriamente para asegurar el comportamiento esperado del algoritmo
    points = points(randperm(n), :);
    [center, radius] = findMEC(points, [], n);
end

function [center, radius] = findMEC(P, R, n)
    % Función recursiva para encontrar el círculo mínimo encerrante
    if n == 0 || size(R, 1) == 3
        [center, radius] = mecFromPoints(R);
    else
        % Escoge un punto aleatorio p de P
        p = P(n, :);
        % Resuelve el problema sin p
        [center, radius] = findMEC(P, R, n - 1);
        % Si p está fuera del círculo obtenido, debe incluirse en el conjunto de soporte
        if norm(center - p) > radius
            R = [R; p];  % Añade p al conjunto de soporte
            [center, radius] = findMEC(P, R, n - 1);
        end
    end
end

function [center, radius] = mecFromPoints(points)
    % Calcula el círculo mínimo encerrante dado un conjunto de hasta tres puntos
    switch size(points, 1)
        case 0
            center = [0, 0];
            radius = 0;
        case 1
            center = points(1, :);
            radius = 0;
        case 2
            center = (points(1, :) + points(2, :)) / 2;
            radius = norm(points(1, :) - points(2, :)) / 2;
        case 3
            % Resolver el círculo circunscrito a tres puntos
            [center, radius] = circumscribedCircle(points(1, :), points(2, :), points(3, :));
    end
end

function [center, radius] = circumscribedCircle(a, b, c)
    % Encuentra el círculo circunscrito para tres puntos en el plano
    % Usando fórmulas derivadas de las bisectrices perpendiculares
    D = 2 * (a(1) * (b(2) - c(2)) + b(1) * (c(2) - a(2)) + c(1) * (a(2) - b(2)));
    if D == 0
        center = [0, 0];
        radius = Inf;
    else
        center(1) = ((a(1)^2 + a(2)^2) * (b(2) - c(2)) + (b(1)^2 + b(2)^2) * (c(2) - a(2)) + (c(1)^2 + c(2)^2) * (a(2) - b(2))) / D;
        center(2) = ((a(1)^2 + a(2)^2) * (c(1) - b(1)) + (b(1)^2 + b(2)^2) * (a(1) - c(1)) + (c(1)^2 + c(2)^2) * (b(1) - a(1))) / D;
        radius = norm(center - a);
    end
end
  function finishSound()
fs = 20000; % frecuencia de muestreo
t = 0:1/fs:0.5; % duración: 0.5 segundos
y = sin(2*pi*1e3*t); % tono A4 (440 Hz)

sound(y, fs); % reproduce el sonido
end