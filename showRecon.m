if ~hacerVideo; figure(2); end
clf

plot(pose(1),pose(2),'sk');hold on
plotEllipse([pose(1) pose(2)], kf2d.p(1:2,1:2)^0.5,'EdgeColor', 'r', 'LineWidth', 1, 'LineStyle', '-');
plot(poses(:,1),poses(:,2),'.k')
plot(pp_(:,1)+pose(1),pp_(:,2)+pose(2),'.b');
% plot(kf1d.x(2:end),kf1dY.x(2:end),'or')
num_landmarks = (length(kf2d.x) - 2) / 2;
for jj = 1:num_landmarks   % Plot landmakrs
    idx_x = 2 + 2*jj - 1; % Índice de lix
    idx_y = 2 + 2*jj;     % Índice de liy
    landmark_pos = [kf2d.x(idx_x), kf2d.x(idx_y)];%-[kf2d.x(1) kf2d.x(2)];
    cov_landmark = kf2d.p(idx_x:idx_y, idx_x:idx_y);
    plotEllipse(landmark_pos, cov_landmark, 'EdgeColor', 'g', 'LineWidth', 1, 'LineStyle', '-');
end
plot([0 2*cos(-fiAcumulado+theta)]+pose(1),[0 2*sin(-fiAcumulado+theta)]+pose(2),'-k'); % orientation
hold off
axis equal


xlim([-5 45]);ylim([-6 6])
grid on
if hacerVideo
    frame = getframe(gcf);
    writeVideo(v, frame);
end
pause(0.1)
%%
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