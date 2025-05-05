% % points filter
points=pointclouds{i};
x=points(:,1);
y=points(:,2);
z=points(:,3);
int=points(:,4);
P=[x,y,z,int];
if Rotar
R=[0 -1 0; 1 0 0;0 0 1];P(:,1:3)=P(:,1:3)*R;% Para cuando el lidar esta girado
end
% %{
% elimina los puntos muy cercanos al origen, debido a que hay ocluciones y
% los lejanos
d_to_origen=sum(P(:,1:3).^2,2).^0.5;
if ~exist('FOV')
idx=d_to_origen<1.5 | d_to_origen>20.0 ; %20
else
 idx=d_to_origen<1.5 | d_to_origen>5.0 ; %20
end

if exist('fov')
    idx=d_to_origen<1.5 | d_to_origen>fov ; %20
end

P(idx,:)=[];
P_=diezmado3D(P(:,1:3),0.5);
% P_=P(:,1:3);
% clear P;
% P=P_;
%}
% diezmado
%%
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