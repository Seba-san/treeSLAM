% Initial configuration
modelPosition_=[0 0];
position=[0 0];
odo=[];
pose=[0 0];
% poses=[];
poses=[0 0 0 0.06];
allPoints=[];
% Opción para optimizar
% options = optimset('Display', 'iter', 'TolX', 1e-6);
options = optimset('MaxFunEvals',1e4);
R3D_=eye(3); % rotacion total inicial
fi0=0;
fiAcumulado=0;
fi0_=0;
base=[];
sigmaX=[];
tic
% parametrosPlantacion=[1.5 4.0]; % sintetico
% parametrosPlantacion=[1.5 4.0]; % distancia entre arboles, ancho de calle Valerio
% parametrosPlantacion=[1.75 4.0]; % distancia entre arboles, ancho de calle Luciana
parametrosPlantacion=[3.5 7.0]; % distancia entre arboles, ancho de calle Separados
% esquema grande  inicial=200
%vuelta4 inicial=359
% vuelta3 inicial=500
objetive_points=[];
referencias=zeros(6,3);
referencia=0;
referencia_=[];
referenciaUpdated=0;
referencia_Updated=0;
modelPosition=[0 0];
offsetX=[];
sigmaLines_=[];
saltosPasados=0;
% kf1d=KF1D();
% kf1dY=KF1D();kf1dY.alpha=50;
kf2d=KF2D();
Rotar=0; % grande 25
if Rotar
    warning('Cuidado, se estan girando los datos 90° en yaw')
end
ptCloudAligned=[];
allTransformations={};
if ~exist('observedCentersHistoric')
    observedCentersHistoric={};
end
if isempty(observedCentersHistoric)
observedCentersHistoric={};
else
    warning('Se comienza con un observedCentersHistoric previo')
end

angleCorrection=[];
showXpos=10;
hacerVideo=0;

if hacerVideo
v = VideoWriter('reconstruction.avi', 'Motion JPEG AVI');
v.FrameRate = 10; % Cuadros por segundo (ajusta según necesites)
v.Quality = 100;
open(v); % Abre el archivo de video
figure('Position', [100, 100, 1920, 1080])
clf
end
clear fov
disp('Iniciando...')