% Esta funcion selecciona las posiciones con baja incertidumbre y
% reconstruye en funcion de esto.
percentile=100;
gridStep = 0.1;%0.05;%1.0;%0.01;
interval=[0 35];
T=[0.1 104]*10;
% pSetted=0.08; % 0.0621
pSetted=0.4;%0.4 % max %0.12


uncertainties=poses(2:end,4);
poses_=poses(2:end,:);


iMaster=1:1:size(allTransformations,2);

fprintf('Percentil seleccionado %.3f \n',percentile)
iFull=[];
%  for i=1:20

idx=poses_(T(1):T(2),1)>interval(1) &poses_(T(1):T(2),1)<interval(2);

%     tranSelected=allTransformations(idx);
iSelected=iMaster(idx);
p30 = prctile(uncertainties(idx), percentile);
p30
idxUncer=uncertainties(idx)<p30;
%     idxUncer=uncertainties(idx)<pSetted;
%     idxUncer=uncertainties(idx)>p30;
%     tranSelected=tranSelected(idxUncer);
iSelected=iSelected(idxUncer);
iFull=[iFull iSelected];
%  end

figure(3)
plot(iMaster,uncertainties,'-b');hold on
plot(iMaster(idx),uncertainties(idx),'-g');
uncer=uncertainties(idx);
plot(iSelected,uncer(idxUncer),'.r');hold off
legend('Todo','Intervalo seleccionado','puntos elegidos')
ylabel('$\tilde{\sigma}$', 'Interpreter', 'latex','FontSize', 23, 'FontWeight', 'bold')
xlabel('$N^o$ de muestra', 'Interpreter', 'latex','FontSize', 23, 'FontWeight', 'bold')
figure(4)
plot(iMaster,poses_(:,1),'.b');hold on
plot(iMaster(idx),poses_(idx,1),'.g')
% poses_=poses(idx,1);
plot(iMaster(iFull),poses_(iFull,1),'.r');
plot(iMaster,uncertainties,'.k');hold off
legend('Todo','Intervalo seleccionado','puntos elegidos','incertidumbre')
grid on
pause(0.1)
% initial_configuration
ptCloudAligned=[];
disp('Iniciando reconstruccion...')
% FOV=1; % Es una bandera que filtra los puntos mas lejos de 5m
fov=5;
tic
for k=1:size(iFull,2)
    i=iFull(k);
    points_filter
    if ~isempty(allTransformations{i})
        pointsRotated=P(:,1:3)*allTransformations{i}.R+allTransformations{i}.T;
        PD=2*normcdf(0.1/(2*uncertainties(i)))-1;% Probabilidad de dispersion, asumiendo gaussiana de una dimension. Ec (6)
        li=log(PD/(1-PD)); % log odds
        pointsRotatedPC=pointCloud(pointsRotated);
        if isempty(ptCloudAligned)
            ptCloudAligned=pointsRotatedPC;
        else
            mergedPointsAligned = [pointsRotatedPC.Location; ptCloudAligned.Location];
            ptCloudAligned=pointCloud(mergedPointsAligned);
        end
        
        ptCloudAligned = pcdownsample(ptCloudAligned, 'gridAverage', gridStep);
        zLimits = [-0.5, 3];
        indices = (ptCloudAligned.Location(:, 3) >= zLimits(1) & ptCloudAligned.Location(:, 3) <= zLimits(2));
        ptCloudAligned = ptCloudAligned.Location(indices, :);
        ptCloudAligned=pointCloud(ptCloudAligned);
        %{
    figure(12)
    clf
    ax=axes;
    pcshow(ptCloudAligned,'MarkerSize',20)
    ax.XAxis.Limits=[interval(1)-5 interval(2)+10];
    ax.YAxis.Limits=[-10 10];
    ax.ZAxis.Limits=[-0.5 3];
%     i
    pause(0.1)
    
    % axis equal
        %}
    end
    if mod(k,50)==0
        tt=toc();
        disp([num2str(i*100/size(iFull,2),'%.2f'), ' % ',num2str(-tt+tt/(i/size(iFull,2)),'%.2f'),' s restantes, ',num2str(tt,'%.2f') ,' s transcurrido'])
    end
    
end
toc
figure(12)
clf
ax=axes;
pcshow(ptCloudAligned,'MarkerSize',20)
ax.XAxis.Limits=[interval(1)-5 interval(2)+10];
ax.YAxis.Limits=[-10 10];
ax.ZAxis.Limits=[-0.5 3];
disp('fin')
clear FOV
finishSound()
%%
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
% view(-85, 2)

view(-80, 12)
% view(-90,90)
% view(-40.8, 88.01)

colorbar
ylabel(colorbar, 'Altura (Z)');

    ax.XAxis.Limits=[-5 35];
    ax.YAxis.Limits=[-6 6];
    ax.ZAxis.Limits=[-0.5 3];

%% Por tiempo e incertidumbre


%% ESTATICO
N=size(pointclouds,2);
ptCloudAligned=[];
disp('Iniciando...')
for i=1:N
    points=pointclouds{i};
    x=points(:,1);
    y=points(:,2);
    z=points(:,3);
    int=points(:,4);
    P=[x,y,z,int];
    d_to_origen=sum(P(:,1:3).^2,2).^0.5;
    idx=d_to_origen<1.5 | d_to_origen>20.0 ; %20
    P(idx,:)=[];P=P(:,1:3);
    
    pointsRotatedPC=pointCloud(P);
    if isempty(ptCloudAligned)
        ptCloudAligned=pointsRotatedPC;
    else
        mergedPointsAligned = [pointsRotatedPC.Location; ptCloudAligned.Location];
        ptCloudAligned=pointCloud(mergedPointsAligned);
    end
    gridStep = 0.01;
    ptCloudAligned = pcdownsample(ptCloudAligned, 'gridAverage', gridStep);
   
end
 figure
    clf
    ax=axes;
    pcshow(ptCloudAligned,'MarkerSize',6)
    %     ax.XAxis.Limits=[interval(1)-5 interval(2)+10];
    %     ax.YAxis.Limits=[-10 10];
    %     ax.ZAxis.Limits=[-0.5 3];
    pause(0.01)
  disp('fin')
  colorbar;
caxis([zMin zMax]); % Ajusta los límites de la barra de color
ylabel(colorbar, 'Altura (Z)');
  %% FASTLIO
  % Especifica la ruta al archivo .pcd
rutaArchivo = '/tmp/scans.pcd'; % Reemplaza con la ruta real de tu archivo
kk=3;
zMin_=-0.5;
zMax_=3;
% Lee el archivo .pcd
nubePuntos = pcread(rutaArchivo);
rotationMatrix = single([
     0.8711,  -0.0299,  0.4902;
    -0.0299,   0.9931,  0.1136;
    -0.4902,  -0.1136,  0.8642
]);
R=[0.9467   -0.3221 0;0.3221    0.9467 0;0 0 1];
T= [0.0274    0.0223    0.7011];
% pointsRotated=nubePuntos.Location*allTransformations{kk}.R+allTransformations{kk}.T;
% pointsRotated=nubePuntos.Location*rotationMatrix+allTransformations{kk}.T;
pointsRotated=nubePuntos.Location*rotationMatrix'*R+T;

    
    pointsRotatedPC=pointCloud(pointsRotated);

% Muestra información básica de la nube de puntos
% disp(pointsRotatedPC);

 gridStep = 0.01;
  pointsRotatedPC = pcdownsample(pointsRotatedPC, 'gridAverage', gridStep);

% Extrae las coordenadas Z
z = pointsRotatedPC.Location(:,3);
% Normaliza los valores Z
zMin = max(min(z),zMin_);
zMax = min(max(z),zMax_);
idx=pointsRotatedPC.Location(:,3)<zMax;
z=pointsRotatedPC.Location(idx,3);
zNorm = (z - zMin) / (zMax - zMin);
zNorm=min(zNorm,1);
zNorm=max(zNorm,0);
% Selecciona un colormap
cmap = jet(1024); % Puedes cambiar 'jet' por otro colormap si lo deseas

% Mapea los valores normalizados a índices de color
colorIndices = round(zNorm * (size(cmap, 1) - 1)) + 1;

% Asigna los colores correspondientes
colorsAsignados = cmap(colorIndices, :);


figure(2);
clf
 ax=axes;
pcshow(pointsRotatedPC.Location(idx,:),colorsAsignados,'MarkerSize',6)
% pcshow(pointsRotatedPC,'MarkerSize',20)

%colormap(jet);
colormap(parula);
colorbar;
caxis([zMin zMax]); % Ajusta los límites de la barra de color
ylabel(colorbar, 'Altura (Z)');

        ax.XAxis.Limits=[interval(1)-5 35];
        ax.YAxis.Limits=[-6 8];
        ax.ZAxis.Limits=[-1 3];


% grid on;
%%
% Utilizando el ajuste del plano para FASTLIO
 pointsRotatedPC=pointCloud(rotatedPoints);
 
% Extrae las coordenadas Z
z = pointsRotatedPC.Location(:,3);
% Normaliza los valores Z
zMin = max(min(z),zMin_);
zMax = min(max(z),zMax_);
idx=pointsRotatedPC.Location(:,3)<zMax;
z=pointsRotatedPC.Location(idx,3);
zNorm = (z - zMin) / (zMax - zMin);
zNorm=min(zNorm,1);
zNorm=max(zNorm,0);
% Selecciona un colormap
cmap = parula(1024); % Puedes cambiar 'jet' por otro colormap si lo deseas

% Mapea los valores normalizados a índices de color
colorIndices = round(zNorm * (size(cmap, 1) - 1)) + 1;

% Asigna los colores correspondientes
colorsAsignados = cmap(colorIndices, :);


figure(2);
clf
 ax=axes;
pcshow(pointsRotatedPC.Location(idx,:),colorsAsignados,'MarkerSize',6)
% pcshow(pointsRotatedPC,'MarkerSize',20)

%colormap(jet);
colormap(parula);
colorbar;
caxis([zMin zMax]); % Ajusta los límites de la barra de color
ylabel(colorbar, 'Altura (Z)');

        ax.XAxis.Limits=[interval(1)-5 35];
        ax.YAxis.Limits=[-6 8];
        ax.ZAxis.Limits=[-2 3];

%%
  function finishSound()
fs = 20000; % frecuencia de muestreo
t = 0:1/fs:0.5; % duración: 0.5 segundos
y = sin(2*pi*1e3*t); % tono A4 (440 Hz)

sound(y, fs); % reproduce el sonido
end