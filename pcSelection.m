point=[0 0 -best_plane(4)/best_plane(3)];
pointRotated=point*rotationMatrix;
offset_z_=pointRotated(3);


R3D=[R [0 0]';[0 0 1]];
R3D=rotationMatrix'*R3D;
allTransformations{i}.R=R3D;
% allTransformations{i}.T=[kf1d.x(1) -mean(modas) offset_z_];
allTransformations{i}.T=[kf2d.x(1) kf2d.x(2) offset_z_];


%%
% offset_z
% mean(modas) % centro de la fila
% kf1d.x(1) % x posicion
%{
R3D=[R [0 0]';[0 0 1]];
R3D=rotationMatrix'*R3D;

pointsRotated=P(:,1:3)*R3D+[kf1d.x(1) -mean(modas) offset_z];

pointsRotatedPC=pointCloud(pointsRotated);
if isempty(ptCloudAligned)
    ptCloudAligned=pointsRotatedPC;
else
mergedPointsAligned = [pointsRotatedPC.Location; ptCloudAligned.Location];
ptCloudAligned=pointCloud(mergedPointsAligned);
end
gridStep = 0.05;
ptCloudAligned = pcdownsample(ptCloudAligned, 'gridAverage', gridStep);
zLimits = [-0.5, 3];
indices = (ptCloudAligned.Location(:, 3) >= zLimits(1) & ptCloudAligned.Location(:, 3) <= zLimits(2));
ptCloudAligned = ptCloudAligned.Location(indices, :);
ptCloudAligned=pointCloud(ptCloudAligned);
figure(12)
clf
ax=axes;
pcshow(ptCloudAligned,'MarkerSize',20)
ax.XAxis.Limits=[-5 20];
ax.YAxis.Limits=[-10 10];
ax.ZAxis.Limits=[-0.5 3];
% axis equal

a=1;
%}