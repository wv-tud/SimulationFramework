clc;
positions = zeros(length(uArena{1}.agents),3);
headings = zeros(length(uArena{1}.agents),1);
for i=1:length(uArena{1}.agents)
    positions(i,:) = uArena{1}.agents{i}.pos(end,:);
    headings(i)    = uArena{1}.agents{i}.heading(end,1);
end
headMap = reshape(meshgrid(headings,ones(200,1)),200,200,1);
posMap = reshape(meshgrid(positions,ones(200,1)),200,200,3);
d(:,:,1) = (posMap(:,:,1))-shiftdim(squeeze(posMap(:,:,1))',2);
d(:,:,2) = (posMap(:,:,2))-shiftdim(squeeze(posMap(:,:,2))',2);
d(:,:,3) = (posMap(:,:,3))-shiftdim(squeeze(posMap(:,:,3))',2);

dAbs = sqrt(sum(d.^2,3));
x_angles = atan2(squeeze(d(:,:,2)),squeeze(d(:,:,1)));
angles = headMap+uArena{1}.agents{1}.cam_dir(1)-x_angles;

dAbs        = dAbs      + diag(ones(1,200))*uArena{1}.agents{1}.cam_range;
angles      = angles    + diag(ones(1,200))*4*pi();

neighbours = (dAbs<uArena{1}.agents{1}.cam_range & abs(angles)<0.5*uArena{1}.agents{1}.cam_fov);

nb1 = uArena{1}.agents{1}.neighbours(end);
nb1 = nb1{1};
sort(nb1(:,1))'
find(neighbours(1,:)==1)