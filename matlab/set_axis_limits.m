function set_axis_limits(XYZ)
% fits a pointcloud in a cube of equal side lengths

mm = minmax(XYZ');
max_range = max(mm(:,2)-mm(:,1));
middle    = (mm(:,2)-mm(:,1))/2+mm(:,1);

xlim([middle(1)-max_range/2 middle(1)+max_range/2]);
ylim([middle(2)-max_range/2 middle(2)+max_range/2]);
zlim([middle(3)-max_range/2 middle(3)+max_range/2]);

end