function intensity = camera_image( I, x )
% I(x = [u, v])

u = x(1);
v = x(2);

% nearest neighbour (int32 already rounds correctly)
intensity = I(int32(v), int32(u));

% bilinear filtering
% TODO: this is not usable for projection (how to filter depth buffer?)
% intensity = interp2(I,v,u);

end