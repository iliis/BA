function [ weights ] = huber_loss( errors, delta )

% Huber loss function

quad_idx = abs(errors) < delta;

weights           = (abs(errors) - delta/2)*delta;
weights(quad_idx) = 0.5*errors(quad_idx).^2;

end

