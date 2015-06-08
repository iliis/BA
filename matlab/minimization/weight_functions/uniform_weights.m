function [ weights ] = uniform_weights( errs )
% identity loss function, doesn't weight errors at all

weights = ones(size(errs));

end

