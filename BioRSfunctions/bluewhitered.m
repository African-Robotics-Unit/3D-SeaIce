function cmap = bluewhitered(n)
    if nargin < 1, n = 256; end
    half = floor(n/2);
    b2w  = [linspace(0,1,half)', linspace(0,1,half)', ones(half,1)];
    w2r  = [ones(half,1), linspace(1,0,half)', linspace(1,0,half)'];
    cmap = [b2w; w2r];
end