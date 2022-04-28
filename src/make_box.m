function out = make_box(x,y,l,w)
out = ...
    [x - l/2, y - w/2;
     x + l/2, y - w/2;
     x + l/2, y + w/2;
     x - l/2, y + w/2];
end