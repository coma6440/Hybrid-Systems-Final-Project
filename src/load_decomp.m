function decomp = load_decomp(fname)
fid = fopen(fname);
c = textscan(fid, "%f");
c = c{1};
n = sum(c == -1);
x = zeros(3*n,1);
y = zeros(3*n,1);
DT = zeros(n,3);
idx = 0;
for i = 1:n
    idx = idx + 2;
    x1 = c(idx);
    y1 = c(idx +1);
    x2 = c(idx + 2);
    y2 = c(idx + 3);
    x3 = c(idx + 4);
    y3 = c(idx + 5);

    j = (3*(i-1) + 1):(3*(i-1) + 3);
    x(j) = [x1; x2; x3];
    y(j) = [y1; y2; y3];
    DT(i, :) = j;
    while(c(idx)~=-1)
        idx = idx + 1;
    end
end
decomp.x = x;
decomp.y = y;
decomp.DT = DT;
end