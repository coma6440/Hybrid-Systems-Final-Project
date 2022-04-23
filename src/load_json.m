function out = load_json(fname)
fid = fopen(fname, 'r');
raw = fread(fid);
str = char(raw');
fclose(fid);
out = jsondecode(str);
end