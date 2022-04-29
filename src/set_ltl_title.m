function set_ltl_title(safety, cosafety)
if ~isempty(safety)
    title("(" + safety  +")" + " & (" + cosafety +")")
else
    title(cosafety)
end
end