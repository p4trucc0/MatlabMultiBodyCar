function outp = parse_tyre_params(txt_in)

f1 = fopen(txt_in, 'r');
s = fscanf(f1, '%c');
fclose(f1);
ll = strsplit(s, newline);

outp = struct();

for ii = 1:length(ll)
    if contains(ll{ii}, '=')
        tl = replace(ll{ii}, ' ', '');
        tls = strsplit(tl, '=');
        outp.(tls{1}) = str2double(tls{2});
    end
end

cnames = {'a', 'b', 'c'};
cnum = [0:20];
for i_l = 1:length(cnames)
    for i_n = 1:length(cnum)
        if ~isfield(outp, [cnames{i_l}, num2str(cnum(i_n))])
            outp.([cnames{i_l}, num2str(cnum(i_n))]) = 0.0;
        end
    end
end



% keyboard