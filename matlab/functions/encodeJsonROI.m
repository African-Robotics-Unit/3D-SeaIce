function out = encodeJsonROI(values)
    out = cell(size(values));

    for i = 1:numel(values)
        if isinf(values(i)) && values(i) < 0
            out{i} = "-inf";
        elseif isinf(values(i)) && values(i) > 0
            out{i} = "inf";
        else
            out{i} = values(i);
        end
    end
end