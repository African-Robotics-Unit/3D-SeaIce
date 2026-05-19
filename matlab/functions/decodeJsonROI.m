function out = decodeJsonROI(values)

    out = zeros(1, numel(values));

    for i = 1:numel(values)

        val = values{i};   

        if strcmp(val, "-inf")
            out(i) = -inf;

        elseif strcmp(val, "inf")
            out(i) = inf;

        else
            out(i) = str2double(val);
        end
  
    end
end