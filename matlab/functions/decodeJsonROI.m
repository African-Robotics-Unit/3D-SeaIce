function out = decodeJsonROI(values)

    out = zeros(1, numel(values));


    for i = 1:numel(values)
            if strcmp(values(i), "-inf")
                out(i) = -inf;

            elseif strcmp(values(i), "inf")
                out(i) = inf;

            else
                out(i) = str2double(values(i));
            end
    end
end