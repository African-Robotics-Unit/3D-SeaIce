function tforms = readTformFile(filename)
    fid = fopen(filename, 'r');
    if fid == -1
        error('Could not open file.');
    end

    tforms = affine3d.empty; % Initialize an empty array of rigid3d objects
    %tforms = rigid3d.empty; % Initialize an empty array of rigid3d objects
    currentMatrix = [];

    while ~feof(fid)
        line = strtrim(fgets(fid)); % Read a line
        
        if contains(line, '# point cloud') % New transformation block
            if ~isempty(currentMatrix) % Store previous matrix if available
                %tforms(end+1) = rigid3d(currentMatrix(1:3,1:3), currentMatrix(1:3,4)');
                tforms(end+1) = affine3d(currentMatrix');
            end
            currentMatrix = []; % Reset for the new matrix
        elseif ~isempty(line) % Read transformation values
            values = sscanf(line, '%f %f %f %f');
            if numel(values) == 4
                currentMatrix = [currentMatrix; values'];
            end
        end
    end

    % Store the last transformation matrix if it exists
    if ~isempty(currentMatrix)
        %tforms(end+1) = rigid3d(currentMatrix(1:3,1:3), currentMatrix(1:3,4)');
        tforms(end+1) = affine3d(currentMatrix');
    end

    fclose(fid);
end
