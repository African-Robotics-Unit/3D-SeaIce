function writeALNtransformRigid(alnFilename, tforms)
%WRITEALNTRANSFORMRIGID Writes a cell array of transforms to an ALN file.
%
% Inputs:
%   alnFilename : output ALN filename
%   tforms      : cell array of 4x4 matrices, rigidtform3d, or affinetform3d

    ntforms = numel(tforms);


    names = "scan_" + string(1:ntforms);

    fid = fopen(alnFilename, 'w');

    if fid == -1
        error("Could not open file for writing: %s", alnFilename);
    end

    fprintf(fid, '%d\n', ntforms);

    for i = 1:ntforms

        fprintf(fid, '# %s\n', names(i));

        T = getTransformMatrix(tforms{i});

        for r = 1:4
            fprintf(fid, '%.12f %.12f %.12f %.12f \n', T(r, :));
        end
    end

    fclose(fid);

end


function T = getTransformMatrix(tform)

    if isa(tform, 'rigidtform3d') || isa(tform, 'affinetform3d')
        T = tform.A;
    elseif isnumeric(tform) && isequal(size(tform), [4 4])
        T = tform;
    else
        error("Unsupported transform type. Expected 4x4 matrix, rigidtform3d, or affinetform3d.");
    end

end