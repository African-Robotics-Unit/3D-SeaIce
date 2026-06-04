function [rigid_tform] = makeRigid(matrix_tform)


% Extract the upper-left 3x3 sub-matrix (linear part)
linear_part = matrix_tform(1:3, 1:3);

% Apply SVD to the linear part
[U, S, V] = svd(linear_part);

% Reconstruct the rotation matrix
rotation_matrix = U * V';

% Ensure the determinant is 1 (proper rotation)
if det(rotation_matrix) < 0
    U(:, end) = -U(:, end); % Adjust the last column of U if needed
    rotation_matrix = U * V'; % Reconstruct the rotation matrix
end

% Construct the new transformation matrix with the rotation and original translation
rigid_tform = matrix_tform; % Copy the original matrix
rigid_tform(1:3, 1:3) = rotation_matrix; % Replace the linear part with the rotation matrix

end