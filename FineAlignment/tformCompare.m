function [rotDiff, transDiff] = tformCompare(T1,T2)
R1 = T1(1:3, 1:3);
    R2 = T2(1:3, 1:3);
    
    % Convert rotation matrices to quaternions
    q1 = rotm2quat(R1); % MATLAB's function to convert rotation matrix to quaternion
    q2 = rotm2quat(R2);

    % Compute quaternion distance (angular difference in radians)
    rotDiff = acos(abs(dot(q1, q2))) * (180/pi); % Convert to degrees

    % Extract translation vectors (last column, excluding bottom row)
    t1 = T1(1:3, 4);
    t2 = T2(1:3, 4);
    
    % Compute Euclidean distance between translations
    transDiff = norm(t1 - t2);
    
    % Display results
    fprintf('Rotation Difference: %.6f degrees\n', rotDiff);
    fprintf('Translation Difference: %.6f units\n', transDiff);

% Convert the rotation matrix to a quaternion
quat1 =rotm2quat(R1);
quat2=rotm2quat(R2);

% Use quat2axang to get the axis and angle
axangle1= quat2axang(quat1);
axis1=axangle1(1:3);
angle1=axangle1(4);
% Convert the angle to degrees
angleDeg1 = rad2deg(angle1);

% Use quat2axang to get the axis and angle
axangle2= quat2axang(quat2);
axis2=axangle2(1:3);
angle2=axangle2(4);
% Convert the angle to degrees
angleDeg2= rad2deg(angle2);

% Display the axis, angle in degrees, and translation
disp('T1');
disp('Rotation axis:');
disp(axis1);
disp('Rotation angle (in degrees):');
disp(angleDeg1);
disp('Translation vector:');
disp(t1);

disp('');
disp('T2');
disp('Rotation axis:');
disp(axis2);
disp('Rotation angle (in degrees):');
disp(angleDeg2);
disp('Translation vector:');
disp(t2);
disp('------------------------')
end