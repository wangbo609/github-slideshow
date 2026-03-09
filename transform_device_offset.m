function data_transformed = transform_device_offset(data, offset_x, offset_y, offset_z)
% TRANSFORM_DEVICE_OFFSET Transform SLAM trajectory to a different device location
%
% This function transforms a SLAM trajectory from one device to another device
% that has a fixed offset relative to the first device. The transformation
% accounts for the orientation of the reference device at each time step.
%
% Inputs:
%   data      - Nx11 matrix where columns are:
%               [timestamp, x, y, z, rx, ry, rz, rw, grav_x, grav_y, grav_z]
%               x, y, z are in meters (x=right, y=forward, z=up) - reference device
%               rx, ry, rz, rw are quaternion components for reference device
%   offset_x  - Offset in x direction (right) in meters
%   offset_y  - Offset in y direction (forward) in meters
%   offset_z  - Offset in z direction (up) in meters
%
% Output:
%   data_transformed - Nx11 matrix with same format as input, but positions
%                      transformed to the second device location
%
% Example:
%   % Second device is 0.09cm forward, -0.05cm right (0.05cm left), 0.23cm down
%   offset_x = -0.0005;  % -0.05 cm in meters (left)
%   offset_y = 0.0009;   % 0.09 cm in meters (forward)
%   offset_z = -0.0023;  % -0.23 cm in meters (down)
%   data_new = transform_device_offset(data, offset_x, offset_y, offset_z);

    % Initialize output matrix
    data_transformed = data;
    
    % Extract position data (columns 2, 3, 4 are x, y, z)
    x = data(:, 2);  % Right (meters)
    y = data(:, 3);  % Forward (meters)
    z = data(:, 4);  % Up (meters)
    
    % Extract quaternion data (columns 5, 6, 7, 8 are rx, ry, rz, rw)
    qx = data(:, 5);
    qy = data(:, 6);
    qz = data(:, 7);
    qw = data(:, 8);
    
    % Number of data points
    n_points = size(data, 1);
    
    % Define offset vector in device body frame
    offset_body = [offset_x; offset_y; offset_z];
    
    % Transform each point
    for i = 1:n_points
        % Normalize quaternion
        q_norm = sqrt(qx(i)^2 + qy(i)^2 + qz(i)^2 + qw(i)^2);
        qx_n = qx(i) / q_norm;
        qy_n = qy(i) / q_norm;
        qz_n = qz(i) / q_norm;
        qw_n = qw(i) / q_norm;
        
        % Convert quaternion to rotation matrix
        % This rotation matrix rotates from body frame to world frame
        R = quat2rotm_custom(qx_n, qy_n, qz_n, qw_n);
        
        % Rotate offset vector from body frame to world frame
        offset_world = R * offset_body;
        
        % Apply offset to position
        data_transformed(i, 2) = x(i) + offset_world(1);  % x
        data_transformed(i, 3) = y(i) + offset_world(2);  % y
        data_transformed(i, 4) = z(i) + offset_world(3);  % z
        
        % Quaternion remains the same (both devices have same orientation)
        % Gravity also remains the same
    end
end

function R = quat2rotm_custom(qx, qy, qz, qw)
% Convert quaternion to rotation matrix
% Quaternion format: [qx, qy, qz, qw] where qw is the scalar part
% Returns 3x3 rotation matrix

    % Calculate rotation matrix elements
    R = zeros(3, 3);
    
    % First row
    R(1,1) = 1 - 2*(qy^2 + qz^2);
    R(1,2) = 2*(qx*qy - qw*qz);
    R(1,3) = 2*(qx*qz + qw*qy);
    
    % Second row
    R(2,1) = 2*(qx*qy + qw*qz);
    R(2,2) = 1 - 2*(qx^2 + qz^2);
    R(2,3) = 2*(qy*qz - qw*qx);
    
    % Third row
    R(3,1) = 2*(qx*qz - qw*qy);
    R(3,2) = 2*(qy*qz + qw*qx);
    R(3,3) = 1 - 2*(qx^2 + qy^2);
end
