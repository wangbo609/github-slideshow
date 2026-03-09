function [lat, lon, altitude, roll, pitch, yaw] = slam_to_gps(data, start_lat, start_lon, start_alt)
% SLAM_TO_GPS Convert SLAM trajectory from local coordinates to GPS coordinates
%
% Inputs:
%   data       - Nx10 matrix where columns are:
%                [timestamp, x, y, z, rx, ry, rz, rw, grav_x, grav_y, grav_z]
%                x, y, z are in meters (x=right, y=forward, z=up)
%                rx, ry, rz, rw are quaternion components
%   start_lat  - Starting latitude in degrees
%   start_lon  - Starting longitude in degrees  
%   start_alt  - Starting altitude in meters (optional, default = 0)
%
% Outputs:
%   lat        - Latitude array in degrees
%   lon        - Longitude array in degrees
%   altitude   - Altitude array in meters
%   roll       - Roll angle array in degrees
%   pitch      - Pitch angle array in degrees
%   yaw        - Yaw angle array in degrees
%
% Note: This assumes a local ENU (East-North-Up) coordinate system where:
%   - SLAM x (right) corresponds to East
%   - SLAM y (forward) corresponds to North
%   - SLAM z (up) corresponds to Up

    % Default altitude if not provided
    if nargin < 4
        start_alt = 0;
    end
    
    % Extract position data (columns 2, 3, 4 are x, y, z)
    x = data(:, 2);  % East (meters)
    y = data(:, 3);  % North (meters)
    z = data(:, 4);  % Up (meters)
    
    % Extract quaternion data (columns 5, 6, 7, 8 are rx, ry, rz, rw)
    qx = data(:, 5);
    qy = data(:, 6);
    qz = data(:, 7);
    qw = data(:, 8);
    
    % Earth radius in meters
    R_earth = 6378137.0;
    
    % Convert starting position to radians
    lat0_rad = deg2rad(start_lat);
    lon0_rad = deg2rad(start_lon);
    
    % Calculate latitude and longitude for each point
    % Using small angle approximation for local coordinates
    n_points = size(data, 1);
    lat = zeros(n_points, 1);
    lon = zeros(n_points, 1);
    altitude = zeros(n_points, 1);
    
    for i = 1:n_points
        % Convert ENU offsets to lat/lon
        % North offset (y) affects latitude
        % East offset (x) affects longitude
        dlat = y(i) / R_earth;
        dlon = x(i) / (R_earth * cos(lat0_rad));
        
        lat(i) = rad2deg(lat0_rad + dlat);
        lon(i) = rad2deg(lon0_rad + dlon);
        altitude(i) = start_alt + z(i);
    end
    
    % Convert quaternions to Euler angles (roll, pitch, yaw)
    roll = zeros(n_points, 1);
    pitch = zeros(n_points, 1);
    yaw = zeros(n_points, 1);
    
    for i = 1:n_points
        % Normalize quaternion
        q_norm = sqrt(qx(i)^2 + qy(i)^2 + qz(i)^2 + qw(i)^2);
        qx_n = qx(i) / q_norm;
        qy_n = qy(i) / q_norm;
        qz_n = qz(i) / q_norm;
        qw_n = qw(i) / q_norm;
        
        % Convert to Euler angles (ZYX convention)
        % Roll (x-axis rotation)
        sinr_cosp = 2 * (qw_n * qx_n + qy_n * qz_n);
        cosr_cosp = 1 - 2 * (qx_n * qx_n + qy_n * qy_n);
        roll(i) = atan2(sinr_cosp, cosr_cosp);
        
        % Pitch (y-axis rotation)
        sinp = 2 * (qw_n * qy_n - qz_n * qx_n);
        if abs(sinp) >= 1
            pitch(i) = sign(sinp) * pi/2; % Use 90 degrees if out of range
        else
            pitch(i) = asin(sinp);
        end
        
        % Yaw (z-axis rotation)
        siny_cosp = 2 * (qw_n * qz_n + qx_n * qy_n);
        cosy_cosp = 1 - 2 * (qy_n * qy_n + qz_n * qz_n);
        yaw(i) = atan2(siny_cosp, cosy_cosp);
    end
    
    % Convert to degrees
    roll = rad2deg(roll);
    pitch = rad2deg(pitch);
    yaw = rad2deg(yaw);
end
