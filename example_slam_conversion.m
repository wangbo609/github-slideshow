% Example script to convert SLAM trajectory data to GPS coordinates
% This script demonstrates how to use the slam_to_gps function

clear all;
close all;
clc;

% Sample trajectory data (from the problem statement)
% Columns: timestamp, x, y, z, rx, ry, rz, rw, grav_x, grav_y, grav_z
data = [
    94772.623022, -0.006730, 0.049222, 0.102169, 0.051367, -0.021900, 0.002431, 0.998437, 0.000000, 0.000000, -9.809000;
    94772.722593, -0.004780, 0.051026, 0.096949, 0.052937, -0.021899, 0.001285, 0.998357, 0.000000, 0.000000, -9.809000;
    94772.822155, -0.003129, 0.054775, 0.085936, 0.053824, -0.020118, 0.002105, 0.998346, 0.000000, 0.000000, -9.809000;
    94772.921798, -0.006221, 0.061535, 0.077625, 0.053816, -0.019115, 0.001022, 0.998367, 0.000000, 0.000000, -9.809000;
    94773.021413, -0.009652, 0.065718, 0.078360, 0.052466, -0.019645, 0.000156, 0.998429, 0.000000, 0.000000, -9.809000;
    94773.120982, -0.014781, 0.064571, 0.084900, 0.051073, -0.020180, 0.000866, 0.998491, 0.000000, 0.000000, -9.809000;
    94773.217446, -0.009721, 0.059963, 0.089560, 0.053024, -0.022715, 0.000714, 0.998335, 0.000000, 0.000000, -9.809000;
    94773.316728, -0.005688, 0.054952, 0.093140, 0.054400, -0.024405, 0.000198, 0.998221, 0.000000, 0.000000, -9.809000;
    94773.419849, -0.005140, 0.052405, 0.094344, 0.055693, -0.024150, 0.002016, 0.998154, 0.000000, 0.000000, -9.809000;
    94773.519618, -0.007686, 0.053862, 0.092054, 0.053751, -0.024895, 0.002086, 0.998242, 0.000000, 0.000000, -9.809000;
    94773.619186, -0.009821, 0.053850, 0.089315, 0.051553, -0.025474, 0.001814, 0.998344, 0.000000, 0.000000, -9.809000;
    94773.718699, -0.011344, 0.050989, 0.089005, 0.051891, -0.026252, 0.000834, 0.998307, 0.000000, 0.000000, -9.809000;
    94773.818329, -0.013061, 0.048540, 0.091575, 0.052111, -0.025375, 0.001955, 0.998317, 0.000000, 0.000000, -9.809000;
    94773.917935, -0.013432, 0.046346, 0.095653, 0.051502, -0.025004, 0.002412, 0.998357, 0.000000, 0.000000, -9.809000;
    94774.017541, -0.013591, 0.045749, 0.103460, 0.048691, -0.024539, 0.002047, 0.998510, 0.000000, 0.000000, -9.809000;
    94774.117187, -0.012205, 0.045153, 0.109459, 0.048173, -0.023855, 0.002679, 0.998551, 0.000000, 0.000000, -9.809000;
    94774.216812, -0.008273, 0.042049, 0.107644, 0.048662, -0.025072, 0.002687, 0.998497, 0.000000, 0.000000, -9.809000;
    94774.316433, -0.005659, 0.040570, 0.099923, 0.050287, -0.024747, 0.004497, 0.998418, 0.000000, 0.000000, -9.809000;
    94774.415986, -0.007066, 0.042603, 0.090954, 0.051048, -0.022923, 0.006810, 0.998410, 0.000000, 0.000000, -9.809000;
    94774.515518, -0.009210, 0.044561, 0.087635, 0.050705, -0.021980, 0.007725, 0.998442, 0.000000, 0.000000, -9.809000;
    94774.615112, -0.012630, 0.043701, 0.091411, 0.050961, -0.023377, 0.006929, 0.998403, 0.000000, 0.000000, -9.809000;
    94774.714769, -0.014754, 0.045647, 0.096654, 0.052110, -0.025836, 0.006430, 0.998286, 0.000000, 0.000000, -9.809000;
    94774.814342, -0.011548, 0.046263, 0.099788, 0.052887, -0.027419, 0.008559, 0.998187, 0.000000, 0.000000, -9.809000;
    94774.913912, -0.010624, 0.048531, 0.099579, 0.052864, -0.029415, 0.007990, 0.998136, 0.000000, 0.000000, -9.809000;
    94775.013492, -0.012294, 0.048387, 0.096714, 0.051994, -0.029569, 0.007411, 0.998182, 0.000000, 0.000000, -9.809000;
    94775.113068, -0.011657, 0.048901, 0.094749, 0.050734, -0.027439, 0.008109, 0.998302, 0.000000, 0.000000, -9.809000;
    94775.212648, -0.009812, 0.049940, 0.093447, 0.049704, -0.028287, 0.004529, 0.998353, 0.000000, 0.000000, -9.809000;
    94775.312150, -0.007867, 0.054093, 0.093737, 0.048979, -0.027158, 0.002904, 0.998426, 0.000000, 0.000000, -9.809000;
    94775.411656, -0.006404, 0.057720, 0.096240, 0.048434, -0.024508, 0.004474, 0.998516, 0.000000, 0.000000, -9.809000;
    94775.511284, -0.007536, 0.058430, 0.102487, 0.048252, -0.024139, 0.003721, 0.998537, 0.000000, 0.000000, -9.809000;
    94775.610917, -0.007845, 0.059047, 0.108097, 0.047047, -0.024099, 0.003358, 0.998596, 0.000000, 0.000000, -9.809000;
    94775.710540, -0.005935, 0.061283, 0.104165, 0.044373, -0.023567, 0.004650, 0.998726, 0.000000, 0.000000, -9.809000;
    94775.810159, -0.000086, 0.062461, 0.096994, 0.044082, -0.022101, 0.005446, 0.998769, 0.000000, 0.000000, -9.809000;
    94775.909791, 0.002206, 0.067612, 0.089085, 0.046891, -0.021837, 0.001443, 0.998660, 0.000000, 0.000000, -9.809000
];

% Set your starting GPS coordinates here (example values - replace with actual)
start_lat = 39.9042;    % Example: Beijing latitude
start_lon = 116.4074;   % Example: Beijing longitude  
start_alt = 50.0;       % Example: 50 meters altitude

% Convert SLAM trajectory to GPS coordinates
[lat, lon, altitude, roll, pitch, yaw] = slam_to_gps(data, start_lat, start_lon, start_alt);

% Display results
fprintf('SLAM Trajectory to GPS Conversion Results\n');
fprintf('==========================================\n\n');
fprintf('Starting Position:\n');
fprintf('  Latitude:  %.6f degrees\n', start_lat);
fprintf('  Longitude: %.6f degrees\n', start_lon);
fprintf('  Altitude:  %.2f meters\n\n', start_alt);

fprintf('Number of trajectory points: %d\n\n', length(lat));

% Display first few points
fprintf('First 5 trajectory points:\n');
fprintf('%-5s %-12s %-12s %-10s %-8s %-8s %-8s\n', ...
        'Index', 'Latitude', 'Longitude', 'Altitude', 'Roll', 'Pitch', 'Yaw');
fprintf('%-5s %-12s %-12s %-10s %-8s %-8s %-8s\n', ...
        '', '(degrees)', '(degrees)', '(meters)', '(deg)', '(deg)', '(deg)');
fprintf('---------------------------------------------------------------------\n');
for i = 1:min(5, length(lat))
    fprintf('%-5d %-12.6f %-12.6f %-10.2f %-8.2f %-8.2f %-8.2f\n', ...
            i, lat(i), lon(i), altitude(i), roll(i), pitch(i), yaw(i));
end

fprintf('\n');

% Create output matrix with all results
% Columns: timestamp, latitude, longitude, altitude, roll, pitch, yaw
output = [data(:,1), lat, lon, altitude, roll, pitch, yaw];

% Save to file
output_filename = 'gps_trajectory_output.txt';
fprintf('Saving results to %s...\n', output_filename);

% Write header and data to file
fid = fopen(output_filename, 'w');
fprintf(fid, '%% SLAM Trajectory Converted to GPS Coordinates\n');
fprintf(fid, '%% Starting position: Lat=%.6f, Lon=%.6f, Alt=%.2f\n', ...
        start_lat, start_lon, start_alt);
fprintf(fid, '%% Columns: timestamp, latitude(deg), longitude(deg), altitude(m), roll(deg), pitch(deg), yaw(deg)\n');
fprintf(fid, '%%timestamp         latitude      longitude     altitude      roll          pitch         yaw\n');
for i = 1:size(output, 1)
    fprintf(fid, '%-15.6f  %-13.8f %-13.8f %-13.4f %-13.6f %-13.6f %-13.6f\n', ...
            output(i,1), output(i,2), output(i,3), output(i,4), ...
            output(i,5), output(i,6), output(i,7));
end
fclose(fid);

fprintf('Done! Results saved to %s\n', output_filename);

% Plot trajectory
figure('Name', 'SLAM Trajectory Visualization');

% Plot 1: 3D position in local coordinates
subplot(2,2,1);
plot3(data(:,2), data(:,3), data(:,4), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('X - East (m)');
ylabel('Y - North (m)');
zlabel('Z - Up (m)');
title('Local Coordinates (ENU)');
axis equal;

% Plot 2: GPS coordinates
subplot(2,2,2);
plot(lon, lat, 'r-', 'LineWidth', 1.5);
hold on;
plot(lon(1), lat(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(lon(end), lat(end), 'rs', 'MarkerSize', 10, 'LineWidth', 2);
grid on;
xlabel('Longitude (degrees)');
ylabel('Latitude (degrees)');
title('GPS Trajectory');
legend('Trajectory', 'Start', 'End', 'Location', 'best');

% Plot 3: Altitude profile
subplot(2,2,3);
plot(1:length(altitude), altitude, 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Point Index');
ylabel('Altitude (m)');
title('Altitude Profile');

% Plot 4: Orientation (Euler angles)
subplot(2,2,4);
plot(1:length(roll), roll, 'r-', 'LineWidth', 1.5);
hold on;
plot(1:length(pitch), pitch, 'g-', 'LineWidth', 1.5);
plot(1:length(yaw), yaw, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Point Index');
ylabel('Angle (degrees)');
title('Orientation (Euler Angles)');
legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');

fprintf('\nVisualization complete!\n');
