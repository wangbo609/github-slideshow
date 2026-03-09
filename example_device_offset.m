% Example script for transforming SLAM trajectory to a different device location
% This demonstrates device offset transformation with the provided specifications

clear all;
close all;
clc;

fprintf('Device Offset Transformation Example\n');
fprintf('=====================================\n\n');

% Sample trajectory data (from the problem statement)
% Columns: timestamp, x, y, z, rx, ry, rz, rw, grav_x, grav_y, grav_z
data_device1 = [
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

% Device 2 offset relative to Device 1 (in Device 1's body frame)
% Device 2 is located at:
%   - Forward (y): 0.09 cm = 0.0009 m
%   - Right (x): -0.05 cm = -0.0005 m (negative = left)
%   - Down (z): 0.23 cm = -0.0023 m (negative = down)
fprintf('Device 2 Offset relative to Device 1:\n');
offset_x = -0.0005;  % meters (left)
offset_y = 0.0009;   % meters (forward)
offset_z = -0.0023;  % meters (down)

fprintf('  X (right):   %+.4f cm (%+.6f m)\n', offset_x*100, offset_x);
fprintf('  Y (forward): %+.4f cm (%+.6f m)\n', offset_y*100, offset_y);
fprintf('  Z (up):      %+.4f cm (%+.6f m)\n\n', offset_z*100, offset_z);

% Transform trajectory to Device 2 location
fprintf('Transforming trajectory to Device 2 location...\n');
data_device2 = transform_device_offset(data_device1, offset_x, offset_y, offset_z);
fprintf('Transformation complete!\n\n');

% Display comparison of first few points
fprintf('Comparison of Device 1 and Device 2 trajectories (first 5 points):\n');
fprintf('=====================================================================\n\n');

fprintf('Device 1 Position (reference):\n');
fprintf('%-5s %-12s %-12s %-12s\n', 'Index', 'X (m)', 'Y (m)', 'Z (m)');
fprintf('-----------------------------------------------------\n');
for i = 1:min(5, size(data_device1, 1))
    fprintf('%-5d %+12.6f %+12.6f %+12.6f\n', ...
            i, data_device1(i,2), data_device1(i,3), data_device1(i,4));
end

fprintf('\nDevice 2 Position (transformed):\n');
fprintf('%-5s %-12s %-12s %-12s\n', 'Index', 'X (m)', 'Y (m)', 'Z (m)');
fprintf('-----------------------------------------------------\n');
for i = 1:min(5, size(data_device2, 1))
    fprintf('%-5d %+12.6f %+12.6f %+12.6f\n', ...
            i, data_device2(i,2), data_device2(i,3), data_device2(i,4));
end

fprintf('\nPosition Difference (Device 2 - Device 1):\n');
fprintf('%-5s %-12s %-12s %-12s\n', 'Index', 'ΔX (m)', 'ΔY (m)', 'ΔZ (m)');
fprintf('-----------------------------------------------------\n');
for i = 1:min(5, size(data_device1, 1))
    dx = data_device2(i,2) - data_device1(i,2);
    dy = data_device2(i,3) - data_device1(i,3);
    dz = data_device2(i,4) - data_device1(i,4);
    fprintf('%-5d %+12.6f %+12.6f %+12.6f\n', i, dx, dy, dz);
end

% Save Device 2 trajectory to file
output_filename = 'device2_trajectory.txt';
fprintf('\nSaving Device 2 trajectory to %s...\n', output_filename);

fid = fopen(output_filename, 'w');
fprintf(fid, '%% Device 2 Trajectory (Transformed from Device 1)\n');
fprintf(fid, '%% Offset: x=%+.6fm, y=%+.6fm, z=%+.6fm\n', offset_x, offset_y, offset_z);
fprintf(fid, '%% Columns: timestamp, x, y, z, rx, ry, rz, rw, grav_x, grav_y, grav_z\n');
fprintf(fid, '%%timestamp         x             y             z             rx            ry            rz            rw            grav_x        grav_y        grav_z\n');
for i = 1:size(data_device2, 1)
    fprintf(fid, '%-15.6f  %+13.6f %+13.6f %+13.6f %+13.6f %+13.6f %+13.6f %+13.6f %+13.6f %+13.6f %+13.6f\n', ...
            data_device2(i,1), data_device2(i,2), data_device2(i,3), data_device2(i,4), ...
            data_device2(i,5), data_device2(i,6), data_device2(i,7), data_device2(i,8), ...
            data_device2(i,9), data_device2(i,10), data_device2(i,11));
end
fclose(fid);
fprintf('Done! Device 2 trajectory saved.\n\n');

% Optional: Convert both trajectories to GPS coordinates
use_gps_conversion = true;  % Set to true to also generate GPS coordinates

if use_gps_conversion
    fprintf('Converting trajectories to GPS coordinates...\n');
    
    % Set starting GPS coordinates (example values - replace with actual)
    start_lat = 39.9042;    % Example: Beijing latitude
    start_lon = 116.4074;   % Example: Beijing longitude
    start_alt = 50.0;       % Example: 50 meters altitude
    
    % Convert Device 1 trajectory
    [lat1, lon1, alt1, roll1, pitch1, yaw1] = slam_to_gps(data_device1, start_lat, start_lon, start_alt);
    
    % Convert Device 2 trajectory
    [lat2, lon2, alt2, roll2, pitch2, yaw2] = slam_to_gps(data_device2, start_lat, start_lon, start_alt);
    
    % Save GPS trajectories
    gps_output1 = 'device1_gps_trajectory.txt';
    gps_output2 = 'device2_gps_trajectory.txt';
    
    % Save Device 1 GPS
    fid = fopen(gps_output1, 'w');
    fprintf(fid, '%% Device 1 GPS Trajectory\n');
    fprintf(fid, '%% Columns: timestamp, latitude(deg), longitude(deg), altitude(m), roll(deg), pitch(deg), yaw(deg)\n');
    for i = 1:length(lat1)
        fprintf(fid, '%-15.6f  %-13.8f %-13.8f %-13.4f %-13.6f %-13.6f %-13.6f\n', ...
                data_device1(i,1), lat1(i), lon1(i), alt1(i), roll1(i), pitch1(i), yaw1(i));
    end
    fclose(fid);
    
    % Save Device 2 GPS
    fid = fopen(gps_output2, 'w');
    fprintf(fid, '%% Device 2 GPS Trajectory\n');
    fprintf(fid, '%% Columns: timestamp, latitude(deg), longitude(deg), altitude(m), roll(deg), pitch(deg), yaw(deg)\n');
    for i = 1:length(lat2)
        fprintf(fid, '%-15.6f  %-13.8f %-13.8f %-13.4f %-13.6f %-13.6f %-13.6f\n', ...
                data_device2(i,1), lat2(i), lon2(i), alt2(i), roll2(i), pitch2(i), yaw2(i));
    end
    fclose(fid);
    
    fprintf('GPS trajectories saved to:\n');
    fprintf('  - %s\n', gps_output1);
    fprintf('  - %s\n\n', gps_output2);
end

% Visualization
figure('Name', 'Device Offset Transformation', 'Position', [100 100 1200 800]);

% Plot 1: 3D trajectories comparison
subplot(2,2,1);
plot3(data_device1(:,2), data_device1(:,3), data_device1(:,4), 'b-', 'LineWidth', 2);
hold on;
plot3(data_device2(:,2), data_device2(:,3), data_device2(:,4), 'r-', 'LineWidth', 2);
plot3(data_device1(1,2), data_device1(1,3), data_device1(1,4), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
plot3(data_device2(1,2), data_device2(1,3), data_device2(1,4), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
grid on;
xlabel('X - Right (m)');
ylabel('Y - Forward (m)');
zlabel('Z - Up (m)');
title('3D Trajectories Comparison');
legend('Device 1', 'Device 2', 'Device 1 Start', 'Device 2 Start', 'Location', 'best');
axis equal;
view(45, 30);

% Plot 2: XY plane view
subplot(2,2,2);
plot(data_device1(:,2), data_device1(:,3), 'b-', 'LineWidth', 2);
hold on;
plot(data_device2(:,2), data_device2(:,3), 'r-', 'LineWidth', 2);
plot(data_device1(1,2), data_device1(1,3), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
plot(data_device2(1,2), data_device2(1,3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
grid on;
xlabel('X - Right (m)');
ylabel('Y - Forward (m)');
title('Top View (XY Plane)');
legend('Device 1', 'Device 2', 'Device 1 Start', 'Device 2 Start', 'Location', 'best');
axis equal;

% Plot 3: Position differences over time
subplot(2,2,3);
diff_x = data_device2(:,2) - data_device1(:,2);
diff_y = data_device2(:,3) - data_device1(:,3);
diff_z = data_device2(:,4) - data_device1(:,4);
plot(1:length(diff_x), diff_x*100, 'r-', 'LineWidth', 1.5);
hold on;
plot(1:length(diff_y), diff_y*100, 'g-', 'LineWidth', 1.5);
plot(1:length(diff_z), diff_z*100, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Point Index');
ylabel('Position Difference (cm)');
title('Device 2 - Device 1 Position Differences');
legend('ΔX', 'ΔY', 'ΔZ', 'Location', 'best');

% Plot 4: Altitude comparison
subplot(2,2,4);
plot(1:size(data_device1,1), data_device1(:,4), 'b-', 'LineWidth', 2);
hold on;
plot(1:size(data_device2,1), data_device2(:,4), 'r-', 'LineWidth', 2);
grid on;
xlabel('Point Index');
ylabel('Z - Up (m)');
title('Altitude Comparison');
legend('Device 1', 'Device 2', 'Location', 'best');

fprintf('Visualization complete!\n');
fprintf('\n=== Summary ===\n');
fprintf('Device 1 trajectory: %d points\n', size(data_device1, 1));
fprintf('Device 2 trajectory: %d points (transformed)\n', size(data_device2, 1));
fprintf('Offset applied: [%+.4fcm, %+.4fcm, %+.4fcm]\n', ...
        offset_x*100, offset_y*100, offset_z*100);
fprintf('Output files created successfully.\n');
