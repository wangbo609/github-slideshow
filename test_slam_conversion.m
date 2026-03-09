% Simple test script to verify slam_to_gps function
% This can be run without MATLAB GUI

clear all;

fprintf('Testing SLAM to GPS Conversion Function\n');
fprintf('========================================\n\n');

% Create simple test data
test_data = [
    % timestamp, x(m), y(m), z(m), qx, qy, qz, qw, grav_x, grav_y, grav_z
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -9.81;  % Origin
    2.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -9.81;  % 1m East
    3.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -9.81;  % 1m North
    4.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -9.81;  % 1m Up
];

% Test starting position (Beijing)
start_lat = 39.9042;
start_lon = 116.4074;
start_alt = 50.0;

% Run conversion
fprintf('Running conversion with test data...\n');
try
    [lat, lon, altitude, roll, pitch, yaw] = slam_to_gps(test_data, start_lat, start_lon, start_alt);
    fprintf('SUCCESS: Function executed without errors\n\n');
    
    % Verify results
    fprintf('Test Results:\n');
    fprintf('-------------\n');
    
    % Check origin point
    fprintf('Point 1 (Origin):\n');
    fprintf('  Lat: %.8f (expected: %.8f)\n', lat(1), start_lat);
    fprintf('  Lon: %.8f (expected: %.8f)\n', lon(1), start_lon);
    fprintf('  Alt: %.4f (expected: %.4f)\n', altitude(1), start_alt);
    
    % Check that East movement increases longitude
    fprintf('\nPoint 2 (1m East):\n');
    fprintf('  Lat: %.8f (change: %.8f)\n', lat(2), lat(2) - lat(1));
    fprintf('  Lon: %.8f (change: %.8f - should be positive)\n', lon(2), lon(2) - lon(1));
    
    % Check that North movement increases latitude
    fprintf('\nPoint 3 (1m North):\n');
    fprintf('  Lat: %.8f (change: %.8f - should be positive)\n', lat(3), lat(3) - lat(1));
    fprintf('  Lon: %.8f (change: %.8f)\n', lon(3), lon(3) - lon(1));
    
    % Check that Up movement increases altitude
    fprintf('\nPoint 4 (1m Up):\n');
    fprintf('  Alt: %.4f (change: %.4f - should be ~1.0)\n', altitude(4), altitude(4) - altitude(1));
    
    % Verify orientation at origin (should be near zero for identity quaternion)
    fprintf('\nOrientation at origin (identity quaternion):\n');
    fprintf('  Roll:  %.4f degrees\n', roll(1));
    fprintf('  Pitch: %.4f degrees\n', pitch(1));
    fprintf('  Yaw:   %.4f degrees\n', yaw(1));
    
    % Overall validation
    fprintf('\n');
    lon_increase = lon(2) > lon(1);
    lat_increase = lat(3) > lat(1);
    alt_increase = abs(altitude(4) - altitude(1) - 1.0) < 0.01;
    
    if lon_increase && lat_increase && alt_increase
        fprintf('✓ ALL BASIC TESTS PASSED\n');
    else
        fprintf('✗ SOME TESTS FAILED\n');
        if ~lon_increase
            fprintf('  - East movement did not increase longitude\n');
        end
        if ~lat_increase
            fprintf('  - North movement did not increase latitude\n');
        end
        if ~alt_increase
            fprintf('  - Up movement did not increase altitude correctly\n');
        end
    end
    
catch ME
    fprintf('ERROR: Function failed\n');
    fprintf('Message: %s\n', ME.message);
    fprintf('Stack trace:\n');
    for i = 1:length(ME.stack)
        fprintf('  In %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
    end
end

fprintf('\nTest complete.\n');
