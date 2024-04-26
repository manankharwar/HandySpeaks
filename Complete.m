arduinoObj = serialport("COM8", 115200);

% Initialize variables for storing collected data
collectedData = [];

while true
    try
        data = readline(arduinoObj);
    catch
        % Handle timeout warning or other read errors
        disp("Timeout or read error occurred. Retrying...");
        continue;  % Skip the rest of the loop and retry reading
    end

    % Check if data is not empty
    if ~isempty(data)
        disp(data);
        
        % Check for the specific string in the received data
        if contains(data, 'Data collection stopped.')
            break;
        end
        
        % Extract live data from the received data
        liveData = str2double(strsplit(data, ' ')); % Assuming data is space-separated

        % Check the length of liveData
        if length(liveData) == 3  % Assuming you expect three values (gx, gy, gz)
            % Store the collected data
            collectedData = [collectedData; liveData];
        else
            disp("Invalid data format. Skipping...");
        end
    else
        disp("Empty data received. Retrying...");
    end
end

delete(arduinoObj);  % Close the serial port connection
clear arduinoObj;

% Define thresholds for each feature
    thresholds = struct(...
    'Mean', [30, 60], ...
    'Variance', [87, 171], ...
    'StdDev', [9, 13], ...
    'Median', [-36, -62], ...
    'Mode', [-41, -58], ...
    'Skewness', [0, 4], ...
    'Energy', [2e6, 8e6], ...
    'Range', [41, 62], ...
    'IQR', [7, 18],...
    'ZCR', [0, 0],...
    'TAT', [1, 1],...
    'AutoCorrection', [0.2e-3, 0.5e-3]...
);

% Assuming your live data is stored in liveData variable with columns gx, gy, gz
gx = collectedData(:, 1);
gy = collectedData(:, 2);
gz = collectedData(:, 3);

thresholdMin = -150;
thresholdMax = 150;

validIndices = (gx >= thresholdMin & gx <= thresholdMax) & ...
            (gy >= thresholdMin & gy <= thresholdMax) & ...
            (gz >= thresholdMin & gz <= thresholdMax);

% Filter out invalid data points
gx = gx(validIndices);
gy = gy(validIndices);
gz = gz(validIndices);

% Calculate statistical features
meanGx = mean(gx);
meanGy = mean(gy);
meanGz = mean(gz);

varianceGx = var(gx);
varianceGy = var(gy);
varianceGz = var(gz);

stdDevGx = std(gx);
stdDevGy = std(gy);
stdDevGz = std(gz);

medianGx = median(gx);
medianGy = median(gy);
medianGz = median(gz);

modeGx = mode(gx);
modeGy = mode(gy);
modeGz = mode(gz);

% Calculate skewness
skewnessX = skewness(gx);
skewnessY = skewness(gy);
skewnessZ = skewness(gz);

% Calculate energy
energyX = sum(gx.^2);
energyY = sum(gy.^2);
energyZ = sum(gz.^2);

rangeGx = range(gx);
rangeGy = range(gy);
rangeGz = range(gz);

iqrGx = iqr(gx);
iqrGy = iqr(gy);
iqrGz = iqr(gz);

% Display the calculated features
disp('Mean:');
disp([meanGx, meanGy, meanGz]);

disp('Variance:');
disp([varianceGx, varianceGy, varianceGz]);

disp('Standard Deviation:');
disp([stdDevGx, stdDevGy, stdDevGz]);

disp('Median:');
disp([medianGx, medianGy, medianGz]);

disp('Mode:');
disp([modeGx, modeGy, modeGz]);

% Display the calculated features
disp('Skewness:');
disp([skewnessX, skewnessY, skewnessZ]);

disp('Energy:');
disp([energyX, energyY, energyZ]);

disp('Range:');
disp([rangeGx, rangeGy, rangeGz]);

disp('Interquartile Range:');
disp([iqrGx, iqrGy, iqrGz]);

% Calculate zero-crossing rates
zeroCrossRateX = sum(diff(gx > 0) ~= 0) / length(gx);
zeroCrossRateY = sum(diff(gy > 0) ~= 0) / length(gy);
zeroCrossRateZ = sum(diff(gz > 0) ~= 0) / length(gz);

disp('Zero-Crossing Rates:');
disp([zeroCrossRateX, zeroCrossRateY, zeroCrossRateZ]);

% Calculate time above a certain threshold
timeAboveThresholdX = sum(gx > thresholdMin & gx < thresholdMax) / length(gx);
timeAboveThresholdY = sum(gy > thresholdMin & gy < thresholdMax) / length(gy);
timeAboveThresholdZ = sum(gz > thresholdMin & gz < thresholdMax) / length(gz);

disp('Time Above Threshold:');
disp([timeAboveThresholdX, timeAboveThresholdY, timeAboveThresholdZ]);

% Calculate autocorrelation coefficients
autocorrelationX = xcorr(gx, 'coeff');
autocorrelationY = xcorr(gy, 'coeff');
autocorrelationZ = xcorr(gz, 'coeff');

disp('Autocorrelation Coefficients:');
disp([autocorrelationX(1), autocorrelationY(1), autocorrelationZ(1)]);

% Create a time vector (assuming the data is sampled at a constant rate)
t = 1:length(gx);

% Plot the filtered gyroscope data
figure;

% Plot gx
subplot(3, 1, 1);
plot(t, gx, 'r');
title('Filtered Gyroscope Data (X-axis)');
xlabel('Time');
ylabel('GyroX');

% Plot gy
subplot(3, 1, 2);
plot(t, gy, 'g');
title('Filtered Gyroscope Data (Y-axis)');
xlabel('Time');
ylabel('GyroY');

% Plot gz
subplot(3, 1, 3);
plot(t, gz, 'b');
title('Filtered Gyroscope Data (Z-axis)');
xlabel('Time');
ylabel('GyroZ');

% Adjust the layout
sgtitle('Gyroscope');

% Check if live data falls within defined thresholds for each feature
matches = 0;

matches = matches + checkThreshold(meanGx, thresholds.Mean);
matches = matches + checkThreshold(varianceGx, thresholds.Variance);
matches = matches + checkThreshold(stdDevGx, thresholds.StdDev);
matches = matches + checkThreshold(medianGx, thresholds.Median);
matches = matches + checkThreshold(modeGx, thresholds.Mode);
matches = matches + checkThreshold(skewnessX, thresholds.Skewness);
matches = matches + checkThreshold(energyX, thresholds.Energy);
matches = matches + checkThreshold(rangeGx, thresholds.Range);
matches = matches + checkThreshold(iqrGx, thresholds.IQR);
matches = matches + checkThreshold(zeroCrossRateX, thresholds.ZCR);
matches = matches + checkThreshold(timeAboveThresholdX, thresholds.TAT);
matches = matches + checkThreshold(autocorrelationX, thresholds.AutoCorrection);


disp(matches);
thresholdMatches = 7;

if matches >= thresholdMatches
    disp('HELLO');
end

% Function to check if a value falls within a specified range
function result = checkThreshold(value, threshold)
    result = value >= threshold(1) & value <= threshold(2);
end