% the 2nd section is where the real thing is because the end bins should be
% removed
% Read the signal data from an Excel file
filename = 'LiDAR_2024-07-13_10-33-52.xlsx'; % Replace with your Excel file name
data = readtable(filename);

% Display the column names to check for correct names
disp('Column names in the Excel file:');
disp(data.Properties.VariableNames);

% Assume the Excel file has columns named 'Signal' and 'Time'
% If different, adjust these names to match the column names in your file
signalColumnName = 'Signal';
timeColumnName = 'Time';

% Check if the expected columns exist
if ismember(signalColumnName, data.Properties.VariableNames) && ismember(timeColumnName, data.Properties.VariableNames)
    % Extract signal and time from the table
    signal = data.(signalColumnName);
    t = data.(timeColumnName);

    % Plot raw data to ensure it's read correctly
    figure;
    subplot(3,1,1);
    plot(t, signal);
    title('Raw Signal Data');
    xlabel('Time');
    ylabel('Signal');
    
    % Check if time values are uniformly spaced
    dt = diff(t);
    if max(dt) - min(dt) > 1e-6
        % Time values are not uniformly spaced, interpolate to uniform grid
        disp('Time values are not uniformly spaced. Interpolating to a uniform grid.');
        uniform_t = linspace(t(1), t(end), length(t));
        uniform_signal = interp1(t, signal, uniform_t, 'linear');
    else
        % Time values are uniformly spaced
        uniform_t = t;
        uniform_signal = signal;
    end

    % Compute the Fourier transform of the signal
    N = length(uniform_signal);    % Length of signal
    fs = 1/mean(diff(uniform_t));  % Calculate the sampling frequency
    Y = fft(uniform_signal);       % Compute Fourier transform
    f = (0:N-1)*(fs/N);            % Frequency vector

    % Compute the magnitude spectrum
    magnitude = abs(Y/N);

    % Bin the magnitude spectrum
    numBins = 50; % Number of bins
    [counts, edges] = histcounts(f, numBins);
    binCenters = edges(1:end-1) + diff(edges)/2;
    binnedMagnitude = accumarray(discretize(f, edges)', magnitude, [], @mean);

    % Find peaks in the magnitude spectrum
    [pks, locs] = findpeaks(magnitude, 'MinPeakHeight', 0.1);

    % Measure distances between peaks
    distances = diff(f(locs));

    % Plot the original or interpolated signal
%     subplot(3,1,2);
%     plot(uniform_t, uniform_signal);
%     title('Interpolated Signal');
%     xlabel('Time (s)');
%     ylabel('Amplitude');

    % Plot the binned magnitude spectrum
    subplot(2,1,2);
    bar(binCenters, binnedMagnitude);
    hold on;
    %plot(f(locs), pks, 'r*'); % Mark peaks
    title('Binned Magnitude Spectrum');
    xlabel('Frequency (Hz)');
    ylabel('Magnitude');
    legend('Binned Magnitude Spectrum', 'Peaks');

    % Display distances between peaks
    disp('Distances between peaks (Hz):');
    disp(distances);

    hold off;
else
    error('The expected columns are not found in the Excel file.');
end

%%
% Read the signal data from an Excel file
filename = 'LiDAR_2024-07-13_10-33-52.xlsx'; % Replace with your Excel file name
data = readtable(filename);

% Display the column names to check for correct names
disp('Column names in the Excel file:');
disp(data.Properties.VariableNames);

% Assume the Excel file has columns named 'Signal' and 'Time'
% If different, adjust these names to match the column names in your file
signalColumnName = 'Signal';
timeColumnName = 'Time';

% Check if the expected columns exist
if ismember(signalColumnName, data.Properties.VariableNames) && ismember(timeColumnName, data.Properties.VariableNames)
    % Extract signal and time from the table
    signal = data.(signalColumnName);
    t = data.(timeColumnName);

    % Plot raw data to ensure it's read correctly
    figure;
    subplot(3,1,1);
    plot(t, signal);
    title('Raw Signal Data');
    xlabel('Time');
    ylabel('Signal');
    
    % Check if time values are uniformly spaced
    dt = diff(t);
    if max(dt) - min(dt) > 1e-6
        % Time values are not uniformly spaced, interpolate to uniform grid
        disp('Time values are not uniformly spaced. Interpolating to a uniform grid.');
        uniform_t = linspace(t(1), t(end), length(t));
        uniform_signal = interp1(t, signal, uniform_t, 'linear');
    else
        % Time values are uniformly spaced
        uniform_t = t;
        uniform_signal = signal;
    end

    % Compute the Fourier transform of the signal
    N = length(uniform_signal);    % Length of signal
    fs = 1/mean(diff(uniform_t));  % Calculate the sampling frequency
    Y = fft(uniform_signal);       % Compute Fourier transform
    f = (0:N-1)*(fs/N);            % Frequency vector

    % Compute the magnitude spectrum
    magnitude = abs(Y/N);

    % Bin the magnitude spectrum
    numBins = 50; % Number of bins
    [counts, edges] = histcounts(f, numBins);
    binCenters = edges(1:end-1) + diff(edges)/2;
    binnedMagnitude = accumarray(discretize(f, edges)', magnitude, [], @mean);

    % Remove the first bin and last
    binCenters([1, end]) = [];
    binnedMagnitude([1, end]) = [];

    % Find peaks in the magnitude spectrum
    [pks, locs] = findpeaks(magnitude, 'MinPeakHeight', 0.1);

    % Measure distances between peaks
    distances = diff(f(locs));

    % Plot the original or interpolated signal
%     subplot(3,1,2);
%     plot(uniform_t, uniform_signal);
%     title('Interpolated Signal');
%     xlabel('Time (s)');
%     ylabel('Amplitude');

    % Plot the binned magnitude spectrum with the first bin removed
    subplot(2,1,2);
    bar(binCenters, binnedMagnitude);
    hold on;
%     plot(f(locs), pks, 'r*'); % Mark peaks
    title('Binned Magnitude Spectrum (First Bin Removed)');
    xlabel('Frequency (Hz)');
    ylabel('Magnitude');
%     legend('Binned Magnitude Spectrum', 'Peaks');

    % Display distances between peaks
    disp('Distances between peaks (Hz):');
    disp(distances);

    hold off;
else
    error('The expected columns are not found in the Excel file.');
end
