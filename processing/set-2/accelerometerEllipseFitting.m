%% Load the data
dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'full-sphere');
[accelerometer, ~, ~, ~] = loadData(dataSetFolder);

% Fetch axes
x = accelerometer.Data(:, 1);
y = accelerometer.Data(:, 2);
z = accelerometer.Data(:, 3);

%% next data set
dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'unmoved-with-x-pointing-forward');
[accelerometer, ~, ~, ~] = loadData(dataSetFolder);

% Fetch axes
x = [x; accelerometer.Data(:, 1)];
y = [y; accelerometer.Data(:, 2)];
z = [z; accelerometer.Data(:, 3)];

%% next data set
dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'unmoved-with-z-pointing-front-and-x-up');
[accelerometer, ~, ~, ~] = loadData(dataSetFolder);

% Fetch axes
x = [x; accelerometer.Data(:, 1)];
y = [y; accelerometer.Data(:, 2)];
z = [z; accelerometer.Data(:, 3)];

%% next data set
dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'unmoved-with-z-pointing-front-and-y-down');
[accelerometer, ~, ~, ~] = loadData(dataSetFolder);

% Fetch axes
x = [x; accelerometer.Data(:, 1)];
y = [y; accelerometer.Data(:, 2)];
z = [z; accelerometer.Data(:, 3)];

%% next data set
dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'rotate-ccw-around-x-pointing-forward');
[accelerometer, ~, ~, ~] = loadData(dataSetFolder);

% Fetch axes
x = [x; accelerometer.Data(:, 1)];
y = [y; accelerometer.Data(:, 2)];
z = [z; accelerometer.Data(:, 3)];

%% next data set
dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'rotate-ccw-around-y-pointing-left');
[accelerometer, ~, ~, ~] = loadData(dataSetFolder);

% Fetch axes
x = [x; accelerometer.Data(:, 1)];
y = [y; accelerometer.Data(:, 2)];
z = [z; accelerometer.Data(:, 3)];

%% next data set
dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'rotate-ccw-around-z-pointing-up');
[accelerometer, ~, ~, ~] = loadData(dataSetFolder);

% Fetch axes
x = [x; accelerometer.Data(:, 1)];
y = [y; accelerometer.Data(:, 2)];
z = [z; accelerometer.Data(:, 3)];


%% Calibrate sensor
calibrateByEllipseFitting(x, y, z);
