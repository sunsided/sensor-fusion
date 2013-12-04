clear all; clc; home;

% define affine gravity vector direction
Gravity = [0; 0; -1; 0];
Gravity = Gravity/norm(Gravity);

% define affine direction of strongest magnetic field
MagneticField = [1; 0; -1; 0];
MagneticField = MagneticField/norm(MagneticField);

% define coordinate system rotation
psi_azimuth     = degtorad(90);
theta_elevation = degtorad(-45);
phi_roll        = degtorad(45);

% define affine base vectors (affine component is zero; direction vectors)
X = [1; 0; 0; 0];
Y = [0; 1; 0; 0];
Z = [0; 0; 1; 0];

% load affine transformation functions
affinePath = fullfile(fileparts(which(mfilename)), '..' , 'affine');
path(affinePath, path);
clear affinePath

% construct rotation matrix
R_theta = affine_rotation_y(theta_elevation);
R_psi   = affine_rotation_z(psi_azimuth);
R_phi   = affine_rotation_x(phi_roll);
R = R_phi*R_theta*R_psi;
clear R_theta R_psi R_phi

% rotate base vectors
x = R*X;
y = R*Y;
z = R*Z;

% rotate base vectors
xgp = R'*X;
ygp = R'*Y;
zgp = R'*Z;

% calculated perceived sensor vectors
% The two scaling operations seem to be pointless as they cancel out
% immediatley but are left here for clarity, as the Z-compensation 
% is exactly what needs to be done for the MPU6050.
acc = R'*Gravity .* [1; 1; -1; 1]; % the sensor does this internally
g = acc          .* [1; 1; -1; 1]; % compensate for sensor
m = R'*MagneticField;

% derive local frame up vector by mirroring Z axis
up = -g(1:3);
up = up/norm(up);

left = cross(m(1:3), g(1:3));
left = left/norm(left);

forward = cross(left, up);
forward = forward/norm(forward);

% print global gravity and magnetic field vectors
fprintf('Forces in reference frame:\n');
fprintf('acc:  % 1.3f % 1.3f % 1.3f\n', Gravity(1), Gravity(2), Gravity(3));
fprintf('mag:  % 1.3f % 1.3f % 1.3f\n', MagneticField(1), MagneticField(2), MagneticField(3));

% print coordinate frame angles
fprintf('\nSimulated orientation:\n');
fprintf('azimuth:   % 4.1f degree\n', radtodeg(psi_azimuth));
fprintf('elevation: % 4.1f degree\n', radtodeg(theta_elevation));
fprintf('roll:      % 4.1f degree\n', radtodeg(phi_roll));

% print coordinate system
fprintf('\nSimulated local coordinate frame:\n');
fprintf('x:    % 1.3f % 1.3f % 1.3f\n', x(1), x(2), x(3));
fprintf('y:    % 1.3f % 1.3f % 1.3f\n', y(1), y(2), y(3));
fprintf('z:    % 1.3f % 1.3f % 1.3f\n', z(1), z(2), z(3));

% print coordinate system
fprintf('\nSimulated perceived global coordinate frame:\n');
fprintf('x:    % 1.3f % 1.3f % 1.3f\n', xgp(1), xgp(2), xgp(3));
fprintf('y:    % 1.3f % 1.3f % 1.3f\n', ygp(1), ygp(2), ygp(3));
fprintf('z:    % 1.3f % 1.3f % 1.3f\n', zgp(1), zgp(2), zgp(3));

% print perceived gravity and magnetic field vectors
fprintf('\nPerceived forces (in local frame):\n');
fprintf('acc:  % 1.3f % 1.3f % 1.3f\n', acc(1), acc(2), acc(3));
fprintf('mag:  % 1.3f % 1.3f % 1.3f\n', m(1), m(2), m(3));

% print derived global frame
fprintf('\nDerived perceived global frame:\n');
fprintf('forw: % 1.3f % 1.3f % 1.3f\n', forward(1), forward(2), forward(3));
fprintf('left: % 1.3f % 1.3f % 1.3f\n', left(1), left(2), left(3));
fprintf('up:   % 1.3f % 1.3f % 1.3f\n', up(1), up(2), up(3));