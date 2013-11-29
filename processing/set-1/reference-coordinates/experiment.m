clear all; clc; home;
% NOTE: In the constellation below, measured Z vector and measured
%       magnetic field vector are (measured) exactly opposite, i.e. g=-m
%       due to the sensor flipping the Z axis!

% define affine gravity vector direction
Gravity = [0; 0; -1; 0];
Gravity = Gravity/norm(Gravity);

% define affine direction of strongest magnetic field
MagneticField = [1; 0; -1; 0];
MagneticField = MagneticField/norm(MagneticField);

% define coordinate system rotation
psi_azimuth     = degtorad(0);
theta_elevation = degtorad(-22.5);
phi_roll        = degtorad(0);

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
g = R'*Gravity;
g = g .* [1; 1; -1; 1]; % adjust for sensor
m = R'*MagneticField;

% derive local frame up vector by mirroring Z axis
up = g(1:3);
up = up/norm(up);

left = cross(g(1:3), m(1:3));
left = left/norm(left);

forward = cross(left, up);
forward = forward/norm(forward);

% print global gravity and magnetic field vectors
fprintf('Forces in reference frame:\n');
fprintf('acc:  % 1.3f % 1.3f % 1.3f\n', Gravity(1), Gravity(2), Gravity(3));
fprintf('mag:  % 1.3f % 1.3f % 1.3f\n', MagneticField(1), MagneticField(2), MagneticField(3));

% print coordinate frame angles
fprintf('\nSimulated orientation:\n');
fprintf('azimuth:   % 4.0f degree\n', radtodeg(psi_azimuth));
fprintf('elevation: % 4.0f degree\n', radtodeg(theta_elevation));
fprintf('roll:      % 4.0f degree\n', radtodeg(phi_roll));

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
fprintf('acc:  % 1.3f % 1.3f % 1.3f\n', g(1), g(2), g(3));
fprintf('mag:  % 1.3f % 1.3f % 1.3f\n', m(1), m(2), m(3));

% print derived global frame
fprintf('\nDerived global frame:\n');
fprintf('forw: % 1.3f % 1.3f % 1.3f\n', forward(1), forward(2), forward(3));
fprintf('left: % 1.3f % 1.3f % 1.3f\n', left(1), left(2), left(3));
fprintf('up:   % 1.3f % 1.3f % 1.3f\n', up(1), up(2), up(3));