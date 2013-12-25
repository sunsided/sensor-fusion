clear all; clc; home;

%{
syms A11 A12 A13 A21 A22 A23 A31 A32 A33
syms B11 B12 B13 B21 B22 B23 B31 B32 B33

A = [A11 A12 A13; A21 A22 A23; A31 A32 A33];
B = [B11 B12 B13; B21 B22 B23; B31 B32 B33];
C = A*B

C13 = C(1,3)
C23 = C(2,3)
C33 = C(3,3)
C12 = C(1,2)
C11 = C(1,1)
%}


syms x y z lx ly lz
cx = [sym('currentx_x') sym('currentx_y') sym('currentx_z')];
cy = [sym('currenty_x') sym('currenty_y') sym('currenty_z')];
cz = [sym('currentz_x') sym('currentz_y') sym('currentz_z')];

lx = [sym('previousx_x') sym('previousx_y') sym('previousx_z')];
ly = [sym('previousy_x') sym('previousy_y') sym('previousy_z')];
lz = [sym('previousz_x') sym('previousz_y') sym('previousz_z')];

%X = [sym(1) sym(0) sym(0)];
%Y = [sym(0) sym(1) sym(0)];
%Z = [sym(0) sym(0) sym(1)];
X = [sym('worldx_x') sym('worldx_y') sym('worldx_z')];
Y = [sym('worldy_x') sym('worldy_y') sym('worldy_z')];
Z = [sym('worldz_x') sym('worldz_y') sym('worldz_z')];

DCM = [ dot(cx, X), dot(cy, X), dot(cz, X);
        dot(cx, Y), dot(cy, Y), dot(cz, Y);
        dot(cx, Z), dot(cy, Z), dot(cz, Z)]
    
previousDCM = [ dot(lx, X), dot(ly, X), dot(lz, X);
            dot(lx, Y), dot(ly, Y), dot(lz, Y);
            dot(lx, Z), dot(ly, Z), dot(lz, Z)]
    
differenceDCM = DCM'*previousDCM

omega_pitchY  = -asin(differenceDCM(1, 3))
omega_rollX   =  atan2(differenceDCM(2, 3), differenceDCM(3, 3))
omega_yawZ    =  atan2(differenceDCM(1, 2), differenceDCM(1, 1))