% state space representation of quadrotor dynamics linearized about
% 0,0,0... equilibrium point

% system params
syms M m L l g real

% state variables
syms x y z alpha beta gamma xdot ydot zdot alphadot betadot gammadot u1 u2 u3 u4 real

% state variable derivatives
syms xddot yddot zddot alphaddot betaddot gammaddot u1dot u2dot u3dot u4dot real

u0 = sqrt(1/4 * (M + 4 * m) * g);
% A-matrix
syms X U real
X = [x y z alpha beta gamma xdot ydot zdot alphadot betadot gammadot]';
A = [
     0 0 0 0     0    0     1    0    0    0        0       0      ;
     0 0 0 0     0    0     0    1    0    0        0       0      ;
     0 0 0 0     0    0     0    0    1    0        0       0      ;
     0 0 0 0     0    0     0    0    0    1        0       0      ;
     0 0 0 0     0    0     0    0    0    0        1       0      ;
     0 0 0 0     0    0     0    0    0    0        0       1      ;
     0 0 0 0     1/(M+4*m)*4*u0^2    0    0    0    0    0        0       0      ;
     0 0 0 0     0    1/(M+4*m)*4*u0^2     0    0    0    0        0       0      ;
     0 0 0 0     0    0     0    0    0    0        0       0      ;
     0 0 0 0     0    0     0    0    0    0        0       0      ;
     0 0 0 0     0    0     0    0    0    0        0       0      ;
     0 0 0 0     0    0     0    0    0    0        0       0      
];
U = [u1 u2 u3 u4]';
B = [
     0  0  0  0 ;
     0  0  0  0 ;
     0  0  0  0 ;
     0  0  0  0 ;
     0  0  0  0 ;
     0  0  0  0 ;
     0  0  0  0 ;
     0  0  0  0 ;
     1/(M+4*m)*2*u0 1/(M+4*m)*-2*u0 1/(M+4*m)*2*u0 1/(M+4*m)*-2*u0
     m*l/((M+4*m)*L) m*l/((M+4*m)*L) m*l/((M+4*m)*L) m*l/((M+4*m)*L)
     1/m*2*u0 0 -1/m*2*u0 0
     0 1/m*2*u0 0 -1/m*2*u0
    ];


controllability_ = simplify([B A*B A*A*B A*A*A*B A*A*A*A*B A*A*A*A*A*B A*A*A*A*A*A*B A*A*A*A*A*A*A*B A*A*A*A*A*A*A*A*B A*A*A*A*A*A*A*A*A*B A*A*A*A*A*A*A*A*A*A*B A*A*A*A*A*A*A*A*A*A*A*B A*A*A*A*A*A*A*A*A*A*A*A*B]);
k = double(simplify(subs(controllability_, [M m L l g], [1 1 0.2 0.15 9.81])));
% rank is 12 -> therefore fully contrallabile!!!!!!!!!!!!!!!!!! :)

syms K K11 K12 K13 K14 K15 K16 K17 K18 K19 K110 K111 K112 real
syms K21 K22 K23 K24 K25 K26 K27 K28 K29 K210 K211 K212 real
syms K31 K32 K33 K34 K35 K36 K37 K38 K39 K310 K311 K312 real
syms K41 K42 K43 K44 K45 K46 K47 K48 K49 K410 K411 K412 real
K = [
    K11 K12 K13 K14 K15 K16 K17 K18 K19 K110 K111 K112;
    K21 K22 K23 K24 K25 K26 K27 K28 K29 K210 K211 K212;
    K31 K32 K33 K34 K35 K36 K37 K38 K39 K310 K311 K312;
    K41 K42 K43 K44 K45 K46 K47 K48 K49 K410 K411 K412
    ];

K = place(A,B,[-1 -1.1 -1.2 -1.3 -1.4 -1.5 -1.6 -1.7 -1.8 -1.9 -1.91 -1.92]);
