function out=Forward_Kin(in)
alpha1 = in(1);
alpha2 = in(2);

syms phi1 phi2 teta1 teta2 
phi = [phi1, phi2];
teta = [teta1; teta2];
d=2;
L = [2.3,1.9];
R = [L(1)/teta(1); L(1)/teta(2)];

%================================
% 90 degree rotation into global
%================================
base1 = [cos(pi/2) -sin(pi/2) 0
         sin(pi/2)  cos(pi/2) 0
         0          0         1];

base2 = [cos(pi/2) -sin(pi/2) d
         sin(pi/2)  cos(pi/2) 0
         0          0         1];

%================================
% Ensure soft link and it's frame 
% start vertical in the global
%================================
x1=R(1)*sin(teta(1));
y1=R(1)*(1-cos(teta(1)));
xy1=[x1;y1;1];
P1=base1*xy1;
x2=R(2)*sin(teta(2));
y2=R(2)*(1-cos(teta(2)));
xy2=[x2;y2;1];
P2=base2*xy2;

T1 = [cos(phi(1)) -sin(phi(1)) P1(1)
      sin(phi(1))  cos(phi(1))  P1(2)
      0             0           1];


T2 = [cos(phi(2)) -sin(phi(2)) P2(1)
      sin(phi(2))  cos(phi(2))  P2(2)
      0             0           1];

%=====================================
% Solve for phi then apply theta input.
% E is end-effector point transformed
% to the global frame
%=====================================
eqn = T1*[L(2);0;1]-T2*[L(2);0;1]==0;
S = solve(eqn, phi);
a = simplify(S.phi1);
b = simplify(S.phi2);
beta1 = eval(subs(a(1), [teta(1) teta(2)], [alpha1, alpha2]));
beta2 = eval(subs(b(1), [teta(1) teta(2)], [alpha1, alpha2]));
TT1 = eval(subs(T1, [phi(1), teta(1)], [beta1, alpha1]));
TT2 = eval(subs(T2, [phi(2), teta(2)], [beta2, alpha2]));
E = TT1*[L(2);0;1];

%========================================
% Calculate radius for the bend angle then
% gather points on curve from 0 to theta
%========================================
r1 = L(1)/alpha1;
r2 = L(1)/alpha2;

theta1 = linspace(0,alpha1,20);
theta2 = linspace(0,alpha2,20);

%======================================
% Rotate soft-link frame 90 degrees to 
% match global frame
%======================================
x1 = r1*sin(theta1);
y1 = r1*(1-cos(theta1));

x2 = r2*sin(theta2);
y2 = r2*(1-cos(theta2));

p1 = base1*[x1;y1;ones(size(theta1))];
p2 = base2*[x2;y2;ones(size(theta2))];

hold on
plot(p1(1,:),p1(2,:), 'color','b','LineWidth',2)
plot(p2(1,:),p2(2,:), 'color','b','LineWidth',2)
plot([p1(1,end),E(1)], [p1(2,end),E(2)],'color','r','LineWidth',1)
plot([p2(1,end),E(1)],[p2(2,end),E(2)],'color','r','LineWidth',1)
plot(E(1),E(2),'*')

axis equal

out=[alpha1, alpha2, E(1), E(2)];

end