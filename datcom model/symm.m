function symAB = symm(A,B)
% the symmetric matrix of vector A and B is defined as symAB = [(A,B)]
xA=A(1);
yA=A(2);
zA=A(3);

xB=B(1);
yB=B(2);
zB=B(3);
symAB = ...
    [yA*yB+zA*zB -1/2*(xA*yB+yA*xB) -1/2*(xA*zB+zA*xB);...
    -1/2*(xA*yB+yA*xB) (xA*xB+zA*zB) -1/2*(yA*zB+zA*yB);...
    -1/2*(xA*zB+zA*xB) -1/2*(yA*zB+zA*yB) (xA*xB+yA*yB)];
end