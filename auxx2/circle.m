%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% calcula um circulo
function ci = circle(r, n)
    ps = linspace(0, 2*pi, n);
    th = linspace(0, 0, n);
    %
    x = r*cos(ps).*cos(th);
    y = r*sin(ps).*cos(th);
    z = r*sin(th);
    %
    ci = [x; y; z];
end