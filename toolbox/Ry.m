function out = Ry( ang )
    out = [
        cos(ang), 0, sin(ang);
        0, 1, 0;
        -sin(ang), 0, cos(ang)
    ];
end