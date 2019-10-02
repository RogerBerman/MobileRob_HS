function [value] = sat(x,d)
    if abs(x) > d
        value = sign(x)*d;
    else
        value = x;
    end
end
