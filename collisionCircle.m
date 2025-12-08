function isFree = collisionCircle(qnear, qnew, center, r)

    v = qnew - qnear;
    w = center - qnear;

    c1 = dot(w, v);
    c2 = dot(v, v);

    t = c1 / c2;

    if t < 0
        h = qnear;
    elseif t > 1
        h = qnew;
    else
        h = qnear + t * v;
    end

    d = norm(h - center);
    isFree = (d > r);
end