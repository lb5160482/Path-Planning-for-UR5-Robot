function rot = eulerxyz(v)
[row, col] = size(v);
if ((row ~= 3) || (col ~= 1))
  error('eulerxyz requires a 3x1 vector argument. Check your dimensions.');
end
phai = v(1);
beta = v(2);
alpha = v(3);
rot = rotz(alpha)*roty(beta)*rotx(phai);