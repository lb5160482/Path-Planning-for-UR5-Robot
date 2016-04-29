function v = eulerzyxinv(M)

[row, col] = size(M);
if ((row ~= 3) || (col ~= 3))
  error('eulerxyzinv requires a 3x3 matrix argument. Check your dimensions.');
end

y =zeros(1,3);
y(1) = atan2(-M(2,3),M(3,3)); % angle_x
y(2) = atan2(M(1,3),sqrt(M(1,1)^2 + M(1,2)^2)); % angle_y
y(3) = atan2(-M(1,2),M(1,1)); %angle _z
v = y;