function invTanDeg = compassDegToInvTanDeg(x)
% convert the compass degrees [0,360]
% to inverse tangent in degrees [-180,180]
% Note:
% compass degrees:
% 0 deg = N, 90 deg = E, 180 deg = S, 270 deg = W, 360 deg = N
% inverse tangent in degrees:
% 0 deg = +x, 90 deg = +y, 180 deg = -x, -90 deg = -y, -180 = -x
invTanDeg = mod((-x + 270 + 360), 360) - 180;
end