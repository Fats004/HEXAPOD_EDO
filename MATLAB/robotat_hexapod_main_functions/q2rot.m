function R = q2rot(q)
    
    % Unit cuaternion check, comment for speed
    unity_norm_tol = 0.1;
    if((numel(q) ~= 4) || (norm(q) - 1 > unity_norm_tol))
        error('Input is NOT a unit quaternion');
    end

    % https://www.ncbi.nlm.nih.gov/pmc/articles/PMC9648712/
    qr = q(1);
    qx = q(2);
    qy = q(3);
    qz = q(4);

    R = [qr^2 + qx^2 - qy^2 - qz^2, -2*qr*qz + 2*qx*qy, 2*qr*qy + 2*qx*qz;
         2*qr*qz + 2*qx*qy, qr^2 - qx^2 + qy^2 - qz^2, -2*qr*qx + 2*qy*qz;
         -2*qr*qy + 2*qx*qz, 2*qr*qx + 2*qy*qz, qr^2 - qx^2 - qy^2 + qz^2];
end