function Total_Lateral_SD = Lateral_conv(SD_Dash,Ixx,Izz,Ixz, Vtot)
% dividing dashed array into tow arrays matrices where first column contains L dash values and second column contains N values
SD_dash = [SD_Dash(5:9) SD_Dash(10:14)];    

% SD_Dash(3:4) = SD_Dash(3:4)*Vtot;

%initializing
L_x = ones(5,1);
N_x = ones(5,1);
G = 1/(1-Ixz^2/Ixx/Izz);

for i = 1:length(SD_dash)
    syms l n
    L_dash = SD_dash(i,1);
    N_dash = SD_dash(i,2);
    eq1 =  L_dash-(Ixz*n/Ixx+l)*G;
    eq2 = N_dash - (Ixz*l/Izz+n)*G;
    eqns = [eq1,eq2];
    [L_x(i),N_x(i)] = solve(eqns,[l,n]);
    % clear l n

end

Total_Lateral_SD = [SD_Dash(1:9); L_x; SD_Dash(10:14); N_x];
end