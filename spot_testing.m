function theta = spot_testing()
    theta = [0 0 0 0 0 0 0];
    limits = [-90,90; -45,90; -135,45; -120,120; -135,125; -30,130; -135,125];
    for j = 1:7
        theta(j) = deg2rad(limits(j,1) + (rand(1) * (limits(j,2)-limits(j,1))));     
    end
end
