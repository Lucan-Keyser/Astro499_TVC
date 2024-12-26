% Propulsion
T=readtable("Estes_F15.csv",'ReadVariableNames',true);
prop.time=T{:,1};
prop.thrust=T{:,2};
save('F15_thrust.mat','prop')
plot(prop.time,prop.thrust)
hold on
title('Thrust Curve'); xlabel('Time, s'); ylabel('Thrust, N');
hold off