T=readtable("Estes_F15.csv",'ReadVariableNames',true);
a=T{:,1};
b=T{:,2};
save('F15_thrust.mat','a','b')