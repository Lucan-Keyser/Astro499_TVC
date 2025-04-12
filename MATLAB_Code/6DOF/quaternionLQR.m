 %diagonal matrix because we're working in the body axis?
I = diag([.05, .1, .1]);
%full state space model, A and B
A = [zeros(3,3), 0.5*I;
    zeros(3,3), zeros(3,3)];
B = [zeros(3,3); I^-1];

%we care about position a little more than velocity
Q = diag([100,100,100,1,1,1]); %[qe1, qe2, qe3, w1, w2, w3]
R = diag([1,1,1]);%equal control cost
K = lqr(A, B, Q, R); %find K

init = [0, 0.5, 0.5]; %orientation (of flight comp. guess)
dt = 0.01;
w0 = [0, 0, 0]; %body angular rates
%lqr referenced: 
%https://www.sciencedirect.com/science/article/pii/S1367578812000387?ref=pdf_download&fr=RR-2&rr=92dbcffa994d1f34