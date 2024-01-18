clear; clc;

load("koop5.mat");
WriteToFile('koopMPC5.dat', reference, states, slipAngles, input, 1, 0);

load("lin5.mat");
WriteToFile('linMPC5.dat', reference, states, slipAngles, input, 1, 0);

load("koop20.mat");
WriteToFile('koopMPC20.dat', reference, states, slipAngles, input, 1, 0);

load("lin20.mat");
WriteToFile('linMPC20.dat', reference, states, slipAngles, input, 1, 0);
