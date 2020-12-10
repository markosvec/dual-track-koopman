% write original data
Y_koop_mpt = Y_koop_mpt';
Y_lin = Y_lin';
Y_lin0 = Y_lin0';

WriteToFile('original1.dat', tout(1:50), X(1:50,:));
WriteToFile('original2.dat', tout(51:100), X(51:100,:));
WriteToFile('original3.dat', tout(101:150), X(101:150,:));
WriteToFile('original4.dat', tout(151:200), X(151:200,:));
WriteToFile('original5.dat', tout(201:250), X(201:250,:));
WriteToFile('original6.dat', tout(251:300), X(251:300,:));
WriteToFile('original7.dat', tout(301:350), X(301:350,:));
WriteToFile('original8.dat', tout(351:400), X(351:400,:));
WriteToFile('original9.dat', tout(401:450), X(401:450,:));
WriteToFile('original10.dat', tout(451:500), X(451:500,:));

% write koopman data
WriteToFile('koopman1.dat', tout(1:50), Y_koop_mpt(1:50,:));
WriteToFile('koopman2.dat', tout(51:100), Y_koop_mpt(51:100,:));
WriteToFile('koopman3.dat', tout(101:150), Y_koop_mpt(101:150,:));
WriteToFile('koopman4.dat', tout(151:200), Y_koop_mpt(151:200,:));
WriteToFile('koopman5.dat', tout(201:250), Y_koop_mpt(201:250,:));
WriteToFile('koopman6.dat', tout(251:300), Y_koop_mpt(251:300,:));
WriteToFile('koopman7.dat', tout(301:350), Y_koop_mpt(301:350,:));
WriteToFile('koopman8.dat', tout(351:400), Y_koop_mpt(351:400,:));
WriteToFile('koopman9.dat', tout(401:450), Y_koop_mpt(401:450,:));
WriteToFile('koopman10.dat', tout(451:500), Y_koop_mpt(451:500,:));

% write lin data
WriteToFile('lin1.dat', tout(1:50), Y_lin(1:50,:));
WriteToFile('lin2.dat', tout(51:100), Y_lin(51:100,:));
WriteToFile('lin3.dat', tout(101:150), Y_lin(101:150,:));
WriteToFile('lin4.dat', tout(151:200), Y_lin(151:200,:));
WriteToFile('lin5.dat', tout(201:250), Y_lin(201:250,:));
WriteToFile('lin6.dat', tout(251:300), Y_lin(251:300,:));
WriteToFile('lin7.dat', tout(301:350), Y_lin(301:350,:));
WriteToFile('lin8.dat', tout(351:400), Y_lin(351:400,:));
WriteToFile('lin9.dat', tout(401:450), Y_lin(401:450,:));
WriteToFile('lin10.dat', tout(451:500), Y_lin(451:500,:));

% write lin0 data
WriteToFile('linInit1.dat', tout(1:50), Y_lin0(1:50,:));
WriteToFile('linInit2.dat', tout(51:100), Y_lin0(51:100,:));
WriteToFile('linInit3.dat', tout(101:150), Y_lin0(101:150,:));
WriteToFile('linInit4.dat', tout(151:200), Y_lin0(151:200,:));
WriteToFile('linInit5.dat', tout(201:250), Y_lin0(201:250,:));
WriteToFile('linInit6.dat', tout(251:300), Y_lin0(251:300,:));
WriteToFile('linInit7.dat', tout(301:350), Y_lin0(301:350,:));
WriteToFile('linInit8.dat', tout(351:400), Y_lin0(351:400,:));
WriteToFile('linInit9.dat', tout(401:450), Y_lin0(401:450,:));
WriteToFile('linInit10.dat', tout(451:500), Y_lin0(451:500,:));



