LOW = 24.5;
pa_profile4461;
pa_profile_dBm = pa_measure;
pa_profile = 10.^(pa_profile_dBm / 10);
%~ pa_profile =pa_profile_dBm; 
figure
plot(pa_profile)
title("Mesure du profil du PA en lin")

figure
plot(pa_profile_dBm)
title("Mesure du profil du PA en dBm")
%~ Enveloppe PA
Y= 0 : pi/(235*2) : pi/2;
Enveloppe_PA= (cos(Y))*(LOW);
figure
plot(Enveloppe_PA)
title("Enveloppe du PA")

figure
plot(pa_profile)
% Quantification de la commande PA
real_PA_cmd_steps = zeros(size(Enveloppe_PA));
for i = 1 : length(Enveloppe_PA)
temp = abs(Enveloppe_PA(i) - pa_profile);	
real_PA_cmd_steps(i) = find(temp == min(temp));
end
figure
hold on
plot(pa_profile(real_PA_cmd_steps))
plot(Enveloppe_PA,"r")
title("Quantification de la commande PA Vs Commande PA ideal")
ylabel("mW")
figure 
plot(real_PA_cmd_steps)

 real_PA_cmd_steps = [real_PA_cmd_steps zeros(1 , 1) ];

%~ %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%~ Ecriture du fichier .h
%~ %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

printf("#define LENGTH_PA %d \n",length(real_PA_cmd_steps));
printf("#define PA_CMD_TAB ");
for i = 1 : length(real_PA_cmd_steps)
printf("%d, ", real_PA_cmd_steps(i))
end

