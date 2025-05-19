figure
plot(aero_sym{1,1}.alpha,aero_sym{1,1}.cl(:,1))
grid on
hold on
plot(aero_sym{1,1}.alpha,aero_sym{1,1}.cl(:,2))
plot(aero_sym{1,1}.alpha,aero_sym{1,1}.cl(:,3))
plot(aero_sym{1,1}.alpha,aero_sym{1,1}.cl(:,4))
plot(aero_sym{1,1}.alpha,aero_sym{1,1}.cl(:,5))
xlabel('\alpha (deg)')
ylabel('CL')
legend('V=6 m/s','V=12 m/s','V=18 m/s','V=24 m/s','V=30 m/s')
figure
plot(aero_sym{1,1}.alpha,aero_sym{1,1}.cd(:,1))
grid on
hold on
plot(aero_sym{1,1}.alpha,aero_sym{1,1}.cd(:,2))
plot(aero_sym{1,1}.alpha,aero_sym{1,1}.cd(:,3))
plot(aero_sym{1,1}.alpha,aero_sym{1,1}.cd(:,4))
plot(aero_sym{1,1}.alpha,aero_sym{1,1}.cd(:,5))
xlabel('\alpha (deg)')
ylabel('CD')
legend('V=6 m/s','V=12 m/s','V=18 m/s','V=24 m/s','V=30 m/s')

figure
plot(aero_sym{1,1}.alpha,aero_sym{1,1}.cm(:,1))
grid on
hold on
plot(aero_sym{1,1}.alpha,aero_sym{1,1}.cm(:,2))
plot(aero_sym{1,1}.alpha,aero_sym{1,1}.cm(:,3))
plot(aero_sym{1,1}.alpha,aero_sym{1,1}.cm(:,4))
plot(aero_sym{1,1}.alpha,aero_sym{1,1}.cm(:,5))
xlabel('\alpha (deg)')
ylabel('CM')
legend('V=6 m/s','V=12 m/s','V=18 m/s','V=24 m/s','V=30 m/s')