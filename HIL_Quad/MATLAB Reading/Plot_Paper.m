close all
clc

figure
%subplot(3,2,1);
plot(Val_COM5(1,:),Val_COM5(8,:),'b','LineWidth',2);
hold on
plot(Val_COM5(1,:),Val_COM5(9,:),'r','LineWidth',2);
hold on
plot(Val_COM5(1,:),Val_COM5(10,:),'c','LineWidth',2);
hold on
plot(Val_COM5(1,:),Val_COM5(11,:),'k','LineWidth',2);
title('U_E (rad/sec)');
legend('U_{E1}','U_{E2}','U_{E3}','U_{E4}');
xlabel('Time (sec)','FontSize',10);
grid
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
%subplot(3,2,2);
plot(Val_COM5(1,:),Val_COM5(5,:),'b','LineWidth',2);
hold on
plot(Val_COM5(1,:),Val_COM5(6,:),'r','LineWidth',2);
hold on
plot(Val_COM5(1,:),Val_COM5(7,:),'c','LineWidth',2);
title('Errors (m)');
legend('Err_x','Err_y','Err_z');
xlabel('Time (sec)','FontSize',10);
grid
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
%subplot(3,2,3);
plot(Val_COM5(1,:),Val_COM5(2,:),'b','LineWidth',2);
hold on
plot(Val_COM5(1,:),Val_COM5(3,:),'k','LineWidth',2);
hold on
plot(Val_COM5(1,:),Val_COM5(4,:),'g','LineWidth',2);
hold on
plot(Val_COM5(1,:),Val_COM7(1,:),'r-','LineWidth',2);
hold on
plot(Val_COM5(1,:),Val_COM7(2,:),'m-','LineWidth',2);
hold on
plot(Val_COM5(1,:),Val_COM7(3,:),'c-','LineWidth',2);
title('X-Y-Z Position (m)');
legend('X_{Ref}','Y_{Ref}','Z_{Ref}','X_{Act}','Y_{Act}','Z_{Act}');
xlabel('Time (sec)','FontSize',10);
grid
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
%subplot(3,2,4);
plot(Val_COM5(1,:),Val_COM7(4,:),'b','LineWidth',2);
hold on
plot(Val_COM5(1,:),Val_COM7(5,:),'r','LineWidth',2);
hold on
plot(Val_COM5(1,:),Val_COM7(6,:),'c','LineWidth',2);
title('X-Y-Z Rotation (rad)');
legend('Phi','Theta','Psi');
xlabel('Time (sec)','FontSize',10);
grid
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
%subplot(3,2,5);
plot(Val_COM5(1,:),Val_COM7(7,:),'b','LineWidth',2);
hold on
plot(Val_COM5(1,:),Val_COM7(8,:),'r','LineWidth',2);
hold on
plot(Val_COM5(1,:),Val_COM7(9,:),'c','LineWidth',2);
title('Linear Velocities (m/sec)');
legend('Vel_x','Vel_y','Vel_z');
xlabel('Time (sec)','FontSize',10);
grid
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
%subplot(3,2,6);
plot(Val_COM5(1,:),Val_COM7(10,:),'b','LineWidth',2);
hold on
plot(Val_COM5(1,:),Val_COM7(11,:),'r','LineWidth',2);
hold on
plot(Val_COM5(1,:),Val_COM7(12,:),'c','LineWidth',2);
title('Rotational Velocities (rad/sec)');
legend('Vel_{phi}','Vel_{theta}','Vel_{psi}');
xlabel('Time (sec)','FontSize',10);
grid
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

