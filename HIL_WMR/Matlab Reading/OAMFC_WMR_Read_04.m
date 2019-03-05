clear 
clc

S1 = serial('COM5','BaudRate',115200);
S2 = serial('COM7','BaudRate',115200);
fopen(S1);
fopen(S2);
pause('on');
pause(1);
Tf = 130;
t_read = 0;

i = 0;
while (t_read<=Tf)
    i = i+1;
    t_read
    for j=1:3
        Data_COM5{j,i} = fscanf(S1);
        b = Data_COM5{j,i};
        if j==1
            Val_COM5(j,i) = str2double(b(7:end));
            t_read = Val_COM5(j,i);
        elseif j<=3
            Val_COM5(j,i) = str2double(b(10:end));               
%         elseif j<=5           
%             Val_COM5(j,i) = str2double(b(10:end));                                     
%         elseif j<=17
%             Val_COM5(j,i) = str2double(b(9:end));
%         else
%             Val_COM5(j,i) = str2double(b(7:end));
        end                
    end
    
    for k=1:5
        Data_COM7{k,i} = fscanf(S2);
        c = Data_COM7{k,i};
        if strcmp(c(1),'X')
            Val_COM7(1,i) = str2double(c(8:end));            
        elseif strcmp(c(1),'Y')
            Val_COM7(2,i) = str2double(c(8:end));               
        elseif strcmp(c(1),'P')            
            Val_COM7(3,i) = str2double(c(6:end)); 
        elseif strcmp(c(4),'1')            
            Val_COM7(4,i) = str2double(c(7:end)); 
        elseif strcmp(c(4),'2')            
            Val_COM7(5,i) = str2double(c(7:end)); 
        end                
    end
    %pause(0.5);    
end

fclose(S1);
fclose(S2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for i=3:(length(Val_COM7)-2)
%     for j=1:5
%         if (isnan(Val_COM7(j,i)))
%             Val_COM7(j,i) = 0.5*(Val_COM7(j,i-2)+Val_COM7(j,i+2));
%         end
%         Diff = (Val_COM7(j,i)-Val_COM7(j,i-1))/...
%             (Val_COM5(1,i)-Val_COM5(1,i-1));
%         if abs(Diff)>10
%            Val_COM7(j,i) = 0.5*(Val_COM7(j,i-2)+Val_COM7(j,i+2));
%         end
%     end
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
X_ref = zeros(length(Val_COM5),1);
Y_ref = zeros(length(Val_COM5),1);
SSE = 0;

for i=1:length(Val_COM5)
    if Val_COM5(1,i) < 10
        X_ref(i,1) = 0;
        Y_ref(i,1) = 0;
    elseif Val_COM5(1,i) < 40
        X_ref(i,1) = 5;
        Y_ref(i,1) = 0;
    elseif Val_COM5(1,i) < 70
        X_ref(i,1) = 5;
        Y_ref(i,1) = 3;
    elseif Val_COM5(1,i) < 100
        X_ref(i,1) = 0;
        Y_ref(i,1) = 3;
    else
        X_ref(i,1) = 0;
        Y_ref(i,1) = 0;
    end    
    SSE = SSE + ( (X_ref(i,1) - Val_COM7(1,i))^2 + (Y_ref(i,1) - Val_COM7(2,i))^2 );
end
MSE = sqrt(SSE) / length(Val_COM5);
        
% save('Data_COM5','Data_COM5');
% save('Data_COM7','Data_COM7');
% save('Val_COM5','Val_COM5');
% save('Val_COM7','Val_COM7');

% figure
% %subplot(3,2,1);
% plot(Val_COM5(1,:),Val_COM5(2,:),'b','LineWidth',2);
% hold on
% plot(Val_COM5(1,:),Val_COM5(3,:),'r','LineWidth',2);
% title('P_{Lin}');
% legend('P(1,1)_{Lin}','P(2,2)_{Lin}');
% xlabel('Time (sec)','FontSize',10);
% grid
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure
% % subplot(3,2,2);
% plot(Val_COM5(1,:),Val_COM5(4,:),'b','LineWidth',2);
% hold on
% plot(Val_COM5(1,:),Val_COM5(5,:),'r','LineWidth',2);
% title('P_{Ang}');
% legend('P(1,1)_{Ang}','P(2,2)_{Ang}');
% xlabel('Time (sec)','FontSize',10);
% grid
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% subplot(3,2,3);
% plot(Val_COM5(1,:),Val_COM5(10,:),'b','LineWidth',2);
% hold on
% plot(Val_COM5(1,:),Val_COM5(11,:),'r','LineWidth',2);
% title('A_{Pos}');
% legend('A1_{Pos}','A2_{Pos}');
% xlabel('Time (sec)','FontSize',10);
% grid
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% subplot(3,2,4);
% plot(Val_COM5(1,:),Val_COM5(12,:),'b','LineWidth',2);
% hold on
% plot(Val_COM5(1,:),Val_COM5(13,:),'r','LineWidth',2);
% title('A_{Rot}');
% legend('A1_{Rot}','A2_{Rot}');
% xlabel('Time (sec)','FontSize',10);
% grid
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% subplot(3,2,5);
% plot(Val_COM5(1,:),Val_COM5(14,:),'b','LineWidth',2);
% hold on
% plot(Val_COM5(1,:),Val_COM5(15,:),'r','LineWidth',2);
% title('P_{Pos}');
% legend('P1_{Pos}','P2_{Pos}');
% xlabel('Time (sec)','FontSize',10);
% grid
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% subplot(3,2,6);
% plot(Val_COM5(1,:),Val_COM5(16,:),'b','LineWidth',2);
% hold on
% plot(Val_COM5(1,:),Val_COM5(17,:),'r','LineWidth',2);
% title('P_{Rot}');
% legend('P1_{Rot}','P2_{Rot}');
% xlabel('Time (sec)','FontSize',10);
% grid
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% figure
% % subplot(3,2,1);
% plot(Val_COM5(1,:),Val_COM7(4,:),'b','LineWidth',4);
% hold on
% plot(Val_COM5(1,:),Val_COM7(5,:),'r','LineWidth',4);
% title('U_E (rad/sec)');
% legend('U_{E1}','U_{E2}');
% xlabel('Time (sec)','FontSize',10);
% grid
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure
% % subplot(3,2,2);
% plot(Val_COM5(1,:),Val_COM5(2,:),'b','LineWidth',4);
% hold on
% plot(Val_COM5(1,:),Val_COM5(3,:),'r','LineWidth',4);
% title('Errors');
% legend('Err_{Pos} (m)','Err_{Rot} (rad)');
% xlabel('Time (sec)','FontSize',10);
% grid
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
%subplot(3,2,3);
plot(Val_COM5(1,:),X_ref,'b','LineWidth',4);
hold on
plot(Val_COM5(1,:),Y_ref,'k','LineWidth',4);
hold on
plot(Val_COM5(1,:),Val_COM7(1,:),'r-','LineWidth',4);
hold on
plot(Val_COM5(1,:),Val_COM7(2,:),'m-','LineWidth',4);
title('X-Y Position (m)');
legend('X_{Ref}','Y_{Ref}','X_{Act}','Y_{Act}');
xlabel('Time (sec)','FontSize',10);
grid
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure
% %subplot(3,2,4);
% plot(Val_COM5(1,:),Val_COM7(3,:),'b','LineWidth',4);
% title('Z Rotation (rad)');
% legend('Psi');
% xlabel('Time (sec)','FontSize',10);
% grid
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure
% %subplot(3,2,5);
% plot(Val_COM5(1,:),Val_COM7(4,:),'b','LineWidth',2);
% hold on
% plot(Val_COM5(1,:),Val_COM7(5,:),'r','LineWidth',2);
% title('Velocities');
% legend('Vel_{Pos}(m/sec)','Vel_{Rot}(rad/sec)');
% xlabel('Time (sec)','FontSize',10);
% grid
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

