clear 
clc

S1 = serial('COM5','BaudRate',115200);
S2 = serial('COM7','BaudRate',115200);
fopen(S1);
fopen(S2);
pause('on');
pause(1);

Tf = 120;
t_read = 0;
N_var5 = 1;
N_var7 = 3;

i = 0;
while (t_read<Tf)
    i = i+1;
    t_read
    for j=1:N_var5
        Data_COM5{j,i} = fscanf(S1);
        b = Data_COM5{j,i};
        if strcmp(b(1),'T')
            Val_COM5(1,i) = str2double(b(7:end));
            t_read = Val_COM5(1,i);
%         elseif strcmp(b(1),'Y')
%             if strcmp(b(4),'x')
%                 Val_COM5(2,i) = str2double(b(7:end)); 
%             elseif strcmp(b(4),'y')
%                 Val_COM5(3,i) = str2double(b(7:end)); 
%             elseif strcmp(b(4),'z')
%                 Val_COM5(4,i) = str2double(b(7:end));
%             end
%         elseif strcmp(b(1),'E')
%             if strcmp(b(5),'x')
%                 Val_COM5(5,i) = str2double(b(8:end)); 
%             elseif strcmp(b(5),'y')
%                 Val_COM5(6,i) = str2double(b(8:end)); 
%             elseif strcmp(b(5),'z')
%                 Val_COM5(7,i) = str2double(b(8:end));
%             end                                     
%         elseif strcmp(b(1),'U')
%             if strcmp(b(4),'1')
%                 Val_COM5(8,i) = str2double(b(7:end)); 
%             elseif strcmp(b(4),'2')
%                 Val_COM5(9,i) = str2double(b(7:end)); 
%             elseif strcmp(b(4),'3')
%                 Val_COM5(10,i) = str2double(b(7:end));
%             elseif strcmp(b(4),'4')
%                 Val_COM5(11,i) = str2double(b(7:end));
%             end 
%         elseif strcmp(b(1),'P')
%            if strcmp(b(2),'1')
% %                 if strcmp(b(3),':')
% %                     Val_COM5(2,i) = str2double(b(5:end));
%                 if strcmp(b(3),'0')
%                     Val_COM5(5,i) = str2double(b(6:end)); 
%                 elseif strcmp(b(3),'1')
%                     Val_COM5(6,i) = str2double(b(6:end));
%                 elseif strcmp(b(3),'2')
%                     Val_COM5(7,i) = str2double(b(6:end));
%                 end
%             elseif strcmp(b(2),'2')
%                 Val_COM5(3,i) = str2double(b(5:end));
%             elseif strcmp(b(2),'3')
%                 Val_COM5(4,i) = str2double(b(5:end));
%             elseif strcmp(b(2),'4')
%                 Val_COM5(5,i) = str2double(b(5:end));
%             elseif strcmp(b(2),'5')
%                 Val_COM5(6,i) = str2double(b(5:end));
%            elseif strcmp(b(2),'6')
%                 Val_COM5(7,i) = str2double(b(5:end));
%            elseif strcmp(b(2),'7')
%                 Val_COM5(2,i) = str2double(b(5:end)); 
%            elseif strcmp(b(2),'8')
%                 Val_COM5(3,i) = str2double(b(5:end)); 
%            elseif strcmp(b(2),'9')
%                 Val_COM5(4,i) = str2double(b(5:end));            
%            end
       end                
   end
    
   for k=1:N_var7
       Data_COM7{k,i} = fscanf(S2);
       c = Data_COM7{k,i};
        if strcmp(c(1),'X')
            Val_COM7(1,i) = str2double(c(8:end));            
        elseif strcmp(c(1),'Y')
            Val_COM7(2,i) = str2double(c(8:end));   
        elseif strcmp(c(1),'Z')
            Val_COM7(3,i) = str2double(c(8:end)); 
%         if strcmp(c(1),'T')
%             Val_COM7(2,i) = str2double(c(7:end));
%         elseif strcmp(c(1),'P')  
%             if strcmp(c(2),'h')
%                 Val_COM7(1,i) = str2double(c(6:end)); 
%             elseif strcmp(c(2),'s')
%                 Val_COM7(3,i) = str2double(c(6:end));
%             end                
%         elseif strcmp(c(1),'V')            
%             if strcmp(c(3),'x')
%                 Val_COM7(7,i) = str2double(c(6:end)); 
%             elseif strcmp(c(3),'y')
%                 Val_COM7(8,i) = str2double(c(6:end));
%             elseif strcmp(c(3),'z')
%                 Val_COM7(9,i) = str2double(c(6:end));
%             end                 
%         elseif strcmp(c(1),'W')            
%             if strcmp(c(3),'t')
%                 Val_COM7(11,i) = str2double(c(10:end)); 
%             elseif strcmp(c(4),'h')
%                 Val_COM7(10,i) = str2double(c(8:end));
%             elseif strcmp(c(4),'s')
%                 Val_COM7(12,i) = str2double(c(8:end));
%             end  
%         if strcmp(c(1),'U')
%             if strcmp(c(4),'1')
%                 Val_COM7(1,i) = str2double(c(7:end)); 
%             elseif strcmp(c(4),'2')
%                 Val_COM7(2,i) = str2double(c(7:end)); 
%             elseif strcmp(c(4),'3')
%                 Val_COM7(3,i) = str2double(c(7:end));
%             elseif strcmp(c(4),'4')
%                 Val_COM7(4,i) = str2double(c(7:end));
%             end 
         end                
   end
    %pause(0.5);    
end

fclose(S1);
fclose(S2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% generating the desired commands   
% X_ref = zeros(length(Val_COM5),1);
% Y_ref = zeros(length(Val_COM5),1);
% Z_ref = zeros(length(Val_COM5),1);
% E_X = zeros(length(Val_COM5),1);
% E_Y = zeros(length(Val_COM5),1);
% E_Z = zeros(length(Val_COM5),1);
% 
% for i=1:length(Val_COM5)
%     Z_ref(i,1) = 0.2 * Val_COM5(1,i);    
%     if (Val_COM5(1,i) < 20)
%         X_ref(i,1) = 0;
%         Y_ref(i,1) = 0;
%     else
%         X_ref(i,1) = 2*cos(0.2*(Val_COM5(1,i)-20)) - 2; 
%         Y_ref(i,1) = 2*sin(0.2*(Val_COM5(1,i)-20)); 
%     end
%     
%     E_X(i,1) = X_ref(i,1) - Val_COM7(1,i);
%     E_Y(i,1) = Y_ref(i,1) - Val_COM7(2,i);
%     E_Z(i,1) = Z_ref(i,1) - Val_COM7(3,i);
% end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for i=3:(length(Val_COM5)-2)
%     if (Val_COM5(1,i)==0)
%         Val_COM5(1,i) = 0.5*(Val_COM5(1,i-1)+Val_COM5(1,i+1));
%     end
%     for j=2:N_var5
%         if (isnan(Val_COM5(j,i)))
%             Val_COM5(j,i) = 0.5*(Val_COM5(j,i-2)+Val_COM5(j,i+2));
%         end
%         Diff = (Val_COM5(j,i)-Val_COM5(j,i-1))/...
%             (Val_COM5(1,i)-Val_COM5(1,i-1));
%         if abs(Diff)>10
%            Val_COM5(j,i) = 0.5*(Val_COM5(j,i-2)+Val_COM5(j,i+2));
%         end
%     end
% end
% 
% for i=3:(length(Val_COM7)-2)
%     for j=1:N_var7
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
% save('Data_COM5','Data_COM5');
% save('Data_COM7','Data_COM7');
% save('Val_COM5','Val_COM5');
% save('Val_COM7','Val_COM7');

X_ref = zeros(length(Val_COM5),1);
Y_ref = zeros(length(Val_COM5),1);
Z_ref = zeros(length(Val_COM5),1);
SSE = 0;

for i=1:length(Val_COM5)
    Z_ref(i,1) = 0.2 * Val_COM5(1,i);
    if Val_COM5(1,i) < 20
        X_ref(i,1) = 0;
        Y_ref(i,1) = 0;
    else
        X_ref(i,1) = 2*cos(0.2*(Val_COM5(1,i)-20)) - 2;
        Y_ref(i,1) = 2*sin(0.2*(Val_COM5(1,i)-20));
    end    
    SSE = SSE + ( (X_ref(i,1) - Val_COM7(1,i))^2 + (Y_ref(i,1) - Val_COM7(2,i))^2 + (Z_ref(i,1) - Val_COM7(3,i))^2);
end
MSE = sqrt(SSE) / length(Val_COM5);

% figure
% %subplot(3,2,1);
% plot(Val_COM5(1,:),Val_COM5(2,:),'b','LineWidth',4);
% hold on
% plot(Val_COM5(1,:),Val_COM5(3,:),'r','LineWidth',4);
% hold on
% plot(Val_COM5(1,:),Val_COM5(4,:),'c','LineWidth',4);
% hold on
% plot(Val_COM5(1,:),Val_COM5(5,:),'k','LineWidth',4);
% hold on
% plot(Val_COM5(1,:),Val_COM5(6,:),'g','LineWidth',4);
% hold on
% plot(Val_COM5(1,:),Val_COM5(7,:),'m','LineWidth',4);
% hold on
% plot(Val_COM5(1,:),Val_COM5(8,:),'b','LineWidth',4);
% hold on
% plot(Val_COM5(1,:),Val_COM5(9,:),'r','LineWidth',4);
% hold on
% plot(Val_COM5(1,:),Val_COM5(10,:),'c','LineWidth',4);
% hold on
% plot(Val_COM5(1,:),Val_COM5(11,:),'k','LineWidth',4);
% hold on
% plot(Val_COM5(1,:),Val_COM5(12,:),'g','LineWidth',4);
% hold on
% plot(Val_COM5(1,:),Val_COM5(13,:),'m','LineWidth',4);
% title('g');
% legend('g7','g8','g9','g10','g11','g12');
% xlabel('Time (sec)','FontSize',20);
% grid
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% subplot(3,2,3);
% plot(Val_COM5(1,:),Val_COM5(14,:),'b','LineWidth',2);
% hold on
% plot(Val_COM5(1,:),Val_COM5(15,:),'r','LineWidth',2);
% hold on
% plot(Val_COM5(1,:),Val_COM5(16,:),'c','LineWidth',2);
% title('A_{Pos}');
% legend('A4_{Pos}','A5_{Pos}','A6_{Pos}');
% xlabel('Time (sec)','FontSize',10);
% grid
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% subplot(3,2,4);
% plot(Val_COM5(1,:),Val_COM5(17,:),'b','LineWidth',2);
% hold on
% plot(Val_COM5(1,:),Val_COM5(18,:),'r','LineWidth',2);
% hold on
% plot(Val_COM5(1,:),Val_COM5(19,:),'c','LineWidth',2);
% title('A_{Rot}');
% legend('A4_{Rot}','A5_{Rot}','A6_{Rot}');
% xlabel('Time (sec)','FontSize',10);
% grid
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% subplot(3,2,5);
% plot(Val_COM5(1,:),Val_COM5(20,:),'b','LineWidth',2);
% hold on
% plot(Val_COM5(1,:),Val_COM5(21,:),'r','LineWidth',2);
% hold on
% plot(Val_COM5(1,:),Val_COM5(22,:),'c','LineWidth',2);
% title('P_{Pos}');
% legend('P4_{Pos}','P5_{Pos}','P6_{Pos}');
% xlabel('Time (sec)','FontSize',10);
% grid
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% subplot(3,2,6);
% plot(Val_COM5(1,:),Val_COM5(23,:),'b','LineWidth',2);
% hold on
% plot(Val_COM5(1,:),Val_COM5(24,:),'r','LineWidth',2);
% hold on
% plot(Val_COM5(1,:),Val_COM5(25,:),'c','LineWidth',2);
% title('P_{Rot}');
% legend('P4_{Rot}','P5_{Rot}','P6_{Rot}');
% xlabel('Time (sec)','FontSize',10);
% grid
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% figure
% subplot(3,2,1);
% plot(Val_COM5(1,:),Val_COM7(1,:),'b','LineWidth',2);
% hold on
% plot(Val_COM5(1,:),Val_COM7(2,:),'r','LineWidth',2);
% hold on
% plot(Val_COM5(1,:),Val_COM7(3,:),'c','LineWidth',2);
% hold on
% plot(Val_COM5(1,:),Val_COM7(4,:),'k','LineWidth',2);
% title('U_E (rad/sec)');
% legend('U_{E1}','U_{E2}','U_{E3}','U_{E4}');
% xlabel('Time (sec)','FontSize',10);
% grid
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% subplot(3,2,2);
% plot(Val_COM5(1,:),E_X,'b','LineWidth',4);
% hold on
% plot(Val_COM5(1,:),E_Y,'r','LineWidth',4);
% hold on
% plot(Val_COM5(1,:),E_Z,'c','LineWidth',4);
% title('Errors (m)');
% legend('Err_x','Err_y','Err_z');
% xlabel('Time (sec)','FontSize',10);
% grid
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% subplot(3,2,3);
%plot(Val_COM5(1,:),Val_COM5(2,:),'b','LineWidth',2);
plot(Val_COM5(1,:),X_ref,'b','LineWidth',2);
hold on
%plot(Val_COM5(1,:),Val_COM5(3,:),'k','LineWidth',2);
plot(Val_COM5(1,:),Y_ref,'b','LineWidth',2);
hold on
%plot(Val_COM5(1,:),Val_COM5(4,:),'g','LineWidth',2);
plot(Val_COM5(1,:),Z_ref,'b','LineWidth',2);
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
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% subplot(3,2,4);
% plot(Val_COM5(1,:),Val_COM7(1,:),'b','LineWidth',2);
% hold on
% plot(Val_COM5(1,:),Val_COM7(2,:),'r','LineWidth',2);
% hold on
% plot(Val_COM5(1,:),Val_COM7(3,:),'c','LineWidth',2);
% title('X-Y-Z Rotation (rad)');
% legend('Phi','Theta','Psi');
% xlabel('Time (sec)','FontSize',10);
% grid
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% subplot(3,2,5);
% plot(Val_COM5(1,:),Val_COM7(7,:),'b','LineWidth',2);
% hold on
% plot(Val_COM5(1,:),Val_COM7(8,:),'r','LineWidth',2);
% hold on
% plot(Val_COM5(1,:),Val_COM7(9,:),'c','LineWidth',2);
% title('Linear Velocities (m/sec)');
% legend('Vel_x','Vel_y','Vel_z');
% xlabel('Time (sec)','FontSize',10);
% grid
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% subplot(3,2,6);
% plot(Val_COM5(1,:),Val_COM7(10,:),'b','LineWidth',2);
% hold on
% plot(Val_COM5(1,:),Val_COM7(11,:),'r','LineWidth',2);
% hold on
% plot(Val_COM5(1,:),Val_COM7(12,:),'c','LineWidth',2);
% title('Rotational Velocities (rad/sec)');
% legend('Vel_{phi}','Vel_{theta}','Vel_{psi}');
% xlabel('Time (sec)','FontSize',10);
% grid
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
