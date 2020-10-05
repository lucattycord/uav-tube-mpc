addpath('../src/')
addpath('../src/utils/')

% fix random seed
rng(0);

%%%%%%������ֵ
%�̶�ֵ

k_e=10^(-5);%������ݺ����ķ���
Jx=2562e-6;%x�����ת������
Jy=2562e-6;
Jz=5124e-6;
m_uav=1.2954;%���˻�����
r=[0.1;0.1;0.01;3];%�ο��㣬�������  phi theta psi z
%���Ʊ������������ں��������

%%��״̬�ռ䷽�̱仯��ֵ
input=4;%������u������
output=4;%�����y������
state=11;%״̬��x������
T=0.2;%sample time
Np=20;%predictive horizon
Nc=8;%control horizon



%%%���״̬�ռ䷽��
%�����Ի�����ɢ�õľ������б����±��Ϊm
%   xm(k+1)=A_m*xm(k)+B_m**u(k)

%A_m
A=eye(6,6);%��λ����
A(1,4)=T;
A(2,5)=T;
A(3,6)=T;

%B_m
B=zeros(6,3);
B(4,1)=T/Jx;
B(5,2)=T/Jy;
B(6,3)=T/Jz;

%C_m
% C=zeros(4,11);
% C(1,1)=1;
% C(2,2)=1;
% C(3,3)=1;
% C(4,7)=1;






% make your own discrete linear system with disturbance

Q = diag([1,1,1,1,1,1]);
R = diag([0.1,0.1,0.1]);

%W_vertex =rand(512,2);
%W_vertex =rand(6,6);

% construct a convex set of disturbance (2dim here)
W= Polyhedron( 'A', [eye(6,6);(-1)*eye(6,6)], 'b', 10^(-4)*[0.0514;0.0582;0.0088;0.5139;0.5822;0.0879;0.0674;0.0566;0.0087;0.6739;0.5658;0.0873]);


% construct disturbance Linear system
disturbance_system = DisturbanceLinearSystem(A, B, Q, R, W);

% constraints on state Xc and input Uc

Xc  =Polyhedron( 'A', [eye(6,6);(-1)*eye(6,6)], 'b',[0.0174;0.0174;0.0174;0.15;0.15;0.15;0.0174;0.0174;0.0174;0.15;0.15;0.15]);
Uc = Polyhedron( 'A', [eye(3,3);(-1)*eye(3,3)], 'b',[0.002;0.002;0.002;0.002;0.002;0.002]);

% create a tube_mpc simulater
% if N_horizon is too small, the path will never reach inside the robust MPI-set X_mpi_robust in time step N_horizon, then the problem becomes infeasible. 
N_horizon = 10;
w_min = -10^(-4)*[0.0674;0.0566;0.0087;0.6739;0.5658;0.0873];
w_max = 10^(-4)*[0.0514;0.0582;0.0088;0.5139;0.5822;0.0879];
mpc = TubeModelPredictiveControl(disturbance_system, Xc, Uc, N_horizon);

% The robust MPC guidances the path inside the robust MPI-set so that the path will reach the robust MPI-set in N_horizon. 
x = [-0.01;-0.01;-0.01;0;0;0];
savedir_name = 'results';
mkdir(savedir_name);

for i = 0:70
    disp(i)%��ӡi��ֵ
    u_next = mpc.solve(x);
    x = disturbance_system.propagate(x, u_next); % additive disturbance is considered inside the method 
%     mpc.show_prediction();
%     saveas(gcf, strcat(savedir_name, '/tmpc_seq', number2string(i), '.png')); % removing this line makes the code much faster
    phi(i+1)=x(1);
    theta(i+1)=x(2);
    psi(i+1)=x(3);
    clf;
end
% subplot(1,3,1);
% plot(phi);
% subplot(1,3,2);
% plot(theta);
% subplot(1,3,3);
% plot(psi);

plot(phi)
