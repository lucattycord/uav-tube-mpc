classdef TubeModelPredictiveControl < handle
    
    properties (SetAccess = public)
        sys % linear system with disturbance
        optcon; % optimal contol solver object
        Xc; % state constraint
        Uc; % input constraint
        Xc_robust; % Xc-Z (Pontryagin diff.)
        Xmpi_robust; % MPI set. Note that this is made using Xc-Z and Uc-KZ (instead of Xc and Uc)
        N; % horizon
        solution_cache
    end
    
    methods (Access = public)
        function obj = TubeModelPredictiveControl(sys, Xc, Uc, N)
            %----------approximation of d-inv set--------%
            %create robust X and U constraints, and construct solver using X and U
            Xc_robust = Xc - sys.Z;
            Uc_robust = Uc - sys.K*sys.Z;
            %optcon.reconstruct_ineq_constraint(Xc_robust, Uc_robust)
            optcon = OptimalControler(sys, Xc_robust, Uc_robust, N);

            %robustize Xmpi set and set it as a terminal constraint
            Xmpi_robust = sys.compute_MPIset(Xc_robust, Uc_robust);
            optcon.add_terminal_constraint(Xmpi_robust);

            %fill properteis
            obj.sys = sys;
            obj.optcon = optcon;
            obj.Xc = Xc;
            obj.Uc = Uc;
            obj.Xc_robust = Xc_robust;
            obj.Xmpi_robust = Xmpi_robust;
            obj.N = N;
            obj.solution_cache = [];
        end

        function u_next = solve(obj, x_init)
            % Note that solution of optimal controler is cached in obj.solution_cache
            Xinit = x_init + obj.sys.Z;
            obj.optcon.add_initial_constraint(Xinit);
            [x_nominal_seq, u_nominal_seq] = obj.optcon.solve();

            obj.solution_cache = struct(...
                'x_init', x_init', 'x_nominal_seq', x_nominal_seq, 'u_nominal_seq', u_nominal_seq);

            u_nominal = u_nominal_seq(:, 1);
            u_feedback = obj.sys.K * (x_init - x_nominal_seq(:, 1));
            u_next = u_nominal + u_feedback;
        end

        function [] = show_prediction(obj)
            assert(~isempty(obj.solution_cache), 'can be used only after solved');
            Graphics.show_convex(obj.Xc, 'm');
            Graphics.show_convex(obj.Xc_robust, 'r');
            Graphics.show_convex(obj.Xmpi_robust + obj.sys.Z, [0.3, 0.3, 0.3]);
            %ĩ��״̬Լ������
            % rgb = [0.3, 0.3, 0.3]
            Graphics.show_convex(obj.Xmpi_robust, [0.5, 0.5, 0.5]); % gray
            x_init = obj.solution_cache.x_init;
            scatter(x_init(1), x_init(2), 50, 'bs', 'filled');
            x_nominal_seq = obj.solution_cache.x_nominal_seq;
            Graphics.show_trajectory(x_nominal_seq, 'gs-');
            for j=1:obj.N+1
                Graphics.show_convex(x_nominal_seq(:, j)+obj.sys.Z, 'g', 'FaceAlpha', 0.1);
            end

            leg = legend({'$X_c$', '$X_c\ominus Z$', '$X_f \oplus Z$', '$X_f (X_{mpi})$', 'current state', 'nominal traj.', 'tube'}, 'position', [0.5 0.15 0.1 0.2]);
            set(leg, 'Interpreter', 'latex');
        end
    end
end
