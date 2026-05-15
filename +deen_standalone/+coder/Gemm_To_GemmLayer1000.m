classdef Gemm_To_GemmLayer1000 < nnet.layer.Layer & nnet.layer.Formattable
    % A custom layer auto-generated while importing an ONNX network.
    %#codegen

    %#ok<*PROPLC>
    %#ok<*NBRAK>
    %#ok<*INUSL>
    %#ok<*VARARG>
    properties (Learnable)
        net_0_weight
        net_0_bias
        net_1_weight
        net_1_bias
        net_4_weight
        net_4_bias
        net_5_weight
        net_5_bias
        net_8_weight
        net_8_bias
        net_9_weight
        net_9_bias
        net_12_weight
        net_12_bias
        net_13_weight
        net_13_bias
        net_16_weight
        net_16_bias
        net_17_weight
        net_17_bias
        net_20_weight
        net_20_bias
    end

    properties (State)
    end

    properties
        Vars
        NumDims
    end

    methods(Static, Hidden)
        % Specify the properties of the class that will not be modified
        % after the first assignment.
        function p = matlabCodegenNontunableProperties(~)
            p = {
                % Constants, i.e., Vars, NumDims and all learnables and states
                'Vars'
                'NumDims'
                };
        end
    end


    methods(Static, Hidden)
        % Instantiate a codegenable layer instance from a MATLAB layer instance
        function this_cg = matlabCodegenToRedirected(mlInstance)
            this_cg = deen_standalone.coder.Gemm_To_GemmLayer1000(mlInstance);
        end
        function this_ml = matlabCodegenFromRedirected(cgInstance)
            this_ml = deen_standalone.Gemm_To_GemmLayer1000(cgInstance.Name);
            if isstruct(cgInstance.Vars)
                names = fieldnames(cgInstance.Vars);
                for i=1:numel(names)
                    fieldname = names{i};
                    this_ml.Vars.(fieldname) = dlarray(cgInstance.Vars.(fieldname));
                end
            else
                this_ml.Vars = [];
            end
            this_ml.NumDims = cgInstance.NumDims;
            this_ml.net_0_weight = cgInstance.net_0_weight;
            this_ml.net_0_bias = cgInstance.net_0_bias;
            this_ml.net_1_weight = cgInstance.net_1_weight;
            this_ml.net_1_bias = cgInstance.net_1_bias;
            this_ml.net_4_weight = cgInstance.net_4_weight;
            this_ml.net_4_bias = cgInstance.net_4_bias;
            this_ml.net_5_weight = cgInstance.net_5_weight;
            this_ml.net_5_bias = cgInstance.net_5_bias;
            this_ml.net_8_weight = cgInstance.net_8_weight;
            this_ml.net_8_bias = cgInstance.net_8_bias;
            this_ml.net_9_weight = cgInstance.net_9_weight;
            this_ml.net_9_bias = cgInstance.net_9_bias;
            this_ml.net_12_weight = cgInstance.net_12_weight;
            this_ml.net_12_bias = cgInstance.net_12_bias;
            this_ml.net_13_weight = cgInstance.net_13_weight;
            this_ml.net_13_bias = cgInstance.net_13_bias;
            this_ml.net_16_weight = cgInstance.net_16_weight;
            this_ml.net_16_bias = cgInstance.net_16_bias;
            this_ml.net_17_weight = cgInstance.net_17_weight;
            this_ml.net_17_bias = cgInstance.net_17_bias;
            this_ml.net_20_weight = cgInstance.net_20_weight;
            this_ml.net_20_bias = cgInstance.net_20_bias;
        end
    end

    methods
        function this = Gemm_To_GemmLayer1000(mlInstance)
            this.Name = mlInstance.Name;
            this.NumInputs = 2;
            this.OutputNames = {'energy_score'};
            if isstruct(mlInstance.Vars)
                names = fieldnames(mlInstance.Vars);
                for i=1:numel(names)
                    fieldname = names{i};
                    this.Vars.(fieldname) = deen_standalone.coder.ops.extractIfDlarray(mlInstance.Vars.(fieldname));
                end
            else
                this.Vars = [];
            end

            this.NumDims = mlInstance.NumDims;
            this.net_0_weight = mlInstance.net_0_weight;
            this.net_0_bias = mlInstance.net_0_bias;
            this.net_1_weight = mlInstance.net_1_weight;
            this.net_1_bias = mlInstance.net_1_bias;
            this.net_4_weight = mlInstance.net_4_weight;
            this.net_4_bias = mlInstance.net_4_bias;
            this.net_5_weight = mlInstance.net_5_weight;
            this.net_5_bias = mlInstance.net_5_bias;
            this.net_8_weight = mlInstance.net_8_weight;
            this.net_8_bias = mlInstance.net_8_bias;
            this.net_9_weight = mlInstance.net_9_weight;
            this.net_9_bias = mlInstance.net_9_bias;
            this.net_12_weight = mlInstance.net_12_weight;
            this.net_12_bias = mlInstance.net_12_bias;
            this.net_13_weight = mlInstance.net_13_weight;
            this.net_13_bias = mlInstance.net_13_bias;
            this.net_16_weight = mlInstance.net_16_weight;
            this.net_16_bias = mlInstance.net_16_bias;
            this.net_17_weight = mlInstance.net_17_weight;
            this.net_17_bias = mlInstance.net_17_bias;
            this.net_20_weight = mlInstance.net_20_weight;
            this.net_20_bias = mlInstance.net_20_bias;
        end

        function [energy_score] = predict(this, input_transition__, input_transitionNumDims__)
            if isdlarray(input_transition__)
                input_transition_ = stripdims(input_transition__);
            else
                input_transition_ = input_transition__;
            end
            input_transitionNumDims = numel(input_transitionNumDims__);
            input_transition = deen_standalone.coder.ops.permuteInputVar(input_transition_, ['as-is'], 2);

            [energy_score__, energy_scoreNumDims__] = Gemm_To_GemmGraph1000(this, input_transition, input_transitionNumDims, false);
            energy_score_ = deen_standalone.coder.ops.permuteOutputVar(energy_score__, ['as-is'], 2);

            energy_score = dlarray(single(energy_score_), repmat('U', 1, max(2, coder.const(energy_scoreNumDims__))));
        end

        function [energy_score, energy_scoreNumDims1013] = Gemm_To_GemmGraph1000(this, input_transition, input_transitionNumDims, Training)

            % Execute the operators:
            % Gemm:
            [A1000, B1001, C1002, alpha1003, beta1004, linearNumDims] = deen_standalone.coder.ops.prepareGemmArgs(input_transition, this.net_0_weight, this.net_0_bias, this.Vars.Gemmalpha1001, this.Vars.Gemmbeta1002, 0, 1, this.NumDims.net_0_bias);
            linear = alpha1003*B1001*A1000 + beta1004*C1002;

            % LayerNormalization:
            [layer_norm, layer_normNumDims] = deen_standalone.coder.ops.onnxLayerNormalization(linear, this.net_1_weight, this.net_1_bias, -1, 1.000000e-05, coder.const(linearNumDims));

            % Sigmoid:
            X1005 = dlarray(deen_standalone.coder.ops.extractIfDlarray(layer_norm));
            Y1006 = sigmoid(X1005);
            val_2 = deen_standalone.coder.ops.extractIfDlarray(Y1006);
            val_2NumDims = coder.const(layer_normNumDims);

            % Mul:
            silu = layer_norm .* val_2;
            siluNumDims = max(coder.const(layer_normNumDims), coder.const(val_2NumDims));

            % Gemm:
            [A1007, B1008, C1009, alpha1010, beta1011, linear_1NumDims] = deen_standalone.coder.ops.prepareGemmArgs(silu, this.net_4_weight, this.net_4_bias, this.Vars.Gemmalpha1003, this.Vars.Gemmbeta1004, 0, 1, this.NumDims.net_4_bias);
            linear_1 = alpha1010*B1008*A1007 + beta1011*C1009;

            % LayerNormalization:
            [layer_norm_1, layer_norm_1NumDims] = deen_standalone.coder.ops.onnxLayerNormalization(linear_1, this.net_5_weight, this.net_5_bias, -1, 1.000000e-05, coder.const(linear_1NumDims));

            % Sigmoid:
            X1012 = dlarray(deen_standalone.coder.ops.extractIfDlarray(layer_norm_1));
            Y1013 = sigmoid(X1012);
            val_5 = deen_standalone.coder.ops.extractIfDlarray(Y1013);
            val_5NumDims = coder.const(layer_norm_1NumDims);

            % Mul:
            silu_1 = layer_norm_1 .* val_5;
            silu_1NumDims = max(coder.const(layer_norm_1NumDims), coder.const(val_5NumDims));

            % Gemm:
            [A1014, B1015, C1016, alpha1017, beta1018, linear_2NumDims] = deen_standalone.coder.ops.prepareGemmArgs(silu_1, this.net_8_weight, this.net_8_bias, this.Vars.Gemmalpha1005, this.Vars.Gemmbeta1006, 0, 1, this.NumDims.net_8_bias);
            linear_2 = alpha1017*B1015*A1014 + beta1018*C1016;

            % LayerNormalization:
            [layer_norm_2, layer_norm_2NumDims] = deen_standalone.coder.ops.onnxLayerNormalization(linear_2, this.net_9_weight, this.net_9_bias, -1, 1.000000e-05, coder.const(linear_2NumDims));

            % Sigmoid:
            X1019 = dlarray(deen_standalone.coder.ops.extractIfDlarray(layer_norm_2));
            Y1020 = sigmoid(X1019);
            val_8 = deen_standalone.coder.ops.extractIfDlarray(Y1020);
            val_8NumDims = coder.const(layer_norm_2NumDims);

            % Mul:
            silu_2 = layer_norm_2 .* val_8;
            silu_2NumDims = max(coder.const(layer_norm_2NumDims), coder.const(val_8NumDims));

            % Gemm:
            [A1021, B1022, C1023, alpha1024, beta1025, linear_3NumDims] = deen_standalone.coder.ops.prepareGemmArgs(silu_2, this.net_12_weight, this.net_12_bias, this.Vars.Gemmalpha1007, this.Vars.Gemmbeta1008, 0, 1, this.NumDims.net_12_bias);
            linear_3 = alpha1024*B1022*A1021 + beta1025*C1023;

            % LayerNormalization:
            [layer_norm_3, layer_norm_3NumDims] = deen_standalone.coder.ops.onnxLayerNormalization(linear_3, this.net_13_weight, this.net_13_bias, -1, 1.000000e-05, coder.const(linear_3NumDims));

            % Sigmoid:
            X1026 = dlarray(deen_standalone.coder.ops.extractIfDlarray(layer_norm_3));
            Y1027 = sigmoid(X1026);
            val_11 = deen_standalone.coder.ops.extractIfDlarray(Y1027);
            val_11NumDims = coder.const(layer_norm_3NumDims);

            % Mul:
            silu_3 = layer_norm_3 .* val_11;
            silu_3NumDims = max(coder.const(layer_norm_3NumDims), coder.const(val_11NumDims));

            % Gemm:
            [A1028, B1029, C1030, alpha1031, beta1032, linear_4NumDims] = deen_standalone.coder.ops.prepareGemmArgs(silu_3, this.net_16_weight, this.net_16_bias, this.Vars.Gemmalpha1009, this.Vars.Gemmbeta1010, 0, 1, this.NumDims.net_16_bias);
            linear_4 = alpha1031*B1029*A1028 + beta1032*C1030;

            % LayerNormalization:
            [layer_norm_4, layer_norm_4NumDims] = deen_standalone.coder.ops.onnxLayerNormalization(linear_4, this.net_17_weight, this.net_17_bias, -1, 1.000000e-05, coder.const(linear_4NumDims));

            % Sigmoid:
            X1033 = dlarray(deen_standalone.coder.ops.extractIfDlarray(layer_norm_4));
            Y1034 = sigmoid(X1033);
            val_14 = deen_standalone.coder.ops.extractIfDlarray(Y1034);
            val_14NumDims = coder.const(layer_norm_4NumDims);

            % Mul:
            silu_4 = layer_norm_4 .* val_14;
            silu_4NumDims = max(coder.const(layer_norm_4NumDims), coder.const(val_14NumDims));

            % Gemm:
            [A1035, B1036, C1037, alpha1038, beta1039, energy_scoreNumDims] = deen_standalone.coder.ops.prepareGemmArgs(silu_4, this.net_20_weight, this.net_20_bias, this.Vars.Gemmalpha1011, this.Vars.Gemmbeta1012, 0, 1, this.NumDims.net_20_bias);
            energy_score = alpha1038*B1036*A1035 + beta1039*C1037;

            % Set graph output arguments
            energy_scoreNumDims1013 = energy_scoreNumDims;

        end

    end

end