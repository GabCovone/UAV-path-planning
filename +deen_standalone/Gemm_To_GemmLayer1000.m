classdef Gemm_To_GemmLayer1000 < nnet.layer.Layer & nnet.layer.Formattable
    % A custom layer auto-generated while importing an ONNX network.

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
        % Specify the path to the class that will be used for codegen
        function name = matlabCodegenRedirect(~)
            name = 'deen_standalone.coder.Gemm_To_GemmLayer1000';
        end
    end


    methods
        function this = Gemm_To_GemmLayer1000(name)
            this.Name = name;
            this.NumInputs = 2;
            this.OutputNames = {'energy_score'};
        end

        function [energy_score] = predict(this, input_transition, input_transitionNumDims)
            if isdlarray(input_transition)
                input_transition = stripdims(input_transition);
            end
            input_transitionNumDims = numel(input_transitionNumDims);
            input_transition = deen_standalone.ops.permuteInputVar(input_transition, ['as-is'], 2);

            [energy_score, energy_scoreNumDims] = Gemm_To_GemmGraph1000(this, input_transition, input_transitionNumDims, false);
            energy_score = deen_standalone.ops.permuteOutputVar(energy_score, ['as-is'], 2);

            energy_score = dlarray(single(energy_score), repmat('U', 1, max(2, energy_scoreNumDims)));
        end

        function [energy_score] = forward(this, input_transition, input_transitionNumDims)
            if isdlarray(input_transition)
                input_transition = stripdims(input_transition);
            end
            input_transitionNumDims = numel(input_transitionNumDims);
            input_transition = deen_standalone.ops.permuteInputVar(input_transition, ['as-is'], 2);

            [energy_score, energy_scoreNumDims] = Gemm_To_GemmGraph1000(this, input_transition, input_transitionNumDims, true);
            energy_score = deen_standalone.ops.permuteOutputVar(energy_score, ['as-is'], 2);

            energy_score = dlarray(single(energy_score), repmat('U', 1, max(2, energy_scoreNumDims)));
        end

        function [energy_score, energy_scoreNumDims1013] = Gemm_To_GemmGraph1000(this, input_transition, input_transitionNumDims, Training)

            % Execute the operators:
            % Gemm:
            [A, B, C, alpha, beta, linearNumDims] = deen_standalone.ops.prepareGemmArgs(input_transition, this.net_0_weight, this.net_0_bias, this.Vars.Gemmalpha1001, this.Vars.Gemmbeta1002, 0, 1, this.NumDims.net_0_bias);
            linear = alpha*B*A + beta*C;

            % LayerNormalization:
            [layer_norm, layer_normNumDims] = deen_standalone.ops.onnxLayerNormalization(linear, this.net_1_weight, this.net_1_bias, -1, 1.000000e-05, linearNumDims);

            % Sigmoid:
            val_2 = sigmoid(dlarray(layer_norm));
            val_2NumDims = layer_normNumDims;

            % Mul:
            silu = layer_norm .* val_2;
            siluNumDims = max(layer_normNumDims, val_2NumDims);

            % Gemm:
            [A, B, C, alpha, beta, linear_1NumDims] = deen_standalone.ops.prepareGemmArgs(silu, this.net_4_weight, this.net_4_bias, this.Vars.Gemmalpha1003, this.Vars.Gemmbeta1004, 0, 1, this.NumDims.net_4_bias);
            linear_1 = alpha*B*A + beta*C;

            % LayerNormalization:
            [layer_norm_1, layer_norm_1NumDims] = deen_standalone.ops.onnxLayerNormalization(linear_1, this.net_5_weight, this.net_5_bias, -1, 1.000000e-05, linear_1NumDims);

            % Sigmoid:
            val_5 = sigmoid(dlarray(layer_norm_1));
            val_5NumDims = layer_norm_1NumDims;

            % Mul:
            silu_1 = layer_norm_1 .* val_5;
            silu_1NumDims = max(layer_norm_1NumDims, val_5NumDims);

            % Gemm:
            [A, B, C, alpha, beta, linear_2NumDims] = deen_standalone.ops.prepareGemmArgs(silu_1, this.net_8_weight, this.net_8_bias, this.Vars.Gemmalpha1005, this.Vars.Gemmbeta1006, 0, 1, this.NumDims.net_8_bias);
            linear_2 = alpha*B*A + beta*C;

            % LayerNormalization:
            [layer_norm_2, layer_norm_2NumDims] = deen_standalone.ops.onnxLayerNormalization(linear_2, this.net_9_weight, this.net_9_bias, -1, 1.000000e-05, linear_2NumDims);

            % Sigmoid:
            val_8 = sigmoid(dlarray(layer_norm_2));
            val_8NumDims = layer_norm_2NumDims;

            % Mul:
            silu_2 = layer_norm_2 .* val_8;
            silu_2NumDims = max(layer_norm_2NumDims, val_8NumDims);

            % Gemm:
            [A, B, C, alpha, beta, linear_3NumDims] = deen_standalone.ops.prepareGemmArgs(silu_2, this.net_12_weight, this.net_12_bias, this.Vars.Gemmalpha1007, this.Vars.Gemmbeta1008, 0, 1, this.NumDims.net_12_bias);
            linear_3 = alpha*B*A + beta*C;

            % LayerNormalization:
            [layer_norm_3, layer_norm_3NumDims] = deen_standalone.ops.onnxLayerNormalization(linear_3, this.net_13_weight, this.net_13_bias, -1, 1.000000e-05, linear_3NumDims);

            % Sigmoid:
            val_11 = sigmoid(dlarray(layer_norm_3));
            val_11NumDims = layer_norm_3NumDims;

            % Mul:
            silu_3 = layer_norm_3 .* val_11;
            silu_3NumDims = max(layer_norm_3NumDims, val_11NumDims);

            % Gemm:
            [A, B, C, alpha, beta, linear_4NumDims] = deen_standalone.ops.prepareGemmArgs(silu_3, this.net_16_weight, this.net_16_bias, this.Vars.Gemmalpha1009, this.Vars.Gemmbeta1010, 0, 1, this.NumDims.net_16_bias);
            linear_4 = alpha*B*A + beta*C;

            % LayerNormalization:
            [layer_norm_4, layer_norm_4NumDims] = deen_standalone.ops.onnxLayerNormalization(linear_4, this.net_17_weight, this.net_17_bias, -1, 1.000000e-05, linear_4NumDims);

            % Sigmoid:
            val_14 = sigmoid(dlarray(layer_norm_4));
            val_14NumDims = layer_norm_4NumDims;

            % Mul:
            silu_4 = layer_norm_4 .* val_14;
            silu_4NumDims = max(layer_norm_4NumDims, val_14NumDims);

            % Gemm:
            [A, B, C, alpha, beta, energy_scoreNumDims] = deen_standalone.ops.prepareGemmArgs(silu_4, this.net_20_weight, this.net_20_bias, this.Vars.Gemmalpha1011, this.Vars.Gemmbeta1012, 0, 1, this.NumDims.net_20_bias);
            energy_score = alpha*B*A + beta*C;

            % Set graph output arguments
            energy_scoreNumDims1013 = energy_scoreNumDims;

        end

    end

end