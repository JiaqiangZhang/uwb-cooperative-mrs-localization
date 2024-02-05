
�root"_tf_keras_sequential*�{"name": "sequential_43", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": false, "class_name": "Sequential", "config": {"name": "sequential_43", "layers": [{"class_name": "InputLayer", "config": {"batch_input_shape": {"class_name": "__tuple__", "items": [null, 10, 3]}, "dtype": "float32", "sparse": false, "ragged": false, "name": "input_44"}}, {"class_name": "MultiScaleMultiHeadAttention", "config": {"name": "multi_scale_multi_head_attention", "num_heads": 4, "d_model": 32, "num_scales": 3}}, {"class_name": "Flatten", "config": {"name": "flatten_43", "trainable": true, "dtype": "float32", "data_format": "channels_last"}}, {"class_name": "Dense", "config": {"name": "dense_43", "trainable": true, "dtype": "float32", "units": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}]}, "shared_object_id": 6, "input_spec": [{"class_name": "InputSpec", "config": {"dtype": null, "shape": {"class_name": "__tuple__", "items": [null, 10, 3]}, "ndim": 3, "max_ndim": null, "min_ndim": null, "axes": {}}}], "build_input_shape": {"class_name": "TensorShape", "items": [null, 10, 3]}, "is_graph_network": true, "full_save_spec": {"class_name": "__tuple__", "items": [[{"class_name": "TypeSpec", "type_spec": "tf.TensorSpec", "serialized": [{"class_name": "TensorShape", "items": [null, 10, 3]}, "float32", "input_44"]}], {}]}, "save_spec": {"class_name": "TypeSpec", "type_spec": "tf.TensorSpec", "serialized": [{"class_name": "TensorShape", "items": [null, 10, 3]}, "float32", "input_44"]}, "keras_version": "2.12.0", "backend": "tensorflow", "model_config": {"class_name": "Sequential", "config": {"name": "sequential_43", "layers": [{"class_name": "InputLayer", "config": {"batch_input_shape": {"class_name": "__tuple__", "items": [null, 10, 3]}, "dtype": "float32", "sparse": false, "ragged": false, "name": "input_44"}, "shared_object_id": 0}, {"class_name": "MultiScaleMultiHeadAttention", "config": {"name": "multi_scale_multi_head_attention", "num_heads": 4, "d_model": 32, "num_scales": 3}, "shared_object_id": 1}, {"class_name": "Flatten", "config": {"name": "flatten_43", "trainable": true, "dtype": "float32", "data_format": "channels_last"}, "shared_object_id": 2}, {"class_name": "Dense", "config": {"name": "dense_43", "trainable": true, "dtype": "float32", "units": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 3}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 4}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "shared_object_id": 5}]}}, "training_config": {"loss": "mse", "metrics": null, "weighted_metrics": null, "loss_weights": null, "optimizer_config": {"class_name": "Custom>Adam", "config": {"name": "Adam", "weight_decay": null, "clipnorm": null, "global_clipnorm": null, "clipvalue": null, "use_ema": false, "ema_momentum": 0.99, "ema_overwrite_frequency": null, "jit_compile": false, "is_legacy_optimizer": false, "learning_rate": 0.0010000000474974513, "beta_1": 0.9, "beta_2": 0.999, "epsilon": 1e-07, "amsgrad": false}}}}2
�root.layer_with_weights-0"_tf_keras_layer*�{"name": "multi_scale_multi_head_attention", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "MultiScaleMultiHeadAttention", "config": {"name": "multi_scale_multi_head_attention", "num_heads": 4, "d_model": 32, "num_scales": 3}, "shared_object_id": 1, "build_input_shape": {"class_name": "TensorShape", "items": [null, 10, 3]}}2
�root.layer-1"_tf_keras_layer*�{"name": "flatten_43", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "Flatten", "config": {"name": "flatten_43", "trainable": true, "dtype": "float32", "data_format": "channels_last"}, "shared_object_id": 2, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 1, "axes": {}}, "shared_object_id": 8}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 10, 9]}}2
�root.layer_with_weights-1"_tf_keras_layer*�{"name": "dense_43", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "Dense", "config": {"name": "dense_43", "trainable": true, "dtype": "float32", "units": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 3}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 4}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "shared_object_id": 5, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 2, "axes": {"-1": 90}}, "shared_object_id": 9}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 90]}}2
�	X,root.layer_with_weights-0.attention_layers.0"_tf_keras_layer*�	{"name": "multi_head_attention_129", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "MultiHeadAttention", "config": {"name": "multi_head_attention_129", "trainable": true, "dtype": "float32", "num_heads": 4, "key_dim": 8, "value_dim": 8, "dropout": 0.0, "use_bias": true, "output_shape": null, "attention_axes": {"class_name": "__tuple__", "items": [1]}, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 10}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 11}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null, "query_shape": {"class_name": "TensorShape", "items": [null, 10, 3]}, "key_shape": {"class_name": "TensorShape", "items": [null, 10, 3]}, "value_shape": {"class_name": "TensorShape", "items": [null, 10, 3]}}, "shared_object_id": 12, "build_input_shape": {"class_name": "TensorShape", "items": [null, 10, 3]}}2
�	Y,root.layer_with_weights-0.attention_layers.1"_tf_keras_layer*�	{"name": "multi_head_attention_130", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "MultiHeadAttention", "config": {"name": "multi_head_attention_130", "trainable": true, "dtype": "float32", "num_heads": 4, "key_dim": 8, "value_dim": 8, "dropout": 0.0, "use_bias": true, "output_shape": null, "attention_axes": {"class_name": "__tuple__", "items": [1]}, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 13}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 14}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null, "query_shape": {"class_name": "TensorShape", "items": [null, 10, 3]}, "key_shape": {"class_name": "TensorShape", "items": [null, 10, 3]}, "value_shape": {"class_name": "TensorShape", "items": [null, 10, 3]}}, "shared_object_id": 15, "build_input_shape": {"class_name": "TensorShape", "items": [null, 10, 3]}}2
�	Z,root.layer_with_weights-0.attention_layers.2"_tf_keras_layer*�	{"name": "multi_head_attention_131", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "MultiHeadAttention", "config": {"name": "multi_head_attention_131", "trainable": true, "dtype": "float32", "num_heads": 4, "key_dim": 8, "value_dim": 8, "dropout": 0.0, "use_bias": true, "output_shape": null, "attention_axes": {"class_name": "__tuple__", "items": [1]}, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 16}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 17}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null, "query_shape": {"class_name": "TensorShape", "items": [null, 10, 3]}, "key_shape": {"class_name": "TensorShape", "items": [null, 10, 3]}, "value_shape": {"class_name": "TensorShape", "items": [null, 10, 3]}}, "shared_object_id": 18, "build_input_shape": {"class_name": "TensorShape", "items": [null, 10, 3]}}2
�iroot.keras_api.metrics.0"_tf_keras_metric*�{"class_name": "Mean", "name": "loss", "dtype": "float32", "config": {"name": "loss", "dtype": "float32"}, "shared_object_id": 19}2
��9root.layer_with_weights-0.attention_layers.0._query_dense"_tf_keras_layer*�{"name": "query", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "EinsumDense", "config": {"name": "query", "trainable": true, "dtype": "float32", "output_shape": [null, 4, 8], "equation": "abc,cde->abde", "activation": "linear", "bias_axes": "de", "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 20}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 21}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "shared_object_id": 22, "build_input_shape": {"class_name": "TensorShape", "items": [null, 10, 3]}}2
��7root.layer_with_weights-0.attention_layers.0._key_dense"_tf_keras_layer*�{"name": "key", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "EinsumDense", "config": {"name": "key", "trainable": true, "dtype": "float32", "output_shape": [null, 4, 8], "equation": "abc,cde->abde", "activation": "linear", "bias_axes": "de", "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 23}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 24}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "shared_object_id": 25, "build_input_shape": {"class_name": "TensorShape", "items": [null, 10, 3]}}2
��9root.layer_with_weights-0.attention_layers.0._value_dense"_tf_keras_layer*�{"name": "value", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "EinsumDense", "config": {"name": "value", "trainable": true, "dtype": "float32", "output_shape": [null, 4, 8], "equation": "abc,cde->abde", "activation": "linear", "bias_axes": "de", "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 26}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 27}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "shared_object_id": 28, "build_input_shape": {"class_name": "TensorShape", "items": [null, 10, 3]}}2
��5root.layer_with_weights-0.attention_layers.0._softmax"_tf_keras_layer*�{"name": "softmax", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "Softmax", "config": {"name": "softmax", "trainable": true, "dtype": "float32", "axis": {"class_name": "__tuple__", "items": [3]}}, "shared_object_id": 29, "build_input_shape": {"class_name": "TensorShape", "items": [null, 4, 10, 10]}}2
��;root.layer_with_weights-0.attention_layers.0._dropout_layer"_tf_keras_layer*�{"name": "dropout", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "Dropout", "config": {"name": "dropout", "trainable": true, "dtype": "float32", "rate": 0.0, "noise_shape": null, "seed": null}, "shared_object_id": 30, "build_input_shape": {"class_name": "TensorShape", "items": [null, 4, 10, 10]}}2
��:root.layer_with_weights-0.attention_layers.0._output_dense"_tf_keras_layer*�{"name": "attention_output", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "EinsumDense", "config": {"name": "attention_output", "trainable": true, "dtype": "float32", "output_shape": [null, 3], "equation": "abcd,cde->abe", "activation": "linear", "bias_axes": "e", "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 31}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 32}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "shared_object_id": 33, "build_input_shape": {"class_name": "TensorShape", "items": [null, 10, 4, 8]}}2
��9root.layer_with_weights-0.attention_layers.1._query_dense"_tf_keras_layer*�{"name": "query", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "EinsumDense", "config": {"name": "query", "trainable": true, "dtype": "float32", "output_shape": [null, 4, 8], "equation": "abc,cde->abde", "activation": "linear", "bias_axes": "de", "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 34}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 35}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "shared_object_id": 36, "build_input_shape": {"class_name": "TensorShape", "items": [null, 10, 3]}}2
��7root.layer_with_weights-0.attention_layers.1._key_dense"_tf_keras_layer*�{"name": "key", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "EinsumDense", "config": {"name": "key", "trainable": true, "dtype": "float32", "output_shape": [null, 4, 8], "equation": "abc,cde->abde", "activation": "linear", "bias_axes": "de", "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 37}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 38}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "shared_object_id": 39, "build_input_shape": {"class_name": "TensorShape", "items": [null, 10, 3]}}2
��9root.layer_with_weights-0.attention_layers.1._value_dense"_tf_keras_layer*�{"name": "value", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "EinsumDense", "config": {"name": "value", "trainable": true, "dtype": "float32", "output_shape": [null, 4, 8], "equation": "abc,cde->abde", "activation": "linear", "bias_axes": "de", "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 40}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 41}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "shared_object_id": 42, "build_input_shape": {"class_name": "TensorShape", "items": [null, 10, 3]}}2
��5root.layer_with_weights-0.attention_layers.1._softmax"_tf_keras_layer*�{"name": "softmax_1", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "Softmax", "config": {"name": "softmax_1", "trainable": true, "dtype": "float32", "axis": {"class_name": "__tuple__", "items": [3]}}, "shared_object_id": 43, "build_input_shape": {"class_name": "TensorShape", "items": [null, 4, 10, 10]}}2
��;root.layer_with_weights-0.attention_layers.1._dropout_layer"_tf_keras_layer*�{"name": "dropout_1", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "Dropout", "config": {"name": "dropout_1", "trainable": true, "dtype": "float32", "rate": 0.0, "noise_shape": null, "seed": null}, "shared_object_id": 44, "build_input_shape": {"class_name": "TensorShape", "items": [null, 4, 10, 10]}}2
��:root.layer_with_weights-0.attention_layers.1._output_dense"_tf_keras_layer*�{"name": "attention_output", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "EinsumDense", "config": {"name": "attention_output", "trainable": true, "dtype": "float32", "output_shape": [null, 3], "equation": "abcd,cde->abe", "activation": "linear", "bias_axes": "e", "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 45}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 46}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "shared_object_id": 47, "build_input_shape": {"class_name": "TensorShape", "items": [null, 10, 4, 8]}}2
��9root.layer_with_weights-0.attention_layers.2._query_dense"_tf_keras_layer*�{"name": "query", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "EinsumDense", "config": {"name": "query", "trainable": true, "dtype": "float32", "output_shape": [null, 4, 8], "equation": "abc,cde->abde", "activation": "linear", "bias_axes": "de", "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 48}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 49}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "shared_object_id": 50, "build_input_shape": {"class_name": "TensorShape", "items": [null, 10, 3]}}2
��7root.layer_with_weights-0.attention_layers.2._key_dense"_tf_keras_layer*�{"name": "key", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "EinsumDense", "config": {"name": "key", "trainable": true, "dtype": "float32", "output_shape": [null, 4, 8], "equation": "abc,cde->abde", "activation": "linear", "bias_axes": "de", "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 51}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 52}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "shared_object_id": 53, "build_input_shape": {"class_name": "TensorShape", "items": [null, 10, 3]}}2
��9root.layer_with_weights-0.attention_layers.2._value_dense"_tf_keras_layer*�{"name": "value", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "EinsumDense", "config": {"name": "value", "trainable": true, "dtype": "float32", "output_shape": [null, 4, 8], "equation": "abc,cde->abde", "activation": "linear", "bias_axes": "de", "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 54}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 55}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "shared_object_id": 56, "build_input_shape": {"class_name": "TensorShape", "items": [null, 10, 3]}}2
��5root.layer_with_weights-0.attention_layers.2._softmax"_tf_keras_layer*�{"name": "softmax_2", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "Softmax", "config": {"name": "softmax_2", "trainable": true, "dtype": "float32", "axis": {"class_name": "__tuple__", "items": [3]}}, "shared_object_id": 57, "build_input_shape": {"class_name": "TensorShape", "items": [null, 4, 10, 10]}}2
��;root.layer_with_weights-0.attention_layers.2._dropout_layer"_tf_keras_layer*�{"name": "dropout_2", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "Dropout", "config": {"name": "dropout_2", "trainable": true, "dtype": "float32", "rate": 0.0, "noise_shape": null, "seed": null}, "shared_object_id": 58, "build_input_shape": {"class_name": "TensorShape", "items": [null, 4, 10, 10]}}2
��:root.layer_with_weights-0.attention_layers.2._output_dense"_tf_keras_layer*�{"name": "attention_output", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "EinsumDense", "config": {"name": "attention_output", "trainable": true, "dtype": "float32", "output_shape": [null, 3], "equation": "abcd,cde->abe", "activation": "linear", "bias_axes": "e", "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 59}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 60}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "shared_object_id": 61, "build_input_shape": {"class_name": "TensorShape", "items": [null, 10, 4, 8]}}2