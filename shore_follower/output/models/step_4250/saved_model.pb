��
��
D
AddV2
x"T
y"T
z"T"
Ttype:
2	��
B
AssignVariableOp
resource
value"dtype"
dtypetype�
~
BiasAdd

value"T	
bias"T
output"T" 
Ttype:
2	"-
data_formatstringNHWC:
NHWCNCHW
8
Const
output"dtype"
valuetensor"
dtypetype
�
Conv2D

input"T
filter"T
output"T"
Ttype:	
2"
strides	list(int)"
use_cudnn_on_gpubool(",
paddingstring:
SAMEVALIDEXPLICIT""
explicit_paddings	list(int)
 "-
data_formatstringNHWC:
NHWCNCHW" 
	dilations	list(int)

^
Fill
dims"
index_type

value"T
output"T"	
Ttype"

index_typetype0:
2	
�
FusedBatchNormV3
x"T

scale"U
offset"U	
mean"U
variance"U
y"T

batch_mean"U
batch_variance"U
reserve_space_1"U
reserve_space_2"U
reserve_space_3"U"
Ttype:
2"
Utype:
2"
epsilonfloat%��8"&
exponential_avg_factorfloat%  �?";
data_formatstringNHWC:
NHWCNCHWNDHWCNCDHW"
is_trainingbool(
.
Identity

input"T
output"T"	
Ttype
:
Less
x"T
y"T
z
"
Ttype:
2	
q
MatMul
a"T
b"T
product"T"
transpose_abool( "
transpose_bbool( "
Ttype:

2	
e
MergeV2Checkpoints
checkpoint_prefixes
destination_prefix"
delete_old_dirsbool(�
?
Mul
x"T
y"T
z"T"
Ttype:
2	�

NoOp
M
Pack
values"T*N
output"T"
Nint(0"	
Ttype"
axisint 
C
Placeholder
output"dtype"
dtypetype"
shapeshape:
@
ReadVariableOp
resource
value"dtype"
dtypetype�
@
RealDiv
x"T
y"T
z"T"
Ttype:
2	
E
Relu
features"T
activations"T"
Ttype:
2	
[
Reshape
tensor"T
shape"Tshape
output"T"	
Ttype"
Tshapetype0:
2	
o
	RestoreV2

prefix
tensor_names
shape_and_slices
tensors2dtypes"
dtypes
list(type)(0�
l
SaveV2

prefix
tensor_names
shape_and_slices
tensors2dtypes"
dtypes
list(type)(0�
?
Select
	condition

t"T
e"T
output"T"	
Ttype
P
Shape

input"T
output"out_type"	
Ttype"
out_typetype0:
2	
H
ShardedFilename
basename	
shard

num_shards
filename
9
Softmax
logits"T
softmax"T"
Ttype:
2
�
StatefulPartitionedCall
args2Tin
output2Tout"
Tin
list(type)("
Tout
list(type)("	
ffunc"
configstring "
config_protostring "
executor_typestring �
@
StaticRegexFullMatch	
input

output
"
patternstring
�
StridedSlice

input"T
begin"Index
end"Index
strides"Index
output"T"	
Ttype"
Indextype:
2	"

begin_maskint "
end_maskint "
ellipsis_maskint "
new_axis_maskint "
shrink_axis_maskint 
N

StringJoin
inputs*N

output"
Nint(0"
	separatorstring 
<
Sub
x"T
y"T
z"T"
Ttype:
2	
�
VarHandleOp
resource"
	containerstring "
shared_namestring "
dtypetype"
shapeshape"#
allowed_deviceslist(string)
 �"serve*2.6.22v2.6.1-9-gc2363d6d0258��
�
cnn_model/conv_1/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape:*(
shared_namecnn_model/conv_1/kernel
�
+cnn_model/conv_1/kernel/Read/ReadVariableOpReadVariableOpcnn_model/conv_1/kernel*&
_output_shapes
:*
dtype0
�
cnn_model/conv_1/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:*&
shared_namecnn_model/conv_1/bias
{
)cnn_model/conv_1/bias/Read/ReadVariableOpReadVariableOpcnn_model/conv_1/bias*
_output_shapes
:*
dtype0
�
cnn_model/layer_norm_1/gammaVarHandleOp*
_output_shapes
: *
dtype0*
shape:*-
shared_namecnn_model/layer_norm_1/gamma
�
0cnn_model/layer_norm_1/gamma/Read/ReadVariableOpReadVariableOpcnn_model/layer_norm_1/gamma*
_output_shapes
:*
dtype0
�
cnn_model/layer_norm_1/betaVarHandleOp*
_output_shapes
: *
dtype0*
shape:*,
shared_namecnn_model/layer_norm_1/beta
�
/cnn_model/layer_norm_1/beta/Read/ReadVariableOpReadVariableOpcnn_model/layer_norm_1/beta*
_output_shapes
:*
dtype0
�
cnn_model/conv_2/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape:*(
shared_namecnn_model/conv_2/kernel
�
+cnn_model/conv_2/kernel/Read/ReadVariableOpReadVariableOpcnn_model/conv_2/kernel*&
_output_shapes
:*
dtype0
�
cnn_model/conv_2/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:*&
shared_namecnn_model/conv_2/bias
{
)cnn_model/conv_2/bias/Read/ReadVariableOpReadVariableOpcnn_model/conv_2/bias*
_output_shapes
:*
dtype0
�
cnn_model/layer_norm_2/gammaVarHandleOp*
_output_shapes
: *
dtype0*
shape:*-
shared_namecnn_model/layer_norm_2/gamma
�
0cnn_model/layer_norm_2/gamma/Read/ReadVariableOpReadVariableOpcnn_model/layer_norm_2/gamma*
_output_shapes
:*
dtype0
�
cnn_model/layer_norm_2/betaVarHandleOp*
_output_shapes
: *
dtype0*
shape:*,
shared_namecnn_model/layer_norm_2/beta
�
/cnn_model/layer_norm_2/beta/Read/ReadVariableOpReadVariableOpcnn_model/layer_norm_2/beta*
_output_shapes
:*
dtype0
�
cnn_model/conv_3/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape:*(
shared_namecnn_model/conv_3/kernel
�
+cnn_model/conv_3/kernel/Read/ReadVariableOpReadVariableOpcnn_model/conv_3/kernel*&
_output_shapes
:*
dtype0
�
cnn_model/conv_3/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:*&
shared_namecnn_model/conv_3/bias
{
)cnn_model/conv_3/bias/Read/ReadVariableOpReadVariableOpcnn_model/conv_3/bias*
_output_shapes
:*
dtype0
�
cnn_model/dense/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape:	�d*'
shared_namecnn_model/dense/kernel
�
*cnn_model/dense/kernel/Read/ReadVariableOpReadVariableOpcnn_model/dense/kernel*
_output_shapes
:	�d*
dtype0
�
cnn_model/dense/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:d*%
shared_namecnn_model/dense/bias
y
(cnn_model/dense/bias/Read/ReadVariableOpReadVariableOpcnn_model/dense/bias*
_output_shapes
:d*
dtype0
�
cnn_model/raw_output/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape
:d*,
shared_namecnn_model/raw_output/kernel
�
/cnn_model/raw_output/kernel/Read/ReadVariableOpReadVariableOpcnn_model/raw_output/kernel*
_output_shapes

:d*
dtype0
�
cnn_model/raw_output/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:**
shared_namecnn_model/raw_output/bias
�
-cnn_model/raw_output/bias/Read/ReadVariableOpReadVariableOpcnn_model/raw_output/bias*
_output_shapes
:*
dtype0
Z
ConstConst*
_output_shapes
:*
dtype0*!
valueB"j��B�VC��%C
\
Const_1Const*
_output_shapes
:*
dtype0*!
valueB"��>N�?��?

NoOpNoOp
�%
Const_2Const"/device:CPU:0*
_output_shapes
: *
dtype0*�$
value�$B�$ B�$
�
	conv1
	norm1
	conv2
	norm2
	conv3
flat

dense1
	drop1
	y_

	variables
trainable_variables
regularization_losses
	keras_api

signatures
h

kernel
bias
	variables
trainable_variables
regularization_losses
	keras_api
q
axis
	gamma
beta
	variables
trainable_variables
regularization_losses
	keras_api
h

kernel
bias
	variables
trainable_variables
 regularization_losses
!	keras_api
q
"axis
	#gamma
$beta
%	variables
&trainable_variables
'regularization_losses
(	keras_api
h

)kernel
*bias
+	variables
,trainable_variables
-regularization_losses
.	keras_api
R
/	variables
0trainable_variables
1regularization_losses
2	keras_api
h

3kernel
4bias
5	variables
6trainable_variables
7regularization_losses
8	keras_api
R
9	variables
:trainable_variables
;regularization_losses
<	keras_api
h

=kernel
>bias
?	variables
@trainable_variables
Aregularization_losses
B	keras_api
f
0
1
2
3
4
5
#6
$7
)8
*9
310
411
=12
>13
f
0
1
2
3
4
5
#6
$7
)8
*9
310
411
=12
>13
 
�
Cmetrics

	variables
Dnon_trainable_variables
trainable_variables
regularization_losses

Elayers
Flayer_metrics
Glayer_regularization_losses
 
TR
VARIABLE_VALUEcnn_model/conv_1/kernel'conv1/kernel/.ATTRIBUTES/VARIABLE_VALUE
PN
VARIABLE_VALUEcnn_model/conv_1/bias%conv1/bias/.ATTRIBUTES/VARIABLE_VALUE

0
1

0
1
 
�
Hmetrics
	variables
Inon_trainable_variables
trainable_variables
regularization_losses

Jlayers
Klayer_metrics
Llayer_regularization_losses
 
XV
VARIABLE_VALUEcnn_model/layer_norm_1/gamma&norm1/gamma/.ATTRIBUTES/VARIABLE_VALUE
VT
VARIABLE_VALUEcnn_model/layer_norm_1/beta%norm1/beta/.ATTRIBUTES/VARIABLE_VALUE

0
1

0
1
 
�
Mmetrics
	variables
Nnon_trainable_variables
trainable_variables
regularization_losses

Olayers
Player_metrics
Qlayer_regularization_losses
TR
VARIABLE_VALUEcnn_model/conv_2/kernel'conv2/kernel/.ATTRIBUTES/VARIABLE_VALUE
PN
VARIABLE_VALUEcnn_model/conv_2/bias%conv2/bias/.ATTRIBUTES/VARIABLE_VALUE

0
1

0
1
 
�
Rmetrics
	variables
Snon_trainable_variables
trainable_variables
 regularization_losses

Tlayers
Ulayer_metrics
Vlayer_regularization_losses
 
XV
VARIABLE_VALUEcnn_model/layer_norm_2/gamma&norm2/gamma/.ATTRIBUTES/VARIABLE_VALUE
VT
VARIABLE_VALUEcnn_model/layer_norm_2/beta%norm2/beta/.ATTRIBUTES/VARIABLE_VALUE

#0
$1

#0
$1
 
�
Wmetrics
%	variables
Xnon_trainable_variables
&trainable_variables
'regularization_losses

Ylayers
Zlayer_metrics
[layer_regularization_losses
TR
VARIABLE_VALUEcnn_model/conv_3/kernel'conv3/kernel/.ATTRIBUTES/VARIABLE_VALUE
PN
VARIABLE_VALUEcnn_model/conv_3/bias%conv3/bias/.ATTRIBUTES/VARIABLE_VALUE

)0
*1

)0
*1
 
�
\metrics
+	variables
]non_trainable_variables
,trainable_variables
-regularization_losses

^layers
_layer_metrics
`layer_regularization_losses
 
 
 
�
ametrics
/	variables
bnon_trainable_variables
0trainable_variables
1regularization_losses

clayers
dlayer_metrics
elayer_regularization_losses
TR
VARIABLE_VALUEcnn_model/dense/kernel(dense1/kernel/.ATTRIBUTES/VARIABLE_VALUE
PN
VARIABLE_VALUEcnn_model/dense/bias&dense1/bias/.ATTRIBUTES/VARIABLE_VALUE

30
41

30
41
 
�
fmetrics
5	variables
gnon_trainable_variables
6trainable_variables
7regularization_losses

hlayers
ilayer_metrics
jlayer_regularization_losses
 
 
 
�
kmetrics
9	variables
lnon_trainable_variables
:trainable_variables
;regularization_losses

mlayers
nlayer_metrics
olayer_regularization_losses
US
VARIABLE_VALUEcnn_model/raw_output/kernel$y_/kernel/.ATTRIBUTES/VARIABLE_VALUE
QO
VARIABLE_VALUEcnn_model/raw_output/bias"y_/bias/.ATTRIBUTES/VARIABLE_VALUE

=0
>1

=0
>1
 
�
pmetrics
?	variables
qnon_trainable_variables
@trainable_variables
Aregularization_losses

rlayers
slayer_metrics
tlayer_regularization_losses
 
 
?
0
1
2
3
4
5
6
7
	8
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
�
serving_default_input_1Placeholder*/
_output_shapes
:���������  *
dtype0*$
shape:���������  
�
StatefulPartitionedCallStatefulPartitionedCallserving_default_input_1ConstConst_1cnn_model/conv_1/kernelcnn_model/conv_1/biascnn_model/layer_norm_1/gammacnn_model/layer_norm_1/betacnn_model/conv_2/kernelcnn_model/conv_2/biascnn_model/layer_norm_2/gammacnn_model/layer_norm_2/betacnn_model/conv_3/kernelcnn_model/conv_3/biascnn_model/dense/kernelcnn_model/dense/biascnn_model/raw_output/kernelcnn_model/raw_output/bias*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:���������*0
_read_only_resource_inputs
	
*-
config_proto

CPU

GPU 2J 8� *,
f'R%
#__inference_signature_wrapper_46728
O
saver_filenamePlaceholder*
_output_shapes
: *
dtype0*
shape: 
�
StatefulPartitionedCall_1StatefulPartitionedCallsaver_filename+cnn_model/conv_1/kernel/Read/ReadVariableOp)cnn_model/conv_1/bias/Read/ReadVariableOp0cnn_model/layer_norm_1/gamma/Read/ReadVariableOp/cnn_model/layer_norm_1/beta/Read/ReadVariableOp+cnn_model/conv_2/kernel/Read/ReadVariableOp)cnn_model/conv_2/bias/Read/ReadVariableOp0cnn_model/layer_norm_2/gamma/Read/ReadVariableOp/cnn_model/layer_norm_2/beta/Read/ReadVariableOp+cnn_model/conv_3/kernel/Read/ReadVariableOp)cnn_model/conv_3/bias/Read/ReadVariableOp*cnn_model/dense/kernel/Read/ReadVariableOp(cnn_model/dense/bias/Read/ReadVariableOp/cnn_model/raw_output/kernel/Read/ReadVariableOp-cnn_model/raw_output/bias/Read/ReadVariableOpConst_2*
Tin
2*
Tout
2*
_collective_manager_ids
 *
_output_shapes
: * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8� *'
f"R 
__inference__traced_save_47820
�
StatefulPartitionedCall_2StatefulPartitionedCallsaver_filenamecnn_model/conv_1/kernelcnn_model/conv_1/biascnn_model/layer_norm_1/gammacnn_model/layer_norm_1/betacnn_model/conv_2/kernelcnn_model/conv_2/biascnn_model/layer_norm_2/gammacnn_model/layer_norm_2/betacnn_model/conv_3/kernelcnn_model/conv_3/biascnn_model/dense/kernelcnn_model/dense/biascnn_model/raw_output/kernelcnn_model/raw_output/bias*
Tin
2*
Tout
2*
_collective_manager_ids
 *
_output_shapes
: * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8� **
f%R#
!__inference__traced_restore_47872��
�
^
B__inference_flatten_layer_call_and_return_conditional_losses_46266

inputs
identity_
ConstConst*
_output_shapes
:*
dtype0*
valueB"�����   2
Consth
ReshapeReshapeinputsConst:output:0*
T0*(
_output_shapes
:����������2	
Reshapee
IdentityIdentityReshape:output:0*
T0*(
_output_shapes
:����������2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*.
_input_shapes
:���������:W S
/
_output_shapes
:���������
 
_user_specified_nameinputs
�

�
@__inference_dense_layer_call_and_return_conditional_losses_47706

inputs1
matmul_readvariableop_resource:	�d-
biasadd_readvariableop_resource:d
identity��BiasAdd/ReadVariableOp�MatMul/ReadVariableOp�
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes
:	�d*
dtype02
MatMul/ReadVariableOps
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������d2
MatMul�
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:d*
dtype02
BiasAdd/ReadVariableOp�
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������d2	
BiasAddk
IdentityIdentityBiasAdd:output:0^NoOp*
T0*'
_output_shapes
:���������d2

Identity
NoOpNoOp^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*+
_input_shapes
:����������: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp:P L
(
_output_shapes
:����������
 
_user_specified_nameinputs
�2
�
G__inference_layer_norm_1_layer_call_and_return_conditional_losses_46158

inputs+
mul_4_readvariableop_resource:)
add_readvariableop_resource:
identity��add/ReadVariableOp�mul_4/ReadVariableOpD
ShapeShapeinputs*
T0*
_output_shapes
:2
Shapet
strided_slice/stackConst*
_output_shapes
:*
dtype0*
valueB: 2
strided_slice/stackx
strided_slice/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice/stack_1x
strided_slice/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice/stack_2�
strided_sliceStridedSliceShape:output:0strided_slice/stack:output:0strided_slice/stack_1:output:0strided_slice/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
strided_sliceP
mul/xConst*
_output_shapes
: *
dtype0*
value	B :2
mul/xZ
mulMulmul/x:output:0strided_slice:output:0*
T0*
_output_shapes
: 2
mulx
strided_slice_1/stackConst*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_1/stack|
strided_slice_1/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_1/stack_1|
strided_slice_1/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_1/stack_2�
strided_slice_1StridedSliceShape:output:0strided_slice_1/stack:output:0 strided_slice_1/stack_1:output:0 strided_slice_1/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
strided_slice_1Y
mul_1Mulmul:z:0strided_slice_1:output:0*
T0*
_output_shapes
: 2
mul_1x
strided_slice_2/stackConst*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_2/stack|
strided_slice_2/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_2/stack_1|
strided_slice_2/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_2/stack_2�
strided_slice_2StridedSliceShape:output:0strided_slice_2/stack:output:0 strided_slice_2/stack_1:output:0 strided_slice_2/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
strided_slice_2[
mul_2Mul	mul_1:z:0strided_slice_2:output:0*
T0*
_output_shapes
: 2
mul_2x
strided_slice_3/stackConst*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_3/stack|
strided_slice_3/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_3/stack_1|
strided_slice_3/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_3/stack_2�
strided_slice_3StridedSliceShape:output:0strided_slice_3/stack:output:0 strided_slice_3/stack_1:output:0 strided_slice_3/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
strided_slice_3T
mul_3/xConst*
_output_shapes
: *
dtype0*
value	B :2	
mul_3/xb
mul_3Mulmul_3/x:output:0strided_slice_3:output:0*
T0*
_output_shapes
: 2
mul_3d
Reshape/shape/0Const*
_output_shapes
: *
dtype0*
value	B :2
Reshape/shape/0d
Reshape/shape/3Const*
_output_shapes
: *
dtype0*
value	B :2
Reshape/shape/3�
Reshape/shapePackReshape/shape/0:output:0	mul_2:z:0	mul_3:z:0Reshape/shape/3:output:0*
N*
T0*
_output_shapes
:2
Reshape/shape�
ReshapeReshapeinputsReshape/shape:output:0*
T0*8
_output_shapes&
$:"������������������2	
Reshape]
ones/Less/yConst*
_output_shapes
: *
dtype0*
value
B :�2
ones/Less/y`
	ones/LessLess	mul_2:z:0ones/Less/y:output:0*
T0*
_output_shapes
: 2
	ones/Less[
ones/packedPack	mul_2:z:0*
N*
T0*
_output_shapes
:2
ones/packed]

ones/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *  �?2

ones/Constm
onesFillones/packed:output:0ones/Const:output:0*
T0*#
_output_shapes
:���������2
ones_
zeros/Less/yConst*
_output_shapes
: *
dtype0*
value
B :�2
zeros/Less/yc

zeros/LessLess	mul_2:z:0zeros/Less/y:output:0*
T0*
_output_shapes
: 2

zeros/Less]
zeros/packedPack	mul_2:z:0*
N*
T0*
_output_shapes
:2
zeros/packed_
zeros/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *    2
zeros/Constq
zerosFillzeros/packed:output:0zeros/Const:output:0*
T0*#
_output_shapes
:���������2
zerosQ
ConstConst*
_output_shapes
: *
dtype0*
valueB 2
ConstU
Const_1Const*
_output_shapes
: *
dtype0*
valueB 2	
Const_1�
FusedBatchNormV3FusedBatchNormV3Reshape:output:0ones:output:0zeros:output:0Const:output:0Const_1:output:0*
T0*
U0*x
_output_shapesf
d:"������������������:���������:���������:���������:���������:*
data_formatNCHW*
epsilon%o�:2
FusedBatchNormV3�
	Reshape_1ReshapeFusedBatchNormV3:y:0Shape:output:0*
T0*/
_output_shapes
:���������2
	Reshape_1�
mul_4/ReadVariableOpReadVariableOpmul_4_readvariableop_resource*
_output_shapes
:*
dtype02
mul_4/ReadVariableOp�
mul_4MulReshape_1:output:0mul_4/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
mul_4�
add/ReadVariableOpReadVariableOpadd_readvariableop_resource*
_output_shapes
:*
dtype02
add/ReadVariableOpt
addAddV2	mul_4:z:0add/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
addj
IdentityIdentityadd:z:0^NoOp*
T0*/
_output_shapes
:���������2

Identityz
NoOpNoOp^add/ReadVariableOp^mul_4/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*2
_input_shapes!
:���������: : 2(
add/ReadVariableOpadd/ReadVariableOp2,
mul_4/ReadVariableOpmul_4/ReadVariableOp:W S
/
_output_shapes
:���������
 
_user_specified_nameinputs
�?
�	
!__inference__traced_restore_47872
file_prefixB
(assignvariableop_cnn_model_conv_1_kernel:6
(assignvariableop_1_cnn_model_conv_1_bias:=
/assignvariableop_2_cnn_model_layer_norm_1_gamma:<
.assignvariableop_3_cnn_model_layer_norm_1_beta:D
*assignvariableop_4_cnn_model_conv_2_kernel:6
(assignvariableop_5_cnn_model_conv_2_bias:=
/assignvariableop_6_cnn_model_layer_norm_2_gamma:<
.assignvariableop_7_cnn_model_layer_norm_2_beta:D
*assignvariableop_8_cnn_model_conv_3_kernel:6
(assignvariableop_9_cnn_model_conv_3_bias:=
*assignvariableop_10_cnn_model_dense_kernel:	�d6
(assignvariableop_11_cnn_model_dense_bias:dA
/assignvariableop_12_cnn_model_raw_output_kernel:d;
-assignvariableop_13_cnn_model_raw_output_bias:
identity_15��AssignVariableOp�AssignVariableOp_1�AssignVariableOp_10�AssignVariableOp_11�AssignVariableOp_12�AssignVariableOp_13�AssignVariableOp_2�AssignVariableOp_3�AssignVariableOp_4�AssignVariableOp_5�AssignVariableOp_6�AssignVariableOp_7�AssignVariableOp_8�AssignVariableOp_9�
RestoreV2/tensor_namesConst"/device:CPU:0*
_output_shapes
:*
dtype0*�
value�B�B'conv1/kernel/.ATTRIBUTES/VARIABLE_VALUEB%conv1/bias/.ATTRIBUTES/VARIABLE_VALUEB&norm1/gamma/.ATTRIBUTES/VARIABLE_VALUEB%norm1/beta/.ATTRIBUTES/VARIABLE_VALUEB'conv2/kernel/.ATTRIBUTES/VARIABLE_VALUEB%conv2/bias/.ATTRIBUTES/VARIABLE_VALUEB&norm2/gamma/.ATTRIBUTES/VARIABLE_VALUEB%norm2/beta/.ATTRIBUTES/VARIABLE_VALUEB'conv3/kernel/.ATTRIBUTES/VARIABLE_VALUEB%conv3/bias/.ATTRIBUTES/VARIABLE_VALUEB(dense1/kernel/.ATTRIBUTES/VARIABLE_VALUEB&dense1/bias/.ATTRIBUTES/VARIABLE_VALUEB$y_/kernel/.ATTRIBUTES/VARIABLE_VALUEB"y_/bias/.ATTRIBUTES/VARIABLE_VALUEB_CHECKPOINTABLE_OBJECT_GRAPH2
RestoreV2/tensor_names�
RestoreV2/shape_and_slicesConst"/device:CPU:0*
_output_shapes
:*
dtype0*1
value(B&B B B B B B B B B B B B B B B 2
RestoreV2/shape_and_slices�
	RestoreV2	RestoreV2file_prefixRestoreV2/tensor_names:output:0#RestoreV2/shape_and_slices:output:0"/device:CPU:0*P
_output_shapes>
<:::::::::::::::*
dtypes
22
	RestoreV2g
IdentityIdentityRestoreV2:tensors:0"/device:CPU:0*
T0*
_output_shapes
:2

Identity�
AssignVariableOpAssignVariableOp(assignvariableop_cnn_model_conv_1_kernelIdentity:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOpk

Identity_1IdentityRestoreV2:tensors:1"/device:CPU:0*
T0*
_output_shapes
:2

Identity_1�
AssignVariableOp_1AssignVariableOp(assignvariableop_1_cnn_model_conv_1_biasIdentity_1:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_1k

Identity_2IdentityRestoreV2:tensors:2"/device:CPU:0*
T0*
_output_shapes
:2

Identity_2�
AssignVariableOp_2AssignVariableOp/assignvariableop_2_cnn_model_layer_norm_1_gammaIdentity_2:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_2k

Identity_3IdentityRestoreV2:tensors:3"/device:CPU:0*
T0*
_output_shapes
:2

Identity_3�
AssignVariableOp_3AssignVariableOp.assignvariableop_3_cnn_model_layer_norm_1_betaIdentity_3:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_3k

Identity_4IdentityRestoreV2:tensors:4"/device:CPU:0*
T0*
_output_shapes
:2

Identity_4�
AssignVariableOp_4AssignVariableOp*assignvariableop_4_cnn_model_conv_2_kernelIdentity_4:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_4k

Identity_5IdentityRestoreV2:tensors:5"/device:CPU:0*
T0*
_output_shapes
:2

Identity_5�
AssignVariableOp_5AssignVariableOp(assignvariableop_5_cnn_model_conv_2_biasIdentity_5:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_5k

Identity_6IdentityRestoreV2:tensors:6"/device:CPU:0*
T0*
_output_shapes
:2

Identity_6�
AssignVariableOp_6AssignVariableOp/assignvariableop_6_cnn_model_layer_norm_2_gammaIdentity_6:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_6k

Identity_7IdentityRestoreV2:tensors:7"/device:CPU:0*
T0*
_output_shapes
:2

Identity_7�
AssignVariableOp_7AssignVariableOp.assignvariableop_7_cnn_model_layer_norm_2_betaIdentity_7:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_7k

Identity_8IdentityRestoreV2:tensors:8"/device:CPU:0*
T0*
_output_shapes
:2

Identity_8�
AssignVariableOp_8AssignVariableOp*assignvariableop_8_cnn_model_conv_3_kernelIdentity_8:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_8k

Identity_9IdentityRestoreV2:tensors:9"/device:CPU:0*
T0*
_output_shapes
:2

Identity_9�
AssignVariableOp_9AssignVariableOp(assignvariableop_9_cnn_model_conv_3_biasIdentity_9:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_9n
Identity_10IdentityRestoreV2:tensors:10"/device:CPU:0*
T0*
_output_shapes
:2
Identity_10�
AssignVariableOp_10AssignVariableOp*assignvariableop_10_cnn_model_dense_kernelIdentity_10:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_10n
Identity_11IdentityRestoreV2:tensors:11"/device:CPU:0*
T0*
_output_shapes
:2
Identity_11�
AssignVariableOp_11AssignVariableOp(assignvariableop_11_cnn_model_dense_biasIdentity_11:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_11n
Identity_12IdentityRestoreV2:tensors:12"/device:CPU:0*
T0*
_output_shapes
:2
Identity_12�
AssignVariableOp_12AssignVariableOp/assignvariableop_12_cnn_model_raw_output_kernelIdentity_12:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_12n
Identity_13IdentityRestoreV2:tensors:13"/device:CPU:0*
T0*
_output_shapes
:2
Identity_13�
AssignVariableOp_13AssignVariableOp-assignvariableop_13_cnn_model_raw_output_biasIdentity_13:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_139
NoOpNoOp"/device:CPU:0*
_output_shapes
 2
NoOp�
Identity_14Identityfile_prefix^AssignVariableOp^AssignVariableOp_1^AssignVariableOp_10^AssignVariableOp_11^AssignVariableOp_12^AssignVariableOp_13^AssignVariableOp_2^AssignVariableOp_3^AssignVariableOp_4^AssignVariableOp_5^AssignVariableOp_6^AssignVariableOp_7^AssignVariableOp_8^AssignVariableOp_9^NoOp"/device:CPU:0*
T0*
_output_shapes
: 2
Identity_14f
Identity_15IdentityIdentity_14:output:0^NoOp_1*
T0*
_output_shapes
: 2
Identity_15�
NoOp_1NoOp^AssignVariableOp^AssignVariableOp_1^AssignVariableOp_10^AssignVariableOp_11^AssignVariableOp_12^AssignVariableOp_13^AssignVariableOp_2^AssignVariableOp_3^AssignVariableOp_4^AssignVariableOp_5^AssignVariableOp_6^AssignVariableOp_7^AssignVariableOp_8^AssignVariableOp_9*"
_acd_function_control_output(*
_output_shapes
 2
NoOp_1"#
identity_15Identity_15:output:0*1
_input_shapes 
: : : : : : : : : : : : : : : 2$
AssignVariableOpAssignVariableOp2(
AssignVariableOp_1AssignVariableOp_12*
AssignVariableOp_10AssignVariableOp_102*
AssignVariableOp_11AssignVariableOp_112*
AssignVariableOp_12AssignVariableOp_122*
AssignVariableOp_13AssignVariableOp_132(
AssignVariableOp_2AssignVariableOp_22(
AssignVariableOp_3AssignVariableOp_32(
AssignVariableOp_4AssignVariableOp_42(
AssignVariableOp_5AssignVariableOp_52(
AssignVariableOp_6AssignVariableOp_62(
AssignVariableOp_7AssignVariableOp_72(
AssignVariableOp_8AssignVariableOp_82(
AssignVariableOp_9AssignVariableOp_9:C ?

_output_shapes
: 
%
_user_specified_namefile_prefix
�
a
B__inference_dropout_layer_call_and_return_conditional_losses_46374

inputs
identity�c
dropout/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *�8�?2
dropout/Consts
dropout/MulMulinputsdropout/Const:output:0*
T0*'
_output_shapes
:���������d2
dropout/MulT
dropout/ShapeShapeinputs*
T0*
_output_shapes
:2
dropout/Shape�
$dropout/random_uniform/RandomUniformRandomUniformdropout/Shape:output:0*
T0*'
_output_shapes
:���������d*
dtype02&
$dropout/random_uniform/RandomUniformu
dropout/GreaterEqual/yConst*
_output_shapes
: *
dtype0*
valueB
 *���=2
dropout/GreaterEqual/y�
dropout/GreaterEqualGreaterEqual-dropout/random_uniform/RandomUniform:output:0dropout/GreaterEqual/y:output:0*
T0*'
_output_shapes
:���������d2
dropout/GreaterEqual
dropout/CastCastdropout/GreaterEqual:z:0*

DstT0*

SrcT0
*'
_output_shapes
:���������d2
dropout/Castz
dropout/Mul_1Muldropout/Mul:z:0dropout/Cast:y:0*
T0*'
_output_shapes
:���������d2
dropout/Mul_1e
IdentityIdentitydropout/Mul_1:z:0*
T0*'
_output_shapes
:���������d2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*&
_input_shapes
:���������d:O K
'
_output_shapes
:���������d
 
_user_specified_nameinputs
�

�
@__inference_dense_layer_call_and_return_conditional_losses_46278

inputs1
matmul_readvariableop_resource:	�d-
biasadd_readvariableop_resource:d
identity��BiasAdd/ReadVariableOp�MatMul/ReadVariableOp�
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes
:	�d*
dtype02
MatMul/ReadVariableOps
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������d2
MatMul�
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:d*
dtype02
BiasAdd/ReadVariableOp�
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������d2	
BiasAddk
IdentityIdentityBiasAdd:output:0^NoOp*
T0*'
_output_shapes
:���������d2

Identity
NoOpNoOp^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*+
_input_shapes
:����������: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp:P L
(
_output_shapes
:����������
 
_user_specified_nameinputs
�
a
B__inference_dropout_layer_call_and_return_conditional_losses_47733

inputs
identity�c
dropout/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *�8�?2
dropout/Consts
dropout/MulMulinputsdropout/Const:output:0*
T0*'
_output_shapes
:���������d2
dropout/MulT
dropout/ShapeShapeinputs*
T0*
_output_shapes
:2
dropout/Shape�
$dropout/random_uniform/RandomUniformRandomUniformdropout/Shape:output:0*
T0*'
_output_shapes
:���������d*
dtype02&
$dropout/random_uniform/RandomUniformu
dropout/GreaterEqual/yConst*
_output_shapes
: *
dtype0*
valueB
 *���=2
dropout/GreaterEqual/y�
dropout/GreaterEqualGreaterEqual-dropout/random_uniform/RandomUniform:output:0dropout/GreaterEqual/y:output:0*
T0*'
_output_shapes
:���������d2
dropout/GreaterEqual
dropout/CastCastdropout/GreaterEqual:z:0*

DstT0*

SrcT0
*'
_output_shapes
:���������d2
dropout/Castz
dropout/Mul_1Muldropout/Mul:z:0dropout/Cast:y:0*
T0*'
_output_shapes
:���������d2
dropout/Mul_1e
IdentityIdentitydropout/Mul_1:z:0*
T0*'
_output_shapes
:���������d2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*&
_input_shapes
:���������d:O K
'
_output_shapes
:���������d
 
_user_specified_nameinputs
�
�
A__inference_conv_1_layer_call_and_return_conditional_losses_47506

inputs8
conv2d_readvariableop_resource:-
biasadd_readvariableop_resource:
identity��BiasAdd/ReadVariableOp�Conv2D/ReadVariableOp�
Conv2D/ReadVariableOpReadVariableOpconv2d_readvariableop_resource*&
_output_shapes
:*
dtype02
Conv2D/ReadVariableOp�
Conv2DConv2DinputsConv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������*
paddingSAME*
strides
2
Conv2D�
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:*
dtype02
BiasAdd/ReadVariableOp�
BiasAddBiasAddConv2D:output:0BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2	
BiasAdd`
ReluReluBiasAdd:output:0*
T0*/
_output_shapes
:���������2
Reluu
IdentityIdentityRelu:activations:0^NoOp*
T0*/
_output_shapes
:���������2

Identity
NoOpNoOp^BiasAdd/ReadVariableOp^Conv2D/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*2
_input_shapes!
:���������  : : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
Conv2D/ReadVariableOpConv2D/ReadVariableOp:W S
/
_output_shapes
:���������  
 
_user_specified_nameinputs
س
�

D__inference_cnn_model_layer_call_and_return_conditional_losses_47330
input_1	
sub_y
	truediv_y?
%conv_1_conv2d_readvariableop_resource:4
&conv_1_biasadd_readvariableop_resource:8
*layer_norm_1_mul_4_readvariableop_resource:6
(layer_norm_1_add_readvariableop_resource:?
%conv_2_conv2d_readvariableop_resource:4
&conv_2_biasadd_readvariableop_resource:8
*layer_norm_2_mul_4_readvariableop_resource:6
(layer_norm_2_add_readvariableop_resource:?
%conv_3_conv2d_readvariableop_resource:4
&conv_3_biasadd_readvariableop_resource:7
$dense_matmul_readvariableop_resource:	�d3
%dense_biasadd_readvariableop_resource:d;
)raw_output_matmul_readvariableop_resource:d8
*raw_output_biasadd_readvariableop_resource:
identity��conv_1/BiasAdd/ReadVariableOp�conv_1/Conv2D/ReadVariableOp�conv_2/BiasAdd/ReadVariableOp�conv_2/Conv2D/ReadVariableOp�conv_3/BiasAdd/ReadVariableOp�conv_3/Conv2D/ReadVariableOp�dense/BiasAdd/ReadVariableOp�dense/MatMul/ReadVariableOp�layer_norm_1/add/ReadVariableOp�!layer_norm_1/mul_4/ReadVariableOp�layer_norm_2/add/ReadVariableOp�!layer_norm_2/mul_4/ReadVariableOp�!raw_output/BiasAdd/ReadVariableOp� raw_output/MatMul/ReadVariableOp[
subSubinput_1sub_y*
T0*/
_output_shapes
:���������  2
subk
truedivRealDivsub:z:0	truediv_y*
T0*/
_output_shapes
:���������  2	
truediv�
conv_1/Conv2D/ReadVariableOpReadVariableOp%conv_1_conv2d_readvariableop_resource*&
_output_shapes
:*
dtype02
conv_1/Conv2D/ReadVariableOp�
conv_1/Conv2DConv2Dtruediv:z:0$conv_1/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������*
paddingSAME*
strides
2
conv_1/Conv2D�
conv_1/BiasAdd/ReadVariableOpReadVariableOp&conv_1_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02
conv_1/BiasAdd/ReadVariableOp�
conv_1/BiasAddBiasAddconv_1/Conv2D:output:0%conv_1/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
conv_1/BiasAddu
conv_1/ReluReluconv_1/BiasAdd:output:0*
T0*/
_output_shapes
:���������2
conv_1/Reluq
layer_norm_1/ShapeShapeconv_1/Relu:activations:0*
T0*
_output_shapes
:2
layer_norm_1/Shape�
 layer_norm_1/strided_slice/stackConst*
_output_shapes
:*
dtype0*
valueB: 2"
 layer_norm_1/strided_slice/stack�
"layer_norm_1/strided_slice/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_1/strided_slice/stack_1�
"layer_norm_1/strided_slice/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_1/strided_slice/stack_2�
layer_norm_1/strided_sliceStridedSlicelayer_norm_1/Shape:output:0)layer_norm_1/strided_slice/stack:output:0+layer_norm_1/strided_slice/stack_1:output:0+layer_norm_1/strided_slice/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_1/strided_slicej
layer_norm_1/mul/xConst*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_1/mul/x�
layer_norm_1/mulMullayer_norm_1/mul/x:output:0#layer_norm_1/strided_slice:output:0*
T0*
_output_shapes
: 2
layer_norm_1/mul�
"layer_norm_1/strided_slice_1/stackConst*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_1/strided_slice_1/stack�
$layer_norm_1/strided_slice_1/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_1/strided_slice_1/stack_1�
$layer_norm_1/strided_slice_1/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_1/strided_slice_1/stack_2�
layer_norm_1/strided_slice_1StridedSlicelayer_norm_1/Shape:output:0+layer_norm_1/strided_slice_1/stack:output:0-layer_norm_1/strided_slice_1/stack_1:output:0-layer_norm_1/strided_slice_1/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_1/strided_slice_1�
layer_norm_1/mul_1Mullayer_norm_1/mul:z:0%layer_norm_1/strided_slice_1:output:0*
T0*
_output_shapes
: 2
layer_norm_1/mul_1�
"layer_norm_1/strided_slice_2/stackConst*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_1/strided_slice_2/stack�
$layer_norm_1/strided_slice_2/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_1/strided_slice_2/stack_1�
$layer_norm_1/strided_slice_2/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_1/strided_slice_2/stack_2�
layer_norm_1/strided_slice_2StridedSlicelayer_norm_1/Shape:output:0+layer_norm_1/strided_slice_2/stack:output:0-layer_norm_1/strided_slice_2/stack_1:output:0-layer_norm_1/strided_slice_2/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_1/strided_slice_2�
layer_norm_1/mul_2Mullayer_norm_1/mul_1:z:0%layer_norm_1/strided_slice_2:output:0*
T0*
_output_shapes
: 2
layer_norm_1/mul_2�
"layer_norm_1/strided_slice_3/stackConst*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_1/strided_slice_3/stack�
$layer_norm_1/strided_slice_3/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_1/strided_slice_3/stack_1�
$layer_norm_1/strided_slice_3/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_1/strided_slice_3/stack_2�
layer_norm_1/strided_slice_3StridedSlicelayer_norm_1/Shape:output:0+layer_norm_1/strided_slice_3/stack:output:0-layer_norm_1/strided_slice_3/stack_1:output:0-layer_norm_1/strided_slice_3/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_1/strided_slice_3n
layer_norm_1/mul_3/xConst*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_1/mul_3/x�
layer_norm_1/mul_3Mullayer_norm_1/mul_3/x:output:0%layer_norm_1/strided_slice_3:output:0*
T0*
_output_shapes
: 2
layer_norm_1/mul_3~
layer_norm_1/Reshape/shape/0Const*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_1/Reshape/shape/0~
layer_norm_1/Reshape/shape/3Const*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_1/Reshape/shape/3�
layer_norm_1/Reshape/shapePack%layer_norm_1/Reshape/shape/0:output:0layer_norm_1/mul_2:z:0layer_norm_1/mul_3:z:0%layer_norm_1/Reshape/shape/3:output:0*
N*
T0*
_output_shapes
:2
layer_norm_1/Reshape/shape�
layer_norm_1/ReshapeReshapeconv_1/Relu:activations:0#layer_norm_1/Reshape/shape:output:0*
T0*8
_output_shapes&
$:"������������������2
layer_norm_1/Reshapew
layer_norm_1/ones/Less/yConst*
_output_shapes
: *
dtype0*
value
B :�2
layer_norm_1/ones/Less/y�
layer_norm_1/ones/LessLesslayer_norm_1/mul_2:z:0!layer_norm_1/ones/Less/y:output:0*
T0*
_output_shapes
: 2
layer_norm_1/ones/Less�
layer_norm_1/ones/packedPacklayer_norm_1/mul_2:z:0*
N*
T0*
_output_shapes
:2
layer_norm_1/ones/packedw
layer_norm_1/ones/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *  �?2
layer_norm_1/ones/Const�
layer_norm_1/onesFill!layer_norm_1/ones/packed:output:0 layer_norm_1/ones/Const:output:0*
T0*#
_output_shapes
:���������2
layer_norm_1/onesy
layer_norm_1/zeros/Less/yConst*
_output_shapes
: *
dtype0*
value
B :�2
layer_norm_1/zeros/Less/y�
layer_norm_1/zeros/LessLesslayer_norm_1/mul_2:z:0"layer_norm_1/zeros/Less/y:output:0*
T0*
_output_shapes
: 2
layer_norm_1/zeros/Less�
layer_norm_1/zeros/packedPacklayer_norm_1/mul_2:z:0*
N*
T0*
_output_shapes
:2
layer_norm_1/zeros/packedy
layer_norm_1/zeros/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *    2
layer_norm_1/zeros/Const�
layer_norm_1/zerosFill"layer_norm_1/zeros/packed:output:0!layer_norm_1/zeros/Const:output:0*
T0*#
_output_shapes
:���������2
layer_norm_1/zerosk
layer_norm_1/ConstConst*
_output_shapes
: *
dtype0*
valueB 2
layer_norm_1/Consto
layer_norm_1/Const_1Const*
_output_shapes
: *
dtype0*
valueB 2
layer_norm_1/Const_1�
layer_norm_1/FusedBatchNormV3FusedBatchNormV3layer_norm_1/Reshape:output:0layer_norm_1/ones:output:0layer_norm_1/zeros:output:0layer_norm_1/Const:output:0layer_norm_1/Const_1:output:0*
T0*
U0*x
_output_shapesf
d:"������������������:���������:���������:���������:���������:*
data_formatNCHW*
epsilon%o�:2
layer_norm_1/FusedBatchNormV3�
layer_norm_1/Reshape_1Reshape!layer_norm_1/FusedBatchNormV3:y:0layer_norm_1/Shape:output:0*
T0*/
_output_shapes
:���������2
layer_norm_1/Reshape_1�
!layer_norm_1/mul_4/ReadVariableOpReadVariableOp*layer_norm_1_mul_4_readvariableop_resource*
_output_shapes
:*
dtype02#
!layer_norm_1/mul_4/ReadVariableOp�
layer_norm_1/mul_4Mullayer_norm_1/Reshape_1:output:0)layer_norm_1/mul_4/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
layer_norm_1/mul_4�
layer_norm_1/add/ReadVariableOpReadVariableOp(layer_norm_1_add_readvariableop_resource*
_output_shapes
:*
dtype02!
layer_norm_1/add/ReadVariableOp�
layer_norm_1/addAddV2layer_norm_1/mul_4:z:0'layer_norm_1/add/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
layer_norm_1/add�
conv_2/Conv2D/ReadVariableOpReadVariableOp%conv_2_conv2d_readvariableop_resource*&
_output_shapes
:*
dtype02
conv_2/Conv2D/ReadVariableOp�
conv_2/Conv2DConv2Dlayer_norm_1/add:z:0$conv_2/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������*
paddingSAME*
strides
2
conv_2/Conv2D�
conv_2/BiasAdd/ReadVariableOpReadVariableOp&conv_2_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02
conv_2/BiasAdd/ReadVariableOp�
conv_2/BiasAddBiasAddconv_2/Conv2D:output:0%conv_2/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
conv_2/BiasAddu
conv_2/ReluReluconv_2/BiasAdd:output:0*
T0*/
_output_shapes
:���������2
conv_2/Reluq
layer_norm_2/ShapeShapeconv_2/Relu:activations:0*
T0*
_output_shapes
:2
layer_norm_2/Shape�
 layer_norm_2/strided_slice/stackConst*
_output_shapes
:*
dtype0*
valueB: 2"
 layer_norm_2/strided_slice/stack�
"layer_norm_2/strided_slice/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_2/strided_slice/stack_1�
"layer_norm_2/strided_slice/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_2/strided_slice/stack_2�
layer_norm_2/strided_sliceStridedSlicelayer_norm_2/Shape:output:0)layer_norm_2/strided_slice/stack:output:0+layer_norm_2/strided_slice/stack_1:output:0+layer_norm_2/strided_slice/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_2/strided_slicej
layer_norm_2/mul/xConst*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_2/mul/x�
layer_norm_2/mulMullayer_norm_2/mul/x:output:0#layer_norm_2/strided_slice:output:0*
T0*
_output_shapes
: 2
layer_norm_2/mul�
"layer_norm_2/strided_slice_1/stackConst*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_2/strided_slice_1/stack�
$layer_norm_2/strided_slice_1/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_2/strided_slice_1/stack_1�
$layer_norm_2/strided_slice_1/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_2/strided_slice_1/stack_2�
layer_norm_2/strided_slice_1StridedSlicelayer_norm_2/Shape:output:0+layer_norm_2/strided_slice_1/stack:output:0-layer_norm_2/strided_slice_1/stack_1:output:0-layer_norm_2/strided_slice_1/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_2/strided_slice_1�
layer_norm_2/mul_1Mullayer_norm_2/mul:z:0%layer_norm_2/strided_slice_1:output:0*
T0*
_output_shapes
: 2
layer_norm_2/mul_1�
"layer_norm_2/strided_slice_2/stackConst*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_2/strided_slice_2/stack�
$layer_norm_2/strided_slice_2/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_2/strided_slice_2/stack_1�
$layer_norm_2/strided_slice_2/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_2/strided_slice_2/stack_2�
layer_norm_2/strided_slice_2StridedSlicelayer_norm_2/Shape:output:0+layer_norm_2/strided_slice_2/stack:output:0-layer_norm_2/strided_slice_2/stack_1:output:0-layer_norm_2/strided_slice_2/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_2/strided_slice_2�
layer_norm_2/mul_2Mullayer_norm_2/mul_1:z:0%layer_norm_2/strided_slice_2:output:0*
T0*
_output_shapes
: 2
layer_norm_2/mul_2�
"layer_norm_2/strided_slice_3/stackConst*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_2/strided_slice_3/stack�
$layer_norm_2/strided_slice_3/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_2/strided_slice_3/stack_1�
$layer_norm_2/strided_slice_3/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_2/strided_slice_3/stack_2�
layer_norm_2/strided_slice_3StridedSlicelayer_norm_2/Shape:output:0+layer_norm_2/strided_slice_3/stack:output:0-layer_norm_2/strided_slice_3/stack_1:output:0-layer_norm_2/strided_slice_3/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_2/strided_slice_3n
layer_norm_2/mul_3/xConst*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_2/mul_3/x�
layer_norm_2/mul_3Mullayer_norm_2/mul_3/x:output:0%layer_norm_2/strided_slice_3:output:0*
T0*
_output_shapes
: 2
layer_norm_2/mul_3~
layer_norm_2/Reshape/shape/0Const*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_2/Reshape/shape/0~
layer_norm_2/Reshape/shape/3Const*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_2/Reshape/shape/3�
layer_norm_2/Reshape/shapePack%layer_norm_2/Reshape/shape/0:output:0layer_norm_2/mul_2:z:0layer_norm_2/mul_3:z:0%layer_norm_2/Reshape/shape/3:output:0*
N*
T0*
_output_shapes
:2
layer_norm_2/Reshape/shape�
layer_norm_2/ReshapeReshapeconv_2/Relu:activations:0#layer_norm_2/Reshape/shape:output:0*
T0*8
_output_shapes&
$:"������������������2
layer_norm_2/Reshapew
layer_norm_2/ones/Less/yConst*
_output_shapes
: *
dtype0*
value
B :�2
layer_norm_2/ones/Less/y�
layer_norm_2/ones/LessLesslayer_norm_2/mul_2:z:0!layer_norm_2/ones/Less/y:output:0*
T0*
_output_shapes
: 2
layer_norm_2/ones/Less�
layer_norm_2/ones/packedPacklayer_norm_2/mul_2:z:0*
N*
T0*
_output_shapes
:2
layer_norm_2/ones/packedw
layer_norm_2/ones/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *  �?2
layer_norm_2/ones/Const�
layer_norm_2/onesFill!layer_norm_2/ones/packed:output:0 layer_norm_2/ones/Const:output:0*
T0*#
_output_shapes
:���������2
layer_norm_2/onesy
layer_norm_2/zeros/Less/yConst*
_output_shapes
: *
dtype0*
value
B :�2
layer_norm_2/zeros/Less/y�
layer_norm_2/zeros/LessLesslayer_norm_2/mul_2:z:0"layer_norm_2/zeros/Less/y:output:0*
T0*
_output_shapes
: 2
layer_norm_2/zeros/Less�
layer_norm_2/zeros/packedPacklayer_norm_2/mul_2:z:0*
N*
T0*
_output_shapes
:2
layer_norm_2/zeros/packedy
layer_norm_2/zeros/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *    2
layer_norm_2/zeros/Const�
layer_norm_2/zerosFill"layer_norm_2/zeros/packed:output:0!layer_norm_2/zeros/Const:output:0*
T0*#
_output_shapes
:���������2
layer_norm_2/zerosk
layer_norm_2/ConstConst*
_output_shapes
: *
dtype0*
valueB 2
layer_norm_2/Consto
layer_norm_2/Const_1Const*
_output_shapes
: *
dtype0*
valueB 2
layer_norm_2/Const_1�
layer_norm_2/FusedBatchNormV3FusedBatchNormV3layer_norm_2/Reshape:output:0layer_norm_2/ones:output:0layer_norm_2/zeros:output:0layer_norm_2/Const:output:0layer_norm_2/Const_1:output:0*
T0*
U0*x
_output_shapesf
d:"������������������:���������:���������:���������:���������:*
data_formatNCHW*
epsilon%o�:2
layer_norm_2/FusedBatchNormV3�
layer_norm_2/Reshape_1Reshape!layer_norm_2/FusedBatchNormV3:y:0layer_norm_2/Shape:output:0*
T0*/
_output_shapes
:���������2
layer_norm_2/Reshape_1�
!layer_norm_2/mul_4/ReadVariableOpReadVariableOp*layer_norm_2_mul_4_readvariableop_resource*
_output_shapes
:*
dtype02#
!layer_norm_2/mul_4/ReadVariableOp�
layer_norm_2/mul_4Mullayer_norm_2/Reshape_1:output:0)layer_norm_2/mul_4/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
layer_norm_2/mul_4�
layer_norm_2/add/ReadVariableOpReadVariableOp(layer_norm_2_add_readvariableop_resource*
_output_shapes
:*
dtype02!
layer_norm_2/add/ReadVariableOp�
layer_norm_2/addAddV2layer_norm_2/mul_4:z:0'layer_norm_2/add/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
layer_norm_2/add�
conv_3/Conv2D/ReadVariableOpReadVariableOp%conv_3_conv2d_readvariableop_resource*&
_output_shapes
:*
dtype02
conv_3/Conv2D/ReadVariableOp�
conv_3/Conv2DConv2Dlayer_norm_2/add:z:0$conv_3/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������*
paddingSAME*
strides
2
conv_3/Conv2D�
conv_3/BiasAdd/ReadVariableOpReadVariableOp&conv_3_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02
conv_3/BiasAdd/ReadVariableOp�
conv_3/BiasAddBiasAddconv_3/Conv2D:output:0%conv_3/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
conv_3/BiasAddu
conv_3/ReluReluconv_3/BiasAdd:output:0*
T0*/
_output_shapes
:���������2
conv_3/Reluo
flatten/ConstConst*
_output_shapes
:*
dtype0*
valueB"�����   2
flatten/Const�
flatten/ReshapeReshapeconv_3/Relu:activations:0flatten/Const:output:0*
T0*(
_output_shapes
:����������2
flatten/Reshape�
dense/MatMul/ReadVariableOpReadVariableOp$dense_matmul_readvariableop_resource*
_output_shapes
:	�d*
dtype02
dense/MatMul/ReadVariableOp�
dense/MatMulMatMulflatten/Reshape:output:0#dense/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������d2
dense/MatMul�
dense/BiasAdd/ReadVariableOpReadVariableOp%dense_biasadd_readvariableop_resource*
_output_shapes
:d*
dtype02
dense/BiasAdd/ReadVariableOp�
dense/BiasAddBiasAdddense/MatMul:product:0$dense/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������d2
dense/BiasAddz
dropout/IdentityIdentitydense/BiasAdd:output:0*
T0*'
_output_shapes
:���������d2
dropout/Identity�
 raw_output/MatMul/ReadVariableOpReadVariableOp)raw_output_matmul_readvariableop_resource*
_output_shapes

:d*
dtype02"
 raw_output/MatMul/ReadVariableOp�
raw_output/MatMulMatMuldropout/Identity:output:0(raw_output/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������2
raw_output/MatMul�
!raw_output/BiasAdd/ReadVariableOpReadVariableOp*raw_output_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02#
!raw_output/BiasAdd/ReadVariableOp�
raw_output/BiasAddBiasAddraw_output/MatMul:product:0)raw_output/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������2
raw_output/BiasAdd�
raw_output/SoftmaxSoftmaxraw_output/BiasAdd:output:0*
T0*'
_output_shapes
:���������2
raw_output/Softmaxw
IdentityIdentityraw_output/Softmax:softmax:0^NoOp*
T0*'
_output_shapes
:���������2

Identity�
NoOpNoOp^conv_1/BiasAdd/ReadVariableOp^conv_1/Conv2D/ReadVariableOp^conv_2/BiasAdd/ReadVariableOp^conv_2/Conv2D/ReadVariableOp^conv_3/BiasAdd/ReadVariableOp^conv_3/Conv2D/ReadVariableOp^dense/BiasAdd/ReadVariableOp^dense/MatMul/ReadVariableOp ^layer_norm_1/add/ReadVariableOp"^layer_norm_1/mul_4/ReadVariableOp ^layer_norm_2/add/ReadVariableOp"^layer_norm_2/mul_4/ReadVariableOp"^raw_output/BiasAdd/ReadVariableOp!^raw_output/MatMul/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*V
_input_shapesE
C:���������  ::: : : : : : : : : : : : : : 2>
conv_1/BiasAdd/ReadVariableOpconv_1/BiasAdd/ReadVariableOp2<
conv_1/Conv2D/ReadVariableOpconv_1/Conv2D/ReadVariableOp2>
conv_2/BiasAdd/ReadVariableOpconv_2/BiasAdd/ReadVariableOp2<
conv_2/Conv2D/ReadVariableOpconv_2/Conv2D/ReadVariableOp2>
conv_3/BiasAdd/ReadVariableOpconv_3/BiasAdd/ReadVariableOp2<
conv_3/Conv2D/ReadVariableOpconv_3/Conv2D/ReadVariableOp2<
dense/BiasAdd/ReadVariableOpdense/BiasAdd/ReadVariableOp2:
dense/MatMul/ReadVariableOpdense/MatMul/ReadVariableOp2B
layer_norm_1/add/ReadVariableOplayer_norm_1/add/ReadVariableOp2F
!layer_norm_1/mul_4/ReadVariableOp!layer_norm_1/mul_4/ReadVariableOp2B
layer_norm_2/add/ReadVariableOplayer_norm_2/add/ReadVariableOp2F
!layer_norm_2/mul_4/ReadVariableOp!layer_norm_2/mul_4/ReadVariableOp2F
!raw_output/BiasAdd/ReadVariableOp!raw_output/BiasAdd/ReadVariableOp2D
 raw_output/MatMul/ReadVariableOp raw_output/MatMul/ReadVariableOp:X T
/
_output_shapes
:���������  
!
_user_specified_name	input_1: 

_output_shapes
:: 

_output_shapes
:
�
C
'__inference_flatten_layer_call_fn_47681

inputs
identity�
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:����������* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8� *K
fFRD
B__inference_flatten_layer_call_and_return_conditional_losses_462662
PartitionedCallm
IdentityIdentityPartitionedCall:output:0*
T0*(
_output_shapes
:����������2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*.
_input_shapes
:���������:W S
/
_output_shapes
:���������
 
_user_specified_nameinputs
�2
�
G__inference_layer_norm_2_layer_call_and_return_conditional_losses_47656

inputs+
mul_4_readvariableop_resource:)
add_readvariableop_resource:
identity��add/ReadVariableOp�mul_4/ReadVariableOpD
ShapeShapeinputs*
T0*
_output_shapes
:2
Shapet
strided_slice/stackConst*
_output_shapes
:*
dtype0*
valueB: 2
strided_slice/stackx
strided_slice/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice/stack_1x
strided_slice/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice/stack_2�
strided_sliceStridedSliceShape:output:0strided_slice/stack:output:0strided_slice/stack_1:output:0strided_slice/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
strided_sliceP
mul/xConst*
_output_shapes
: *
dtype0*
value	B :2
mul/xZ
mulMulmul/x:output:0strided_slice:output:0*
T0*
_output_shapes
: 2
mulx
strided_slice_1/stackConst*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_1/stack|
strided_slice_1/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_1/stack_1|
strided_slice_1/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_1/stack_2�
strided_slice_1StridedSliceShape:output:0strided_slice_1/stack:output:0 strided_slice_1/stack_1:output:0 strided_slice_1/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
strided_slice_1Y
mul_1Mulmul:z:0strided_slice_1:output:0*
T0*
_output_shapes
: 2
mul_1x
strided_slice_2/stackConst*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_2/stack|
strided_slice_2/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_2/stack_1|
strided_slice_2/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_2/stack_2�
strided_slice_2StridedSliceShape:output:0strided_slice_2/stack:output:0 strided_slice_2/stack_1:output:0 strided_slice_2/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
strided_slice_2[
mul_2Mul	mul_1:z:0strided_slice_2:output:0*
T0*
_output_shapes
: 2
mul_2x
strided_slice_3/stackConst*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_3/stack|
strided_slice_3/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_3/stack_1|
strided_slice_3/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_3/stack_2�
strided_slice_3StridedSliceShape:output:0strided_slice_3/stack:output:0 strided_slice_3/stack_1:output:0 strided_slice_3/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
strided_slice_3T
mul_3/xConst*
_output_shapes
: *
dtype0*
value	B :2	
mul_3/xb
mul_3Mulmul_3/x:output:0strided_slice_3:output:0*
T0*
_output_shapes
: 2
mul_3d
Reshape/shape/0Const*
_output_shapes
: *
dtype0*
value	B :2
Reshape/shape/0d
Reshape/shape/3Const*
_output_shapes
: *
dtype0*
value	B :2
Reshape/shape/3�
Reshape/shapePackReshape/shape/0:output:0	mul_2:z:0	mul_3:z:0Reshape/shape/3:output:0*
N*
T0*
_output_shapes
:2
Reshape/shape�
ReshapeReshapeinputsReshape/shape:output:0*
T0*8
_output_shapes&
$:"������������������2	
Reshape]
ones/Less/yConst*
_output_shapes
: *
dtype0*
value
B :�2
ones/Less/y`
	ones/LessLess	mul_2:z:0ones/Less/y:output:0*
T0*
_output_shapes
: 2
	ones/Less[
ones/packedPack	mul_2:z:0*
N*
T0*
_output_shapes
:2
ones/packed]

ones/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *  �?2

ones/Constm
onesFillones/packed:output:0ones/Const:output:0*
T0*#
_output_shapes
:���������2
ones_
zeros/Less/yConst*
_output_shapes
: *
dtype0*
value
B :�2
zeros/Less/yc

zeros/LessLess	mul_2:z:0zeros/Less/y:output:0*
T0*
_output_shapes
: 2

zeros/Less]
zeros/packedPack	mul_2:z:0*
N*
T0*
_output_shapes
:2
zeros/packed_
zeros/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *    2
zeros/Constq
zerosFillzeros/packed:output:0zeros/Const:output:0*
T0*#
_output_shapes
:���������2
zerosQ
ConstConst*
_output_shapes
: *
dtype0*
valueB 2
ConstU
Const_1Const*
_output_shapes
: *
dtype0*
valueB 2	
Const_1�
FusedBatchNormV3FusedBatchNormV3Reshape:output:0ones:output:0zeros:output:0Const:output:0Const_1:output:0*
T0*
U0*x
_output_shapesf
d:"������������������:���������:���������:���������:���������:*
data_formatNCHW*
epsilon%o�:2
FusedBatchNormV3�
	Reshape_1ReshapeFusedBatchNormV3:y:0Shape:output:0*
T0*/
_output_shapes
:���������2
	Reshape_1�
mul_4/ReadVariableOpReadVariableOpmul_4_readvariableop_resource*
_output_shapes
:*
dtype02
mul_4/ReadVariableOp�
mul_4MulReshape_1:output:0mul_4/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
mul_4�
add/ReadVariableOpReadVariableOpadd_readvariableop_resource*
_output_shapes
:*
dtype02
add/ReadVariableOpt
addAddV2	mul_4:z:0add/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
addj
IdentityIdentityadd:z:0^NoOp*
T0*/
_output_shapes
:���������2

Identityz
NoOpNoOp^add/ReadVariableOp^mul_4/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*2
_input_shapes!
:���������: : 2(
add/ReadVariableOpadd/ReadVariableOp2,
mul_4/ReadVariableOpmul_4/ReadVariableOp:W S
/
_output_shapes
:���������
 
_user_specified_nameinputs
Ѽ
�

D__inference_cnn_model_layer_call_and_return_conditional_losses_47181

inputs	
sub_y
	truediv_y?
%conv_1_conv2d_readvariableop_resource:4
&conv_1_biasadd_readvariableop_resource:8
*layer_norm_1_mul_4_readvariableop_resource:6
(layer_norm_1_add_readvariableop_resource:?
%conv_2_conv2d_readvariableop_resource:4
&conv_2_biasadd_readvariableop_resource:8
*layer_norm_2_mul_4_readvariableop_resource:6
(layer_norm_2_add_readvariableop_resource:?
%conv_3_conv2d_readvariableop_resource:4
&conv_3_biasadd_readvariableop_resource:7
$dense_matmul_readvariableop_resource:	�d3
%dense_biasadd_readvariableop_resource:d;
)raw_output_matmul_readvariableop_resource:d8
*raw_output_biasadd_readvariableop_resource:
identity��conv_1/BiasAdd/ReadVariableOp�conv_1/Conv2D/ReadVariableOp�conv_2/BiasAdd/ReadVariableOp�conv_2/Conv2D/ReadVariableOp�conv_3/BiasAdd/ReadVariableOp�conv_3/Conv2D/ReadVariableOp�dense/BiasAdd/ReadVariableOp�dense/MatMul/ReadVariableOp�layer_norm_1/add/ReadVariableOp�!layer_norm_1/mul_4/ReadVariableOp�layer_norm_2/add/ReadVariableOp�!layer_norm_2/mul_4/ReadVariableOp�!raw_output/BiasAdd/ReadVariableOp� raw_output/MatMul/ReadVariableOpZ
subSubinputssub_y*
T0*/
_output_shapes
:���������  2
subk
truedivRealDivsub:z:0	truediv_y*
T0*/
_output_shapes
:���������  2	
truediv�
conv_1/Conv2D/ReadVariableOpReadVariableOp%conv_1_conv2d_readvariableop_resource*&
_output_shapes
:*
dtype02
conv_1/Conv2D/ReadVariableOp�
conv_1/Conv2DConv2Dtruediv:z:0$conv_1/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������*
paddingSAME*
strides
2
conv_1/Conv2D�
conv_1/BiasAdd/ReadVariableOpReadVariableOp&conv_1_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02
conv_1/BiasAdd/ReadVariableOp�
conv_1/BiasAddBiasAddconv_1/Conv2D:output:0%conv_1/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
conv_1/BiasAddu
conv_1/ReluReluconv_1/BiasAdd:output:0*
T0*/
_output_shapes
:���������2
conv_1/Reluq
layer_norm_1/ShapeShapeconv_1/Relu:activations:0*
T0*
_output_shapes
:2
layer_norm_1/Shape�
 layer_norm_1/strided_slice/stackConst*
_output_shapes
:*
dtype0*
valueB: 2"
 layer_norm_1/strided_slice/stack�
"layer_norm_1/strided_slice/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_1/strided_slice/stack_1�
"layer_norm_1/strided_slice/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_1/strided_slice/stack_2�
layer_norm_1/strided_sliceStridedSlicelayer_norm_1/Shape:output:0)layer_norm_1/strided_slice/stack:output:0+layer_norm_1/strided_slice/stack_1:output:0+layer_norm_1/strided_slice/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_1/strided_slicej
layer_norm_1/mul/xConst*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_1/mul/x�
layer_norm_1/mulMullayer_norm_1/mul/x:output:0#layer_norm_1/strided_slice:output:0*
T0*
_output_shapes
: 2
layer_norm_1/mul�
"layer_norm_1/strided_slice_1/stackConst*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_1/strided_slice_1/stack�
$layer_norm_1/strided_slice_1/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_1/strided_slice_1/stack_1�
$layer_norm_1/strided_slice_1/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_1/strided_slice_1/stack_2�
layer_norm_1/strided_slice_1StridedSlicelayer_norm_1/Shape:output:0+layer_norm_1/strided_slice_1/stack:output:0-layer_norm_1/strided_slice_1/stack_1:output:0-layer_norm_1/strided_slice_1/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_1/strided_slice_1�
layer_norm_1/mul_1Mullayer_norm_1/mul:z:0%layer_norm_1/strided_slice_1:output:0*
T0*
_output_shapes
: 2
layer_norm_1/mul_1�
"layer_norm_1/strided_slice_2/stackConst*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_1/strided_slice_2/stack�
$layer_norm_1/strided_slice_2/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_1/strided_slice_2/stack_1�
$layer_norm_1/strided_slice_2/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_1/strided_slice_2/stack_2�
layer_norm_1/strided_slice_2StridedSlicelayer_norm_1/Shape:output:0+layer_norm_1/strided_slice_2/stack:output:0-layer_norm_1/strided_slice_2/stack_1:output:0-layer_norm_1/strided_slice_2/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_1/strided_slice_2�
layer_norm_1/mul_2Mullayer_norm_1/mul_1:z:0%layer_norm_1/strided_slice_2:output:0*
T0*
_output_shapes
: 2
layer_norm_1/mul_2�
"layer_norm_1/strided_slice_3/stackConst*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_1/strided_slice_3/stack�
$layer_norm_1/strided_slice_3/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_1/strided_slice_3/stack_1�
$layer_norm_1/strided_slice_3/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_1/strided_slice_3/stack_2�
layer_norm_1/strided_slice_3StridedSlicelayer_norm_1/Shape:output:0+layer_norm_1/strided_slice_3/stack:output:0-layer_norm_1/strided_slice_3/stack_1:output:0-layer_norm_1/strided_slice_3/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_1/strided_slice_3n
layer_norm_1/mul_3/xConst*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_1/mul_3/x�
layer_norm_1/mul_3Mullayer_norm_1/mul_3/x:output:0%layer_norm_1/strided_slice_3:output:0*
T0*
_output_shapes
: 2
layer_norm_1/mul_3~
layer_norm_1/Reshape/shape/0Const*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_1/Reshape/shape/0~
layer_norm_1/Reshape/shape/3Const*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_1/Reshape/shape/3�
layer_norm_1/Reshape/shapePack%layer_norm_1/Reshape/shape/0:output:0layer_norm_1/mul_2:z:0layer_norm_1/mul_3:z:0%layer_norm_1/Reshape/shape/3:output:0*
N*
T0*
_output_shapes
:2
layer_norm_1/Reshape/shape�
layer_norm_1/ReshapeReshapeconv_1/Relu:activations:0#layer_norm_1/Reshape/shape:output:0*
T0*8
_output_shapes&
$:"������������������2
layer_norm_1/Reshapew
layer_norm_1/ones/Less/yConst*
_output_shapes
: *
dtype0*
value
B :�2
layer_norm_1/ones/Less/y�
layer_norm_1/ones/LessLesslayer_norm_1/mul_2:z:0!layer_norm_1/ones/Less/y:output:0*
T0*
_output_shapes
: 2
layer_norm_1/ones/Less�
layer_norm_1/ones/packedPacklayer_norm_1/mul_2:z:0*
N*
T0*
_output_shapes
:2
layer_norm_1/ones/packedw
layer_norm_1/ones/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *  �?2
layer_norm_1/ones/Const�
layer_norm_1/onesFill!layer_norm_1/ones/packed:output:0 layer_norm_1/ones/Const:output:0*
T0*#
_output_shapes
:���������2
layer_norm_1/onesy
layer_norm_1/zeros/Less/yConst*
_output_shapes
: *
dtype0*
value
B :�2
layer_norm_1/zeros/Less/y�
layer_norm_1/zeros/LessLesslayer_norm_1/mul_2:z:0"layer_norm_1/zeros/Less/y:output:0*
T0*
_output_shapes
: 2
layer_norm_1/zeros/Less�
layer_norm_1/zeros/packedPacklayer_norm_1/mul_2:z:0*
N*
T0*
_output_shapes
:2
layer_norm_1/zeros/packedy
layer_norm_1/zeros/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *    2
layer_norm_1/zeros/Const�
layer_norm_1/zerosFill"layer_norm_1/zeros/packed:output:0!layer_norm_1/zeros/Const:output:0*
T0*#
_output_shapes
:���������2
layer_norm_1/zerosk
layer_norm_1/ConstConst*
_output_shapes
: *
dtype0*
valueB 2
layer_norm_1/Consto
layer_norm_1/Const_1Const*
_output_shapes
: *
dtype0*
valueB 2
layer_norm_1/Const_1�
layer_norm_1/FusedBatchNormV3FusedBatchNormV3layer_norm_1/Reshape:output:0layer_norm_1/ones:output:0layer_norm_1/zeros:output:0layer_norm_1/Const:output:0layer_norm_1/Const_1:output:0*
T0*
U0*x
_output_shapesf
d:"������������������:���������:���������:���������:���������:*
data_formatNCHW*
epsilon%o�:2
layer_norm_1/FusedBatchNormV3�
layer_norm_1/Reshape_1Reshape!layer_norm_1/FusedBatchNormV3:y:0layer_norm_1/Shape:output:0*
T0*/
_output_shapes
:���������2
layer_norm_1/Reshape_1�
!layer_norm_1/mul_4/ReadVariableOpReadVariableOp*layer_norm_1_mul_4_readvariableop_resource*
_output_shapes
:*
dtype02#
!layer_norm_1/mul_4/ReadVariableOp�
layer_norm_1/mul_4Mullayer_norm_1/Reshape_1:output:0)layer_norm_1/mul_4/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
layer_norm_1/mul_4�
layer_norm_1/add/ReadVariableOpReadVariableOp(layer_norm_1_add_readvariableop_resource*
_output_shapes
:*
dtype02!
layer_norm_1/add/ReadVariableOp�
layer_norm_1/addAddV2layer_norm_1/mul_4:z:0'layer_norm_1/add/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
layer_norm_1/add�
conv_2/Conv2D/ReadVariableOpReadVariableOp%conv_2_conv2d_readvariableop_resource*&
_output_shapes
:*
dtype02
conv_2/Conv2D/ReadVariableOp�
conv_2/Conv2DConv2Dlayer_norm_1/add:z:0$conv_2/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������*
paddingSAME*
strides
2
conv_2/Conv2D�
conv_2/BiasAdd/ReadVariableOpReadVariableOp&conv_2_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02
conv_2/BiasAdd/ReadVariableOp�
conv_2/BiasAddBiasAddconv_2/Conv2D:output:0%conv_2/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
conv_2/BiasAddu
conv_2/ReluReluconv_2/BiasAdd:output:0*
T0*/
_output_shapes
:���������2
conv_2/Reluq
layer_norm_2/ShapeShapeconv_2/Relu:activations:0*
T0*
_output_shapes
:2
layer_norm_2/Shape�
 layer_norm_2/strided_slice/stackConst*
_output_shapes
:*
dtype0*
valueB: 2"
 layer_norm_2/strided_slice/stack�
"layer_norm_2/strided_slice/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_2/strided_slice/stack_1�
"layer_norm_2/strided_slice/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_2/strided_slice/stack_2�
layer_norm_2/strided_sliceStridedSlicelayer_norm_2/Shape:output:0)layer_norm_2/strided_slice/stack:output:0+layer_norm_2/strided_slice/stack_1:output:0+layer_norm_2/strided_slice/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_2/strided_slicej
layer_norm_2/mul/xConst*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_2/mul/x�
layer_norm_2/mulMullayer_norm_2/mul/x:output:0#layer_norm_2/strided_slice:output:0*
T0*
_output_shapes
: 2
layer_norm_2/mul�
"layer_norm_2/strided_slice_1/stackConst*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_2/strided_slice_1/stack�
$layer_norm_2/strided_slice_1/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_2/strided_slice_1/stack_1�
$layer_norm_2/strided_slice_1/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_2/strided_slice_1/stack_2�
layer_norm_2/strided_slice_1StridedSlicelayer_norm_2/Shape:output:0+layer_norm_2/strided_slice_1/stack:output:0-layer_norm_2/strided_slice_1/stack_1:output:0-layer_norm_2/strided_slice_1/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_2/strided_slice_1�
layer_norm_2/mul_1Mullayer_norm_2/mul:z:0%layer_norm_2/strided_slice_1:output:0*
T0*
_output_shapes
: 2
layer_norm_2/mul_1�
"layer_norm_2/strided_slice_2/stackConst*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_2/strided_slice_2/stack�
$layer_norm_2/strided_slice_2/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_2/strided_slice_2/stack_1�
$layer_norm_2/strided_slice_2/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_2/strided_slice_2/stack_2�
layer_norm_2/strided_slice_2StridedSlicelayer_norm_2/Shape:output:0+layer_norm_2/strided_slice_2/stack:output:0-layer_norm_2/strided_slice_2/stack_1:output:0-layer_norm_2/strided_slice_2/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_2/strided_slice_2�
layer_norm_2/mul_2Mullayer_norm_2/mul_1:z:0%layer_norm_2/strided_slice_2:output:0*
T0*
_output_shapes
: 2
layer_norm_2/mul_2�
"layer_norm_2/strided_slice_3/stackConst*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_2/strided_slice_3/stack�
$layer_norm_2/strided_slice_3/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_2/strided_slice_3/stack_1�
$layer_norm_2/strided_slice_3/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_2/strided_slice_3/stack_2�
layer_norm_2/strided_slice_3StridedSlicelayer_norm_2/Shape:output:0+layer_norm_2/strided_slice_3/stack:output:0-layer_norm_2/strided_slice_3/stack_1:output:0-layer_norm_2/strided_slice_3/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_2/strided_slice_3n
layer_norm_2/mul_3/xConst*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_2/mul_3/x�
layer_norm_2/mul_3Mullayer_norm_2/mul_3/x:output:0%layer_norm_2/strided_slice_3:output:0*
T0*
_output_shapes
: 2
layer_norm_2/mul_3~
layer_norm_2/Reshape/shape/0Const*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_2/Reshape/shape/0~
layer_norm_2/Reshape/shape/3Const*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_2/Reshape/shape/3�
layer_norm_2/Reshape/shapePack%layer_norm_2/Reshape/shape/0:output:0layer_norm_2/mul_2:z:0layer_norm_2/mul_3:z:0%layer_norm_2/Reshape/shape/3:output:0*
N*
T0*
_output_shapes
:2
layer_norm_2/Reshape/shape�
layer_norm_2/ReshapeReshapeconv_2/Relu:activations:0#layer_norm_2/Reshape/shape:output:0*
T0*8
_output_shapes&
$:"������������������2
layer_norm_2/Reshapew
layer_norm_2/ones/Less/yConst*
_output_shapes
: *
dtype0*
value
B :�2
layer_norm_2/ones/Less/y�
layer_norm_2/ones/LessLesslayer_norm_2/mul_2:z:0!layer_norm_2/ones/Less/y:output:0*
T0*
_output_shapes
: 2
layer_norm_2/ones/Less�
layer_norm_2/ones/packedPacklayer_norm_2/mul_2:z:0*
N*
T0*
_output_shapes
:2
layer_norm_2/ones/packedw
layer_norm_2/ones/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *  �?2
layer_norm_2/ones/Const�
layer_norm_2/onesFill!layer_norm_2/ones/packed:output:0 layer_norm_2/ones/Const:output:0*
T0*#
_output_shapes
:���������2
layer_norm_2/onesy
layer_norm_2/zeros/Less/yConst*
_output_shapes
: *
dtype0*
value
B :�2
layer_norm_2/zeros/Less/y�
layer_norm_2/zeros/LessLesslayer_norm_2/mul_2:z:0"layer_norm_2/zeros/Less/y:output:0*
T0*
_output_shapes
: 2
layer_norm_2/zeros/Less�
layer_norm_2/zeros/packedPacklayer_norm_2/mul_2:z:0*
N*
T0*
_output_shapes
:2
layer_norm_2/zeros/packedy
layer_norm_2/zeros/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *    2
layer_norm_2/zeros/Const�
layer_norm_2/zerosFill"layer_norm_2/zeros/packed:output:0!layer_norm_2/zeros/Const:output:0*
T0*#
_output_shapes
:���������2
layer_norm_2/zerosk
layer_norm_2/ConstConst*
_output_shapes
: *
dtype0*
valueB 2
layer_norm_2/Consto
layer_norm_2/Const_1Const*
_output_shapes
: *
dtype0*
valueB 2
layer_norm_2/Const_1�
layer_norm_2/FusedBatchNormV3FusedBatchNormV3layer_norm_2/Reshape:output:0layer_norm_2/ones:output:0layer_norm_2/zeros:output:0layer_norm_2/Const:output:0layer_norm_2/Const_1:output:0*
T0*
U0*x
_output_shapesf
d:"������������������:���������:���������:���������:���������:*
data_formatNCHW*
epsilon%o�:2
layer_norm_2/FusedBatchNormV3�
layer_norm_2/Reshape_1Reshape!layer_norm_2/FusedBatchNormV3:y:0layer_norm_2/Shape:output:0*
T0*/
_output_shapes
:���������2
layer_norm_2/Reshape_1�
!layer_norm_2/mul_4/ReadVariableOpReadVariableOp*layer_norm_2_mul_4_readvariableop_resource*
_output_shapes
:*
dtype02#
!layer_norm_2/mul_4/ReadVariableOp�
layer_norm_2/mul_4Mullayer_norm_2/Reshape_1:output:0)layer_norm_2/mul_4/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
layer_norm_2/mul_4�
layer_norm_2/add/ReadVariableOpReadVariableOp(layer_norm_2_add_readvariableop_resource*
_output_shapes
:*
dtype02!
layer_norm_2/add/ReadVariableOp�
layer_norm_2/addAddV2layer_norm_2/mul_4:z:0'layer_norm_2/add/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
layer_norm_2/add�
conv_3/Conv2D/ReadVariableOpReadVariableOp%conv_3_conv2d_readvariableop_resource*&
_output_shapes
:*
dtype02
conv_3/Conv2D/ReadVariableOp�
conv_3/Conv2DConv2Dlayer_norm_2/add:z:0$conv_3/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������*
paddingSAME*
strides
2
conv_3/Conv2D�
conv_3/BiasAdd/ReadVariableOpReadVariableOp&conv_3_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02
conv_3/BiasAdd/ReadVariableOp�
conv_3/BiasAddBiasAddconv_3/Conv2D:output:0%conv_3/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
conv_3/BiasAddu
conv_3/ReluReluconv_3/BiasAdd:output:0*
T0*/
_output_shapes
:���������2
conv_3/Reluo
flatten/ConstConst*
_output_shapes
:*
dtype0*
valueB"�����   2
flatten/Const�
flatten/ReshapeReshapeconv_3/Relu:activations:0flatten/Const:output:0*
T0*(
_output_shapes
:����������2
flatten/Reshape�
dense/MatMul/ReadVariableOpReadVariableOp$dense_matmul_readvariableop_resource*
_output_shapes
:	�d*
dtype02
dense/MatMul/ReadVariableOp�
dense/MatMulMatMulflatten/Reshape:output:0#dense/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������d2
dense/MatMul�
dense/BiasAdd/ReadVariableOpReadVariableOp%dense_biasadd_readvariableop_resource*
_output_shapes
:d*
dtype02
dense/BiasAdd/ReadVariableOp�
dense/BiasAddBiasAdddense/MatMul:product:0$dense/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������d2
dense/BiasAdds
dropout/dropout/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *�8�?2
dropout/dropout/Const�
dropout/dropout/MulMuldense/BiasAdd:output:0dropout/dropout/Const:output:0*
T0*'
_output_shapes
:���������d2
dropout/dropout/Mult
dropout/dropout/ShapeShapedense/BiasAdd:output:0*
T0*
_output_shapes
:2
dropout/dropout/Shape�
,dropout/dropout/random_uniform/RandomUniformRandomUniformdropout/dropout/Shape:output:0*
T0*'
_output_shapes
:���������d*
dtype02.
,dropout/dropout/random_uniform/RandomUniform�
dropout/dropout/GreaterEqual/yConst*
_output_shapes
: *
dtype0*
valueB
 *���=2 
dropout/dropout/GreaterEqual/y�
dropout/dropout/GreaterEqualGreaterEqual5dropout/dropout/random_uniform/RandomUniform:output:0'dropout/dropout/GreaterEqual/y:output:0*
T0*'
_output_shapes
:���������d2
dropout/dropout/GreaterEqual�
dropout/dropout/CastCast dropout/dropout/GreaterEqual:z:0*

DstT0*

SrcT0
*'
_output_shapes
:���������d2
dropout/dropout/Cast�
dropout/dropout/Mul_1Muldropout/dropout/Mul:z:0dropout/dropout/Cast:y:0*
T0*'
_output_shapes
:���������d2
dropout/dropout/Mul_1�
 raw_output/MatMul/ReadVariableOpReadVariableOp)raw_output_matmul_readvariableop_resource*
_output_shapes

:d*
dtype02"
 raw_output/MatMul/ReadVariableOp�
raw_output/MatMulMatMuldropout/dropout/Mul_1:z:0(raw_output/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������2
raw_output/MatMul�
!raw_output/BiasAdd/ReadVariableOpReadVariableOp*raw_output_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02#
!raw_output/BiasAdd/ReadVariableOp�
raw_output/BiasAddBiasAddraw_output/MatMul:product:0)raw_output/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������2
raw_output/BiasAdd�
raw_output/SoftmaxSoftmaxraw_output/BiasAdd:output:0*
T0*'
_output_shapes
:���������2
raw_output/Softmaxw
IdentityIdentityraw_output/Softmax:softmax:0^NoOp*
T0*'
_output_shapes
:���������2

Identity�
NoOpNoOp^conv_1/BiasAdd/ReadVariableOp^conv_1/Conv2D/ReadVariableOp^conv_2/BiasAdd/ReadVariableOp^conv_2/Conv2D/ReadVariableOp^conv_3/BiasAdd/ReadVariableOp^conv_3/Conv2D/ReadVariableOp^dense/BiasAdd/ReadVariableOp^dense/MatMul/ReadVariableOp ^layer_norm_1/add/ReadVariableOp"^layer_norm_1/mul_4/ReadVariableOp ^layer_norm_2/add/ReadVariableOp"^layer_norm_2/mul_4/ReadVariableOp"^raw_output/BiasAdd/ReadVariableOp!^raw_output/MatMul/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*V
_input_shapesE
C:���������  ::: : : : : : : : : : : : : : 2>
conv_1/BiasAdd/ReadVariableOpconv_1/BiasAdd/ReadVariableOp2<
conv_1/Conv2D/ReadVariableOpconv_1/Conv2D/ReadVariableOp2>
conv_2/BiasAdd/ReadVariableOpconv_2/BiasAdd/ReadVariableOp2<
conv_2/Conv2D/ReadVariableOpconv_2/Conv2D/ReadVariableOp2>
conv_3/BiasAdd/ReadVariableOpconv_3/BiasAdd/ReadVariableOp2<
conv_3/Conv2D/ReadVariableOpconv_3/Conv2D/ReadVariableOp2<
dense/BiasAdd/ReadVariableOpdense/BiasAdd/ReadVariableOp2:
dense/MatMul/ReadVariableOpdense/MatMul/ReadVariableOp2B
layer_norm_1/add/ReadVariableOplayer_norm_1/add/ReadVariableOp2F
!layer_norm_1/mul_4/ReadVariableOp!layer_norm_1/mul_4/ReadVariableOp2B
layer_norm_2/add/ReadVariableOplayer_norm_2/add/ReadVariableOp2F
!layer_norm_2/mul_4/ReadVariableOp!layer_norm_2/mul_4/ReadVariableOp2F
!raw_output/BiasAdd/ReadVariableOp!raw_output/BiasAdd/ReadVariableOp2D
 raw_output/MatMul/ReadVariableOp raw_output/MatMul/ReadVariableOp:W S
/
_output_shapes
:���������  
 
_user_specified_nameinputs: 

_output_shapes
:: 

_output_shapes
:
�
�
)__inference_cnn_model_layer_call_fn_46765
input_1
unknown
	unknown_0#
	unknown_1:
	unknown_2:
	unknown_3:
	unknown_4:#
	unknown_5:
	unknown_6:
	unknown_7:
	unknown_8:#
	unknown_9:

unknown_10:

unknown_11:	�d

unknown_12:d

unknown_13:d

unknown_14:
identity��StatefulPartitionedCall�
StatefulPartitionedCallStatefulPartitionedCallinput_1unknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4	unknown_5	unknown_6	unknown_7	unknown_8	unknown_9
unknown_10
unknown_11
unknown_12
unknown_13
unknown_14*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:���������*0
_read_only_resource_inputs
	
*-
config_proto

CPU

GPU 2J 8� *M
fHRF
D__inference_cnn_model_layer_call_and_return_conditional_losses_463092
StatefulPartitionedCall{
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:���������2

Identityh
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*V
_input_shapesE
C:���������  ::: : : : : : : : : : : : : : 22
StatefulPartitionedCallStatefulPartitionedCall:X T
/
_output_shapes
:���������  
!
_user_specified_name	input_1: 

_output_shapes
:: 

_output_shapes
:
�2
�
G__inference_layer_norm_2_layer_call_and_return_conditional_losses_46237

inputs+
mul_4_readvariableop_resource:)
add_readvariableop_resource:
identity��add/ReadVariableOp�mul_4/ReadVariableOpD
ShapeShapeinputs*
T0*
_output_shapes
:2
Shapet
strided_slice/stackConst*
_output_shapes
:*
dtype0*
valueB: 2
strided_slice/stackx
strided_slice/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice/stack_1x
strided_slice/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice/stack_2�
strided_sliceStridedSliceShape:output:0strided_slice/stack:output:0strided_slice/stack_1:output:0strided_slice/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
strided_sliceP
mul/xConst*
_output_shapes
: *
dtype0*
value	B :2
mul/xZ
mulMulmul/x:output:0strided_slice:output:0*
T0*
_output_shapes
: 2
mulx
strided_slice_1/stackConst*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_1/stack|
strided_slice_1/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_1/stack_1|
strided_slice_1/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_1/stack_2�
strided_slice_1StridedSliceShape:output:0strided_slice_1/stack:output:0 strided_slice_1/stack_1:output:0 strided_slice_1/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
strided_slice_1Y
mul_1Mulmul:z:0strided_slice_1:output:0*
T0*
_output_shapes
: 2
mul_1x
strided_slice_2/stackConst*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_2/stack|
strided_slice_2/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_2/stack_1|
strided_slice_2/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_2/stack_2�
strided_slice_2StridedSliceShape:output:0strided_slice_2/stack:output:0 strided_slice_2/stack_1:output:0 strided_slice_2/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
strided_slice_2[
mul_2Mul	mul_1:z:0strided_slice_2:output:0*
T0*
_output_shapes
: 2
mul_2x
strided_slice_3/stackConst*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_3/stack|
strided_slice_3/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_3/stack_1|
strided_slice_3/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_3/stack_2�
strided_slice_3StridedSliceShape:output:0strided_slice_3/stack:output:0 strided_slice_3/stack_1:output:0 strided_slice_3/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
strided_slice_3T
mul_3/xConst*
_output_shapes
: *
dtype0*
value	B :2	
mul_3/xb
mul_3Mulmul_3/x:output:0strided_slice_3:output:0*
T0*
_output_shapes
: 2
mul_3d
Reshape/shape/0Const*
_output_shapes
: *
dtype0*
value	B :2
Reshape/shape/0d
Reshape/shape/3Const*
_output_shapes
: *
dtype0*
value	B :2
Reshape/shape/3�
Reshape/shapePackReshape/shape/0:output:0	mul_2:z:0	mul_3:z:0Reshape/shape/3:output:0*
N*
T0*
_output_shapes
:2
Reshape/shape�
ReshapeReshapeinputsReshape/shape:output:0*
T0*8
_output_shapes&
$:"������������������2	
Reshape]
ones/Less/yConst*
_output_shapes
: *
dtype0*
value
B :�2
ones/Less/y`
	ones/LessLess	mul_2:z:0ones/Less/y:output:0*
T0*
_output_shapes
: 2
	ones/Less[
ones/packedPack	mul_2:z:0*
N*
T0*
_output_shapes
:2
ones/packed]

ones/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *  �?2

ones/Constm
onesFillones/packed:output:0ones/Const:output:0*
T0*#
_output_shapes
:���������2
ones_
zeros/Less/yConst*
_output_shapes
: *
dtype0*
value
B :�2
zeros/Less/yc

zeros/LessLess	mul_2:z:0zeros/Less/y:output:0*
T0*
_output_shapes
: 2

zeros/Less]
zeros/packedPack	mul_2:z:0*
N*
T0*
_output_shapes
:2
zeros/packed_
zeros/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *    2
zeros/Constq
zerosFillzeros/packed:output:0zeros/Const:output:0*
T0*#
_output_shapes
:���������2
zerosQ
ConstConst*
_output_shapes
: *
dtype0*
valueB 2
ConstU
Const_1Const*
_output_shapes
: *
dtype0*
valueB 2	
Const_1�
FusedBatchNormV3FusedBatchNormV3Reshape:output:0ones:output:0zeros:output:0Const:output:0Const_1:output:0*
T0*
U0*x
_output_shapesf
d:"������������������:���������:���������:���������:���������:*
data_formatNCHW*
epsilon%o�:2
FusedBatchNormV3�
	Reshape_1ReshapeFusedBatchNormV3:y:0Shape:output:0*
T0*/
_output_shapes
:���������2
	Reshape_1�
mul_4/ReadVariableOpReadVariableOpmul_4_readvariableop_resource*
_output_shapes
:*
dtype02
mul_4/ReadVariableOp�
mul_4MulReshape_1:output:0mul_4/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
mul_4�
add/ReadVariableOpReadVariableOpadd_readvariableop_resource*
_output_shapes
:*
dtype02
add/ReadVariableOpt
addAddV2	mul_4:z:0add/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
addj
IdentityIdentityadd:z:0^NoOp*
T0*/
_output_shapes
:���������2

Identityz
NoOpNoOp^add/ReadVariableOp^mul_4/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*2
_input_shapes!
:���������: : 2(
add/ReadVariableOpadd/ReadVariableOp2,
mul_4/ReadVariableOpmul_4/ReadVariableOp:W S
/
_output_shapes
:���������
 
_user_specified_nameinputs
�1
�
D__inference_cnn_model_layer_call_and_return_conditional_losses_46527

inputs	
sub_y
	truediv_y&
conv_1_46489:
conv_1_46491: 
layer_norm_1_46494: 
layer_norm_1_46496:&
conv_2_46499:
conv_2_46501: 
layer_norm_2_46504: 
layer_norm_2_46506:&
conv_3_46509:
conv_3_46511:
dense_46515:	�d
dense_46517:d"
raw_output_46521:d
raw_output_46523:
identity��conv_1/StatefulPartitionedCall�conv_2/StatefulPartitionedCall�conv_3/StatefulPartitionedCall�dense/StatefulPartitionedCall�dropout/StatefulPartitionedCall�$layer_norm_1/StatefulPartitionedCall�$layer_norm_2/StatefulPartitionedCall�"raw_output/StatefulPartitionedCallZ
subSubinputssub_y*
T0*/
_output_shapes
:���������  2
subk
truedivRealDivsub:z:0	truediv_y*
T0*/
_output_shapes
:���������  2	
truediv�
conv_1/StatefulPartitionedCallStatefulPartitionedCalltruediv:z:0conv_1_46489conv_1_46491*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:���������*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8� *J
fERC
A__inference_conv_1_layer_call_and_return_conditional_losses_460962 
conv_1/StatefulPartitionedCall�
$layer_norm_1/StatefulPartitionedCallStatefulPartitionedCall'conv_1/StatefulPartitionedCall:output:0layer_norm_1_46494layer_norm_1_46496*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:���������*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8� *P
fKRI
G__inference_layer_norm_1_layer_call_and_return_conditional_losses_461582&
$layer_norm_1/StatefulPartitionedCall�
conv_2/StatefulPartitionedCallStatefulPartitionedCall-layer_norm_1/StatefulPartitionedCall:output:0conv_2_46499conv_2_46501*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:���������*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8� *J
fERC
A__inference_conv_2_layer_call_and_return_conditional_losses_461752 
conv_2/StatefulPartitionedCall�
$layer_norm_2/StatefulPartitionedCallStatefulPartitionedCall'conv_2/StatefulPartitionedCall:output:0layer_norm_2_46504layer_norm_2_46506*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:���������*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8� *P
fKRI
G__inference_layer_norm_2_layer_call_and_return_conditional_losses_462372&
$layer_norm_2/StatefulPartitionedCall�
conv_3/StatefulPartitionedCallStatefulPartitionedCall-layer_norm_2/StatefulPartitionedCall:output:0conv_3_46509conv_3_46511*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:���������*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8� *J
fERC
A__inference_conv_3_layer_call_and_return_conditional_losses_462542 
conv_3/StatefulPartitionedCall�
flatten/PartitionedCallPartitionedCall'conv_3/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:����������* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8� *K
fFRD
B__inference_flatten_layer_call_and_return_conditional_losses_462662
flatten/PartitionedCall�
dense/StatefulPartitionedCallStatefulPartitionedCall flatten/PartitionedCall:output:0dense_46515dense_46517*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:���������d*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8� *I
fDRB
@__inference_dense_layer_call_and_return_conditional_losses_462782
dense/StatefulPartitionedCall�
dropout/StatefulPartitionedCallStatefulPartitionedCall&dense/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:���������d* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8� *K
fFRD
B__inference_dropout_layer_call_and_return_conditional_losses_463742!
dropout/StatefulPartitionedCall�
"raw_output/StatefulPartitionedCallStatefulPartitionedCall(dropout/StatefulPartitionedCall:output:0raw_output_46521raw_output_46523*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:���������*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8� *N
fIRG
E__inference_raw_output_layer_call_and_return_conditional_losses_463022$
"raw_output/StatefulPartitionedCall�
IdentityIdentity+raw_output/StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:���������2

Identity�
NoOpNoOp^conv_1/StatefulPartitionedCall^conv_2/StatefulPartitionedCall^conv_3/StatefulPartitionedCall^dense/StatefulPartitionedCall ^dropout/StatefulPartitionedCall%^layer_norm_1/StatefulPartitionedCall%^layer_norm_2/StatefulPartitionedCall#^raw_output/StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*V
_input_shapesE
C:���������  ::: : : : : : : : : : : : : : 2@
conv_1/StatefulPartitionedCallconv_1/StatefulPartitionedCall2@
conv_2/StatefulPartitionedCallconv_2/StatefulPartitionedCall2@
conv_3/StatefulPartitionedCallconv_3/StatefulPartitionedCall2>
dense/StatefulPartitionedCalldense/StatefulPartitionedCall2B
dropout/StatefulPartitionedCalldropout/StatefulPartitionedCall2L
$layer_norm_1/StatefulPartitionedCall$layer_norm_1/StatefulPartitionedCall2L
$layer_norm_2/StatefulPartitionedCall$layer_norm_2/StatefulPartitionedCall2H
"raw_output/StatefulPartitionedCall"raw_output/StatefulPartitionedCall:W S
/
_output_shapes
:���������  
 
_user_specified_nameinputs: 

_output_shapes
:: 

_output_shapes
:
�
�
A__inference_conv_3_layer_call_and_return_conditional_losses_46254

inputs8
conv2d_readvariableop_resource:-
biasadd_readvariableop_resource:
identity��BiasAdd/ReadVariableOp�Conv2D/ReadVariableOp�
Conv2D/ReadVariableOpReadVariableOpconv2d_readvariableop_resource*&
_output_shapes
:*
dtype02
Conv2D/ReadVariableOp�
Conv2DConv2DinputsConv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������*
paddingSAME*
strides
2
Conv2D�
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:*
dtype02
BiasAdd/ReadVariableOp�
BiasAddBiasAddConv2D:output:0BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2	
BiasAdd`
ReluReluBiasAdd:output:0*
T0*/
_output_shapes
:���������2
Reluu
IdentityIdentityRelu:activations:0^NoOp*
T0*/
_output_shapes
:���������2

Identity
NoOpNoOp^BiasAdd/ReadVariableOp^Conv2D/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*2
_input_shapes!
:���������: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
Conv2D/ReadVariableOpConv2D/ReadVariableOp:W S
/
_output_shapes
:���������
 
_user_specified_nameinputs
�
�
&__inference_conv_3_layer_call_fn_47665

inputs!
unknown:
	unknown_0:
identity��StatefulPartitionedCall�
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:���������*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8� *J
fERC
A__inference_conv_3_layer_call_and_return_conditional_losses_462542
StatefulPartitionedCall�
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*/
_output_shapes
:���������2

Identityh
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*2
_input_shapes!
:���������: : 22
StatefulPartitionedCallStatefulPartitionedCall:W S
/
_output_shapes
:���������
 
_user_specified_nameinputs
�
�
#__inference_signature_wrapper_46728
input_1
unknown
	unknown_0#
	unknown_1:
	unknown_2:
	unknown_3:
	unknown_4:#
	unknown_5:
	unknown_6:
	unknown_7:
	unknown_8:#
	unknown_9:

unknown_10:

unknown_11:	�d

unknown_12:d

unknown_13:d

unknown_14:
identity��StatefulPartitionedCall�
StatefulPartitionedCallStatefulPartitionedCallinput_1unknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4	unknown_5	unknown_6	unknown_7	unknown_8	unknown_9
unknown_10
unknown_11
unknown_12
unknown_13
unknown_14*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:���������*0
_read_only_resource_inputs
	
*-
config_proto

CPU

GPU 2J 8� *)
f$R"
 __inference__wrapped_model_460742
StatefulPartitionedCall{
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:���������2

Identityh
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*V
_input_shapesE
C:���������  ::: : : : : : : : : : : : : : 22
StatefulPartitionedCallStatefulPartitionedCall:X T
/
_output_shapes
:���������  
!
_user_specified_name	input_1: 

_output_shapes
:: 

_output_shapes
:
�/
�
D__inference_cnn_model_layer_call_and_return_conditional_losses_46309

inputs	
sub_y
	truediv_y&
conv_1_46097:
conv_1_46099: 
layer_norm_1_46159: 
layer_norm_1_46161:&
conv_2_46176:
conv_2_46178: 
layer_norm_2_46238: 
layer_norm_2_46240:&
conv_3_46255:
conv_3_46257:
dense_46279:	�d
dense_46281:d"
raw_output_46303:d
raw_output_46305:
identity��conv_1/StatefulPartitionedCall�conv_2/StatefulPartitionedCall�conv_3/StatefulPartitionedCall�dense/StatefulPartitionedCall�$layer_norm_1/StatefulPartitionedCall�$layer_norm_2/StatefulPartitionedCall�"raw_output/StatefulPartitionedCallZ
subSubinputssub_y*
T0*/
_output_shapes
:���������  2
subk
truedivRealDivsub:z:0	truediv_y*
T0*/
_output_shapes
:���������  2	
truediv�
conv_1/StatefulPartitionedCallStatefulPartitionedCalltruediv:z:0conv_1_46097conv_1_46099*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:���������*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8� *J
fERC
A__inference_conv_1_layer_call_and_return_conditional_losses_460962 
conv_1/StatefulPartitionedCall�
$layer_norm_1/StatefulPartitionedCallStatefulPartitionedCall'conv_1/StatefulPartitionedCall:output:0layer_norm_1_46159layer_norm_1_46161*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:���������*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8� *P
fKRI
G__inference_layer_norm_1_layer_call_and_return_conditional_losses_461582&
$layer_norm_1/StatefulPartitionedCall�
conv_2/StatefulPartitionedCallStatefulPartitionedCall-layer_norm_1/StatefulPartitionedCall:output:0conv_2_46176conv_2_46178*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:���������*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8� *J
fERC
A__inference_conv_2_layer_call_and_return_conditional_losses_461752 
conv_2/StatefulPartitionedCall�
$layer_norm_2/StatefulPartitionedCallStatefulPartitionedCall'conv_2/StatefulPartitionedCall:output:0layer_norm_2_46238layer_norm_2_46240*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:���������*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8� *P
fKRI
G__inference_layer_norm_2_layer_call_and_return_conditional_losses_462372&
$layer_norm_2/StatefulPartitionedCall�
conv_3/StatefulPartitionedCallStatefulPartitionedCall-layer_norm_2/StatefulPartitionedCall:output:0conv_3_46255conv_3_46257*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:���������*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8� *J
fERC
A__inference_conv_3_layer_call_and_return_conditional_losses_462542 
conv_3/StatefulPartitionedCall�
flatten/PartitionedCallPartitionedCall'conv_3/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:����������* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8� *K
fFRD
B__inference_flatten_layer_call_and_return_conditional_losses_462662
flatten/PartitionedCall�
dense/StatefulPartitionedCallStatefulPartitionedCall flatten/PartitionedCall:output:0dense_46279dense_46281*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:���������d*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8� *I
fDRB
@__inference_dense_layer_call_and_return_conditional_losses_462782
dense/StatefulPartitionedCall�
dropout/PartitionedCallPartitionedCall&dense/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:���������d* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8� *K
fFRD
B__inference_dropout_layer_call_and_return_conditional_losses_462892
dropout/PartitionedCall�
"raw_output/StatefulPartitionedCallStatefulPartitionedCall dropout/PartitionedCall:output:0raw_output_46303raw_output_46305*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:���������*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8� *N
fIRG
E__inference_raw_output_layer_call_and_return_conditional_losses_463022$
"raw_output/StatefulPartitionedCall�
IdentityIdentity+raw_output/StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:���������2

Identity�
NoOpNoOp^conv_1/StatefulPartitionedCall^conv_2/StatefulPartitionedCall^conv_3/StatefulPartitionedCall^dense/StatefulPartitionedCall%^layer_norm_1/StatefulPartitionedCall%^layer_norm_2/StatefulPartitionedCall#^raw_output/StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*V
_input_shapesE
C:���������  ::: : : : : : : : : : : : : : 2@
conv_1/StatefulPartitionedCallconv_1/StatefulPartitionedCall2@
conv_2/StatefulPartitionedCallconv_2/StatefulPartitionedCall2@
conv_3/StatefulPartitionedCallconv_3/StatefulPartitionedCall2>
dense/StatefulPartitionedCalldense/StatefulPartitionedCall2L
$layer_norm_1/StatefulPartitionedCall$layer_norm_1/StatefulPartitionedCall2L
$layer_norm_2/StatefulPartitionedCall$layer_norm_2/StatefulPartitionedCall2H
"raw_output/StatefulPartitionedCall"raw_output/StatefulPartitionedCall:W S
/
_output_shapes
:���������  
 
_user_specified_nameinputs: 

_output_shapes
:: 

_output_shapes
:
�
�
E__inference_raw_output_layer_call_and_return_conditional_losses_46302

inputs0
matmul_readvariableop_resource:d-
biasadd_readvariableop_resource:
identity��BiasAdd/ReadVariableOp�MatMul/ReadVariableOp�
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes

:d*
dtype02
MatMul/ReadVariableOps
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������2
MatMul�
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:*
dtype02
BiasAdd/ReadVariableOp�
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������2	
BiasAdda
SoftmaxSoftmaxBiasAdd:output:0*
T0*'
_output_shapes
:���������2	
Softmaxl
IdentityIdentitySoftmax:softmax:0^NoOp*
T0*'
_output_shapes
:���������2

Identity
NoOpNoOp^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime**
_input_shapes
:���������d: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp:O K
'
_output_shapes
:���������d
 
_user_specified_nameinputs
Լ
�

D__inference_cnn_model_layer_call_and_return_conditional_losses_47486
input_1	
sub_y
	truediv_y?
%conv_1_conv2d_readvariableop_resource:4
&conv_1_biasadd_readvariableop_resource:8
*layer_norm_1_mul_4_readvariableop_resource:6
(layer_norm_1_add_readvariableop_resource:?
%conv_2_conv2d_readvariableop_resource:4
&conv_2_biasadd_readvariableop_resource:8
*layer_norm_2_mul_4_readvariableop_resource:6
(layer_norm_2_add_readvariableop_resource:?
%conv_3_conv2d_readvariableop_resource:4
&conv_3_biasadd_readvariableop_resource:7
$dense_matmul_readvariableop_resource:	�d3
%dense_biasadd_readvariableop_resource:d;
)raw_output_matmul_readvariableop_resource:d8
*raw_output_biasadd_readvariableop_resource:
identity��conv_1/BiasAdd/ReadVariableOp�conv_1/Conv2D/ReadVariableOp�conv_2/BiasAdd/ReadVariableOp�conv_2/Conv2D/ReadVariableOp�conv_3/BiasAdd/ReadVariableOp�conv_3/Conv2D/ReadVariableOp�dense/BiasAdd/ReadVariableOp�dense/MatMul/ReadVariableOp�layer_norm_1/add/ReadVariableOp�!layer_norm_1/mul_4/ReadVariableOp�layer_norm_2/add/ReadVariableOp�!layer_norm_2/mul_4/ReadVariableOp�!raw_output/BiasAdd/ReadVariableOp� raw_output/MatMul/ReadVariableOp[
subSubinput_1sub_y*
T0*/
_output_shapes
:���������  2
subk
truedivRealDivsub:z:0	truediv_y*
T0*/
_output_shapes
:���������  2	
truediv�
conv_1/Conv2D/ReadVariableOpReadVariableOp%conv_1_conv2d_readvariableop_resource*&
_output_shapes
:*
dtype02
conv_1/Conv2D/ReadVariableOp�
conv_1/Conv2DConv2Dtruediv:z:0$conv_1/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������*
paddingSAME*
strides
2
conv_1/Conv2D�
conv_1/BiasAdd/ReadVariableOpReadVariableOp&conv_1_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02
conv_1/BiasAdd/ReadVariableOp�
conv_1/BiasAddBiasAddconv_1/Conv2D:output:0%conv_1/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
conv_1/BiasAddu
conv_1/ReluReluconv_1/BiasAdd:output:0*
T0*/
_output_shapes
:���������2
conv_1/Reluq
layer_norm_1/ShapeShapeconv_1/Relu:activations:0*
T0*
_output_shapes
:2
layer_norm_1/Shape�
 layer_norm_1/strided_slice/stackConst*
_output_shapes
:*
dtype0*
valueB: 2"
 layer_norm_1/strided_slice/stack�
"layer_norm_1/strided_slice/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_1/strided_slice/stack_1�
"layer_norm_1/strided_slice/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_1/strided_slice/stack_2�
layer_norm_1/strided_sliceStridedSlicelayer_norm_1/Shape:output:0)layer_norm_1/strided_slice/stack:output:0+layer_norm_1/strided_slice/stack_1:output:0+layer_norm_1/strided_slice/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_1/strided_slicej
layer_norm_1/mul/xConst*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_1/mul/x�
layer_norm_1/mulMullayer_norm_1/mul/x:output:0#layer_norm_1/strided_slice:output:0*
T0*
_output_shapes
: 2
layer_norm_1/mul�
"layer_norm_1/strided_slice_1/stackConst*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_1/strided_slice_1/stack�
$layer_norm_1/strided_slice_1/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_1/strided_slice_1/stack_1�
$layer_norm_1/strided_slice_1/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_1/strided_slice_1/stack_2�
layer_norm_1/strided_slice_1StridedSlicelayer_norm_1/Shape:output:0+layer_norm_1/strided_slice_1/stack:output:0-layer_norm_1/strided_slice_1/stack_1:output:0-layer_norm_1/strided_slice_1/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_1/strided_slice_1�
layer_norm_1/mul_1Mullayer_norm_1/mul:z:0%layer_norm_1/strided_slice_1:output:0*
T0*
_output_shapes
: 2
layer_norm_1/mul_1�
"layer_norm_1/strided_slice_2/stackConst*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_1/strided_slice_2/stack�
$layer_norm_1/strided_slice_2/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_1/strided_slice_2/stack_1�
$layer_norm_1/strided_slice_2/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_1/strided_slice_2/stack_2�
layer_norm_1/strided_slice_2StridedSlicelayer_norm_1/Shape:output:0+layer_norm_1/strided_slice_2/stack:output:0-layer_norm_1/strided_slice_2/stack_1:output:0-layer_norm_1/strided_slice_2/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_1/strided_slice_2�
layer_norm_1/mul_2Mullayer_norm_1/mul_1:z:0%layer_norm_1/strided_slice_2:output:0*
T0*
_output_shapes
: 2
layer_norm_1/mul_2�
"layer_norm_1/strided_slice_3/stackConst*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_1/strided_slice_3/stack�
$layer_norm_1/strided_slice_3/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_1/strided_slice_3/stack_1�
$layer_norm_1/strided_slice_3/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_1/strided_slice_3/stack_2�
layer_norm_1/strided_slice_3StridedSlicelayer_norm_1/Shape:output:0+layer_norm_1/strided_slice_3/stack:output:0-layer_norm_1/strided_slice_3/stack_1:output:0-layer_norm_1/strided_slice_3/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_1/strided_slice_3n
layer_norm_1/mul_3/xConst*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_1/mul_3/x�
layer_norm_1/mul_3Mullayer_norm_1/mul_3/x:output:0%layer_norm_1/strided_slice_3:output:0*
T0*
_output_shapes
: 2
layer_norm_1/mul_3~
layer_norm_1/Reshape/shape/0Const*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_1/Reshape/shape/0~
layer_norm_1/Reshape/shape/3Const*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_1/Reshape/shape/3�
layer_norm_1/Reshape/shapePack%layer_norm_1/Reshape/shape/0:output:0layer_norm_1/mul_2:z:0layer_norm_1/mul_3:z:0%layer_norm_1/Reshape/shape/3:output:0*
N*
T0*
_output_shapes
:2
layer_norm_1/Reshape/shape�
layer_norm_1/ReshapeReshapeconv_1/Relu:activations:0#layer_norm_1/Reshape/shape:output:0*
T0*8
_output_shapes&
$:"������������������2
layer_norm_1/Reshapew
layer_norm_1/ones/Less/yConst*
_output_shapes
: *
dtype0*
value
B :�2
layer_norm_1/ones/Less/y�
layer_norm_1/ones/LessLesslayer_norm_1/mul_2:z:0!layer_norm_1/ones/Less/y:output:0*
T0*
_output_shapes
: 2
layer_norm_1/ones/Less�
layer_norm_1/ones/packedPacklayer_norm_1/mul_2:z:0*
N*
T0*
_output_shapes
:2
layer_norm_1/ones/packedw
layer_norm_1/ones/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *  �?2
layer_norm_1/ones/Const�
layer_norm_1/onesFill!layer_norm_1/ones/packed:output:0 layer_norm_1/ones/Const:output:0*
T0*#
_output_shapes
:���������2
layer_norm_1/onesy
layer_norm_1/zeros/Less/yConst*
_output_shapes
: *
dtype0*
value
B :�2
layer_norm_1/zeros/Less/y�
layer_norm_1/zeros/LessLesslayer_norm_1/mul_2:z:0"layer_norm_1/zeros/Less/y:output:0*
T0*
_output_shapes
: 2
layer_norm_1/zeros/Less�
layer_norm_1/zeros/packedPacklayer_norm_1/mul_2:z:0*
N*
T0*
_output_shapes
:2
layer_norm_1/zeros/packedy
layer_norm_1/zeros/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *    2
layer_norm_1/zeros/Const�
layer_norm_1/zerosFill"layer_norm_1/zeros/packed:output:0!layer_norm_1/zeros/Const:output:0*
T0*#
_output_shapes
:���������2
layer_norm_1/zerosk
layer_norm_1/ConstConst*
_output_shapes
: *
dtype0*
valueB 2
layer_norm_1/Consto
layer_norm_1/Const_1Const*
_output_shapes
: *
dtype0*
valueB 2
layer_norm_1/Const_1�
layer_norm_1/FusedBatchNormV3FusedBatchNormV3layer_norm_1/Reshape:output:0layer_norm_1/ones:output:0layer_norm_1/zeros:output:0layer_norm_1/Const:output:0layer_norm_1/Const_1:output:0*
T0*
U0*x
_output_shapesf
d:"������������������:���������:���������:���������:���������:*
data_formatNCHW*
epsilon%o�:2
layer_norm_1/FusedBatchNormV3�
layer_norm_1/Reshape_1Reshape!layer_norm_1/FusedBatchNormV3:y:0layer_norm_1/Shape:output:0*
T0*/
_output_shapes
:���������2
layer_norm_1/Reshape_1�
!layer_norm_1/mul_4/ReadVariableOpReadVariableOp*layer_norm_1_mul_4_readvariableop_resource*
_output_shapes
:*
dtype02#
!layer_norm_1/mul_4/ReadVariableOp�
layer_norm_1/mul_4Mullayer_norm_1/Reshape_1:output:0)layer_norm_1/mul_4/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
layer_norm_1/mul_4�
layer_norm_1/add/ReadVariableOpReadVariableOp(layer_norm_1_add_readvariableop_resource*
_output_shapes
:*
dtype02!
layer_norm_1/add/ReadVariableOp�
layer_norm_1/addAddV2layer_norm_1/mul_4:z:0'layer_norm_1/add/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
layer_norm_1/add�
conv_2/Conv2D/ReadVariableOpReadVariableOp%conv_2_conv2d_readvariableop_resource*&
_output_shapes
:*
dtype02
conv_2/Conv2D/ReadVariableOp�
conv_2/Conv2DConv2Dlayer_norm_1/add:z:0$conv_2/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������*
paddingSAME*
strides
2
conv_2/Conv2D�
conv_2/BiasAdd/ReadVariableOpReadVariableOp&conv_2_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02
conv_2/BiasAdd/ReadVariableOp�
conv_2/BiasAddBiasAddconv_2/Conv2D:output:0%conv_2/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
conv_2/BiasAddu
conv_2/ReluReluconv_2/BiasAdd:output:0*
T0*/
_output_shapes
:���������2
conv_2/Reluq
layer_norm_2/ShapeShapeconv_2/Relu:activations:0*
T0*
_output_shapes
:2
layer_norm_2/Shape�
 layer_norm_2/strided_slice/stackConst*
_output_shapes
:*
dtype0*
valueB: 2"
 layer_norm_2/strided_slice/stack�
"layer_norm_2/strided_slice/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_2/strided_slice/stack_1�
"layer_norm_2/strided_slice/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_2/strided_slice/stack_2�
layer_norm_2/strided_sliceStridedSlicelayer_norm_2/Shape:output:0)layer_norm_2/strided_slice/stack:output:0+layer_norm_2/strided_slice/stack_1:output:0+layer_norm_2/strided_slice/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_2/strided_slicej
layer_norm_2/mul/xConst*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_2/mul/x�
layer_norm_2/mulMullayer_norm_2/mul/x:output:0#layer_norm_2/strided_slice:output:0*
T0*
_output_shapes
: 2
layer_norm_2/mul�
"layer_norm_2/strided_slice_1/stackConst*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_2/strided_slice_1/stack�
$layer_norm_2/strided_slice_1/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_2/strided_slice_1/stack_1�
$layer_norm_2/strided_slice_1/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_2/strided_slice_1/stack_2�
layer_norm_2/strided_slice_1StridedSlicelayer_norm_2/Shape:output:0+layer_norm_2/strided_slice_1/stack:output:0-layer_norm_2/strided_slice_1/stack_1:output:0-layer_norm_2/strided_slice_1/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_2/strided_slice_1�
layer_norm_2/mul_1Mullayer_norm_2/mul:z:0%layer_norm_2/strided_slice_1:output:0*
T0*
_output_shapes
: 2
layer_norm_2/mul_1�
"layer_norm_2/strided_slice_2/stackConst*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_2/strided_slice_2/stack�
$layer_norm_2/strided_slice_2/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_2/strided_slice_2/stack_1�
$layer_norm_2/strided_slice_2/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_2/strided_slice_2/stack_2�
layer_norm_2/strided_slice_2StridedSlicelayer_norm_2/Shape:output:0+layer_norm_2/strided_slice_2/stack:output:0-layer_norm_2/strided_slice_2/stack_1:output:0-layer_norm_2/strided_slice_2/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_2/strided_slice_2�
layer_norm_2/mul_2Mullayer_norm_2/mul_1:z:0%layer_norm_2/strided_slice_2:output:0*
T0*
_output_shapes
: 2
layer_norm_2/mul_2�
"layer_norm_2/strided_slice_3/stackConst*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_2/strided_slice_3/stack�
$layer_norm_2/strided_slice_3/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_2/strided_slice_3/stack_1�
$layer_norm_2/strided_slice_3/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_2/strided_slice_3/stack_2�
layer_norm_2/strided_slice_3StridedSlicelayer_norm_2/Shape:output:0+layer_norm_2/strided_slice_3/stack:output:0-layer_norm_2/strided_slice_3/stack_1:output:0-layer_norm_2/strided_slice_3/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_2/strided_slice_3n
layer_norm_2/mul_3/xConst*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_2/mul_3/x�
layer_norm_2/mul_3Mullayer_norm_2/mul_3/x:output:0%layer_norm_2/strided_slice_3:output:0*
T0*
_output_shapes
: 2
layer_norm_2/mul_3~
layer_norm_2/Reshape/shape/0Const*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_2/Reshape/shape/0~
layer_norm_2/Reshape/shape/3Const*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_2/Reshape/shape/3�
layer_norm_2/Reshape/shapePack%layer_norm_2/Reshape/shape/0:output:0layer_norm_2/mul_2:z:0layer_norm_2/mul_3:z:0%layer_norm_2/Reshape/shape/3:output:0*
N*
T0*
_output_shapes
:2
layer_norm_2/Reshape/shape�
layer_norm_2/ReshapeReshapeconv_2/Relu:activations:0#layer_norm_2/Reshape/shape:output:0*
T0*8
_output_shapes&
$:"������������������2
layer_norm_2/Reshapew
layer_norm_2/ones/Less/yConst*
_output_shapes
: *
dtype0*
value
B :�2
layer_norm_2/ones/Less/y�
layer_norm_2/ones/LessLesslayer_norm_2/mul_2:z:0!layer_norm_2/ones/Less/y:output:0*
T0*
_output_shapes
: 2
layer_norm_2/ones/Less�
layer_norm_2/ones/packedPacklayer_norm_2/mul_2:z:0*
N*
T0*
_output_shapes
:2
layer_norm_2/ones/packedw
layer_norm_2/ones/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *  �?2
layer_norm_2/ones/Const�
layer_norm_2/onesFill!layer_norm_2/ones/packed:output:0 layer_norm_2/ones/Const:output:0*
T0*#
_output_shapes
:���������2
layer_norm_2/onesy
layer_norm_2/zeros/Less/yConst*
_output_shapes
: *
dtype0*
value
B :�2
layer_norm_2/zeros/Less/y�
layer_norm_2/zeros/LessLesslayer_norm_2/mul_2:z:0"layer_norm_2/zeros/Less/y:output:0*
T0*
_output_shapes
: 2
layer_norm_2/zeros/Less�
layer_norm_2/zeros/packedPacklayer_norm_2/mul_2:z:0*
N*
T0*
_output_shapes
:2
layer_norm_2/zeros/packedy
layer_norm_2/zeros/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *    2
layer_norm_2/zeros/Const�
layer_norm_2/zerosFill"layer_norm_2/zeros/packed:output:0!layer_norm_2/zeros/Const:output:0*
T0*#
_output_shapes
:���������2
layer_norm_2/zerosk
layer_norm_2/ConstConst*
_output_shapes
: *
dtype0*
valueB 2
layer_norm_2/Consto
layer_norm_2/Const_1Const*
_output_shapes
: *
dtype0*
valueB 2
layer_norm_2/Const_1�
layer_norm_2/FusedBatchNormV3FusedBatchNormV3layer_norm_2/Reshape:output:0layer_norm_2/ones:output:0layer_norm_2/zeros:output:0layer_norm_2/Const:output:0layer_norm_2/Const_1:output:0*
T0*
U0*x
_output_shapesf
d:"������������������:���������:���������:���������:���������:*
data_formatNCHW*
epsilon%o�:2
layer_norm_2/FusedBatchNormV3�
layer_norm_2/Reshape_1Reshape!layer_norm_2/FusedBatchNormV3:y:0layer_norm_2/Shape:output:0*
T0*/
_output_shapes
:���������2
layer_norm_2/Reshape_1�
!layer_norm_2/mul_4/ReadVariableOpReadVariableOp*layer_norm_2_mul_4_readvariableop_resource*
_output_shapes
:*
dtype02#
!layer_norm_2/mul_4/ReadVariableOp�
layer_norm_2/mul_4Mullayer_norm_2/Reshape_1:output:0)layer_norm_2/mul_4/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
layer_norm_2/mul_4�
layer_norm_2/add/ReadVariableOpReadVariableOp(layer_norm_2_add_readvariableop_resource*
_output_shapes
:*
dtype02!
layer_norm_2/add/ReadVariableOp�
layer_norm_2/addAddV2layer_norm_2/mul_4:z:0'layer_norm_2/add/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
layer_norm_2/add�
conv_3/Conv2D/ReadVariableOpReadVariableOp%conv_3_conv2d_readvariableop_resource*&
_output_shapes
:*
dtype02
conv_3/Conv2D/ReadVariableOp�
conv_3/Conv2DConv2Dlayer_norm_2/add:z:0$conv_3/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������*
paddingSAME*
strides
2
conv_3/Conv2D�
conv_3/BiasAdd/ReadVariableOpReadVariableOp&conv_3_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02
conv_3/BiasAdd/ReadVariableOp�
conv_3/BiasAddBiasAddconv_3/Conv2D:output:0%conv_3/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
conv_3/BiasAddu
conv_3/ReluReluconv_3/BiasAdd:output:0*
T0*/
_output_shapes
:���������2
conv_3/Reluo
flatten/ConstConst*
_output_shapes
:*
dtype0*
valueB"�����   2
flatten/Const�
flatten/ReshapeReshapeconv_3/Relu:activations:0flatten/Const:output:0*
T0*(
_output_shapes
:����������2
flatten/Reshape�
dense/MatMul/ReadVariableOpReadVariableOp$dense_matmul_readvariableop_resource*
_output_shapes
:	�d*
dtype02
dense/MatMul/ReadVariableOp�
dense/MatMulMatMulflatten/Reshape:output:0#dense/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������d2
dense/MatMul�
dense/BiasAdd/ReadVariableOpReadVariableOp%dense_biasadd_readvariableop_resource*
_output_shapes
:d*
dtype02
dense/BiasAdd/ReadVariableOp�
dense/BiasAddBiasAdddense/MatMul:product:0$dense/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������d2
dense/BiasAdds
dropout/dropout/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *�8�?2
dropout/dropout/Const�
dropout/dropout/MulMuldense/BiasAdd:output:0dropout/dropout/Const:output:0*
T0*'
_output_shapes
:���������d2
dropout/dropout/Mult
dropout/dropout/ShapeShapedense/BiasAdd:output:0*
T0*
_output_shapes
:2
dropout/dropout/Shape�
,dropout/dropout/random_uniform/RandomUniformRandomUniformdropout/dropout/Shape:output:0*
T0*'
_output_shapes
:���������d*
dtype02.
,dropout/dropout/random_uniform/RandomUniform�
dropout/dropout/GreaterEqual/yConst*
_output_shapes
: *
dtype0*
valueB
 *���=2 
dropout/dropout/GreaterEqual/y�
dropout/dropout/GreaterEqualGreaterEqual5dropout/dropout/random_uniform/RandomUniform:output:0'dropout/dropout/GreaterEqual/y:output:0*
T0*'
_output_shapes
:���������d2
dropout/dropout/GreaterEqual�
dropout/dropout/CastCast dropout/dropout/GreaterEqual:z:0*

DstT0*

SrcT0
*'
_output_shapes
:���������d2
dropout/dropout/Cast�
dropout/dropout/Mul_1Muldropout/dropout/Mul:z:0dropout/dropout/Cast:y:0*
T0*'
_output_shapes
:���������d2
dropout/dropout/Mul_1�
 raw_output/MatMul/ReadVariableOpReadVariableOp)raw_output_matmul_readvariableop_resource*
_output_shapes

:d*
dtype02"
 raw_output/MatMul/ReadVariableOp�
raw_output/MatMulMatMuldropout/dropout/Mul_1:z:0(raw_output/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������2
raw_output/MatMul�
!raw_output/BiasAdd/ReadVariableOpReadVariableOp*raw_output_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02#
!raw_output/BiasAdd/ReadVariableOp�
raw_output/BiasAddBiasAddraw_output/MatMul:product:0)raw_output/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������2
raw_output/BiasAdd�
raw_output/SoftmaxSoftmaxraw_output/BiasAdd:output:0*
T0*'
_output_shapes
:���������2
raw_output/Softmaxw
IdentityIdentityraw_output/Softmax:softmax:0^NoOp*
T0*'
_output_shapes
:���������2

Identity�
NoOpNoOp^conv_1/BiasAdd/ReadVariableOp^conv_1/Conv2D/ReadVariableOp^conv_2/BiasAdd/ReadVariableOp^conv_2/Conv2D/ReadVariableOp^conv_3/BiasAdd/ReadVariableOp^conv_3/Conv2D/ReadVariableOp^dense/BiasAdd/ReadVariableOp^dense/MatMul/ReadVariableOp ^layer_norm_1/add/ReadVariableOp"^layer_norm_1/mul_4/ReadVariableOp ^layer_norm_2/add/ReadVariableOp"^layer_norm_2/mul_4/ReadVariableOp"^raw_output/BiasAdd/ReadVariableOp!^raw_output/MatMul/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*V
_input_shapesE
C:���������  ::: : : : : : : : : : : : : : 2>
conv_1/BiasAdd/ReadVariableOpconv_1/BiasAdd/ReadVariableOp2<
conv_1/Conv2D/ReadVariableOpconv_1/Conv2D/ReadVariableOp2>
conv_2/BiasAdd/ReadVariableOpconv_2/BiasAdd/ReadVariableOp2<
conv_2/Conv2D/ReadVariableOpconv_2/Conv2D/ReadVariableOp2>
conv_3/BiasAdd/ReadVariableOpconv_3/BiasAdd/ReadVariableOp2<
conv_3/Conv2D/ReadVariableOpconv_3/Conv2D/ReadVariableOp2<
dense/BiasAdd/ReadVariableOpdense/BiasAdd/ReadVariableOp2:
dense/MatMul/ReadVariableOpdense/MatMul/ReadVariableOp2B
layer_norm_1/add/ReadVariableOplayer_norm_1/add/ReadVariableOp2F
!layer_norm_1/mul_4/ReadVariableOp!layer_norm_1/mul_4/ReadVariableOp2B
layer_norm_2/add/ReadVariableOplayer_norm_2/add/ReadVariableOp2F
!layer_norm_2/mul_4/ReadVariableOp!layer_norm_2/mul_4/ReadVariableOp2F
!raw_output/BiasAdd/ReadVariableOp!raw_output/BiasAdd/ReadVariableOp2D
 raw_output/MatMul/ReadVariableOp raw_output/MatMul/ReadVariableOp:X T
/
_output_shapes
:���������  
!
_user_specified_name	input_1: 

_output_shapes
:: 

_output_shapes
:
�
�
)__inference_cnn_model_layer_call_fn_46839

inputs
unknown
	unknown_0#
	unknown_1:
	unknown_2:
	unknown_3:
	unknown_4:#
	unknown_5:
	unknown_6:
	unknown_7:
	unknown_8:#
	unknown_9:

unknown_10:

unknown_11:	�d

unknown_12:d

unknown_13:d

unknown_14:
identity��StatefulPartitionedCall�
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4	unknown_5	unknown_6	unknown_7	unknown_8	unknown_9
unknown_10
unknown_11
unknown_12
unknown_13
unknown_14*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:���������*0
_read_only_resource_inputs
	
*-
config_proto

CPU

GPU 2J 8� *M
fHRF
D__inference_cnn_model_layer_call_and_return_conditional_losses_465272
StatefulPartitionedCall{
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:���������2

Identityh
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*V
_input_shapesE
C:���������  ::: : : : : : : : : : : : : : 22
StatefulPartitionedCallStatefulPartitionedCall:W S
/
_output_shapes
:���������  
 
_user_specified_nameinputs: 

_output_shapes
:: 

_output_shapes
:
ճ
�

D__inference_cnn_model_layer_call_and_return_conditional_losses_47025

inputs	
sub_y
	truediv_y?
%conv_1_conv2d_readvariableop_resource:4
&conv_1_biasadd_readvariableop_resource:8
*layer_norm_1_mul_4_readvariableop_resource:6
(layer_norm_1_add_readvariableop_resource:?
%conv_2_conv2d_readvariableop_resource:4
&conv_2_biasadd_readvariableop_resource:8
*layer_norm_2_mul_4_readvariableop_resource:6
(layer_norm_2_add_readvariableop_resource:?
%conv_3_conv2d_readvariableop_resource:4
&conv_3_biasadd_readvariableop_resource:7
$dense_matmul_readvariableop_resource:	�d3
%dense_biasadd_readvariableop_resource:d;
)raw_output_matmul_readvariableop_resource:d8
*raw_output_biasadd_readvariableop_resource:
identity��conv_1/BiasAdd/ReadVariableOp�conv_1/Conv2D/ReadVariableOp�conv_2/BiasAdd/ReadVariableOp�conv_2/Conv2D/ReadVariableOp�conv_3/BiasAdd/ReadVariableOp�conv_3/Conv2D/ReadVariableOp�dense/BiasAdd/ReadVariableOp�dense/MatMul/ReadVariableOp�layer_norm_1/add/ReadVariableOp�!layer_norm_1/mul_4/ReadVariableOp�layer_norm_2/add/ReadVariableOp�!layer_norm_2/mul_4/ReadVariableOp�!raw_output/BiasAdd/ReadVariableOp� raw_output/MatMul/ReadVariableOpZ
subSubinputssub_y*
T0*/
_output_shapes
:���������  2
subk
truedivRealDivsub:z:0	truediv_y*
T0*/
_output_shapes
:���������  2	
truediv�
conv_1/Conv2D/ReadVariableOpReadVariableOp%conv_1_conv2d_readvariableop_resource*&
_output_shapes
:*
dtype02
conv_1/Conv2D/ReadVariableOp�
conv_1/Conv2DConv2Dtruediv:z:0$conv_1/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������*
paddingSAME*
strides
2
conv_1/Conv2D�
conv_1/BiasAdd/ReadVariableOpReadVariableOp&conv_1_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02
conv_1/BiasAdd/ReadVariableOp�
conv_1/BiasAddBiasAddconv_1/Conv2D:output:0%conv_1/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
conv_1/BiasAddu
conv_1/ReluReluconv_1/BiasAdd:output:0*
T0*/
_output_shapes
:���������2
conv_1/Reluq
layer_norm_1/ShapeShapeconv_1/Relu:activations:0*
T0*
_output_shapes
:2
layer_norm_1/Shape�
 layer_norm_1/strided_slice/stackConst*
_output_shapes
:*
dtype0*
valueB: 2"
 layer_norm_1/strided_slice/stack�
"layer_norm_1/strided_slice/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_1/strided_slice/stack_1�
"layer_norm_1/strided_slice/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_1/strided_slice/stack_2�
layer_norm_1/strided_sliceStridedSlicelayer_norm_1/Shape:output:0)layer_norm_1/strided_slice/stack:output:0+layer_norm_1/strided_slice/stack_1:output:0+layer_norm_1/strided_slice/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_1/strided_slicej
layer_norm_1/mul/xConst*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_1/mul/x�
layer_norm_1/mulMullayer_norm_1/mul/x:output:0#layer_norm_1/strided_slice:output:0*
T0*
_output_shapes
: 2
layer_norm_1/mul�
"layer_norm_1/strided_slice_1/stackConst*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_1/strided_slice_1/stack�
$layer_norm_1/strided_slice_1/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_1/strided_slice_1/stack_1�
$layer_norm_1/strided_slice_1/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_1/strided_slice_1/stack_2�
layer_norm_1/strided_slice_1StridedSlicelayer_norm_1/Shape:output:0+layer_norm_1/strided_slice_1/stack:output:0-layer_norm_1/strided_slice_1/stack_1:output:0-layer_norm_1/strided_slice_1/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_1/strided_slice_1�
layer_norm_1/mul_1Mullayer_norm_1/mul:z:0%layer_norm_1/strided_slice_1:output:0*
T0*
_output_shapes
: 2
layer_norm_1/mul_1�
"layer_norm_1/strided_slice_2/stackConst*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_1/strided_slice_2/stack�
$layer_norm_1/strided_slice_2/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_1/strided_slice_2/stack_1�
$layer_norm_1/strided_slice_2/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_1/strided_slice_2/stack_2�
layer_norm_1/strided_slice_2StridedSlicelayer_norm_1/Shape:output:0+layer_norm_1/strided_slice_2/stack:output:0-layer_norm_1/strided_slice_2/stack_1:output:0-layer_norm_1/strided_slice_2/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_1/strided_slice_2�
layer_norm_1/mul_2Mullayer_norm_1/mul_1:z:0%layer_norm_1/strided_slice_2:output:0*
T0*
_output_shapes
: 2
layer_norm_1/mul_2�
"layer_norm_1/strided_slice_3/stackConst*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_1/strided_slice_3/stack�
$layer_norm_1/strided_slice_3/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_1/strided_slice_3/stack_1�
$layer_norm_1/strided_slice_3/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_1/strided_slice_3/stack_2�
layer_norm_1/strided_slice_3StridedSlicelayer_norm_1/Shape:output:0+layer_norm_1/strided_slice_3/stack:output:0-layer_norm_1/strided_slice_3/stack_1:output:0-layer_norm_1/strided_slice_3/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_1/strided_slice_3n
layer_norm_1/mul_3/xConst*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_1/mul_3/x�
layer_norm_1/mul_3Mullayer_norm_1/mul_3/x:output:0%layer_norm_1/strided_slice_3:output:0*
T0*
_output_shapes
: 2
layer_norm_1/mul_3~
layer_norm_1/Reshape/shape/0Const*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_1/Reshape/shape/0~
layer_norm_1/Reshape/shape/3Const*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_1/Reshape/shape/3�
layer_norm_1/Reshape/shapePack%layer_norm_1/Reshape/shape/0:output:0layer_norm_1/mul_2:z:0layer_norm_1/mul_3:z:0%layer_norm_1/Reshape/shape/3:output:0*
N*
T0*
_output_shapes
:2
layer_norm_1/Reshape/shape�
layer_norm_1/ReshapeReshapeconv_1/Relu:activations:0#layer_norm_1/Reshape/shape:output:0*
T0*8
_output_shapes&
$:"������������������2
layer_norm_1/Reshapew
layer_norm_1/ones/Less/yConst*
_output_shapes
: *
dtype0*
value
B :�2
layer_norm_1/ones/Less/y�
layer_norm_1/ones/LessLesslayer_norm_1/mul_2:z:0!layer_norm_1/ones/Less/y:output:0*
T0*
_output_shapes
: 2
layer_norm_1/ones/Less�
layer_norm_1/ones/packedPacklayer_norm_1/mul_2:z:0*
N*
T0*
_output_shapes
:2
layer_norm_1/ones/packedw
layer_norm_1/ones/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *  �?2
layer_norm_1/ones/Const�
layer_norm_1/onesFill!layer_norm_1/ones/packed:output:0 layer_norm_1/ones/Const:output:0*
T0*#
_output_shapes
:���������2
layer_norm_1/onesy
layer_norm_1/zeros/Less/yConst*
_output_shapes
: *
dtype0*
value
B :�2
layer_norm_1/zeros/Less/y�
layer_norm_1/zeros/LessLesslayer_norm_1/mul_2:z:0"layer_norm_1/zeros/Less/y:output:0*
T0*
_output_shapes
: 2
layer_norm_1/zeros/Less�
layer_norm_1/zeros/packedPacklayer_norm_1/mul_2:z:0*
N*
T0*
_output_shapes
:2
layer_norm_1/zeros/packedy
layer_norm_1/zeros/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *    2
layer_norm_1/zeros/Const�
layer_norm_1/zerosFill"layer_norm_1/zeros/packed:output:0!layer_norm_1/zeros/Const:output:0*
T0*#
_output_shapes
:���������2
layer_norm_1/zerosk
layer_norm_1/ConstConst*
_output_shapes
: *
dtype0*
valueB 2
layer_norm_1/Consto
layer_norm_1/Const_1Const*
_output_shapes
: *
dtype0*
valueB 2
layer_norm_1/Const_1�
layer_norm_1/FusedBatchNormV3FusedBatchNormV3layer_norm_1/Reshape:output:0layer_norm_1/ones:output:0layer_norm_1/zeros:output:0layer_norm_1/Const:output:0layer_norm_1/Const_1:output:0*
T0*
U0*x
_output_shapesf
d:"������������������:���������:���������:���������:���������:*
data_formatNCHW*
epsilon%o�:2
layer_norm_1/FusedBatchNormV3�
layer_norm_1/Reshape_1Reshape!layer_norm_1/FusedBatchNormV3:y:0layer_norm_1/Shape:output:0*
T0*/
_output_shapes
:���������2
layer_norm_1/Reshape_1�
!layer_norm_1/mul_4/ReadVariableOpReadVariableOp*layer_norm_1_mul_4_readvariableop_resource*
_output_shapes
:*
dtype02#
!layer_norm_1/mul_4/ReadVariableOp�
layer_norm_1/mul_4Mullayer_norm_1/Reshape_1:output:0)layer_norm_1/mul_4/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
layer_norm_1/mul_4�
layer_norm_1/add/ReadVariableOpReadVariableOp(layer_norm_1_add_readvariableop_resource*
_output_shapes
:*
dtype02!
layer_norm_1/add/ReadVariableOp�
layer_norm_1/addAddV2layer_norm_1/mul_4:z:0'layer_norm_1/add/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
layer_norm_1/add�
conv_2/Conv2D/ReadVariableOpReadVariableOp%conv_2_conv2d_readvariableop_resource*&
_output_shapes
:*
dtype02
conv_2/Conv2D/ReadVariableOp�
conv_2/Conv2DConv2Dlayer_norm_1/add:z:0$conv_2/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������*
paddingSAME*
strides
2
conv_2/Conv2D�
conv_2/BiasAdd/ReadVariableOpReadVariableOp&conv_2_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02
conv_2/BiasAdd/ReadVariableOp�
conv_2/BiasAddBiasAddconv_2/Conv2D:output:0%conv_2/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
conv_2/BiasAddu
conv_2/ReluReluconv_2/BiasAdd:output:0*
T0*/
_output_shapes
:���������2
conv_2/Reluq
layer_norm_2/ShapeShapeconv_2/Relu:activations:0*
T0*
_output_shapes
:2
layer_norm_2/Shape�
 layer_norm_2/strided_slice/stackConst*
_output_shapes
:*
dtype0*
valueB: 2"
 layer_norm_2/strided_slice/stack�
"layer_norm_2/strided_slice/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_2/strided_slice/stack_1�
"layer_norm_2/strided_slice/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_2/strided_slice/stack_2�
layer_norm_2/strided_sliceStridedSlicelayer_norm_2/Shape:output:0)layer_norm_2/strided_slice/stack:output:0+layer_norm_2/strided_slice/stack_1:output:0+layer_norm_2/strided_slice/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_2/strided_slicej
layer_norm_2/mul/xConst*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_2/mul/x�
layer_norm_2/mulMullayer_norm_2/mul/x:output:0#layer_norm_2/strided_slice:output:0*
T0*
_output_shapes
: 2
layer_norm_2/mul�
"layer_norm_2/strided_slice_1/stackConst*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_2/strided_slice_1/stack�
$layer_norm_2/strided_slice_1/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_2/strided_slice_1/stack_1�
$layer_norm_2/strided_slice_1/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_2/strided_slice_1/stack_2�
layer_norm_2/strided_slice_1StridedSlicelayer_norm_2/Shape:output:0+layer_norm_2/strided_slice_1/stack:output:0-layer_norm_2/strided_slice_1/stack_1:output:0-layer_norm_2/strided_slice_1/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_2/strided_slice_1�
layer_norm_2/mul_1Mullayer_norm_2/mul:z:0%layer_norm_2/strided_slice_1:output:0*
T0*
_output_shapes
: 2
layer_norm_2/mul_1�
"layer_norm_2/strided_slice_2/stackConst*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_2/strided_slice_2/stack�
$layer_norm_2/strided_slice_2/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_2/strided_slice_2/stack_1�
$layer_norm_2/strided_slice_2/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_2/strided_slice_2/stack_2�
layer_norm_2/strided_slice_2StridedSlicelayer_norm_2/Shape:output:0+layer_norm_2/strided_slice_2/stack:output:0-layer_norm_2/strided_slice_2/stack_1:output:0-layer_norm_2/strided_slice_2/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_2/strided_slice_2�
layer_norm_2/mul_2Mullayer_norm_2/mul_1:z:0%layer_norm_2/strided_slice_2:output:0*
T0*
_output_shapes
: 2
layer_norm_2/mul_2�
"layer_norm_2/strided_slice_3/stackConst*
_output_shapes
:*
dtype0*
valueB:2$
"layer_norm_2/strided_slice_3/stack�
$layer_norm_2/strided_slice_3/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_2/strided_slice_3/stack_1�
$layer_norm_2/strided_slice_3/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2&
$layer_norm_2/strided_slice_3/stack_2�
layer_norm_2/strided_slice_3StridedSlicelayer_norm_2/Shape:output:0+layer_norm_2/strided_slice_3/stack:output:0-layer_norm_2/strided_slice_3/stack_1:output:0-layer_norm_2/strided_slice_3/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
layer_norm_2/strided_slice_3n
layer_norm_2/mul_3/xConst*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_2/mul_3/x�
layer_norm_2/mul_3Mullayer_norm_2/mul_3/x:output:0%layer_norm_2/strided_slice_3:output:0*
T0*
_output_shapes
: 2
layer_norm_2/mul_3~
layer_norm_2/Reshape/shape/0Const*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_2/Reshape/shape/0~
layer_norm_2/Reshape/shape/3Const*
_output_shapes
: *
dtype0*
value	B :2
layer_norm_2/Reshape/shape/3�
layer_norm_2/Reshape/shapePack%layer_norm_2/Reshape/shape/0:output:0layer_norm_2/mul_2:z:0layer_norm_2/mul_3:z:0%layer_norm_2/Reshape/shape/3:output:0*
N*
T0*
_output_shapes
:2
layer_norm_2/Reshape/shape�
layer_norm_2/ReshapeReshapeconv_2/Relu:activations:0#layer_norm_2/Reshape/shape:output:0*
T0*8
_output_shapes&
$:"������������������2
layer_norm_2/Reshapew
layer_norm_2/ones/Less/yConst*
_output_shapes
: *
dtype0*
value
B :�2
layer_norm_2/ones/Less/y�
layer_norm_2/ones/LessLesslayer_norm_2/mul_2:z:0!layer_norm_2/ones/Less/y:output:0*
T0*
_output_shapes
: 2
layer_norm_2/ones/Less�
layer_norm_2/ones/packedPacklayer_norm_2/mul_2:z:0*
N*
T0*
_output_shapes
:2
layer_norm_2/ones/packedw
layer_norm_2/ones/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *  �?2
layer_norm_2/ones/Const�
layer_norm_2/onesFill!layer_norm_2/ones/packed:output:0 layer_norm_2/ones/Const:output:0*
T0*#
_output_shapes
:���������2
layer_norm_2/onesy
layer_norm_2/zeros/Less/yConst*
_output_shapes
: *
dtype0*
value
B :�2
layer_norm_2/zeros/Less/y�
layer_norm_2/zeros/LessLesslayer_norm_2/mul_2:z:0"layer_norm_2/zeros/Less/y:output:0*
T0*
_output_shapes
: 2
layer_norm_2/zeros/Less�
layer_norm_2/zeros/packedPacklayer_norm_2/mul_2:z:0*
N*
T0*
_output_shapes
:2
layer_norm_2/zeros/packedy
layer_norm_2/zeros/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *    2
layer_norm_2/zeros/Const�
layer_norm_2/zerosFill"layer_norm_2/zeros/packed:output:0!layer_norm_2/zeros/Const:output:0*
T0*#
_output_shapes
:���������2
layer_norm_2/zerosk
layer_norm_2/ConstConst*
_output_shapes
: *
dtype0*
valueB 2
layer_norm_2/Consto
layer_norm_2/Const_1Const*
_output_shapes
: *
dtype0*
valueB 2
layer_norm_2/Const_1�
layer_norm_2/FusedBatchNormV3FusedBatchNormV3layer_norm_2/Reshape:output:0layer_norm_2/ones:output:0layer_norm_2/zeros:output:0layer_norm_2/Const:output:0layer_norm_2/Const_1:output:0*
T0*
U0*x
_output_shapesf
d:"������������������:���������:���������:���������:���������:*
data_formatNCHW*
epsilon%o�:2
layer_norm_2/FusedBatchNormV3�
layer_norm_2/Reshape_1Reshape!layer_norm_2/FusedBatchNormV3:y:0layer_norm_2/Shape:output:0*
T0*/
_output_shapes
:���������2
layer_norm_2/Reshape_1�
!layer_norm_2/mul_4/ReadVariableOpReadVariableOp*layer_norm_2_mul_4_readvariableop_resource*
_output_shapes
:*
dtype02#
!layer_norm_2/mul_4/ReadVariableOp�
layer_norm_2/mul_4Mullayer_norm_2/Reshape_1:output:0)layer_norm_2/mul_4/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
layer_norm_2/mul_4�
layer_norm_2/add/ReadVariableOpReadVariableOp(layer_norm_2_add_readvariableop_resource*
_output_shapes
:*
dtype02!
layer_norm_2/add/ReadVariableOp�
layer_norm_2/addAddV2layer_norm_2/mul_4:z:0'layer_norm_2/add/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
layer_norm_2/add�
conv_3/Conv2D/ReadVariableOpReadVariableOp%conv_3_conv2d_readvariableop_resource*&
_output_shapes
:*
dtype02
conv_3/Conv2D/ReadVariableOp�
conv_3/Conv2DConv2Dlayer_norm_2/add:z:0$conv_3/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������*
paddingSAME*
strides
2
conv_3/Conv2D�
conv_3/BiasAdd/ReadVariableOpReadVariableOp&conv_3_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02
conv_3/BiasAdd/ReadVariableOp�
conv_3/BiasAddBiasAddconv_3/Conv2D:output:0%conv_3/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
conv_3/BiasAddu
conv_3/ReluReluconv_3/BiasAdd:output:0*
T0*/
_output_shapes
:���������2
conv_3/Reluo
flatten/ConstConst*
_output_shapes
:*
dtype0*
valueB"�����   2
flatten/Const�
flatten/ReshapeReshapeconv_3/Relu:activations:0flatten/Const:output:0*
T0*(
_output_shapes
:����������2
flatten/Reshape�
dense/MatMul/ReadVariableOpReadVariableOp$dense_matmul_readvariableop_resource*
_output_shapes
:	�d*
dtype02
dense/MatMul/ReadVariableOp�
dense/MatMulMatMulflatten/Reshape:output:0#dense/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������d2
dense/MatMul�
dense/BiasAdd/ReadVariableOpReadVariableOp%dense_biasadd_readvariableop_resource*
_output_shapes
:d*
dtype02
dense/BiasAdd/ReadVariableOp�
dense/BiasAddBiasAdddense/MatMul:product:0$dense/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������d2
dense/BiasAddz
dropout/IdentityIdentitydense/BiasAdd:output:0*
T0*'
_output_shapes
:���������d2
dropout/Identity�
 raw_output/MatMul/ReadVariableOpReadVariableOp)raw_output_matmul_readvariableop_resource*
_output_shapes

:d*
dtype02"
 raw_output/MatMul/ReadVariableOp�
raw_output/MatMulMatMuldropout/Identity:output:0(raw_output/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������2
raw_output/MatMul�
!raw_output/BiasAdd/ReadVariableOpReadVariableOp*raw_output_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02#
!raw_output/BiasAdd/ReadVariableOp�
raw_output/BiasAddBiasAddraw_output/MatMul:product:0)raw_output/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������2
raw_output/BiasAdd�
raw_output/SoftmaxSoftmaxraw_output/BiasAdd:output:0*
T0*'
_output_shapes
:���������2
raw_output/Softmaxw
IdentityIdentityraw_output/Softmax:softmax:0^NoOp*
T0*'
_output_shapes
:���������2

Identity�
NoOpNoOp^conv_1/BiasAdd/ReadVariableOp^conv_1/Conv2D/ReadVariableOp^conv_2/BiasAdd/ReadVariableOp^conv_2/Conv2D/ReadVariableOp^conv_3/BiasAdd/ReadVariableOp^conv_3/Conv2D/ReadVariableOp^dense/BiasAdd/ReadVariableOp^dense/MatMul/ReadVariableOp ^layer_norm_1/add/ReadVariableOp"^layer_norm_1/mul_4/ReadVariableOp ^layer_norm_2/add/ReadVariableOp"^layer_norm_2/mul_4/ReadVariableOp"^raw_output/BiasAdd/ReadVariableOp!^raw_output/MatMul/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*V
_input_shapesE
C:���������  ::: : : : : : : : : : : : : : 2>
conv_1/BiasAdd/ReadVariableOpconv_1/BiasAdd/ReadVariableOp2<
conv_1/Conv2D/ReadVariableOpconv_1/Conv2D/ReadVariableOp2>
conv_2/BiasAdd/ReadVariableOpconv_2/BiasAdd/ReadVariableOp2<
conv_2/Conv2D/ReadVariableOpconv_2/Conv2D/ReadVariableOp2>
conv_3/BiasAdd/ReadVariableOpconv_3/BiasAdd/ReadVariableOp2<
conv_3/Conv2D/ReadVariableOpconv_3/Conv2D/ReadVariableOp2<
dense/BiasAdd/ReadVariableOpdense/BiasAdd/ReadVariableOp2:
dense/MatMul/ReadVariableOpdense/MatMul/ReadVariableOp2B
layer_norm_1/add/ReadVariableOplayer_norm_1/add/ReadVariableOp2F
!layer_norm_1/mul_4/ReadVariableOp!layer_norm_1/mul_4/ReadVariableOp2B
layer_norm_2/add/ReadVariableOplayer_norm_2/add/ReadVariableOp2F
!layer_norm_2/mul_4/ReadVariableOp!layer_norm_2/mul_4/ReadVariableOp2F
!raw_output/BiasAdd/ReadVariableOp!raw_output/BiasAdd/ReadVariableOp2D
 raw_output/MatMul/ReadVariableOp raw_output/MatMul/ReadVariableOp:W S
/
_output_shapes
:���������  
 
_user_specified_nameinputs: 

_output_shapes
:: 

_output_shapes
:
�
�
)__inference_cnn_model_layer_call_fn_46802

inputs
unknown
	unknown_0#
	unknown_1:
	unknown_2:
	unknown_3:
	unknown_4:#
	unknown_5:
	unknown_6:
	unknown_7:
	unknown_8:#
	unknown_9:

unknown_10:

unknown_11:	�d

unknown_12:d

unknown_13:d

unknown_14:
identity��StatefulPartitionedCall�
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4	unknown_5	unknown_6	unknown_7	unknown_8	unknown_9
unknown_10
unknown_11
unknown_12
unknown_13
unknown_14*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:���������*0
_read_only_resource_inputs
	
*-
config_proto

CPU

GPU 2J 8� *M
fHRF
D__inference_cnn_model_layer_call_and_return_conditional_losses_463092
StatefulPartitionedCall{
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:���������2

Identityh
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*V
_input_shapesE
C:���������  ::: : : : : : : : : : : : : : 22
StatefulPartitionedCallStatefulPartitionedCall:W S
/
_output_shapes
:���������  
 
_user_specified_nameinputs: 

_output_shapes
:: 

_output_shapes
:
�
�
&__inference_conv_2_layer_call_fn_47580

inputs!
unknown:
	unknown_0:
identity��StatefulPartitionedCall�
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:���������*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8� *J
fERC
A__inference_conv_2_layer_call_and_return_conditional_losses_461752
StatefulPartitionedCall�
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*/
_output_shapes
:���������2

Identityh
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*2
_input_shapes!
:���������: : 22
StatefulPartitionedCallStatefulPartitionedCall:W S
/
_output_shapes
:���������
 
_user_specified_nameinputs
�2
�
G__inference_layer_norm_1_layer_call_and_return_conditional_losses_47571

inputs+
mul_4_readvariableop_resource:)
add_readvariableop_resource:
identity��add/ReadVariableOp�mul_4/ReadVariableOpD
ShapeShapeinputs*
T0*
_output_shapes
:2
Shapet
strided_slice/stackConst*
_output_shapes
:*
dtype0*
valueB: 2
strided_slice/stackx
strided_slice/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice/stack_1x
strided_slice/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice/stack_2�
strided_sliceStridedSliceShape:output:0strided_slice/stack:output:0strided_slice/stack_1:output:0strided_slice/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
strided_sliceP
mul/xConst*
_output_shapes
: *
dtype0*
value	B :2
mul/xZ
mulMulmul/x:output:0strided_slice:output:0*
T0*
_output_shapes
: 2
mulx
strided_slice_1/stackConst*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_1/stack|
strided_slice_1/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_1/stack_1|
strided_slice_1/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_1/stack_2�
strided_slice_1StridedSliceShape:output:0strided_slice_1/stack:output:0 strided_slice_1/stack_1:output:0 strided_slice_1/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
strided_slice_1Y
mul_1Mulmul:z:0strided_slice_1:output:0*
T0*
_output_shapes
: 2
mul_1x
strided_slice_2/stackConst*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_2/stack|
strided_slice_2/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_2/stack_1|
strided_slice_2/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_2/stack_2�
strided_slice_2StridedSliceShape:output:0strided_slice_2/stack:output:0 strided_slice_2/stack_1:output:0 strided_slice_2/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
strided_slice_2[
mul_2Mul	mul_1:z:0strided_slice_2:output:0*
T0*
_output_shapes
: 2
mul_2x
strided_slice_3/stackConst*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_3/stack|
strided_slice_3/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_3/stack_1|
strided_slice_3/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2
strided_slice_3/stack_2�
strided_slice_3StridedSliceShape:output:0strided_slice_3/stack:output:0 strided_slice_3/stack_1:output:0 strided_slice_3/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2
strided_slice_3T
mul_3/xConst*
_output_shapes
: *
dtype0*
value	B :2	
mul_3/xb
mul_3Mulmul_3/x:output:0strided_slice_3:output:0*
T0*
_output_shapes
: 2
mul_3d
Reshape/shape/0Const*
_output_shapes
: *
dtype0*
value	B :2
Reshape/shape/0d
Reshape/shape/3Const*
_output_shapes
: *
dtype0*
value	B :2
Reshape/shape/3�
Reshape/shapePackReshape/shape/0:output:0	mul_2:z:0	mul_3:z:0Reshape/shape/3:output:0*
N*
T0*
_output_shapes
:2
Reshape/shape�
ReshapeReshapeinputsReshape/shape:output:0*
T0*8
_output_shapes&
$:"������������������2	
Reshape]
ones/Less/yConst*
_output_shapes
: *
dtype0*
value
B :�2
ones/Less/y`
	ones/LessLess	mul_2:z:0ones/Less/y:output:0*
T0*
_output_shapes
: 2
	ones/Less[
ones/packedPack	mul_2:z:0*
N*
T0*
_output_shapes
:2
ones/packed]

ones/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *  �?2

ones/Constm
onesFillones/packed:output:0ones/Const:output:0*
T0*#
_output_shapes
:���������2
ones_
zeros/Less/yConst*
_output_shapes
: *
dtype0*
value
B :�2
zeros/Less/yc

zeros/LessLess	mul_2:z:0zeros/Less/y:output:0*
T0*
_output_shapes
: 2

zeros/Less]
zeros/packedPack	mul_2:z:0*
N*
T0*
_output_shapes
:2
zeros/packed_
zeros/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *    2
zeros/Constq
zerosFillzeros/packed:output:0zeros/Const:output:0*
T0*#
_output_shapes
:���������2
zerosQ
ConstConst*
_output_shapes
: *
dtype0*
valueB 2
ConstU
Const_1Const*
_output_shapes
: *
dtype0*
valueB 2	
Const_1�
FusedBatchNormV3FusedBatchNormV3Reshape:output:0ones:output:0zeros:output:0Const:output:0Const_1:output:0*
T0*
U0*x
_output_shapesf
d:"������������������:���������:���������:���������:���������:*
data_formatNCHW*
epsilon%o�:2
FusedBatchNormV3�
	Reshape_1ReshapeFusedBatchNormV3:y:0Shape:output:0*
T0*/
_output_shapes
:���������2
	Reshape_1�
mul_4/ReadVariableOpReadVariableOpmul_4_readvariableop_resource*
_output_shapes
:*
dtype02
mul_4/ReadVariableOp�
mul_4MulReshape_1:output:0mul_4/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
mul_4�
add/ReadVariableOpReadVariableOpadd_readvariableop_resource*
_output_shapes
:*
dtype02
add/ReadVariableOpt
addAddV2	mul_4:z:0add/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
addj
IdentityIdentityadd:z:0^NoOp*
T0*/
_output_shapes
:���������2

Identityz
NoOpNoOp^add/ReadVariableOp^mul_4/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*2
_input_shapes!
:���������: : 2(
add/ReadVariableOpadd/ReadVariableOp2,
mul_4/ReadVariableOpmul_4/ReadVariableOp:W S
/
_output_shapes
:���������
 
_user_specified_nameinputs
�
�
*__inference_raw_output_layer_call_fn_47742

inputs
unknown:d
	unknown_0:
identity��StatefulPartitionedCall�
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:���������*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8� *N
fIRG
E__inference_raw_output_layer_call_and_return_conditional_losses_463022
StatefulPartitionedCall{
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:���������2

Identityh
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime**
_input_shapes
:���������d: : 22
StatefulPartitionedCallStatefulPartitionedCall:O K
'
_output_shapes
:���������d
 
_user_specified_nameinputs
�
C
'__inference_dropout_layer_call_fn_47711

inputs
identity�
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:���������d* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8� *K
fFRD
B__inference_dropout_layer_call_and_return_conditional_losses_462892
PartitionedCalll
IdentityIdentityPartitionedCall:output:0*
T0*'
_output_shapes
:���������d2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*&
_input_shapes
:���������d:O K
'
_output_shapes
:���������d
 
_user_specified_nameinputs
�
�
,__inference_layer_norm_2_layer_call_fn_47600

inputs
unknown:
	unknown_0:
identity��StatefulPartitionedCall�
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:���������*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8� *P
fKRI
G__inference_layer_norm_2_layer_call_and_return_conditional_losses_462372
StatefulPartitionedCall�
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*/
_output_shapes
:���������2

Identityh
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*2
_input_shapes!
:���������: : 22
StatefulPartitionedCallStatefulPartitionedCall:W S
/
_output_shapes
:���������
 
_user_specified_nameinputs
�
�
&__inference_conv_1_layer_call_fn_47495

inputs!
unknown:
	unknown_0:
identity��StatefulPartitionedCall�
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:���������*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8� *J
fERC
A__inference_conv_1_layer_call_and_return_conditional_losses_460962
StatefulPartitionedCall�
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*/
_output_shapes
:���������2

Identityh
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*2
_input_shapes!
:���������  : : 22
StatefulPartitionedCallStatefulPartitionedCall:W S
/
_output_shapes
:���������  
 
_user_specified_nameinputs
�
�
%__inference_dense_layer_call_fn_47696

inputs
unknown:	�d
	unknown_0:d
identity��StatefulPartitionedCall�
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:���������d*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8� *I
fDRB
@__inference_dense_layer_call_and_return_conditional_losses_462782
StatefulPartitionedCall{
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:���������d2

Identityh
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*+
_input_shapes
:����������: : 22
StatefulPartitionedCallStatefulPartitionedCall:P L
(
_output_shapes
:����������
 
_user_specified_nameinputs
�
^
B__inference_flatten_layer_call_and_return_conditional_losses_47687

inputs
identity_
ConstConst*
_output_shapes
:*
dtype0*
valueB"�����   2
Consth
ReshapeReshapeinputsConst:output:0*
T0*(
_output_shapes
:����������2	
Reshapee
IdentityIdentityReshape:output:0*
T0*(
_output_shapes
:����������2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*.
_input_shapes
:���������:W S
/
_output_shapes
:���������
 
_user_specified_nameinputs
�
�
A__inference_conv_2_layer_call_and_return_conditional_losses_47591

inputs8
conv2d_readvariableop_resource:-
biasadd_readvariableop_resource:
identity��BiasAdd/ReadVariableOp�Conv2D/ReadVariableOp�
Conv2D/ReadVariableOpReadVariableOpconv2d_readvariableop_resource*&
_output_shapes
:*
dtype02
Conv2D/ReadVariableOp�
Conv2DConv2DinputsConv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������*
paddingSAME*
strides
2
Conv2D�
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:*
dtype02
BiasAdd/ReadVariableOp�
BiasAddBiasAddConv2D:output:0BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2	
BiasAdd`
ReluReluBiasAdd:output:0*
T0*/
_output_shapes
:���������2
Reluu
IdentityIdentityRelu:activations:0^NoOp*
T0*/
_output_shapes
:���������2

Identity
NoOpNoOp^BiasAdd/ReadVariableOp^Conv2D/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*2
_input_shapes!
:���������: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
Conv2D/ReadVariableOpConv2D/ReadVariableOp:W S
/
_output_shapes
:���������
 
_user_specified_nameinputs
�)
�
__inference__traced_save_47820
file_prefix6
2savev2_cnn_model_conv_1_kernel_read_readvariableop4
0savev2_cnn_model_conv_1_bias_read_readvariableop;
7savev2_cnn_model_layer_norm_1_gamma_read_readvariableop:
6savev2_cnn_model_layer_norm_1_beta_read_readvariableop6
2savev2_cnn_model_conv_2_kernel_read_readvariableop4
0savev2_cnn_model_conv_2_bias_read_readvariableop;
7savev2_cnn_model_layer_norm_2_gamma_read_readvariableop:
6savev2_cnn_model_layer_norm_2_beta_read_readvariableop6
2savev2_cnn_model_conv_3_kernel_read_readvariableop4
0savev2_cnn_model_conv_3_bias_read_readvariableop5
1savev2_cnn_model_dense_kernel_read_readvariableop3
/savev2_cnn_model_dense_bias_read_readvariableop:
6savev2_cnn_model_raw_output_kernel_read_readvariableop8
4savev2_cnn_model_raw_output_bias_read_readvariableop
savev2_const_2

identity_1��MergeV2Checkpoints�
StaticRegexFullMatchStaticRegexFullMatchfile_prefix"/device:CPU:**
_output_shapes
: *
pattern
^s3://.*2
StaticRegexFullMatchc
ConstConst"/device:CPU:**
_output_shapes
: *
dtype0*
valueB B.part2
Constl
Const_1Const"/device:CPU:**
_output_shapes
: *
dtype0*
valueB B
_temp/part2	
Const_1�
SelectSelectStaticRegexFullMatch:output:0Const:output:0Const_1:output:0"/device:CPU:**
T0*
_output_shapes
: 2
Selectt

StringJoin
StringJoinfile_prefixSelect:output:0"/device:CPU:**
N*
_output_shapes
: 2

StringJoinZ

num_shardsConst*
_output_shapes
: *
dtype0*
value	B :2

num_shards
ShardedFilename/shardConst"/device:CPU:0*
_output_shapes
: *
dtype0*
value	B : 2
ShardedFilename/shard�
ShardedFilenameShardedFilenameStringJoin:output:0ShardedFilename/shard:output:0num_shards:output:0"/device:CPU:0*
_output_shapes
: 2
ShardedFilename�
SaveV2/tensor_namesConst"/device:CPU:0*
_output_shapes
:*
dtype0*�
value�B�B'conv1/kernel/.ATTRIBUTES/VARIABLE_VALUEB%conv1/bias/.ATTRIBUTES/VARIABLE_VALUEB&norm1/gamma/.ATTRIBUTES/VARIABLE_VALUEB%norm1/beta/.ATTRIBUTES/VARIABLE_VALUEB'conv2/kernel/.ATTRIBUTES/VARIABLE_VALUEB%conv2/bias/.ATTRIBUTES/VARIABLE_VALUEB&norm2/gamma/.ATTRIBUTES/VARIABLE_VALUEB%norm2/beta/.ATTRIBUTES/VARIABLE_VALUEB'conv3/kernel/.ATTRIBUTES/VARIABLE_VALUEB%conv3/bias/.ATTRIBUTES/VARIABLE_VALUEB(dense1/kernel/.ATTRIBUTES/VARIABLE_VALUEB&dense1/bias/.ATTRIBUTES/VARIABLE_VALUEB$y_/kernel/.ATTRIBUTES/VARIABLE_VALUEB"y_/bias/.ATTRIBUTES/VARIABLE_VALUEB_CHECKPOINTABLE_OBJECT_GRAPH2
SaveV2/tensor_names�
SaveV2/shape_and_slicesConst"/device:CPU:0*
_output_shapes
:*
dtype0*1
value(B&B B B B B B B B B B B B B B B 2
SaveV2/shape_and_slices�
SaveV2SaveV2ShardedFilename:filename:0SaveV2/tensor_names:output:0 SaveV2/shape_and_slices:output:02savev2_cnn_model_conv_1_kernel_read_readvariableop0savev2_cnn_model_conv_1_bias_read_readvariableop7savev2_cnn_model_layer_norm_1_gamma_read_readvariableop6savev2_cnn_model_layer_norm_1_beta_read_readvariableop2savev2_cnn_model_conv_2_kernel_read_readvariableop0savev2_cnn_model_conv_2_bias_read_readvariableop7savev2_cnn_model_layer_norm_2_gamma_read_readvariableop6savev2_cnn_model_layer_norm_2_beta_read_readvariableop2savev2_cnn_model_conv_3_kernel_read_readvariableop0savev2_cnn_model_conv_3_bias_read_readvariableop1savev2_cnn_model_dense_kernel_read_readvariableop/savev2_cnn_model_dense_bias_read_readvariableop6savev2_cnn_model_raw_output_kernel_read_readvariableop4savev2_cnn_model_raw_output_bias_read_readvariableopsavev2_const_2"/device:CPU:0*
_output_shapes
 *
dtypes
22
SaveV2�
&MergeV2Checkpoints/checkpoint_prefixesPackShardedFilename:filename:0^SaveV2"/device:CPU:0*
N*
T0*
_output_shapes
:2(
&MergeV2Checkpoints/checkpoint_prefixes�
MergeV2CheckpointsMergeV2Checkpoints/MergeV2Checkpoints/checkpoint_prefixes:output:0file_prefix"/device:CPU:0*
_output_shapes
 2
MergeV2Checkpointsr
IdentityIdentityfile_prefix^MergeV2Checkpoints"/device:CPU:0*
T0*
_output_shapes
: 2

Identity_

Identity_1IdentityIdentity:output:0^NoOp*
T0*
_output_shapes
: 2

Identity_1c
NoOpNoOp^MergeV2Checkpoints*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"!

identity_1Identity_1:output:0*�
_input_shapes�
�: :::::::::::	�d:d:d:: 2(
MergeV2CheckpointsMergeV2Checkpoints:C ?

_output_shapes
: 
%
_user_specified_namefile_prefix:,(
&
_output_shapes
:: 

_output_shapes
:: 

_output_shapes
:: 

_output_shapes
::,(
&
_output_shapes
:: 

_output_shapes
:: 

_output_shapes
:: 

_output_shapes
::,	(
&
_output_shapes
:: 


_output_shapes
::%!

_output_shapes
:	�d: 

_output_shapes
:d:$ 

_output_shapes

:d: 

_output_shapes
::

_output_shapes
: 
�
`
B__inference_dropout_layer_call_and_return_conditional_losses_46289

inputs

identity_1Z
IdentityIdentityinputs*
T0*'
_output_shapes
:���������d2

Identityi

Identity_1IdentityIdentity:output:0*
T0*'
_output_shapes
:���������d2

Identity_1"!

identity_1Identity_1:output:0*(
_construction_contextkEagerRuntime*&
_input_shapes
:���������d:O K
'
_output_shapes
:���������d
 
_user_specified_nameinputs
�
�
)__inference_cnn_model_layer_call_fn_46876
input_1
unknown
	unknown_0#
	unknown_1:
	unknown_2:
	unknown_3:
	unknown_4:#
	unknown_5:
	unknown_6:
	unknown_7:
	unknown_8:#
	unknown_9:

unknown_10:

unknown_11:	�d

unknown_12:d

unknown_13:d

unknown_14:
identity��StatefulPartitionedCall�
StatefulPartitionedCallStatefulPartitionedCallinput_1unknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4	unknown_5	unknown_6	unknown_7	unknown_8	unknown_9
unknown_10
unknown_11
unknown_12
unknown_13
unknown_14*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:���������*0
_read_only_resource_inputs
	
*-
config_proto

CPU

GPU 2J 8� *M
fHRF
D__inference_cnn_model_layer_call_and_return_conditional_losses_465272
StatefulPartitionedCall{
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:���������2

Identityh
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*V
_input_shapesE
C:���������  ::: : : : : : : : : : : : : : 22
StatefulPartitionedCallStatefulPartitionedCall:X T
/
_output_shapes
:���������  
!
_user_specified_name	input_1: 

_output_shapes
:: 

_output_shapes
:
�
�
A__inference_conv_2_layer_call_and_return_conditional_losses_46175

inputs8
conv2d_readvariableop_resource:-
biasadd_readvariableop_resource:
identity��BiasAdd/ReadVariableOp�Conv2D/ReadVariableOp�
Conv2D/ReadVariableOpReadVariableOpconv2d_readvariableop_resource*&
_output_shapes
:*
dtype02
Conv2D/ReadVariableOp�
Conv2DConv2DinputsConv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������*
paddingSAME*
strides
2
Conv2D�
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:*
dtype02
BiasAdd/ReadVariableOp�
BiasAddBiasAddConv2D:output:0BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2	
BiasAdd`
ReluReluBiasAdd:output:0*
T0*/
_output_shapes
:���������2
Reluu
IdentityIdentityRelu:activations:0^NoOp*
T0*/
_output_shapes
:���������2

Identity
NoOpNoOp^BiasAdd/ReadVariableOp^Conv2D/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*2
_input_shapes!
:���������: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
Conv2D/ReadVariableOpConv2D/ReadVariableOp:W S
/
_output_shapes
:���������
 
_user_specified_nameinputs
��
�
 __inference__wrapped_model_46074
input_1
cnn_model_sub_y
cnn_model_truediv_yI
/cnn_model_conv_1_conv2d_readvariableop_resource:>
0cnn_model_conv_1_biasadd_readvariableop_resource:B
4cnn_model_layer_norm_1_mul_4_readvariableop_resource:@
2cnn_model_layer_norm_1_add_readvariableop_resource:I
/cnn_model_conv_2_conv2d_readvariableop_resource:>
0cnn_model_conv_2_biasadd_readvariableop_resource:B
4cnn_model_layer_norm_2_mul_4_readvariableop_resource:@
2cnn_model_layer_norm_2_add_readvariableop_resource:I
/cnn_model_conv_3_conv2d_readvariableop_resource:>
0cnn_model_conv_3_biasadd_readvariableop_resource:A
.cnn_model_dense_matmul_readvariableop_resource:	�d=
/cnn_model_dense_biasadd_readvariableop_resource:dE
3cnn_model_raw_output_matmul_readvariableop_resource:dB
4cnn_model_raw_output_biasadd_readvariableop_resource:
identity��'cnn_model/conv_1/BiasAdd/ReadVariableOp�&cnn_model/conv_1/Conv2D/ReadVariableOp�'cnn_model/conv_2/BiasAdd/ReadVariableOp�&cnn_model/conv_2/Conv2D/ReadVariableOp�'cnn_model/conv_3/BiasAdd/ReadVariableOp�&cnn_model/conv_3/Conv2D/ReadVariableOp�&cnn_model/dense/BiasAdd/ReadVariableOp�%cnn_model/dense/MatMul/ReadVariableOp�)cnn_model/layer_norm_1/add/ReadVariableOp�+cnn_model/layer_norm_1/mul_4/ReadVariableOp�)cnn_model/layer_norm_2/add/ReadVariableOp�+cnn_model/layer_norm_2/mul_4/ReadVariableOp�+cnn_model/raw_output/BiasAdd/ReadVariableOp�*cnn_model/raw_output/MatMul/ReadVariableOpy
cnn_model/subSubinput_1cnn_model_sub_y*
T0*/
_output_shapes
:���������  2
cnn_model/sub�
cnn_model/truedivRealDivcnn_model/sub:z:0cnn_model_truediv_y*
T0*/
_output_shapes
:���������  2
cnn_model/truediv�
&cnn_model/conv_1/Conv2D/ReadVariableOpReadVariableOp/cnn_model_conv_1_conv2d_readvariableop_resource*&
_output_shapes
:*
dtype02(
&cnn_model/conv_1/Conv2D/ReadVariableOp�
cnn_model/conv_1/Conv2DConv2Dcnn_model/truediv:z:0.cnn_model/conv_1/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������*
paddingSAME*
strides
2
cnn_model/conv_1/Conv2D�
'cnn_model/conv_1/BiasAdd/ReadVariableOpReadVariableOp0cnn_model_conv_1_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02)
'cnn_model/conv_1/BiasAdd/ReadVariableOp�
cnn_model/conv_1/BiasAddBiasAdd cnn_model/conv_1/Conv2D:output:0/cnn_model/conv_1/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
cnn_model/conv_1/BiasAdd�
cnn_model/conv_1/ReluRelu!cnn_model/conv_1/BiasAdd:output:0*
T0*/
_output_shapes
:���������2
cnn_model/conv_1/Relu�
cnn_model/layer_norm_1/ShapeShape#cnn_model/conv_1/Relu:activations:0*
T0*
_output_shapes
:2
cnn_model/layer_norm_1/Shape�
*cnn_model/layer_norm_1/strided_slice/stackConst*
_output_shapes
:*
dtype0*
valueB: 2,
*cnn_model/layer_norm_1/strided_slice/stack�
,cnn_model/layer_norm_1/strided_slice/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2.
,cnn_model/layer_norm_1/strided_slice/stack_1�
,cnn_model/layer_norm_1/strided_slice/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2.
,cnn_model/layer_norm_1/strided_slice/stack_2�
$cnn_model/layer_norm_1/strided_sliceStridedSlice%cnn_model/layer_norm_1/Shape:output:03cnn_model/layer_norm_1/strided_slice/stack:output:05cnn_model/layer_norm_1/strided_slice/stack_1:output:05cnn_model/layer_norm_1/strided_slice/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2&
$cnn_model/layer_norm_1/strided_slice~
cnn_model/layer_norm_1/mul/xConst*
_output_shapes
: *
dtype0*
value	B :2
cnn_model/layer_norm_1/mul/x�
cnn_model/layer_norm_1/mulMul%cnn_model/layer_norm_1/mul/x:output:0-cnn_model/layer_norm_1/strided_slice:output:0*
T0*
_output_shapes
: 2
cnn_model/layer_norm_1/mul�
,cnn_model/layer_norm_1/strided_slice_1/stackConst*
_output_shapes
:*
dtype0*
valueB:2.
,cnn_model/layer_norm_1/strided_slice_1/stack�
.cnn_model/layer_norm_1/strided_slice_1/stack_1Const*
_output_shapes
:*
dtype0*
valueB:20
.cnn_model/layer_norm_1/strided_slice_1/stack_1�
.cnn_model/layer_norm_1/strided_slice_1/stack_2Const*
_output_shapes
:*
dtype0*
valueB:20
.cnn_model/layer_norm_1/strided_slice_1/stack_2�
&cnn_model/layer_norm_1/strided_slice_1StridedSlice%cnn_model/layer_norm_1/Shape:output:05cnn_model/layer_norm_1/strided_slice_1/stack:output:07cnn_model/layer_norm_1/strided_slice_1/stack_1:output:07cnn_model/layer_norm_1/strided_slice_1/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2(
&cnn_model/layer_norm_1/strided_slice_1�
cnn_model/layer_norm_1/mul_1Mulcnn_model/layer_norm_1/mul:z:0/cnn_model/layer_norm_1/strided_slice_1:output:0*
T0*
_output_shapes
: 2
cnn_model/layer_norm_1/mul_1�
,cnn_model/layer_norm_1/strided_slice_2/stackConst*
_output_shapes
:*
dtype0*
valueB:2.
,cnn_model/layer_norm_1/strided_slice_2/stack�
.cnn_model/layer_norm_1/strided_slice_2/stack_1Const*
_output_shapes
:*
dtype0*
valueB:20
.cnn_model/layer_norm_1/strided_slice_2/stack_1�
.cnn_model/layer_norm_1/strided_slice_2/stack_2Const*
_output_shapes
:*
dtype0*
valueB:20
.cnn_model/layer_norm_1/strided_slice_2/stack_2�
&cnn_model/layer_norm_1/strided_slice_2StridedSlice%cnn_model/layer_norm_1/Shape:output:05cnn_model/layer_norm_1/strided_slice_2/stack:output:07cnn_model/layer_norm_1/strided_slice_2/stack_1:output:07cnn_model/layer_norm_1/strided_slice_2/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2(
&cnn_model/layer_norm_1/strided_slice_2�
cnn_model/layer_norm_1/mul_2Mul cnn_model/layer_norm_1/mul_1:z:0/cnn_model/layer_norm_1/strided_slice_2:output:0*
T0*
_output_shapes
: 2
cnn_model/layer_norm_1/mul_2�
,cnn_model/layer_norm_1/strided_slice_3/stackConst*
_output_shapes
:*
dtype0*
valueB:2.
,cnn_model/layer_norm_1/strided_slice_3/stack�
.cnn_model/layer_norm_1/strided_slice_3/stack_1Const*
_output_shapes
:*
dtype0*
valueB:20
.cnn_model/layer_norm_1/strided_slice_3/stack_1�
.cnn_model/layer_norm_1/strided_slice_3/stack_2Const*
_output_shapes
:*
dtype0*
valueB:20
.cnn_model/layer_norm_1/strided_slice_3/stack_2�
&cnn_model/layer_norm_1/strided_slice_3StridedSlice%cnn_model/layer_norm_1/Shape:output:05cnn_model/layer_norm_1/strided_slice_3/stack:output:07cnn_model/layer_norm_1/strided_slice_3/stack_1:output:07cnn_model/layer_norm_1/strided_slice_3/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2(
&cnn_model/layer_norm_1/strided_slice_3�
cnn_model/layer_norm_1/mul_3/xConst*
_output_shapes
: *
dtype0*
value	B :2 
cnn_model/layer_norm_1/mul_3/x�
cnn_model/layer_norm_1/mul_3Mul'cnn_model/layer_norm_1/mul_3/x:output:0/cnn_model/layer_norm_1/strided_slice_3:output:0*
T0*
_output_shapes
: 2
cnn_model/layer_norm_1/mul_3�
&cnn_model/layer_norm_1/Reshape/shape/0Const*
_output_shapes
: *
dtype0*
value	B :2(
&cnn_model/layer_norm_1/Reshape/shape/0�
&cnn_model/layer_norm_1/Reshape/shape/3Const*
_output_shapes
: *
dtype0*
value	B :2(
&cnn_model/layer_norm_1/Reshape/shape/3�
$cnn_model/layer_norm_1/Reshape/shapePack/cnn_model/layer_norm_1/Reshape/shape/0:output:0 cnn_model/layer_norm_1/mul_2:z:0 cnn_model/layer_norm_1/mul_3:z:0/cnn_model/layer_norm_1/Reshape/shape/3:output:0*
N*
T0*
_output_shapes
:2&
$cnn_model/layer_norm_1/Reshape/shape�
cnn_model/layer_norm_1/ReshapeReshape#cnn_model/conv_1/Relu:activations:0-cnn_model/layer_norm_1/Reshape/shape:output:0*
T0*8
_output_shapes&
$:"������������������2 
cnn_model/layer_norm_1/Reshape�
"cnn_model/layer_norm_1/ones/Less/yConst*
_output_shapes
: *
dtype0*
value
B :�2$
"cnn_model/layer_norm_1/ones/Less/y�
 cnn_model/layer_norm_1/ones/LessLess cnn_model/layer_norm_1/mul_2:z:0+cnn_model/layer_norm_1/ones/Less/y:output:0*
T0*
_output_shapes
: 2"
 cnn_model/layer_norm_1/ones/Less�
"cnn_model/layer_norm_1/ones/packedPack cnn_model/layer_norm_1/mul_2:z:0*
N*
T0*
_output_shapes
:2$
"cnn_model/layer_norm_1/ones/packed�
!cnn_model/layer_norm_1/ones/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *  �?2#
!cnn_model/layer_norm_1/ones/Const�
cnn_model/layer_norm_1/onesFill+cnn_model/layer_norm_1/ones/packed:output:0*cnn_model/layer_norm_1/ones/Const:output:0*
T0*#
_output_shapes
:���������2
cnn_model/layer_norm_1/ones�
#cnn_model/layer_norm_1/zeros/Less/yConst*
_output_shapes
: *
dtype0*
value
B :�2%
#cnn_model/layer_norm_1/zeros/Less/y�
!cnn_model/layer_norm_1/zeros/LessLess cnn_model/layer_norm_1/mul_2:z:0,cnn_model/layer_norm_1/zeros/Less/y:output:0*
T0*
_output_shapes
: 2#
!cnn_model/layer_norm_1/zeros/Less�
#cnn_model/layer_norm_1/zeros/packedPack cnn_model/layer_norm_1/mul_2:z:0*
N*
T0*
_output_shapes
:2%
#cnn_model/layer_norm_1/zeros/packed�
"cnn_model/layer_norm_1/zeros/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *    2$
"cnn_model/layer_norm_1/zeros/Const�
cnn_model/layer_norm_1/zerosFill,cnn_model/layer_norm_1/zeros/packed:output:0+cnn_model/layer_norm_1/zeros/Const:output:0*
T0*#
_output_shapes
:���������2
cnn_model/layer_norm_1/zeros
cnn_model/layer_norm_1/ConstConst*
_output_shapes
: *
dtype0*
valueB 2
cnn_model/layer_norm_1/Const�
cnn_model/layer_norm_1/Const_1Const*
_output_shapes
: *
dtype0*
valueB 2 
cnn_model/layer_norm_1/Const_1�
'cnn_model/layer_norm_1/FusedBatchNormV3FusedBatchNormV3'cnn_model/layer_norm_1/Reshape:output:0$cnn_model/layer_norm_1/ones:output:0%cnn_model/layer_norm_1/zeros:output:0%cnn_model/layer_norm_1/Const:output:0'cnn_model/layer_norm_1/Const_1:output:0*
T0*
U0*x
_output_shapesf
d:"������������������:���������:���������:���������:���������:*
data_formatNCHW*
epsilon%o�:2)
'cnn_model/layer_norm_1/FusedBatchNormV3�
 cnn_model/layer_norm_1/Reshape_1Reshape+cnn_model/layer_norm_1/FusedBatchNormV3:y:0%cnn_model/layer_norm_1/Shape:output:0*
T0*/
_output_shapes
:���������2"
 cnn_model/layer_norm_1/Reshape_1�
+cnn_model/layer_norm_1/mul_4/ReadVariableOpReadVariableOp4cnn_model_layer_norm_1_mul_4_readvariableop_resource*
_output_shapes
:*
dtype02-
+cnn_model/layer_norm_1/mul_4/ReadVariableOp�
cnn_model/layer_norm_1/mul_4Mul)cnn_model/layer_norm_1/Reshape_1:output:03cnn_model/layer_norm_1/mul_4/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
cnn_model/layer_norm_1/mul_4�
)cnn_model/layer_norm_1/add/ReadVariableOpReadVariableOp2cnn_model_layer_norm_1_add_readvariableop_resource*
_output_shapes
:*
dtype02+
)cnn_model/layer_norm_1/add/ReadVariableOp�
cnn_model/layer_norm_1/addAddV2 cnn_model/layer_norm_1/mul_4:z:01cnn_model/layer_norm_1/add/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
cnn_model/layer_norm_1/add�
&cnn_model/conv_2/Conv2D/ReadVariableOpReadVariableOp/cnn_model_conv_2_conv2d_readvariableop_resource*&
_output_shapes
:*
dtype02(
&cnn_model/conv_2/Conv2D/ReadVariableOp�
cnn_model/conv_2/Conv2DConv2Dcnn_model/layer_norm_1/add:z:0.cnn_model/conv_2/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������*
paddingSAME*
strides
2
cnn_model/conv_2/Conv2D�
'cnn_model/conv_2/BiasAdd/ReadVariableOpReadVariableOp0cnn_model_conv_2_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02)
'cnn_model/conv_2/BiasAdd/ReadVariableOp�
cnn_model/conv_2/BiasAddBiasAdd cnn_model/conv_2/Conv2D:output:0/cnn_model/conv_2/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
cnn_model/conv_2/BiasAdd�
cnn_model/conv_2/ReluRelu!cnn_model/conv_2/BiasAdd:output:0*
T0*/
_output_shapes
:���������2
cnn_model/conv_2/Relu�
cnn_model/layer_norm_2/ShapeShape#cnn_model/conv_2/Relu:activations:0*
T0*
_output_shapes
:2
cnn_model/layer_norm_2/Shape�
*cnn_model/layer_norm_2/strided_slice/stackConst*
_output_shapes
:*
dtype0*
valueB: 2,
*cnn_model/layer_norm_2/strided_slice/stack�
,cnn_model/layer_norm_2/strided_slice/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2.
,cnn_model/layer_norm_2/strided_slice/stack_1�
,cnn_model/layer_norm_2/strided_slice/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2.
,cnn_model/layer_norm_2/strided_slice/stack_2�
$cnn_model/layer_norm_2/strided_sliceStridedSlice%cnn_model/layer_norm_2/Shape:output:03cnn_model/layer_norm_2/strided_slice/stack:output:05cnn_model/layer_norm_2/strided_slice/stack_1:output:05cnn_model/layer_norm_2/strided_slice/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2&
$cnn_model/layer_norm_2/strided_slice~
cnn_model/layer_norm_2/mul/xConst*
_output_shapes
: *
dtype0*
value	B :2
cnn_model/layer_norm_2/mul/x�
cnn_model/layer_norm_2/mulMul%cnn_model/layer_norm_2/mul/x:output:0-cnn_model/layer_norm_2/strided_slice:output:0*
T0*
_output_shapes
: 2
cnn_model/layer_norm_2/mul�
,cnn_model/layer_norm_2/strided_slice_1/stackConst*
_output_shapes
:*
dtype0*
valueB:2.
,cnn_model/layer_norm_2/strided_slice_1/stack�
.cnn_model/layer_norm_2/strided_slice_1/stack_1Const*
_output_shapes
:*
dtype0*
valueB:20
.cnn_model/layer_norm_2/strided_slice_1/stack_1�
.cnn_model/layer_norm_2/strided_slice_1/stack_2Const*
_output_shapes
:*
dtype0*
valueB:20
.cnn_model/layer_norm_2/strided_slice_1/stack_2�
&cnn_model/layer_norm_2/strided_slice_1StridedSlice%cnn_model/layer_norm_2/Shape:output:05cnn_model/layer_norm_2/strided_slice_1/stack:output:07cnn_model/layer_norm_2/strided_slice_1/stack_1:output:07cnn_model/layer_norm_2/strided_slice_1/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2(
&cnn_model/layer_norm_2/strided_slice_1�
cnn_model/layer_norm_2/mul_1Mulcnn_model/layer_norm_2/mul:z:0/cnn_model/layer_norm_2/strided_slice_1:output:0*
T0*
_output_shapes
: 2
cnn_model/layer_norm_2/mul_1�
,cnn_model/layer_norm_2/strided_slice_2/stackConst*
_output_shapes
:*
dtype0*
valueB:2.
,cnn_model/layer_norm_2/strided_slice_2/stack�
.cnn_model/layer_norm_2/strided_slice_2/stack_1Const*
_output_shapes
:*
dtype0*
valueB:20
.cnn_model/layer_norm_2/strided_slice_2/stack_1�
.cnn_model/layer_norm_2/strided_slice_2/stack_2Const*
_output_shapes
:*
dtype0*
valueB:20
.cnn_model/layer_norm_2/strided_slice_2/stack_2�
&cnn_model/layer_norm_2/strided_slice_2StridedSlice%cnn_model/layer_norm_2/Shape:output:05cnn_model/layer_norm_2/strided_slice_2/stack:output:07cnn_model/layer_norm_2/strided_slice_2/stack_1:output:07cnn_model/layer_norm_2/strided_slice_2/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2(
&cnn_model/layer_norm_2/strided_slice_2�
cnn_model/layer_norm_2/mul_2Mul cnn_model/layer_norm_2/mul_1:z:0/cnn_model/layer_norm_2/strided_slice_2:output:0*
T0*
_output_shapes
: 2
cnn_model/layer_norm_2/mul_2�
,cnn_model/layer_norm_2/strided_slice_3/stackConst*
_output_shapes
:*
dtype0*
valueB:2.
,cnn_model/layer_norm_2/strided_slice_3/stack�
.cnn_model/layer_norm_2/strided_slice_3/stack_1Const*
_output_shapes
:*
dtype0*
valueB:20
.cnn_model/layer_norm_2/strided_slice_3/stack_1�
.cnn_model/layer_norm_2/strided_slice_3/stack_2Const*
_output_shapes
:*
dtype0*
valueB:20
.cnn_model/layer_norm_2/strided_slice_3/stack_2�
&cnn_model/layer_norm_2/strided_slice_3StridedSlice%cnn_model/layer_norm_2/Shape:output:05cnn_model/layer_norm_2/strided_slice_3/stack:output:07cnn_model/layer_norm_2/strided_slice_3/stack_1:output:07cnn_model/layer_norm_2/strided_slice_3/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_mask2(
&cnn_model/layer_norm_2/strided_slice_3�
cnn_model/layer_norm_2/mul_3/xConst*
_output_shapes
: *
dtype0*
value	B :2 
cnn_model/layer_norm_2/mul_3/x�
cnn_model/layer_norm_2/mul_3Mul'cnn_model/layer_norm_2/mul_3/x:output:0/cnn_model/layer_norm_2/strided_slice_3:output:0*
T0*
_output_shapes
: 2
cnn_model/layer_norm_2/mul_3�
&cnn_model/layer_norm_2/Reshape/shape/0Const*
_output_shapes
: *
dtype0*
value	B :2(
&cnn_model/layer_norm_2/Reshape/shape/0�
&cnn_model/layer_norm_2/Reshape/shape/3Const*
_output_shapes
: *
dtype0*
value	B :2(
&cnn_model/layer_norm_2/Reshape/shape/3�
$cnn_model/layer_norm_2/Reshape/shapePack/cnn_model/layer_norm_2/Reshape/shape/0:output:0 cnn_model/layer_norm_2/mul_2:z:0 cnn_model/layer_norm_2/mul_3:z:0/cnn_model/layer_norm_2/Reshape/shape/3:output:0*
N*
T0*
_output_shapes
:2&
$cnn_model/layer_norm_2/Reshape/shape�
cnn_model/layer_norm_2/ReshapeReshape#cnn_model/conv_2/Relu:activations:0-cnn_model/layer_norm_2/Reshape/shape:output:0*
T0*8
_output_shapes&
$:"������������������2 
cnn_model/layer_norm_2/Reshape�
"cnn_model/layer_norm_2/ones/Less/yConst*
_output_shapes
: *
dtype0*
value
B :�2$
"cnn_model/layer_norm_2/ones/Less/y�
 cnn_model/layer_norm_2/ones/LessLess cnn_model/layer_norm_2/mul_2:z:0+cnn_model/layer_norm_2/ones/Less/y:output:0*
T0*
_output_shapes
: 2"
 cnn_model/layer_norm_2/ones/Less�
"cnn_model/layer_norm_2/ones/packedPack cnn_model/layer_norm_2/mul_2:z:0*
N*
T0*
_output_shapes
:2$
"cnn_model/layer_norm_2/ones/packed�
!cnn_model/layer_norm_2/ones/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *  �?2#
!cnn_model/layer_norm_2/ones/Const�
cnn_model/layer_norm_2/onesFill+cnn_model/layer_norm_2/ones/packed:output:0*cnn_model/layer_norm_2/ones/Const:output:0*
T0*#
_output_shapes
:���������2
cnn_model/layer_norm_2/ones�
#cnn_model/layer_norm_2/zeros/Less/yConst*
_output_shapes
: *
dtype0*
value
B :�2%
#cnn_model/layer_norm_2/zeros/Less/y�
!cnn_model/layer_norm_2/zeros/LessLess cnn_model/layer_norm_2/mul_2:z:0,cnn_model/layer_norm_2/zeros/Less/y:output:0*
T0*
_output_shapes
: 2#
!cnn_model/layer_norm_2/zeros/Less�
#cnn_model/layer_norm_2/zeros/packedPack cnn_model/layer_norm_2/mul_2:z:0*
N*
T0*
_output_shapes
:2%
#cnn_model/layer_norm_2/zeros/packed�
"cnn_model/layer_norm_2/zeros/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *    2$
"cnn_model/layer_norm_2/zeros/Const�
cnn_model/layer_norm_2/zerosFill,cnn_model/layer_norm_2/zeros/packed:output:0+cnn_model/layer_norm_2/zeros/Const:output:0*
T0*#
_output_shapes
:���������2
cnn_model/layer_norm_2/zeros
cnn_model/layer_norm_2/ConstConst*
_output_shapes
: *
dtype0*
valueB 2
cnn_model/layer_norm_2/Const�
cnn_model/layer_norm_2/Const_1Const*
_output_shapes
: *
dtype0*
valueB 2 
cnn_model/layer_norm_2/Const_1�
'cnn_model/layer_norm_2/FusedBatchNormV3FusedBatchNormV3'cnn_model/layer_norm_2/Reshape:output:0$cnn_model/layer_norm_2/ones:output:0%cnn_model/layer_norm_2/zeros:output:0%cnn_model/layer_norm_2/Const:output:0'cnn_model/layer_norm_2/Const_1:output:0*
T0*
U0*x
_output_shapesf
d:"������������������:���������:���������:���������:���������:*
data_formatNCHW*
epsilon%o�:2)
'cnn_model/layer_norm_2/FusedBatchNormV3�
 cnn_model/layer_norm_2/Reshape_1Reshape+cnn_model/layer_norm_2/FusedBatchNormV3:y:0%cnn_model/layer_norm_2/Shape:output:0*
T0*/
_output_shapes
:���������2"
 cnn_model/layer_norm_2/Reshape_1�
+cnn_model/layer_norm_2/mul_4/ReadVariableOpReadVariableOp4cnn_model_layer_norm_2_mul_4_readvariableop_resource*
_output_shapes
:*
dtype02-
+cnn_model/layer_norm_2/mul_4/ReadVariableOp�
cnn_model/layer_norm_2/mul_4Mul)cnn_model/layer_norm_2/Reshape_1:output:03cnn_model/layer_norm_2/mul_4/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
cnn_model/layer_norm_2/mul_4�
)cnn_model/layer_norm_2/add/ReadVariableOpReadVariableOp2cnn_model_layer_norm_2_add_readvariableop_resource*
_output_shapes
:*
dtype02+
)cnn_model/layer_norm_2/add/ReadVariableOp�
cnn_model/layer_norm_2/addAddV2 cnn_model/layer_norm_2/mul_4:z:01cnn_model/layer_norm_2/add/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
cnn_model/layer_norm_2/add�
&cnn_model/conv_3/Conv2D/ReadVariableOpReadVariableOp/cnn_model_conv_3_conv2d_readvariableop_resource*&
_output_shapes
:*
dtype02(
&cnn_model/conv_3/Conv2D/ReadVariableOp�
cnn_model/conv_3/Conv2DConv2Dcnn_model/layer_norm_2/add:z:0.cnn_model/conv_3/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������*
paddingSAME*
strides
2
cnn_model/conv_3/Conv2D�
'cnn_model/conv_3/BiasAdd/ReadVariableOpReadVariableOp0cnn_model_conv_3_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02)
'cnn_model/conv_3/BiasAdd/ReadVariableOp�
cnn_model/conv_3/BiasAddBiasAdd cnn_model/conv_3/Conv2D:output:0/cnn_model/conv_3/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2
cnn_model/conv_3/BiasAdd�
cnn_model/conv_3/ReluRelu!cnn_model/conv_3/BiasAdd:output:0*
T0*/
_output_shapes
:���������2
cnn_model/conv_3/Relu�
cnn_model/flatten/ConstConst*
_output_shapes
:*
dtype0*
valueB"�����   2
cnn_model/flatten/Const�
cnn_model/flatten/ReshapeReshape#cnn_model/conv_3/Relu:activations:0 cnn_model/flatten/Const:output:0*
T0*(
_output_shapes
:����������2
cnn_model/flatten/Reshape�
%cnn_model/dense/MatMul/ReadVariableOpReadVariableOp.cnn_model_dense_matmul_readvariableop_resource*
_output_shapes
:	�d*
dtype02'
%cnn_model/dense/MatMul/ReadVariableOp�
cnn_model/dense/MatMulMatMul"cnn_model/flatten/Reshape:output:0-cnn_model/dense/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������d2
cnn_model/dense/MatMul�
&cnn_model/dense/BiasAdd/ReadVariableOpReadVariableOp/cnn_model_dense_biasadd_readvariableop_resource*
_output_shapes
:d*
dtype02(
&cnn_model/dense/BiasAdd/ReadVariableOp�
cnn_model/dense/BiasAddBiasAdd cnn_model/dense/MatMul:product:0.cnn_model/dense/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������d2
cnn_model/dense/BiasAdd�
cnn_model/dropout/IdentityIdentity cnn_model/dense/BiasAdd:output:0*
T0*'
_output_shapes
:���������d2
cnn_model/dropout/Identity�
*cnn_model/raw_output/MatMul/ReadVariableOpReadVariableOp3cnn_model_raw_output_matmul_readvariableop_resource*
_output_shapes

:d*
dtype02,
*cnn_model/raw_output/MatMul/ReadVariableOp�
cnn_model/raw_output/MatMulMatMul#cnn_model/dropout/Identity:output:02cnn_model/raw_output/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������2
cnn_model/raw_output/MatMul�
+cnn_model/raw_output/BiasAdd/ReadVariableOpReadVariableOp4cnn_model_raw_output_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02-
+cnn_model/raw_output/BiasAdd/ReadVariableOp�
cnn_model/raw_output/BiasAddBiasAdd%cnn_model/raw_output/MatMul:product:03cnn_model/raw_output/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������2
cnn_model/raw_output/BiasAdd�
cnn_model/raw_output/SoftmaxSoftmax%cnn_model/raw_output/BiasAdd:output:0*
T0*'
_output_shapes
:���������2
cnn_model/raw_output/Softmax�
IdentityIdentity&cnn_model/raw_output/Softmax:softmax:0^NoOp*
T0*'
_output_shapes
:���������2

Identity�
NoOpNoOp(^cnn_model/conv_1/BiasAdd/ReadVariableOp'^cnn_model/conv_1/Conv2D/ReadVariableOp(^cnn_model/conv_2/BiasAdd/ReadVariableOp'^cnn_model/conv_2/Conv2D/ReadVariableOp(^cnn_model/conv_3/BiasAdd/ReadVariableOp'^cnn_model/conv_3/Conv2D/ReadVariableOp'^cnn_model/dense/BiasAdd/ReadVariableOp&^cnn_model/dense/MatMul/ReadVariableOp*^cnn_model/layer_norm_1/add/ReadVariableOp,^cnn_model/layer_norm_1/mul_4/ReadVariableOp*^cnn_model/layer_norm_2/add/ReadVariableOp,^cnn_model/layer_norm_2/mul_4/ReadVariableOp,^cnn_model/raw_output/BiasAdd/ReadVariableOp+^cnn_model/raw_output/MatMul/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*V
_input_shapesE
C:���������  ::: : : : : : : : : : : : : : 2R
'cnn_model/conv_1/BiasAdd/ReadVariableOp'cnn_model/conv_1/BiasAdd/ReadVariableOp2P
&cnn_model/conv_1/Conv2D/ReadVariableOp&cnn_model/conv_1/Conv2D/ReadVariableOp2R
'cnn_model/conv_2/BiasAdd/ReadVariableOp'cnn_model/conv_2/BiasAdd/ReadVariableOp2P
&cnn_model/conv_2/Conv2D/ReadVariableOp&cnn_model/conv_2/Conv2D/ReadVariableOp2R
'cnn_model/conv_3/BiasAdd/ReadVariableOp'cnn_model/conv_3/BiasAdd/ReadVariableOp2P
&cnn_model/conv_3/Conv2D/ReadVariableOp&cnn_model/conv_3/Conv2D/ReadVariableOp2P
&cnn_model/dense/BiasAdd/ReadVariableOp&cnn_model/dense/BiasAdd/ReadVariableOp2N
%cnn_model/dense/MatMul/ReadVariableOp%cnn_model/dense/MatMul/ReadVariableOp2V
)cnn_model/layer_norm_1/add/ReadVariableOp)cnn_model/layer_norm_1/add/ReadVariableOp2Z
+cnn_model/layer_norm_1/mul_4/ReadVariableOp+cnn_model/layer_norm_1/mul_4/ReadVariableOp2V
)cnn_model/layer_norm_2/add/ReadVariableOp)cnn_model/layer_norm_2/add/ReadVariableOp2Z
+cnn_model/layer_norm_2/mul_4/ReadVariableOp+cnn_model/layer_norm_2/mul_4/ReadVariableOp2Z
+cnn_model/raw_output/BiasAdd/ReadVariableOp+cnn_model/raw_output/BiasAdd/ReadVariableOp2X
*cnn_model/raw_output/MatMul/ReadVariableOp*cnn_model/raw_output/MatMul/ReadVariableOp:X T
/
_output_shapes
:���������  
!
_user_specified_name	input_1: 

_output_shapes
:: 

_output_shapes
:
�
`
B__inference_dropout_layer_call_and_return_conditional_losses_47721

inputs

identity_1Z
IdentityIdentityinputs*
T0*'
_output_shapes
:���������d2

Identityi

Identity_1IdentityIdentity:output:0*
T0*'
_output_shapes
:���������d2

Identity_1"!

identity_1Identity_1:output:0*(
_construction_contextkEagerRuntime*&
_input_shapes
:���������d:O K
'
_output_shapes
:���������d
 
_user_specified_nameinputs
�
�
A__inference_conv_1_layer_call_and_return_conditional_losses_46096

inputs8
conv2d_readvariableop_resource:-
biasadd_readvariableop_resource:
identity��BiasAdd/ReadVariableOp�Conv2D/ReadVariableOp�
Conv2D/ReadVariableOpReadVariableOpconv2d_readvariableop_resource*&
_output_shapes
:*
dtype02
Conv2D/ReadVariableOp�
Conv2DConv2DinputsConv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������*
paddingSAME*
strides
2
Conv2D�
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:*
dtype02
BiasAdd/ReadVariableOp�
BiasAddBiasAddConv2D:output:0BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2	
BiasAdd`
ReluReluBiasAdd:output:0*
T0*/
_output_shapes
:���������2
Reluu
IdentityIdentityRelu:activations:0^NoOp*
T0*/
_output_shapes
:���������2

Identity
NoOpNoOp^BiasAdd/ReadVariableOp^Conv2D/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*2
_input_shapes!
:���������  : : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
Conv2D/ReadVariableOpConv2D/ReadVariableOp:W S
/
_output_shapes
:���������  
 
_user_specified_nameinputs
�
`
'__inference_dropout_layer_call_fn_47716

inputs
identity��StatefulPartitionedCall�
StatefulPartitionedCallStatefulPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:���������d* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8� *K
fFRD
B__inference_dropout_layer_call_and_return_conditional_losses_463742
StatefulPartitionedCall{
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:���������d2

Identityh
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*&
_input_shapes
:���������d22
StatefulPartitionedCallStatefulPartitionedCall:O K
'
_output_shapes
:���������d
 
_user_specified_nameinputs
�
�
,__inference_layer_norm_1_layer_call_fn_47515

inputs
unknown:
	unknown_0:
identity��StatefulPartitionedCall�
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:���������*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8� *P
fKRI
G__inference_layer_norm_1_layer_call_and_return_conditional_losses_461582
StatefulPartitionedCall�
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*/
_output_shapes
:���������2

Identityh
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*2
_input_shapes!
:���������: : 22
StatefulPartitionedCallStatefulPartitionedCall:W S
/
_output_shapes
:���������
 
_user_specified_nameinputs
�
�
A__inference_conv_3_layer_call_and_return_conditional_losses_47676

inputs8
conv2d_readvariableop_resource:-
biasadd_readvariableop_resource:
identity��BiasAdd/ReadVariableOp�Conv2D/ReadVariableOp�
Conv2D/ReadVariableOpReadVariableOpconv2d_readvariableop_resource*&
_output_shapes
:*
dtype02
Conv2D/ReadVariableOp�
Conv2DConv2DinputsConv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������*
paddingSAME*
strides
2
Conv2D�
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:*
dtype02
BiasAdd/ReadVariableOp�
BiasAddBiasAddConv2D:output:0BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:���������2	
BiasAdd`
ReluReluBiasAdd:output:0*
T0*/
_output_shapes
:���������2
Reluu
IdentityIdentityRelu:activations:0^NoOp*
T0*/
_output_shapes
:���������2

Identity
NoOpNoOp^BiasAdd/ReadVariableOp^Conv2D/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*2
_input_shapes!
:���������: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
Conv2D/ReadVariableOpConv2D/ReadVariableOp:W S
/
_output_shapes
:���������
 
_user_specified_nameinputs
�
�
E__inference_raw_output_layer_call_and_return_conditional_losses_47753

inputs0
matmul_readvariableop_resource:d-
biasadd_readvariableop_resource:
identity��BiasAdd/ReadVariableOp�MatMul/ReadVariableOp�
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes

:d*
dtype02
MatMul/ReadVariableOps
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������2
MatMul�
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:*
dtype02
BiasAdd/ReadVariableOp�
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������2	
BiasAdda
SoftmaxSoftmaxBiasAdd:output:0*
T0*'
_output_shapes
:���������2	
Softmaxl
IdentityIdentitySoftmax:softmax:0^NoOp*
T0*'
_output_shapes
:���������2

Identity
NoOpNoOp^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime**
_input_shapes
:���������d: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp:O K
'
_output_shapes
:���������d
 
_user_specified_nameinputs"�L
saver_filename:0StatefulPartitionedCall_1:0StatefulPartitionedCall_28"
saved_model_main_op

NoOp*>
__saved_model_init_op%#
__saved_model_init_op

NoOp*�
serving_default�
C
input_18
serving_default_input_1:0���������  <
output_10
StatefulPartitionedCall:0���������tensorflow/serving/predict:��
�
	conv1
	norm1
	conv2
	norm2
	conv3
flat

dense1
	drop1
	y_

	variables
trainable_variables
regularization_losses
	keras_api

signatures
u__call__
v_default_save_signature
*w&call_and_return_all_conditional_losses"
_tf_keras_model
�

kernel
bias
	variables
trainable_variables
regularization_losses
	keras_api
x__call__
*y&call_and_return_all_conditional_losses"
_tf_keras_layer
�
axis
	gamma
beta
	variables
trainable_variables
regularization_losses
	keras_api
z__call__
*{&call_and_return_all_conditional_losses"
_tf_keras_layer
�

kernel
bias
	variables
trainable_variables
 regularization_losses
!	keras_api
|__call__
*}&call_and_return_all_conditional_losses"
_tf_keras_layer
�
"axis
	#gamma
$beta
%	variables
&trainable_variables
'regularization_losses
(	keras_api
~__call__
*&call_and_return_all_conditional_losses"
_tf_keras_layer
�

)kernel
*bias
+	variables
,trainable_variables
-regularization_losses
.	keras_api
�__call__
+�&call_and_return_all_conditional_losses"
_tf_keras_layer
�
/	variables
0trainable_variables
1regularization_losses
2	keras_api
�__call__
+�&call_and_return_all_conditional_losses"
_tf_keras_layer
�

3kernel
4bias
5	variables
6trainable_variables
7regularization_losses
8	keras_api
�__call__
+�&call_and_return_all_conditional_losses"
_tf_keras_layer
�
9	variables
:trainable_variables
;regularization_losses
<	keras_api
�__call__
+�&call_and_return_all_conditional_losses"
_tf_keras_layer
�

=kernel
>bias
?	variables
@trainable_variables
Aregularization_losses
B	keras_api
�__call__
+�&call_and_return_all_conditional_losses"
_tf_keras_layer
�
0
1
2
3
4
5
#6
$7
)8
*9
310
411
=12
>13"
trackable_list_wrapper
�
0
1
2
3
4
5
#6
$7
)8
*9
310
411
=12
>13"
trackable_list_wrapper
 "
trackable_list_wrapper
�
Cmetrics

	variables
Dnon_trainable_variables
trainable_variables
regularization_losses

Elayers
Flayer_metrics
Glayer_regularization_losses
u__call__
v_default_save_signature
*w&call_and_return_all_conditional_losses
&w"call_and_return_conditional_losses"
_generic_user_object
-
�serving_default"
signature_map
1:/2cnn_model/conv_1/kernel
#:!2cnn_model/conv_1/bias
.
0
1"
trackable_list_wrapper
.
0
1"
trackable_list_wrapper
 "
trackable_list_wrapper
�
Hmetrics
	variables
Inon_trainable_variables
trainable_variables
regularization_losses

Jlayers
Klayer_metrics
Llayer_regularization_losses
x__call__
*y&call_and_return_all_conditional_losses
&y"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
*:(2cnn_model/layer_norm_1/gamma
):'2cnn_model/layer_norm_1/beta
.
0
1"
trackable_list_wrapper
.
0
1"
trackable_list_wrapper
 "
trackable_list_wrapper
�
Mmetrics
	variables
Nnon_trainable_variables
trainable_variables
regularization_losses

Olayers
Player_metrics
Qlayer_regularization_losses
z__call__
*{&call_and_return_all_conditional_losses
&{"call_and_return_conditional_losses"
_generic_user_object
1:/2cnn_model/conv_2/kernel
#:!2cnn_model/conv_2/bias
.
0
1"
trackable_list_wrapper
.
0
1"
trackable_list_wrapper
 "
trackable_list_wrapper
�
Rmetrics
	variables
Snon_trainable_variables
trainable_variables
 regularization_losses

Tlayers
Ulayer_metrics
Vlayer_regularization_losses
|__call__
*}&call_and_return_all_conditional_losses
&}"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
*:(2cnn_model/layer_norm_2/gamma
):'2cnn_model/layer_norm_2/beta
.
#0
$1"
trackable_list_wrapper
.
#0
$1"
trackable_list_wrapper
 "
trackable_list_wrapper
�
Wmetrics
%	variables
Xnon_trainable_variables
&trainable_variables
'regularization_losses

Ylayers
Zlayer_metrics
[layer_regularization_losses
~__call__
*&call_and_return_all_conditional_losses
&"call_and_return_conditional_losses"
_generic_user_object
1:/2cnn_model/conv_3/kernel
#:!2cnn_model/conv_3/bias
.
)0
*1"
trackable_list_wrapper
.
)0
*1"
trackable_list_wrapper
 "
trackable_list_wrapper
�
\metrics
+	variables
]non_trainable_variables
,trainable_variables
-regularization_losses

^layers
_layer_metrics
`layer_regularization_losses
�__call__
+�&call_and_return_all_conditional_losses
'�"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
�
ametrics
/	variables
bnon_trainable_variables
0trainable_variables
1regularization_losses

clayers
dlayer_metrics
elayer_regularization_losses
�__call__
+�&call_and_return_all_conditional_losses
'�"call_and_return_conditional_losses"
_generic_user_object
):'	�d2cnn_model/dense/kernel
": d2cnn_model/dense/bias
.
30
41"
trackable_list_wrapper
.
30
41"
trackable_list_wrapper
 "
trackable_list_wrapper
�
fmetrics
5	variables
gnon_trainable_variables
6trainable_variables
7regularization_losses

hlayers
ilayer_metrics
jlayer_regularization_losses
�__call__
+�&call_and_return_all_conditional_losses
'�"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
�
kmetrics
9	variables
lnon_trainable_variables
:trainable_variables
;regularization_losses

mlayers
nlayer_metrics
olayer_regularization_losses
�__call__
+�&call_and_return_all_conditional_losses
'�"call_and_return_conditional_losses"
_generic_user_object
-:+d2cnn_model/raw_output/kernel
':%2cnn_model/raw_output/bias
.
=0
>1"
trackable_list_wrapper
.
=0
>1"
trackable_list_wrapper
 "
trackable_list_wrapper
�
pmetrics
?	variables
qnon_trainable_variables
@trainable_variables
Aregularization_losses

rlayers
slayer_metrics
tlayer_regularization_losses
�__call__
+�&call_and_return_all_conditional_losses
'�"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
_
0
1
2
3
4
5
6
7
	8"
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
�2�
)__inference_cnn_model_layer_call_fn_46765
)__inference_cnn_model_layer_call_fn_46802
)__inference_cnn_model_layer_call_fn_46839
)__inference_cnn_model_layer_call_fn_46876�
���
FullArgSpec)
args!�
jself
jinputs

jtraining
varargs
 
varkw
 
defaults�
p 

kwonlyargs� 
kwonlydefaults� 
annotations� *
 
�B�
 __inference__wrapped_model_46074input_1"�
���
FullArgSpec
args� 
varargsjargs
varkwjkwargs
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�2�
D__inference_cnn_model_layer_call_and_return_conditional_losses_47025
D__inference_cnn_model_layer_call_and_return_conditional_losses_47181
D__inference_cnn_model_layer_call_and_return_conditional_losses_47330
D__inference_cnn_model_layer_call_and_return_conditional_losses_47486�
���
FullArgSpec)
args!�
jself
jinputs

jtraining
varargs
 
varkw
 
defaults�
p 

kwonlyargs� 
kwonlydefaults� 
annotations� *
 
�2�
&__inference_conv_1_layer_call_fn_47495�
���
FullArgSpec
args�
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�2�
A__inference_conv_1_layer_call_and_return_conditional_losses_47506�
���
FullArgSpec
args�
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�2�
,__inference_layer_norm_1_layer_call_fn_47515�
���
FullArgSpec
args�
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�2�
G__inference_layer_norm_1_layer_call_and_return_conditional_losses_47571�
���
FullArgSpec
args�
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�2�
&__inference_conv_2_layer_call_fn_47580�
���
FullArgSpec
args�
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�2�
A__inference_conv_2_layer_call_and_return_conditional_losses_47591�
���
FullArgSpec
args�
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�2�
,__inference_layer_norm_2_layer_call_fn_47600�
���
FullArgSpec
args�
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�2�
G__inference_layer_norm_2_layer_call_and_return_conditional_losses_47656�
���
FullArgSpec
args�
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�2�
&__inference_conv_3_layer_call_fn_47665�
���
FullArgSpec
args�
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�2�
A__inference_conv_3_layer_call_and_return_conditional_losses_47676�
���
FullArgSpec
args�
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�2�
'__inference_flatten_layer_call_fn_47681�
���
FullArgSpec
args�
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�2�
B__inference_flatten_layer_call_and_return_conditional_losses_47687�
���
FullArgSpec
args�
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�2�
%__inference_dense_layer_call_fn_47696�
���
FullArgSpec
args�
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�2�
@__inference_dense_layer_call_and_return_conditional_losses_47706�
���
FullArgSpec
args�
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�2�
'__inference_dropout_layer_call_fn_47711
'__inference_dropout_layer_call_fn_47716�
���
FullArgSpec)
args!�
jself
jinputs

jtraining
varargs
 
varkw
 
defaults�
p 

kwonlyargs� 
kwonlydefaults� 
annotations� *
 
�2�
B__inference_dropout_layer_call_and_return_conditional_losses_47721
B__inference_dropout_layer_call_and_return_conditional_losses_47733�
���
FullArgSpec)
args!�
jself
jinputs

jtraining
varargs
 
varkw
 
defaults�
p 

kwonlyargs� 
kwonlydefaults� 
annotations� *
 
�2�
*__inference_raw_output_layer_call_fn_47742�
���
FullArgSpec
args�
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�2�
E__inference_raw_output_layer_call_and_return_conditional_losses_47753�
���
FullArgSpec
args�
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�B�
#__inference_signature_wrapper_46728input_1"�
���
FullArgSpec
args� 
varargs
 
varkwjkwargs
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
	J
Const
J	
Const_1�
 __inference__wrapped_model_46074���#$)*34=>8�5
.�+
)�&
input_1���������  
� "3�0
.
output_1"�
output_1����������
D__inference_cnn_model_layer_call_and_return_conditional_losses_47025x��#$)*34=>;�8
1�.
(�%
inputs���������  
p 
� "%�"
�
0���������
� �
D__inference_cnn_model_layer_call_and_return_conditional_losses_47181x��#$)*34=>;�8
1�.
(�%
inputs���������  
p
� "%�"
�
0���������
� �
D__inference_cnn_model_layer_call_and_return_conditional_losses_47330y��#$)*34=><�9
2�/
)�&
input_1���������  
p 
� "%�"
�
0���������
� �
D__inference_cnn_model_layer_call_and_return_conditional_losses_47486y��#$)*34=><�9
2�/
)�&
input_1���������  
p
� "%�"
�
0���������
� �
)__inference_cnn_model_layer_call_fn_46765l��#$)*34=><�9
2�/
)�&
input_1���������  
p 
� "�����������
)__inference_cnn_model_layer_call_fn_46802k��#$)*34=>;�8
1�.
(�%
inputs���������  
p 
� "�����������
)__inference_cnn_model_layer_call_fn_46839k��#$)*34=>;�8
1�.
(�%
inputs���������  
p
� "�����������
)__inference_cnn_model_layer_call_fn_46876l��#$)*34=><�9
2�/
)�&
input_1���������  
p
� "�����������
A__inference_conv_1_layer_call_and_return_conditional_losses_47506l7�4
-�*
(�%
inputs���������  
� "-�*
#� 
0���������
� �
&__inference_conv_1_layer_call_fn_47495_7�4
-�*
(�%
inputs���������  
� " �����������
A__inference_conv_2_layer_call_and_return_conditional_losses_47591l7�4
-�*
(�%
inputs���������
� "-�*
#� 
0���������
� �
&__inference_conv_2_layer_call_fn_47580_7�4
-�*
(�%
inputs���������
� " �����������
A__inference_conv_3_layer_call_and_return_conditional_losses_47676l)*7�4
-�*
(�%
inputs���������
� "-�*
#� 
0���������
� �
&__inference_conv_3_layer_call_fn_47665_)*7�4
-�*
(�%
inputs���������
� " �����������
@__inference_dense_layer_call_and_return_conditional_losses_47706]340�-
&�#
!�
inputs����������
� "%�"
�
0���������d
� y
%__inference_dense_layer_call_fn_47696P340�-
&�#
!�
inputs����������
� "����������d�
B__inference_dropout_layer_call_and_return_conditional_losses_47721\3�0
)�&
 �
inputs���������d
p 
� "%�"
�
0���������d
� �
B__inference_dropout_layer_call_and_return_conditional_losses_47733\3�0
)�&
 �
inputs���������d
p
� "%�"
�
0���������d
� z
'__inference_dropout_layer_call_fn_47711O3�0
)�&
 �
inputs���������d
p 
� "����������dz
'__inference_dropout_layer_call_fn_47716O3�0
)�&
 �
inputs���������d
p
� "����������d�
B__inference_flatten_layer_call_and_return_conditional_losses_47687a7�4
-�*
(�%
inputs���������
� "&�#
�
0����������
� 
'__inference_flatten_layer_call_fn_47681T7�4
-�*
(�%
inputs���������
� "������������
G__inference_layer_norm_1_layer_call_and_return_conditional_losses_47571l7�4
-�*
(�%
inputs���������
� "-�*
#� 
0���������
� �
,__inference_layer_norm_1_layer_call_fn_47515_7�4
-�*
(�%
inputs���������
� " �����������
G__inference_layer_norm_2_layer_call_and_return_conditional_losses_47656l#$7�4
-�*
(�%
inputs���������
� "-�*
#� 
0���������
� �
,__inference_layer_norm_2_layer_call_fn_47600_#$7�4
-�*
(�%
inputs���������
� " �����������
E__inference_raw_output_layer_call_and_return_conditional_losses_47753\=>/�,
%�"
 �
inputs���������d
� "%�"
�
0���������
� }
*__inference_raw_output_layer_call_fn_47742O=>/�,
%�"
 �
inputs���������d
� "�����������
#__inference_signature_wrapper_46728���#$)*34=>C�@
� 
9�6
4
input_1)�&
input_1���������  "3�0
.
output_1"�
output_1���������