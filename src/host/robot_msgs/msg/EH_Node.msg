int32 node_id
string node_name
int32 level_id
string level_name
string[] node_text
int32 sensor_sg_id
bool isNew
# Inference Model
float64 probability
float64 variance
bool isUncertain # This means that the node was picked with same probability and variance as another node
int32[] competingParentIds
float64[] competingParentProbabilities
float64[] competingParentVariances
# SMILE BN
int32 node_type_id
string node_type_name
int32[] node_state_id
string[] node_state_name
float64[] node_state_value
#int32[] parent_node_id #Do not use. Use child_parent_map instead 
##old:
int32 id
string code
EH_Node_Text[] texts
int32 type
bool unknown_error
