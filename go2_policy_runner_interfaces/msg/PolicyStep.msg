# One policy step, published on ~/policy_step (only while an rl_policy mode is active).
# Useful for logging and sim-to-sim / sim-to-real comparison.

std_msgs/Header header
string mode
uint64 step                         # step index since the mode was entered

string[] observation_groups         # ONNX input names, e.g. ["obs"]
uint32[] observation_sizes          # size of each group in `observation`
float32[] observation               # all groups concatenated (after clip/scale/history)

float32[] action                    # raw network output (IsaacLab "last_action")
float32[] joint_position_target     # SDK motor order, after scale/offset/clip

geometry_msgs/Vector3 command       # velocity command used: x = vx, y = vy, z = wz (before clamping)
float32 inference_time              # [ms]
