# State of the policy controller, published on ~/state (latched, on change and at state_rate).

std_msgs/Header header

string mode                     # active mode
string mode_type                # passive | fix_stand | rl_policy
float32 mode_time               # [s] time spent in the active mode
string[] modes                  # all configured modes
string[] allowed_transitions    # modes reachable from the active one (fallback is always reachable)
string last_transition_reason

bool ready                      # startup finished (lowstate received, motion switcher released)
bool lowstate_ok                # lowstate is fresh
float32 lowstate_age            # [s] since last /lowstate
float32 lowstate_rate           # [Hz] measured /lowstate rate
bool cmd_vel_ok                 # cmd_vel is fresh (otherwise zero command is used)
float32 control_rate            # [Hz] measured control loop rate
float32 max_cycle_time          # [ms] worst control cycle since last state message
