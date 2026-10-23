# Third-party notices

go2_policy_runner contains code and data adapted from the projects listed below. Every
adapted source file keeps the original copyright notice in its header and lists the
upstream files it was derived from and the changes made.

## unitree_rl_lab — Apache License 2.0

https://github.com/unitreerobotics/unitree_rl_lab
Copyright (c) 2025, Unitree Robotics Co., Ltd.

| This repository | Derived from (`deploy/include/...`) |
|---|---|
| `src/policy/observation_manager.cpp`, `include/.../policy/observation_manager.hpp` | `isaaclab/manager/observation_manager.h`, `isaaclab/manager/manager_term_cfg.h`, `isaaclab/envs/mdp/observations/observations.h` |
| `src/policy/action_manager.cpp`, `include/.../policy/action_manager.hpp` | `isaaclab/manager/action_manager.h`, `isaaclab/envs/mdp/actions/joint_actions.h` |
| `src/policy/policy_env.cpp`, `include/.../policy/policy_env.hpp` | `isaaclab/envs/manager_based_rl_env.h`, `param.h` |
| `include/.../policy/articulation.hpp` | `isaaclab/assets/articulation/articulation.h`, `unitree_articulation.h` |
| `src/policy/onnx_policy.cpp`, `include/.../policy/policy_model.hpp` | `isaaclab/algorithms/algorithms.h` |
| `src/remote/remote_dsl.cpp`, `include/.../remote/remote_dsl.hpp` | `unitree_joystick_dsl.hpp` |
| `src/fsm/*`, `include/.../fsm/*` | `FSM/BaseState.h`, `FSM/FSMState.h`, `FSM/CtrlFSM.h`, `FSM/State_Passive.h`, `FSM/State_FixStand.h`, `FSM/State_RLBase.h`, `LinearInterpolator.h`, `isaaclab/envs/mdp/terminations.h` |
| `config/go2.yaml` (gains, poses, remote combinations) | `deploy/robots/go2/config/config.yaml` |

The Apache License 2.0 text is in [LICENSE](LICENSE). unitree_rl_lab does not ship a NOTICE file.

## unitree_sdk2, unitree_ros2, unitree_mujoco — BSD 3-Clause License

https://github.com/unitreerobotics/unitree_sdk2, https://github.com/unitreerobotics/unitree_ros2,
https://github.com/unitreerobotics/unitree_mujoco

| This repository | Derived from |
|---|---|
| `src/remote/unitree_remote.cpp`, `include/.../remote/unitree_remote.hpp` | unitree_sdk2 `include/unitree/dds_wrapper/common/unitree_joystick.hpp` |
| `src/ros/unitree_lowcmd.cpp`, `include/.../ros/unitree_lowcmd.hpp` | unitree_ros2 `example/src/include/common/motor_crc.h`, `example/src/src/common/motor_crc.cpp` |
| `src/ros/motion_switcher_client.cpp`, `include/.../ros/motion_switcher_client.hpp` | unitree_ros2 `example/src/include/b2/b2_motion_switch_client.hpp`, `example/src/include/common/base_client.hpp` |
| `config/go2.yaml` (joint limits) | unitree_mujoco `unitree_robots/go2/go2.xml` |

### unitree_sdk2 license

```
BSD 3-Clause License

Copyright (c) 2016-2023 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics")
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

### unitree_ros2 license

```
BSD 3-Clause License

Copyright (c) 2016-2024 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics")
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

### unitree_mujoco license

```
BSD 3-Clause License

Copyright (c) 2016-2024 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics")
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```
