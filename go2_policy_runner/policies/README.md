# Policies

Relative `policy_dir` values in the config are resolved against this directory
(installed to `share/go2_policy_runner/policies`). Expected layout (what unitree_rl_lab
`play.py` / `train.py` writes into a run folder):

```
<policy_dir>/
├── exported/policy.onnx
└── params/deploy.yaml
```

Pointing `policy_dir` to `unitree_rl_lab/logs/rsl_rl/<experiment>` selects the latest run
that has an `exported/` folder. Usually it is simpler to pass an absolute path with the
launch argument `policy_dir:=...` instead of copying policies here.
