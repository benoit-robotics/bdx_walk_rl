# Isaac sim environment to train the BDX robot

This repository contains the code to train the [BDX](https://la.disneyresearch.com/publication/design-and-control-of-a-bipedal-robotic-character/) robot  to walk on flat or rough terrains using Isaac Lab [Isaac Lab](https://github.com/isaac-sim/IsaacLab). The model of the robot was found on the [AWD](https://github.com/rimim/AWD) repository and the idea of this work comes from the [Open_Duck_Mini](https://github.com/apirrone/Open_Duck_Mini) repository. The robot is trained using the [RSL-RL](https://github.com/leggedrobotics/rsl_rl) implementation of RL algorithms.
This repository was created using the [IsaacLab template](https://github.com/isaac-sim/IsaacLabExtensionTemplate/tree/main).



https://github.com/user-attachments/assets/bb7887ef-2252-470d-8ace-fc5ef8cbae35

### Repository structure and important files

The repository contains the configuration files for a flat environment and a rough environment. We're training the robot to perform a `task` which in this case is a velocity task. The robot is trained to walk with a certain velocity. The configuration files define the environment and the task for the robot. The configuration files are located in the `exts/bdx_walk/bdx_walk/tasks/locomotion/velocity` folder.
The repository is structured as follows:

```
    ├── data
    ├── exts/bdx_walk/bdx_walk/tasks/locomotion/velocity
    |   ├── velocity_env_cfg.py        # Configuration file for the velocity task
    │   ├── config/bdx/          
    │   │   ├── flat_env_cfg.py        # Configuration file for the flat terrain environment
    │   │   └── rough_env_cfg.py       # Configuration file for the rough terrain environment
    └── scripts/rsl_rl
        ├── train.py            # Train the BDX robot
        └── play.py             # Play the trained model
```

### How to start training

Install [Isaac Lab](https://github.com/isaac-sim/IsaacLab).


Install the package:
```sh
python -m pip install -e exts/bdx_walk/.
```

Train the model for the `flat` terrain using the following command:
```sh
python scripts/rsl_rl/train.py --task Isaac-Velocity-Flat-BDX-v0  --headless --num_envs 4096
```

During training, for debugging purposes it can be useful to output the full logs:
```sh
export HYDRA_FULL_ERROR=1
```


### Visualize training information
For visualizing the training information, use tensorboard. To start tensorboard, run the following command:
```sh
tensorboard --logdir logs/rsl_rl/bdx_flat/
```
It will start a server on `localhost:6006` where you can visualize the training information for the `flat` terrain.

### Play the trained model

The training process will save checkpoints (`pt` files) inside the `logs` folder. To play the trained model, use the following command:
```sh
python scripts/rsl_rl/play.py --task Isaac-Velocity-Flat-BDX-Play-v0 --num_envs 32 --checkpoint [/PATH/TO/model.pt]
```
