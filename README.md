# Isaac Lab - Go2

This repository was forked from [NVIDIA's IsaacLab](https://github.com/isaac-sim/IsaacLab) to edit the reward structure, environments and more to enable the Sim2Real transfer using my ROS2 Humble RL Deployment framework, [go2_rl_ws](https://github.com/eppl-erau-db/go2_rl_ws). Currently, the training how allowed for deployment of a low level and navigation control policies, although only sporatic pronking has been achieved by the low level policies. Feel free to contribute.

You can find the related documentation here: [Isaac Lab Documentation Page](https://isaac-sim.github.io/IsaacLab)

## Licensing

The Isaac Lab framework is released under [BSD-3 License](LICENSE). The license files of its dependencies and assets are present in the [`docs/licenses`](docs/licenses) directory.

## Acknowledgement

Isaac Lab development initiated from the [Orbit](https://isaac-orbit.github.io/) framework. We would appreciate if you would cite it in academic publications as well:

```
@article{mittal2023orbit,
   author={Mittal, Mayank and Yu, Calvin and Yu, Qinxi and Liu, Jingzhou and Rudin, Nikita and Hoeller, David and Yuan, Jia Lin and Singh, Ritvik and Guo, Yunrong and Mazhar, Hammad and Mandlekar, Ajay and Babich, Buck and State, Gavriel and Hutter, Marco and Garg, Animesh},
   journal={IEEE Robotics and Automation Letters},
   title={Orbit: A Unified Simulation Framework for Interactive Robot Learning Environments},
   year={2023},
   volume={8},
   number={6},
   pages={3740-3747},
   doi={10.1109/LRA.2023.3270034}
}
```