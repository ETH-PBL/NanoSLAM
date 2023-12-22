[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# NanoSLAM: Enabling Fully Onboard SLAM for Tiny Robots

**Authors: *Vlad Niculescu*, *Tommaso Polonelli*, *Michele Magno*, *Luca Benini*** 

Corresponding author: *Vlad Niculescu* <vladn@iis.ee.ethz.ch>  

## About the Project
Perceiving and mapping the surroundings are essential for autonomous navigation in any robotic platform. The algorithm class that enables accurate mapping while correcting the odometry errors present in most robotics systems is Simultaneous Localization and Mapping (SLAM). Today, fully onboard mapping is only achievable on robotic platforms that can host high-wattage processors, mainly due to the significant computational load and memory demands required for executing SLAM algorithms. For this reason, pocket-size hardware-constrained robots offload the execution of SLAM to external infrastructures. To address the challenge of enabling SLAM algorithms on resource-constrained processors, this paper proposes NanoSLAM, a lightweight and optimized end-to-end SLAM approach specifically designed to operate on centimeter-size robots at a power budget of only 87.9 mW. We demonstrate the mapping capabilities in real-world scenarios and deploy NanoSLAM on a nano-drone weighing 44 g and equipped with a novel commercial RISC-V low-power parallel processor called GAP9. The algorithm, designed to leverage the parallel capabilities of the RISC-V processing cores, enables mapping of a general environment with an accuracy of 4.5 cm and an end-toend execution time of less than 250 ms.

## Demonstration Video
Our video briefly explains how our system works and showcases **NanoSLAM** operation onboard a 44 g nano-drone. [**Video available here**](https://www.youtube.com/watch?v=XUSVLHJ87J0).

## Publications
If you use **NanoSLAM** in an academic or industrial context, please cite the following publications:

Publications: 
* *NanoSLAM: Enabling Fully Onboard SLAM for Tiny Robots* [IEEE IoT Journal](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=10343110) -- [IEEE IoT Journal]
* *Fully Onboard SLAM for Distributed Mapping with a Swarm of Nano-Drones* [arXiv preprint](https://arxiv.org/pdf/2309.03678.pdf)


(https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=10343110)
```bibtex
@article{niculescu2023nanoslam,
  title={NanoSLAM: Enabling Fully Onboard SLAM for Tiny Robots},
  author={Niculescu, Vlad and Polonelli, Tommaso and Magno, Michele and Benini, Luca},
  journal={IEEE Internet of Things Journal},
  year={2023},
  publisher={IEEE}
}
```

```bibtex
@article{friess2023fully,
  title={Fully Onboard SLAM for Distributed Mapping with a Swarm of Nano-Drones},
  author={Friess, Carl and Niculescu, Vlad and Polonelli, Tommaso and Magno, Michele and Benini, Luca},
  journal={arXiv preprint arXiv:2309.03678},
  year={2023}
}
```

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->


[contributors-shield]: https://img.shields.io/github/contributors/ETH-PBL/NanoSLAM.svg?style=flat-square
[contributors-url]: https://github.com/ETH-PBL/NanoSLAM/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/ETH-PBL/NanoSLAM.svg?style=flat-square
[forks-url]: https://github.com/ETH-PBL/NanoSLAM/network/members
[stars-shield]: https://img.shields.io/github/stars/ETH-PBL/NanoSLAM.svg?style=flat-square
[stars-url]: https://github.com/ETH-PBL/NanoSLAM/stargazers
[issues-shield]: https://img.shields.io/github/issues/ETH-PBL/NanoSLAM.svg?style=flat-square
[issues-url]: https://github.com/ETH-PBL/NanoSLAM/issues
[license-shield]: https://img.shields.io/github/license/ETH-PBL/NanoSLAM.svg?style=flat-square
[license-url]: https://github.com/ETH-PBL/NanoSLAM/blob/master/LICENSE
[product-screenshot]: pics/drone.png
