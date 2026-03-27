# 🚀 FAR-RIO: A Fast and Robust Radar-Inertial Odometry

**FAR-RIO** is the official open-source implementation of the paper: *"FAR-RIO: A Fast and Robust Radar-Inertial Odometry with Isotropic Uncertainty Model and Dual-Observation Update Pipeline"*.

This project provides a fast and highly robust Radar-Inertial Odometry (RIO) solution for robotic localization in challenging and visually degraded environments (e.g., rain, snow, heavy fog, and unilluminated scenes). It achieves this by integrating an innovative **Isotropic Uncertainty Model** and a **Dual-Observation Update Pipeline**.

---

## 🚧 Current Status: Under Construction (WIP)

*Note: This repository is currently being heavily refactored from the [FasterLIO](https://github.com/gaoxiang12/faster-lio) framework. The core radar processing modules are being merged and will be available soon.*

### 📋 TO-DO List (Migration from FasterLIO)
Before this code can be run as FAR-RIO, the following FasterLIO components need to be replaced/adapted:

- [ ] **Frontend Replacement:** Swap out the Lidar point cloud preprocessing (e.g., deskewing, feature extraction) with our FMCW Radar processing pipeline.
- [ ] **Isotropic Uncertainty Model Integration:** Inject the new noise modeling logic tailored for sparse, noisy radar point clouds into the state estimation.
- [ ] **Update the Backend Pipeline:** Replace the standard Lidar-Inertial update step with our **Dual-Observation Update Pipeline** (utilizing both Doppler velocity and Geometric features).
- [ ] **Project Configuration:** Update `CMakeLists.txt` and `package.xml` (rename project to `far_rio`, remove Lidar-specific dependencies, adjust iVox configurations if necessary).
- [ ] **Config & Launch Files:** Create new `.yaml` and `.launch` files containing Radar-IMU extrinsics and algorithm parameters.
- [ ] **Dataset Links:** Replace Avia/NCLT Lidar dataset links with our sample Radar-Inertial datasets.

---


## 🛠️ Dependencies & Build
*Coming soon...* *(Will generally require ROS, Eigen, PCL, etc. Detailed instructions will be updated once the codebase is finalized.)*

## 🚀 Running FAR-RIO
*Coming soon...*

---

<!-- ## 🙏 Acknowledgements & Citation

This project is built upon the excellent foundational framework of [FasterLIO](https://github.com/gaoxiang12/faster-lio) and [FastLIO2](https://github.com/hku-mars/FAST_LIO). We sincerely thank the authors for their open-source contributions to the community.

If you find FAR-RIO helpful in your academic research, please consider citing our work:

```bibtex
@article{YourName2026FARRIO,
  title={FAR-RIO: A Fast and Robust Radar-Inertial Odometry with Isotropic Uncertainty Model and Dual-Observation Update Pipeline},
  author={Author A and Author B and Author C},
  journal={Journal/Conference Name},
  year={2026},
  volume={x},
  pages={xxx-xxx}
} -->
