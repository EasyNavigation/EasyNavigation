# 🗺️ EasyNavigation Roadmap

This document defines the development roadmap for **EasyNavigation (EasyNav)** and its ecosystem, including related repositories such as **Yaets**, **NavMap**, and **easynav_plugins**.  
It is organized into **six-month plans**, updated regularly to reflect current priorities and progress.

---

## 📆 November 2025 – April 2026

Each item is numbered (`RD###`) for easier reference and tracking. Items not completed in this period were moved to the next one.

- [x] **RD001:** Display test coverage levels in all repositories and increase them to at least **70%**
- [x] **RD002:** Implement and validate a **GPS-based Localizer plugin**, tested in outdoor environments
- [x] **RD003:** Develop the **MPC Controller plugin** for **differential-drive robots**
- [ ] ~~**RD006:** Integrate **LLM-based analysis** for runtime execution review and improvement suggestions~~
- [x] **RD009:** Add **route-based navigation tools** for predefined path execution
- [x] **RD010:** Perform release of **EasyNav**, **Yaets**, **easynav_plugins**, and **NavMap** for **ROS 2 Kilted**
- [x] **RD011:** Perform release of **EasyNav**, **Yaets**, **easynav_plugins**, and **NavMap** for **ROS 2 Jazzy**
- [ ] ~~**RD012:** Perform release of **EasyNav**, **Yaets**, **easynav_plugins**, and **NavMap** for **ROS 2 Humble**~~ (see RD018)
- [ ] ~~**RD013:** Perform release of **EasyNav**, **Yaets**, **easynav_plugins**, and **NavMap** for **ROS 2 Rolling**~~
- [x] **RD014:** Complete and consolidate documentation with **HowTos** and **API references**
- [x] **RD051:** NavMap: goal pose tool, occupancy grid conventions, faster NavCel location and releases 0.3 and 0.4
- [x] **RD052:** NavMap: RViz plugin moved to **Qt6**, and CI on container workers and Ubuntu 26.04 (also Yaets)
- RD004, RD005, RD007, RD008, RD015 and RD016 → moved to May 2026 – October 2026.

---

## 📆 May 2026 – October 2026

These are the main development goals for this semester. Items not completed in this period were moved to the next one.

Carried over from November 2025 – April 2026:

- [ ] **RD004:** Develop the **MPC Controller plugin** for **Ackermann-steered robots**
- [ ] **RD005:** Develop the **MPC Controller plugin** for **omnidirectional robots**
- [ ] **RD007:** Create **Test Case plugins** for **underwater robots**
- [ ] **RD008:** Create **Test Case plugins** for **aerial robots**
- [x] **RD015:** Write the EasyNav reference paper (submitted to ICRA)
- [ ] **RD016:** Write the NavMap reference paper

New in this period:

- [x] **RD017:** Perform release of **EasyNav**, **Yaets**, **easynav_plugins**, and **NavMap** for **ROS 2 Lyrical**, with CI
- [x] **RD018:** Support **ROS 2 Humble** with backport branches and CI
- [x] **RD019:** Move CI to **Ubuntu 26.04**, and offer installation with **Pixi** besides APT and source
- [x] **RD020:** Increase test coverage in EasyNavigation above **85%**
- [x] **RD021:** Develop a **Regulated Pure Pursuit Controller plugin**, with the Dynamic Window extension
- [x] **RD022:** Develop a **Multi-Hypothesis AMCL (MH-AMCL) Localizer plugin** for global localization
- [x] **RD023:** Extend **route-based navigation** to receive routes at run time and save them
- [x] **RD024:** Allow to **pause and resume** navigation from clients and tools
- [x] **RD025:** Create a single **velocity output** in the Controller node: robot limits shared by every controller, velocity multiplexer and smoother
- [x] **RD026:** Configure the **robot geometry** once and share it with every component
- [x] **RD027:** Develop a **recovery system** with pluggable recovery managers, a diagnosis-driven manager (safety reflexes, evaluators and mitigations by priority) and a set of recovery plugins
- [x] **RD028:** Support **run-time reconfiguration** and plugin switching, keeping the mission and the localization
- [x] **RD029:** Make EasyNav robust to **concurrency** between its real-time and non-real-time cycles
- [x] **RD030:** Analyze the feasibility of **functional-safety** certification (IEC 61508-3 SIL 2) and define the integration roadmap
- [x] **RD031:** Make the **velocity command** robust: command timeout, keepalive, QoS with deadline and liveliness, non-finite commands discarded
- [x] **RD032:** Add a **safety mode**: configuration checks, configuration fingerprint, frozen configuration, real-time scheduling and memory locking
- [x] **RD033:** Publish a **heartbeat** from the real-time cycle and **monitor** the real-time cycle
- [x] **RD034:** Integrate the **safety channel's state**: protective stop and safely limited speed
- [x] **RD035:** Bound the **age of the data** used (perceptions and robot pose), and add **fault-injection** plugins for every component
- [x] **RD036:** Remove priority inversions from **Yaets** tracing in the real-time cycle
- [x] **RD037:** Develop a **Nav2 bridge**, so Nav2 clients can drive EasyNav
- [x] **RD038:** Write a **Migration Guide** for Nav2 users, and the Safety and Recovery documentation
- [x] **RD039:** Show diagnostics and recovery mitigations in the **TUI**
- [x] **RD040:** Generate a simulated world and its **NavMap from satellite imagery**
- [x] **RD053:** Perform releases 0.5.0 and 0.5.1 of **NavMap**, with the Jazzy port and compatibility with current Rolling
- [x] **RD054:** Perform release 1.1.0 of **Yaets**, with CI for Lyrical
- RD004, RD005, RD007, RD008 and RD016 → moved to November 2026 – April 2027.

---

## 📆 November 2026 – April 2027

Carried over:

- [ ] **RD004:** Develop the **MPC Controller plugin** for **Ackermann-steered robots**
- [ ] **RD005:** Develop the **MPC Controller plugin** for **omnidirectional robots**
- [ ] **RD007:** Create **Test Case plugins** for **underwater robots**
- [ ] **RD008:** Create **Test Case plugins** for **aerial robots**
- [ ] **RD016:** Write the NavMap reference paper

Proposed:

- [ ] **RD041:** Release **EasyNav 0.5** (recovery system, velocity pipeline, safety) for **Jazzy**, **Kilted** and **Lyrical**, and port the recovery system to **Humble**
- [ ] **RD042:** Fault-injection **simulation scenarios** with `ros2_fault_injection` (sensor and TF drops and delays)
- [ ] **RD043:** No middleware calls in the real-time cycle: `RTTFBuffer::publish()` and debug publications moved out of `update_rt()`
- [ ] **RD044:** **Timing campaign** per release: worst-case execution time per cycle and plugin, under load, on reference hardware
- [ ] **RD045:** Mandatory **static analysis** in CI (clang-tidy, cppcheck) and a coverage gate that fails on regressions
- [ ] **RD046:** **Safety manual for integrators**: assumptions of use, safety-related parameters and interfaces, validated configurations
- [ ] **RD047:** **Polygon footprints**, beyond the circular robot geometry
- [ ] **RD048:** Nav2 bridge: `NavigateThroughPoses`, and complete navigation feedback (estimated time remaining, distance covered)
- [ ] **RD049:** **Multi-robot convoy**: a controller that follows another robot at a fixed distance, with a howto
- [ ] **RD050:** Thread-safety review of the localizers (e.g. AMCL prediction and correction)
- [ ] **RD055:** NavMap: incremental updates from live sensor data (dynamic obstacles on the mesh) and a NavMap-based collision check for the safety reflexes
- [ ] **RD057:** Maps Manager plugin for **large maps**, possibly with fractal (multi-resolution) approximations
- [ ] **RD058:** `RTTFBuffer` that **extrapolates transforms to the future**, so the real-time cycle does not wait for or use stale transforms
- [ ] **RD059:** Integrate a **Recovery Manager plugin** for the configuration and adaptation of the whole system

---

## 🧭 Notes

- Progress will be tracked directly in this document.  
- Each roadmap item may have a corresponding issue or pull request linked for more details.  
- When an item is completed, mark it as done (`[x]`) and link the related PR or release.

---

📎 **Related repositories:**  
[EasyNavigation](https://github.com/EasyNavigation/EasyNavigation) •  
[NavMap](https://github.com/EasyNavigation/NavMap) •  
[easynav_plugins](https://github.com/EasyNavigation/easynav_plugins) •  
[Yaets](https://github.com/fmrico/yaets) •  
[easynav_nav2_bridge](https://github.com/EasyNavigation/easynav_nav2_bridge)
