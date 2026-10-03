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

Carried over from the previous period:

- [ ] **RD004:** Develop the **MPC Controller plugin** for **Ackermann-steered robots** (→ November 2026)
- [ ] **RD005:** Develop the **MPC Controller plugin** for **omnidirectional robots** (→ November 2026)
- [ ] **RD007:** Create **Test Case plugins** for **underwater robots** (→ November 2026)
- [ ] **RD008:** Create **Test Case plugins** for **aerial robots** (→ November 2026)
- [x] **RD015:** Write the EasyNav reference paper (submitted to ICRA)
- [ ] **RD016:** Write the NavMap reference paper (→ November 2026)

Developed in this period:

**Releases and infrastructure**

- [x] **RD017:** Release of **EasyNav**, **Yaets**, **easynav_plugins**, and **NavMap** for **ROS 2 Lyrical**, with CI (EasyNav 0.4.x, NavMap 0.5.x, Yaets 1.1.0)
- [x] **RD018:** **ROS 2 Humble** support: backport branches and CI
- [x] **RD019:** CI on **Ubuntu 26.04**, and installation with **Pixi** besides APT and source
- [x] **RD020:** Test coverage above **85%** in EasyNavigation

**Navigation**

- [x] **RD021:** **Regulated Pure Pursuit** controller (port of Nav2's), with the Dynamic Window extension
- [x] **RD022:** **Multi-Hypothesis AMCL** localizer, for global localization
- [x] **RD023:** Routes: accept incoming routes and save them
- [x] **RD024:** **Pause and resume** navigation, from clients and tools
- [x] **RD025:** **Velocity pipeline** in `ControllerNode`: robot limits shared by every controller, velocity multiplexer and smoother
- [x] **RD026:** **Robot geometry** configured once and shared by every component

**Robustness and recovery**

- [x] **RD027:** **Recovery system**: `RecoveryManagerNode` with pluggable managers; `DiagnosticRecoveryManager` (safety reflexes, evaluators, mitigations by priority) and `SimpleRecoveryManager`; recovery plugins in `easynav_plugins` (collision reflex, stuck, no path, obstacle too close, ROS graph, AMCL convergence, advance, retreat, relocalize, human assistance, cancel, shutdown)
- [x] **RD028:** **Run-time reconfiguration** and plugin switching (`PluginSwitcher`), keeping the mission and the localization across it
- [x] **RD029:** Concurrency robustness: thread-safe `NavState` access (`get_safe`), race fixes in `RTTFBuffer` and perceptions, safe shutdown on signals

**Safety** (see the Safety page in the website)

- [x] **RD030:** Functional-safety feasibility analysis (IEC 61508-3 SIL 2) and integration roadmap
- [x] **RD031:** Velocity command robustness: command timeout, keepalive, `cmd_vel` QoS with deadline and liveliness, non-finite commands discarded
- [x] **RD032:** **Safety mode**: configuration checks, SHA-256 configuration fingerprint, frozen configuration, real-time scheduling required, memory locking
- [x] **RD033:** **Heartbeat** from the real-time cycle and **real-time cycle monitoring**
- [x] **RD034:** Integration of the safety channel's state (`SafetyStatus`): protective stop and safely limited speed
- [ ] **RD035:** Maximum **data age** (perceptions and robot pose), fail-safe collision reflex, and **fault-injection** plugins for every component (under review)
- [x] **RD036:** Yaets: trace producers never wait for the trace file (no priority inversion in the real-time cycle)

**Tools and documentation**

- [x] **RD037:** **Nav2 bridge** (`easynav_nav2_bridge`): a `NavigateToPose` action server on top of EasyNav
- [x] **RD038:** **Migration Guide** for Nav2 users, and Safety, Recovery and installation pages in the website
- [x] **RD039:** TUI: diagnostics and mitigation panels
- [x] **RD040:** NavMap: generation of a simulated world and its NavMap from satellite imagery
- [x] **RD053:** NavMap: releases 0.5.0 and 0.5.1, Jazzy port, and compatibility with current Rolling (message generation, RViz/Ogre headers)
- [x] **RD054:** Yaets: release 1.1.0 and CI for Lyrical

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
- [ ] **RD056:** Yaets: trace analysis tools (per-function timing histograms and worst cases, comparison between runs) to support the timing campaign (RD044)

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
