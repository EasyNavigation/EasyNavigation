# easynav_core

Core base classes for all Easy Navigation method plugins (controllers, planners, localizers, map managers).  
This package provides a common lifecycle, timing utilities, and (for some bases) shared behaviors like collision checking.

## Base Classes Overview

| Base Class | Header | Typical Derived Plugins |
|---|---|---|
| `easynav::MethodBase` | `easynav_core/MethodBase.hpp` | All method plugins (controllers, planners, localizers, maps managers) |
| `easynav::ControllerMethodBase` | `easynav_core/ControllerMethodBase.hpp` | `easynav_simple_controller`, `easynav_mppi_controller`, `easynav_serest_controller`, `easynav_vff_controller`, ... |
| `easynav::PlannerMethodBase` | `easynav_core/PlannerMethodBase.hpp` | `easynav_costmap_planner`, `easynav_navmap_planner`, `easynav_simple_planner`, ... |
| `easynav::LocalizerMethodBase` | `easynav_core/LocalizerMethodBase.hpp` | `easynav_costmap_localizer`, `easynav_navmap_localizer`, `easynav_simple_localizer`, ... |
| `easynav::MapsManagerBase` | `easynav_core/MapsManagerBase.hpp` | `easynav_costmap_maps_manager`, `easynav_navmap_maps_manager`, `easynav_simple_maps_manager`, ... |

Each plugin README in `easynav_plugins` can refer to these sections instead of duplicating the shared behavior.

---

## `easynav::MethodBase`

**Header:** `easynav_core/MethodBase.hpp`  
**Role:** Common lifecycle and timing utilities for all method plugins.

### MethodBase Responsibilities

- Store a pointer to the parent `rclcpp_lifecycle::LifecycleNode`.
- Keep the plugin name and TF prefix.
- Provide `initialize()` + virtual `on_initialize()` hook for derived classes.
- Provide update-rate helpers for real-time and non-real-time loops.

### Parameters

All parameters are declared under each derived plugin namespace; `MethodBase` just expects them to exist. Typical parameters (declared by derived classes) include:

| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.rt_frequency` | `double` | implementation-specific | Desired real-time loop frequency (Hz). Used by `isTime2RunRT()`. |
| `<plugin>.frequency` | `double` | implementation-specific | Desired non-RT loop frequency (Hz). Used by `isTime2Run()`. |

> Note: The exact parameter names and defaults are defined in each derived plugin; `MethodBase` only consumes the configured frequencies.

### MethodBase Public API

| Method | Description |
|---|---|
| `initialize(parent_node, plugin_name, tf_prefix)` | Stores node pointer, plugin name and TF prefix, reads frequencies, and calls `on_initialize()`. |
| `on_initialize()` | Virtual hook for derived classes to perform extra setup (declare parameters, create pubs/subs, etc.). |
| `get_node()` | Returns shared pointer to parent lifecycle node. |
| `get_plugin_name()` | Returns the plugin identifier used for namespacing parameters and logs. |
| `get_tf_prefix()` | Returns the TF namespace (with trailing `/`). |
| `isTime2RunRT()` | Returns true if enough time has elapsed to run a real-time update. |
| `isTime2Run()` | Returns true if enough time has elapsed to run a non-RT update. |
| `setRunRT()` / `setRun()` | Mark that an RT / non-RT iteration has just been executed. |
| `get_last_rt_execution_ts()` / `get_last_execution_ts()` | Access the last execution timestamps. |

### NavState / Topics

`MethodBase` itself does not read or write `NavState` and does not create publishers or subscriptions. All such interfaces are defined in derived base classes (see below) and their plugins.

---

## `easynav::ControllerMethodBase`

**Header:** `easynav_core/ControllerMethodBase.hpp`  
**Role:** Base class for controllers that generate velocity commands and optionally perform collision checking.

Typical derived plugins: `easynav_simple_controller`, `easynav_mppi_controller`, `easynav_serest_controller`, `easynav_vff_controller`.

### Controller Responsibilities

- Extend `MethodBase` with a real-time control loop (`update_rt`).
- Provide a standard collision-checking utility based on point clouds.
- Optionally publish visualization markers for the collision zone.

### Robot geometry

The robot's shape is configured once, in `system_node`, and shared with every component (collision
checking, costmap inflation, planners, recovery...). Plugins read it with
`MethodBase::get_robot_geometry()`; other code, with `easynav::get_robot_geometry()`
(`easynav_common/RobotGeometry.hpp`).

| Name | Type | Default | Description |
|---|---|---:|---|
| `robot_geometry.radius` | `double` | `0.3` | Circumscribed radius: smallest circle containing the robot (m). |
| `robot_geometry.inscribed_radius` | `double` | `radius` | Largest circle inside the robot (m). |
| `robot_geometry.height` | `double` | `0.5` | Top of the robot, above the robot frame (m). |

A component's former geometry parameter (e.g. `colision_checker.robot_radius`) still applies, with a
deprecation warning, where `robot_geometry` does not configure that field; `robot_geometry` takes
precedence when both are set.

### Parameters (common collision checker)

Derived controllers usually expose these parameters (names may vary slightly per plugin):

| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.debug_markers` | `bool` | `false` | Enable/disable publication of collision debug markers. |
| `<plugin>.active` | `bool` | `false` | Enable/disable collision checking. |
| `colision_checker.robot_radius` | `double` | — | **Deprecated**: the robot radius is `system_node.robot_geometry.radius`. Still applied, with a warning, if `robot_geometry.radius` is not configured. |
| `colision_checker.robot_height` | `double` | — | **Deprecated**: the robot height is `system_node.robot_geometry.height`. Still applied, with a warning, if `robot_geometry.height` is not configured. |
| `<plugin>.z_min_filter` | `double` | `0.0` | Minimum Z to keep points in the collision check (m). |
| `<plugin>.brake_acc` | `double` | `0.5` | Maximum braking deceleration used to compute stopping distance (m/s²). |
| `<plugin>.safety_margin` | `double` | `0.1` | Extra margin added to the braking distance (m). |
| `<plugin>.downsample_leaf_size` | `double` | `0.1` | Voxel leaf size for downsampling the collision point cloud (m). |

(Exact declaration and defaults live in each controller plugin; this section documents the shared semantics.)

### Interfaces (Topics)

| Direction | Topic | Type | Purpose |
|---|---|---|---|
| Publisher | `<node_fqn>/<plugin>/collision_zone` | `visualization_msgs/msg/MarkerArray` | Publishes collision region and filtered points for debugging. |

The actual topic name is built inside the controller plugin using `get_node()` and `get_plugin_name()`, but all controllers sharing `ControllerMethodBase` use a similar pattern.

### NavState Keys (collision helper)

`ControllerMethodBase` expects derived controllers to provide certain NavState entries when using its collision checker:

| Key | Type | Access | Description |
|---|---|---|---|
| `robot_pose` | `nav_msgs::msg::Odometry` | **Read** | Current robot pose and twist used to estimate braking distance. |
| `points` | `sensor_msgs::msg::PointCloud2` or filtered cloud | **Read** | 3D points around the robot used for collision prediction. |
| `cmd_vel` | `geometry_msgs::msg::TwistStamped` | **Write** (via `on_inminent_collision`) | Default handler stops the robot on imminent collision. |

### Controller Public API

| Method | Description |
|---|---|
| `initialize(parent_node, plugin_name, tf_prefix)` | Declares common controller parameters, creates collision marker publisher, and calls `MethodBase::initialize()`. |
| `internal_update_rt(nav_state, trigger)` | Checks timing and calls `update_rt(nav_state)` when appropriate; returns true if executed. |
| `update_rt(nav_state)` | **To implement in derived controller.** Computes and writes the `cmd_vel` command. |
| `is_inminent_collision(nav_state)` | Runs collision prediction using current velocity, braking distance, safety margin, and point cloud. |
| `on_inminent_collision(nav_state)` | Default handler: logs a warning and writes zero `cmd_vel`. Can be overridden. |
| `publish_collision_zone_marker(min, max, cloud, imminent_collision)` | Publishes `MarkerArray` with the collision box and points for RViz. |

---

## `easynav::PlannerMethodBase`

**Header:** `easynav_core/PlannerMethodBase.hpp`  
**Role:** Base class for planners that build a path from robot pose to goal.

Typical derived plugins: `easynav_costmap_planner`, `easynav_navmap_planner`, `easynav_simple_planner`.

### Planner Responsibilities

- Extend `MethodBase` with non-RT `update()` callbacks.
- Provide convenience helpers to respect the configured planning frequency.

### Planner Public API

| Method | Description |
|---|---|
| `internal_update(nav_state)` | Checks timing via `isTime2Run()` and calls `update(nav_state)` when due. |
| `force_update(nav_state)` | Forces a call to `update(nav_state)` regardless of timing constraints. |
| `update(nav_state)` | **Pure virtual.** Implement the planning algorithm and write the path into `NavState`. |

### Parameters / NavState / Topics

`PlannerMethodBase` itself does not declare parameters, navstate keys, or topics. These are defined by each planner plugin (see their READMEs). This base only standardizes when and how often the planner is executed.

---

## `easynav::LocalizerMethodBase`

**Header:** `easynav_core/LocalizerMethodBase.hpp`  
**Role:** Base class for localization methods (AMCL variants, sensor fusion, etc.).

Typical derived plugins: `easynav_costmap_localizer`, `easynav_navmap_localizer`, `easynav_simple_localizer`.

### Localizer Responsibilities

- Extend `MethodBase` with both RT and non-RT update hooks.
- Provide helpers to respect configured frequencies for each.

### Localizer Public API

| Method | Description |
|---|---|
| `internal_update_rt(nav_state, trigger)` | Checks RT timing and calls `update_rt(nav_state)` if due or if `trigger` is true. Returns true if executed. |
| `internal_update(nav_state)` | Checks non-RT timing and calls `update(nav_state)` if due. |
| `update_rt(nav_state)` | **Pure virtual.** Real-time localization update (e.g., predict step). |
| `update(nav_state)` | **Pure virtual.** Non-RT update (e.g., sensor correction, map alignment). |

### Localizer Parameters / NavState / Topics

Like `PlannerMethodBase`, `LocalizerMethodBase` itself does not fix specific parameters or topics. Each concrete localizer plugin documents:

- Parameters (e.g., particle filter sizes, noise models, initial pose).
- NavState keys (e.g., `map.*`, `robot_pose`, sensor inputs).
- Topics/TF frames used.

---

## `easynav::MapsManagerBase`

**Header:** `easynav_core/MapsManagerBase.hpp`  
**Role:** Base class for components that own or maintain maps (2D costmaps, NavMaps, simple maps, etc.).

Typical derived plugins: `easynav_costmap_maps_manager`, `easynav_navmap_maps_manager`, `easynav_simple_maps_manager`, `easynav_bonxai_maps_manager`, ...

### MapsManager Responsibilities

- Extend `MethodBase` with a single non-RT `update()` hook.
- Provide timing helper to run map updates at a configured frequency.

### MapsManager Public API

| Method | Description |
|---|---|
| `internal_update(nav_state)` | Checks update timing and calls `update(nav_state)` when due. |
| `update(nav_state)` | **Pure virtual.** Implement the logic to build or update map representations in `NavState`. |

### MapsManager Parameters / NavState / Topics

`MapsManagerBase` itself does not declare parameters or topics. Concrete managers define:

- Map-specific parameters (files, layers, filters, resolutions, etc.).
- NavState keys (e.g., `map.static`, `map.dynamic`, `map.navmap`, ...).
- Subscriptions/publishers for map I/O.

---

## How to Link From Plugin READMEs

For any plugin whose base class is in `easynav_core`, you can avoid duplicating common behavior and instead add a short note such as:

> This plugin derives from `easynav::ControllerMethodBase`.  
> See the **ControllerMethodBase** section in [`easynav_core`](https://github.com/EasyNavigation/EasyNavigation/tree/rolling/easynav_core#easynavcontrollermethodbase) for shared collision-checking parameters, NavState usage and debug markers.

Similarly for planners, localizers, and maps managers:

- **Planners:** link to the `PlannerMethodBase` section.
- **Localizers:** link to the `LocalizerMethodBase` section.
- **Maps managers:** link to the `MapsManagerBase` section.
