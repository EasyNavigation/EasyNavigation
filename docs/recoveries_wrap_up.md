# Sistema de recuperación ante errores — resumen de cierre

Este documento resume, de una sentada, todo lo construido en la rama `easynav_recovery` frente
a `rolling`, en los dos repositorios implicados (`EasyNavigation` y `easynav_plugins`). Es una
foto final; el relato sesión a sesión, con el porqué de cada decisión, sigue en
[`recoveries_easynav_implementation.md`](recoveries_easynav_implementation.md), y el diseño
original en [`recoveries_easynav.md`](recoveries_easynav.md).

## Alcance de la comparación

| Repositorio | Rama | Base (`rolling`) | Punta | Diff |
|---|---|---|---|---|
| `EasyNavigation` | `easynav_recovery` | `1ef5833` | `0ec7fd9` "Recovery system complete" | 51 ficheros, +6528/−377 |
| `easynav_plugins` | `easynav_recovery` | `07758f3` | `96adaf3` "Recovery system complete" | 66 ficheros, +3868/−4 |

Todo lo descrito aquí está commiteado en ambos repos (`git diff rolling...HEAD` limpio de
sorpresas: no queda trabajo suelto en el árbol de trabajo).

## El diseño obtenido

El sistema es una jerarquía de dos niveles, más un patrón de "recuperación especializada"
co-localizada con el componente que la origina — exactamente lo que planteaba
`recoveries_easynav.md`, con las desviaciones puntuales que se explican más abajo.

**Nivel 0 — reflejos en tiempo real.** Un `SafetyReflexBase` corre en el ciclo RT de
`SystemNode`, justo antes de publicar `cmd_vel`, sea cual sea quien lo haya producido (el
controlador nominal o una mitigación de nivel 1 que haya tomado el control). Es la única capa
que puede pisar `cmd_vel` de forma incondicional; su propio fallo se trata como inseguro
(fail-safe: para el robot en vez de asumir que no hay peligro). El único reflejo implementado,
`CollisionSafetyReflex`, sustituye por completo al viejo mecanismo de colisión que vivía dentro
de `ControllerMethodBase` (migración completa, no coexistencia — ver más abajo).

**Nivel 1 — diagnóstico y mitigación deliberativos, en un nodo propio.** `RecoveryManagerNode`
(paquete nuevo `easynav_recovery`) carga, por `pluginlib`, un número arbitrario de
`RecoveryEvaluatorBase` (solo diagnostican, nunca actúan) y `RecoveryMitigationBase` (actúan,
nunca diagnostican). Cada ciclo no-RT: los evaluadores corren todos; si hay una mitigación
activa se la deja continuar; si no, se escanea el primer diagnóstico no-`OK` y se selecciona el
primer mitigador, en orden de prioridad, que `can_handle()` esa entrada y no esté ya excluido
para ella. Una mitigación que `requires_control()` toma `cmd_vel` vía `control_owner` y se cicla
a ritmo RT (`cycle_rt()`); una que no, se cicla en el propio ciclo no-RT.

**Recuperación especializada co-localizada.** Cuando la señal de fallo es interna a un
componente concreto (p. ej. la covarianza de partículas de AMCL), el evaluador/mitigador
correspondiente vive en el mismo paquete que ese componente (`easynav_costmap_localizer`), no en
el catálogo genérico — solo el autor del plugin conoce esa señal.

**Arbitraje: prioridad + exclusión, sin tabla de *cooldown*.** Cada mitigador se configura con un
`priority` (menor = antes); a igualdad, gana el orden de la lista. Un mitigador que falla
(`RecoveryStatus::FAILED`) para un diagnóstico concreto queda excluido de ser reseleccionado para
esa misma aparición del diagnóstico — hasta que este vuelve a `OK`, momento en que la exclusión se
olvida y una futura recurrencia empieza la escalada desde el principio. No hay *cooldown* por
tiempo ni razonamiento que cruce distintos diagnósticos (pendiente, ver más abajo).

**Handoff de control, en lugar de llamadas directas entre subsistemas.** Un mitigador nunca
llama directamente a otro subsistema (`PlannerNode`, `GoalManager`, etc.) — no tiene referencia a
ellos. Se comunica exclusivamente escribiendo señales en `NavState` que `SystemNode` u otro
subsistema con la referencia real consume y resetea. Es el mecanismo central de todo el diseño;
ver "Mecanismos de coordinación" abajo.

## Componentes añadidos

### Núcleo (`easynav_core`, `EasyNavigation`)

| Componente | Qué es |
|---|---|
| `SafetyReflexBase` | Base de los reflejos de nivel 0. `check()`/`mitigate()`; fallo-seguro ante excepción. |
| `RecoveryEvaluatorBase` | Base de los evaluadores de nivel 1. Solo lee `NavState`; `publish_diagnostic()` escribe/sobrescribe su entrada en el grupo `"diagnostics"`. |
| `RecoveryMitigationBase` | Base de los mitigadores de nivel 1. `can_handle()`, `requires_control()`, ciclo `on_start()`/`on_cycle()`/`on_stop()`; `report()` (ver más abajo) para narrar su actividad. |
| `ObstacleProximity` (`compute_nearest_obstacle()`) | Distancia + *bearing* al obstáculo más cercano, independiente de la geometría de frenada de `CollisionChecker`. Compartida por evaluador y mitigador de proximidad. |
| `MitigationReport` / `RecoveryMitigationBase::report()` | Ver "Observabilidad" más abajo. |

### Nodo nuevo: `easynav_recovery`

`RecoveryManagerNode` (lifecycle node `"recovery_node"`, **no** `"system_node"` — detalle de
despliegue que costó una sesión entera detectar, ver Sesión 15). Carga evaluadores y
mitigadores, arbitra la selección, publica `/diagnostics` y `"mitigation"`. Incluye
`DummyEvaluator`/`DummyMitigation` como plugins de referencia para sus propios tests.

### Reflejo: `CollisionSafetyReflex` (`easynav_controller`)

Sustituye al mecanismo de colisión que antes vivía en `ControllerMethodBase`. Migración
completa: los tres controladores que tenían uso propio de esos parámetros
(`RegulatedPurePursuitController`, `MPCController`) se desacoplaron declarando sus propios
parámetros en vez de heredarlos.

### Evaluadores (`easynav_plugins/recovery_evaluators/`)

- **`NoPathEvaluator`** — diagnostica `"path"` ausente/vacío; `OK` sin goal activo.
- **`ObstacleTooCloseEvaluator`** — condición compuesta: robot ya parado (medido por
  `robot_pose`, con *debounce*) *y* obstáculo por debajo de `safe_distance`.
- **`ControllerStuckEvaluator`** — comandado a moverse pero sin progreso durante
  `stuck_time_threshold`; cuatro precondiciones (sin goal, `control_owner` ajeno, reflejo activo,
  navegación pausada) que lo silencian a `OK` sin diagnosticar nada.

### Mitigadores (`easynav_plugins/recovery_mitigations/`)

- **`SafeRetreatRecovery`** — retrocede en línea recta de un obstáculo demasiado cerca (simplificación deliberada: solo reversa, como el `BackUp` de Nav2).
- **`AdvanceRecovery`** — avanza una distancia corta cuando el controlador está atascado; nunca declara el problema resuelto, solo la activación en curso; acumula tiempo total de episodio para escalar.
- **`HumanAssistanceRecovery`** — catch-all genérico (cualquier `ERROR`), para y espera a que un humano resuelva el problema; `timeout` opcional antes de darse por vencido.
- **`CancelMissionRecovery`** — último escalón: cancela la misión activa (`GoalManager::set_error()`) sin mover el robot; siempre devuelve `FAILED` (nunca resuelve el diagnóstico que lo disparó).

### Especializados, co-localizados (`easynav_costmap_localizer`)

- **`AmclConvergenceEvaluator`** / **`AmclRelocalizeMitigation`** — divergencia del filtro de
  partículas de AMCL (traza de covarianza); gira in situ para relocalizar, con *timeout*.

### Observabilidad

- **`/diagnostics` real** (`diagnostic_msgs/DiagnosticArray`), publicado por `RecoveryManagerNode`
  a partir del grupo interno `"diagnostics"` de `NavState` — para herramientas estándar
  (`rqt_robot_monitor`, `diagnostic_aggregator`) además de la TUI propia.
- **Topic `"mitigation"`** (`rcl_interfaces/msg/Log` — el mismo mensaje que ya usa `/rosout`,
  reutilizado en vez de definir uno propio): narrativa de lo que cada mitigador va haciendo,
  además de rosout, en un canal dedicado y de bajo ruido. `RecoveryMitigationBase::report()` es
  el único punto de llamada para ambos canales a la vez.
- **Panel "Mitigation" en la TUI** (`RichLog`, con *scroll*), bajo el panel "Diagnostics" (ahora
  más pequeño para dejarle sitio), que se limpia al recibir el *sentinel* de "resuelto".
- Printer de `DiagnosticStatus` registrado para que `easynav_navstate`/la TUI lo muestren legible.

### Retirado en el camino

La **Fase 4** completa (`ForceReplanRecovery`, `ClearMapRecovery`, `SafeWaypointRecovery`,
`NotifyAndHoldRecovery`) se implementó y se revirtió (Sesión 11): el mecanismo de comunicación
que usaban (tres señales de un solo disparo en `NavState`, consumidas por `SystemNode`) no
convenció como sitio correcto, y tres de los cuatro mitigadores no aplicaban a este proyecto en
concreto (replan continuo ya activo, costmap sin memoria persistente de obstáculos, sin caso de
uso de *waypoint* seguro). Queda como pregunta abierta, no como decisión cerrada, cómo debería
ser esa comunicación si hiciera falta en el futuro.

## Mecanismos de coordinación

Todo el sistema se apoya en un puñado de patrones repetidos, no en llamadas directas entre
subsistemas:

| Mecanismo | Tipo | Quién escribe | Quién lee/consume | Para qué |
|---|---|---|---|---|
| `"diagnostics.<plugin>"` + grupo `"diagnostics"` | Snapshot, sobrescrito cada ciclo | Cada evaluador (y cada reflejo, solo en las transiciones) | `RecoveryManagerNode` (arbitraje y `/diagnostics`), otros evaluadores | Vocabulario común de "qué está mal ahora mismo" |
| `control_owner` | Cesión continua | `RecoveryManagerNode` al seleccionar una mitigación con `requires_control()` | `SystemNode::system_cycle_rt()` (decide quién produce `cmd_vel`) | Ceder el mando de movimiento sin que el mitigador conozca al controlador |
| `excluded_mitigations_` / `priority` | Estado interno de arbitraje | `RecoveryManagerNode` | `RecoveryManagerNode` (`try_select_mitigation()`) | Evitar reseleccionar en bucle lo que ya falló; orden determinista entre candidatos |
| `mission_cancel_requested` | Señal de un solo disparo | `CancelMissionRecovery::on_start()` | `GoalManager::update()` (llama a `set_error()` y resetea la señal) | El mitigador no tiene (ni debe tener) referencia a `GoalManager` |
| `navigation_paused` | Estado continuo, ya existente | `GoalManager::update()` (refleja `GoalManagerClient::pause()`) | `ControllerStuckEvaluator` | Que pausar la navegación no se confunda con "atascado" |
| `MitigationReport` (`seq` + `rcl_interfaces::msg::Log`) | Slot único sobrescrito, no cola | `RecoveryMitigationBase::report()` | `RecoveryManagerNode::publish_mitigation_log()` | Puente entre el hilo que informa (RT o no-RT según el mitigador) y el publicador (siempre no-RT) sin necesitar una cola *thread-safe* |
| *Sentinel* `DEBUG` en `"mitigation"` | Evento único | `RecoveryManagerNode` (cuando un diagnóstico con `keys_with_mitigation_history_` vuelve a `OK`) | TUI (`MitigationProcessor`) | Saber cuándo limpiar el panel de mitigación |

El hilo conductor: **cualquier comunicación entre un plugin de recuperación y "el resto del
sistema" pasa por `NavState`, nunca por una referencia directa** — es lo que permite que
mitigadores y evaluadores se escriban, compilen y prueben sin conocer `GoalManager`,
`PlannerNode` ni `MapsManagerNode`.

## Configuración de despliegue real

Pipeline activo en `easynav_indoor_testcase/robots_params/costmap.rpp.params.yaml` (repositorio
`easynav_indoor_testcase`, aparte de los dos comparados en este documento — se incluye aquí solo
como contexto de cómo se usa lo construido):

```yaml
system_node:
  ros__parameters:
    safety_reflex_types: [collision]
    collision:
      plugin: easynav_controller/CollisionSafetyReflex

recovery_node:
  ros__parameters:
    evaluator_types: [no_path, obstacle_close, amcl_convergence, controller_stuck]
    mitigation_types: [retreat, amcl_relocalize, advance, human_assistance, cancel_mission]
    # prioridades: retreat/advance/amcl_relocalize (10) < human_assistance (1000) < cancel_mission (2000)
```

Escalada resultante para una pérdida de localización: `amcl_relocalize` (gira 5 s) → excluido →
`human_assistance` (para y espera 10 s) → excluido → `cancel_mission` (cancela la misión,
informando de qué diagnósticos seguían en `ERROR`). En paralelo, `retreat`/`advance` atienden sus
propios diagnósticos (proximidad y atasco) sin interferir.

## Desviaciones deliberadas frente al documento de diseño

Ninguna invalida el diseño — son ajustes que solo se ven al implementar, todos documentados en su
sesión correspondiente:

- **`SafeRetreatRecovery` solo retrocede recto**, no "se aleja por el lado del obstáculo" — coincide con Nav2 (`BackUp`) y con que los robots reales de este workspace son de tracción diferencial.
- **`HumanAssistanceRecovery` sin modo `teleop` ni `ack` con id de episodio** — el "ack" es físico (el diagnóstico volviendo a `OK`).
- **Prioridad de mitigadores como parámetro leído por `RecoveryManagerNode`**, no como método virtual `score()` del propio mitigador — más simple, sin que el mitigador necesite saber nada de arbitraje.
- **`CancelMissionRecovery` devuelve `FAILED`, no `SUCCEEDED`** — cumple el contrato real de `RecoveryStatus` (SUCCEEDED implica que el diagnóstico se resolvió, y cancelar la misión no arregla, p. ej., una localización perdida).
- **Sin tabla de prioridad/*cooldown*/reintento por código de diagnóstico completa (§5.6)** — solo prioridad + exclusión binaria.
- **Fase 4 completa, revertida** (ver arriba).
- **Fase 6 (detección de mala configuración), descartada** por alcance — más ambiciosa que lo necesario ahora mismo.

## Estado del roadmap (`recoveries_easynav.md` §5.12)

| Fase | Estado |
|---|---|
| 0 — cimientos | ✅ Hecho |
| 1 — nivel 0, reflejos RT | ✅ Hecho |
| 2 — nivel 1, evaluación | ✅ Hecho |
| 3 — mitigación y arbitraje de control | ✅ Hecho |
| 4 — mitigación de mapa/planner y de misión | ⬜ Revertida (ver arriba) |
| 5 — recuperación especializada y pulido | ✅ Hecho, con matices |
| 6 — detección de mala configuración | ⬜ Descartada |
| 7 — asistencia humana | ✅ Hecho, descopeada (sin `teleop`) |

## Limitaciones conocidas / trabajo futuro

- **Calibración en robot real** de `covariance_threshold`, `rotation_speed`, umbrales de
  proximidad/atasco, *timeouts* — todos son puntos de partida razonables, no medidos.
- **Sin *cooldown* por tiempo** entre reintentos del mismo diagnóstico, ni razonamiento que cruce
  distintos códigos de diagnóstico (§5.6 completo).
- **`hardware_id`/`values` exactos del reflejo de nivel 0** (§5.13) siguen sin ese detalle —
  publica un diagnóstico genérico.
- **Entradas de recuperación en `easynav_tools plugins`** (el verbo CLI) — no lista
  `RecoveryEvaluatorBase`/`RecoveryMitigationBase`/`SafetyReflexBase`, solo las categorías de
  navegación "normal".
- **`CycleOverrunEvaluator`/`BlackboardFaultEvaluator`** (catálogo original de la Fase 2) nunca se
  implementaron — el segundo, en particular, exigiría que los `catch` de la Fase 0 dejaran
  constancia en `NavState` de la excepción, no solo en el log.
- **Sin evaluador dedicado a salud de mapa/costmap** — quedó huérfano al revertirse
  `ClearMapRecovery`.
- **Modo `teleop` de `HumanAssistanceRecovery`** queda como extensión futura si algún día hace
  falta control remoto real durante la asistencia.

## Ficheros tocados

Listado completo por `git diff rolling...HEAD --stat` en cada repo (commit final
`0ec7fd9`/`96adaf3`, "Recovery system complete"):

**`EasyNavigation`** (51 ficheros): documentación nueva (`docs/recoveries_easynav*.md`); paquete
nuevo `easynav_recovery` (11 ficheros); `easynav_core` extendido (`SafetyReflexBase`,
`RecoveryEvaluatorBase`, `RecoveryMitigationBase`, `ObstacleProximity`, y los cuatro
`*MethodBase` existentes para la captura de excepciones); `easynav_controller` extendido
(`CollisionSafetyReflex`); `easynav_system` (`SystemNode`, `GoalManager`); `easynav_common`
(`NavState::get_group_keys()`); `easynav_tools` (TUI + controladores ROS); `easynav_sensors`
(*throttle* de un log, fuera de fase); `easynav/package.xml` (nueva dependencia).

**`easynav_plugins`** (66 ficheros): tres controladores existentes desacoplados de la colisión
retirada (`RegulatedPurePursuitController`, `MPCController`, `VffController`); `CostmapPlanner`
(vacía y publica el path al cancelar); siete paquetes nuevos completos, cada uno con su propio
`CMakeLists.txt`/`package.xml`/manifiesto de plugin/tests: `easynav_controller_stuck_evaluator`,
`easynav_no_path_evaluator`, `easynav_obstacle_too_close_evaluator`, `easynav_advance_recovery`,
`easynav_cancel_mission_recovery`, `easynav_human_assistance_recovery`,
`easynav_safe_retreat_recovery`; más `AmclConvergenceEvaluator`/`AmclRelocalizeMitigation` dentro
del paquete existente `easynav_costmap_localizer`.

Para el detalle línea a línea, `git diff rolling...HEAD` en cada repo (o `--stat` para solo la
lista de ficheros) es la fuente autoritativa — este documento resume, no sustituye al diff real.

## Verificación

Cada sesión se verificó con el mismo procedimiento: build del paquete tocado, sus tests, luego
`pixi run -e rolling build --packages-above easynav_core` (todo lo que depende, directa o
transitivamente, de lo tocado) y la batería de tests de los paquetes relevantes con
`--parallel-workers 1` (evita falsos positivos de contención de recursos al lanzar muchos
binarios `gtest`/`rclcpp` en paralelo). Estado final: **37 paquetes de `EasyNavigation`
compilan**, **1501 tests, 0 fallos nuevos** (los 28 fallos presentes son preexistentes de
`robotnik_sensors`, un paquete Python no relacionado con este desarrollo). La TUI se verificó
lanzándola manualmente — no tiene suite automatizada en este repo.
