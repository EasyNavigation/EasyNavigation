# Implementación del sistema de recuperación ante errores — bitácora

Este fichero documenta el **proceso de implementación** de la propuesta descrita en
[`recoveries_easynav.md`](recoveries_easynav.md), deliberadamente en un documento aparte para no
mezclar diseño razonado con el registro de lo que realmente se ha construido, verificado o
tenido que corregir sobre la marcha.

Convenciones de este documento:

- Se organiza por las **fases del roadmap** de `recoveries_easynav.md` §5.12 (Fase 0 a Fase 7).
- Cada fase tiene una tabla de estado y, debajo, una entrada de bitácora por sesión de trabajo
  con: qué se hizo, qué se verificó (build/tests), y qué se descubrió que obliga a matizar o
  corregir algo del documento de diseño.
- Los commits los gestiona el usuario directamente; este documento no asume ni registra
  operaciones de git — solo el estado del código en el árbol de trabajo.

## Estado general

| Fase (ver §5.12 del diseño) | Estado |
|---|---|
| Fase 0 — cimientos | ✅ Hecho salvo un pendiente reabierto (excepciones capturadas desde la Fase 0; dependencia a `diagnostic_msgs` añadida en la Fase 2; `GoalManager::set_failed/set_error` se conectó en la Fase 4 vía `NotifyAndHoldRecovery` pero ese mitigador se retiró en la Sesión 11 — vuelve a estar pendiente) |
| Fase 1 — nivel 0, reflejos RT | ✅ Hecho (`SafetyReflexBase`, `CollisionSafetyReflex`, wiring en `SystemNode`, mecanismo antiguo retirado y migrado, reflejo publica en `"diagnostics"` desde la Sesión 9) |
| Fase 2 — nivel 1, evaluación | ✅ Hecho (`RecoveryEvaluatorBase`, `RecoveryManagerNode`, wiring en `SystemNode`, primer evaluador real `NoPathEvaluator`) |
| Fase 3 — mitigación y arbitraje de control | ✅ Hecho (`RecoveryMitigationBase`, `control_owner`, único par evaluador+mitigador: `ObstacleTooCloseEvaluator`/`SafeRetreatRecovery`, con *debounce* desde la Sesión 10) |
| Fase 4 — mitigación de mapa/planner y de misión | ⬜ Revertida (Sesión 11) — implementada y luego retirada en su mayor parte; ver esa sesión para el porqué |
| Fase 5 — recuperación especializada y pulido | ✅ Hecho, con matices (Sesión 20) frente a los 4 puntos de §5.12.6 — ver esa sesión |
| Fase 6 — detección de mala configuración | ⬜ Descartada (Sesión 20) — fuera de alcance por ahora, ver esa sesión |
| Fase 7 — asistencia humana | ✅ Hecho, descopeada (Sesión 20) — sin modo `teleop` ni `ack` con id de episodio, ver esa sesión |

---

## Fase 0 — Cimientos

Objetivo (recoveries_easynav.md §5.12, punto 1): capturar las excepciones de `NavState` en el
punto de invocación de cada plugin en lugar de dejarlas tumbar el proceso; conectar
`GoalManager::set_failed/set_error` a al menos un fallo real; añadir dependencia a
`diagnostic_msgs`.

| Tarea | Estado |
|---|---|
| Capturar excepciones en el punto de invocación de cada plugin | ✅ Hecho (este documento, sesión 1) |
| Tests de regresión (un plugin que lanza no debe propagar la excepción) | ✅ Hecho (sesión 1) |
| Conectar `GoalManager::set_failed/set_error` a un fallo real | ✅ Hecho de nuevo — vía `CancelMissionRecovery` (Fase 5, Sesión 19), tras haberse retirado en la Sesión 11 junto con `NotifyAndHoldRecovery` |
| Añadir dependencia a `diagnostic_msgs` | ✅ Hecho — añadida en la Fase 2 (`easynav_recovery`, Sesión 5) |

### Sesión 1 — capturar excepciones en el punto de invocación de los plugins

**Qué se encontró al mirar el código real.** El documento de diseño hablaba en genérico de "el
punto de invocación de cada plugin". En la práctica, ese punto no está en los `*Node` (
`ControllerNode`, `PlannerNode`, `LocalizerNode`, `MapsManagerNode`) sino un nivel más abajo, en
las clases base de `easynav_core` que ya envuelven la llamada al método virtual de cada plugin:

| Clase base | Método envoltorio | Llama a |
|---|---|---|
| `ControllerMethodBase` | `internal_update_rt()` | `update_rt(nav_state)` |
| `LocalizerMethodBase` | `internal_update_rt()` | `update_rt(nav_state)` |
| `LocalizerMethodBase` | `internal_update()` | `update(nav_state)` |
| `PlannerMethodBase` | `internal_update()` | `update(nav_state)` |
| `PlannerMethodBase` | `force_update()` | `update(nav_state)` |
| `MapsManagerBase` | `internal_update()` | `update(nav_state)` |

Esto es en realidad mejor que parchear cada `*Node`: al estar en la clase base compartida por
**todos** los plugins de una categoría, la protección se aplica una sola vez y cubre
automáticamente cualquier plugin presente o futuro (`easynav_plugins/controllers/...`,
`.../planners/...`, etc.), sin tener que tocar cada implementación concreta.

**Cambio aplicado.** En los seis puntos de la tabla, la llamada al método virtual del plugin
(`update_rt(...)`/`update(...)`) queda envuelta en `try { ... } catch (const std::exception & e)`,
que registra el error con `RCLCPP_ERROR_THROTTLE` (mismo patrón ya usado en
`ControllerMethodBase::on_inminent_collision`, para no inundar el log si el plugin falla en
cada ciclo a 200 Hz) y deja que la ejecución continúe con normalidad:

- En `ControllerMethodBase::internal_update_rt`, el chequeo de colisión inminente
  (`is_inminent_collision`/`on_inminent_collision`) sigue ejecutándose **después** del `catch`,
  sobre lo que haya en `nav_state` en ese momento — la red de seguridad no depende de que el
  plugin haya tenido éxito.
- En el resto de casos, tras el `catch` la función simplemente termina con normalidad (como si
  el ciclo no hubiera producido cambios), sin relanzar la excepción.

Ficheros tocados: `easynav_core/src/easynav_core/ControllerMethodBase.cpp`,
`LocalizerMethodBase.cpp`, `PlannerMethodBase.cpp`, `MapsManagerBase.cpp`.

**Tests añadidos.** En `easynav_core/test/core_method_test.cpp`: cuatro plugins mínimos
(`ThrowingLocalizer`, `ThrowingPlanner`, `ThrowingMapsManager`, `ThrowingController`) cuyos
`update()`/`update_rt()` incrementan un contador y después lanzan `std::runtime_error`, y seis
tests (`*ExceptionDoesNotPropagate`) que comprueban con `EXPECT_NO_THROW` que la excepción no
sale de `internal_update()`/`internal_update_rt()`/`force_update()`, y que el contador sí se
incrementó (o sea, que el plugin sí se llegó a invocar antes de fallar).

**Verificación.**

```
pixi run -e rolling build --packages-select easynav_core
pixi run -e rolling colcon test --packages-select easynav_core
```

Resultado: build limpio, **33/33 tests pasan** (27 preexistentes + 6 nuevos), incluidos los
linters del paquete (`uncrustify`, `cppcheck`, `lint_cmake`, `xmllint`) sin avisos nuevos.

**Alcance no cubierto.** Los sensores (`easynav_sensors`) no siguen el patrón
`internal_update()`/`update()` de `MethodBase`: usan `PerceptionHandler`, orientado a
suscripciones/callbacks por tipo de percepción, no a un ciclo periódico. Aplicarles la misma
protección requiere mirar ese código con más calma y queda pendiente como una tarea aparte
(no bloquea el resto de la Fase 0, que ya cubre los cuatro subsistemas que participan
directamente en el bucle RT/no-RT: localización, mapas, planificación y control).

### Corrección al documento de diseño: la API real de `set_group`

Al revisar `NavState.hpp` para preparar el grupo `"diagnostics"` (§5.5 del diseño) se descubre
que `set_group()` **no** tiene la forma que el diseño asumía
(`nav_state.set_group<T>("diagnostics", plugin_name, status)`, tomando una clave de plugin y un
valor). La firma real es:

```cpp
void set_group(const std::string & key, const std::vector<std::string> & group_keys);
```

Es decir, un grupo es una lista de **claves que ya existen** en `NavState` (así es como
`SensorsNode` agrupa las percepciones de varios sensores). Para que un `RecoveryEvaluatorBase`
publique su diagnóstico habrá que, en la Fase 2:

1. Guardar el propio `diagnostic_msgs::msg::DiagnosticStatus` bajo una clave propia del
   evaluador, p. ej. `nav_state.set("diagnostics." + get_plugin_name(), status)`.
2. Mantener aparte la lista de claves que componen el grupo `"diagnostics"` (añadiendo la clave
   propia si es la primera vez) y llamar a `set_group("diagnostics", esa_lista)`.

Esto es un detalle de implementación, no cambia nada del diseño en `recoveries_easynav.md`
(que ya hablaba de "la misma primitiva `set_group()` que ya usa `SensorsNode`" — la primitiva es
la correcta, la forma de usarla es la que había que precisar). Se deja anotado aquí para no
repetir la misma exploración cuando llegue la Fase 2, y probablemente convenga entonces un
pequeño helper (p. ej. en `NavState` o en `RecoveryEvaluatorBase`) que encapsule este patrón de
"añadir una entrada a un grupo que puede no existir todavía" en vez de repetirlo en cada
evaluador.

### Cierre de la Fase 0 (Sesión 7, dentro de la Fase 4) — reabierto en parte en la Sesión 11

Los dos pendientes de esta fase se cerraron finalmente en la Fase 4, no antes, porque los dos
necesitaban maquinaria que todavía no existía. **El segundo ha vuelto a quedar pendiente**: la
Sesión 11 retiró `NotifyAndHoldRecovery` junto con el resto de la Fase 4 (ver esa sesión), así
que la conexión que describe este apartado ya no existe en el código — se deja el relato tal
cual como registro de lo que se probó y por qué, no como estado actual.

- **Dependencia a `diagnostic_msgs`.** Se resolvió en la Fase 2 (Sesión 4), en cuanto existió
  código que de verdad construye un `diagnostic_msgs::msg::DiagnosticStatus`.
- **Conectar `GoalManager::set_failed/set_error` a un fallo real.** El wiring evaluado y
  descartado aquí en su momento (cambiar la firma de `cycle()`/`cycle_rt()` de los cuatro
  `*Node` para propagar "hubo una excepción") nunca hizo falta: la ruta real que finalmente lo
  conecta es `NotifyAndHoldRecovery` (Fase 4, Sesión 7) escribiendo una petición en `NavState`
  que `SystemNode::system_cycle()` traduce a una llamada directa a `goal_manager_->set_failed()`/
  `set_error()` — el mismo patrón que ya usa `control_owner`, aplicado aquí a una acción de un
  solo disparo en vez de a una cesión continua de `cmd_vel`. Ver Sesión 7 para el detalle.

---

## Fase 1 — Nivel 0: reflejos de seguridad en tiempo real

Objetivo (recoveries_easynav.md §5.12, punto 2): extraer `SafetyReflexBase` de `easynav_core`,
dar a `SystemNode` su propio `pluginlib::ClassLoader<SafetyReflexBase>` (`safety_reflex_types`),
y refactorizar `is_inminent_collision`/`on_inminent_collision` como el plugin de referencia
`CollisionSafetyReflex` — sin tocar su ubicación real en el ciclo RT ni su coste.

| Tarea | Estado |
|---|---|
| `SafetyReflexBase` en `easynav_core`, con fallo-seguro ante excepción | ✅ Hecho |
| Reflejo publica su severidad en el grupo `"diagnostics"` de `NavState` (§5.2) | ✅ Hecho — Sesión 9, ver más abajo |
| Extraer la geometría de colisión a `CollisionChecker` (compartida, sin duplicar) | ✅ Hecho — consolidada de vuelta en `CollisionSafetyReflex` en la Sesión 8, ver más abajo |
| `CollisionSafetyReflex` como plugin de referencia en `easynav_controller` | ✅ Hecho |
| `SystemNode` carga y ejecuta los reflejos cada ciclo RT, antes de publicar `cmd_vel` | ✅ Hecho |
| Comprobar que ningún otro paquete rompe por el refactor de `ControllerMethodBase` | ✅ Hecho (encontrado y corregido 1 caso, ver más abajo) |

### Sesión 2 — `SafetyReflexBase`, `CollisionChecker` y el reflejo en `SystemNode`

**Diseño de `SafetyReflexBase`** (`easynav_core/include/easynav_core/SafetyReflexBase.hpp`).
Interfaz mínima —`check(nav_state)`/`mitigate(nav_state)`— envuelta por un método público
`internal_check_and_mitigate()` que:

- No está limitado por `MethodBase::isTime2RunRT()`: `SystemNode` ya controla la frecuencia
  global del ciclo RT, así que un reflejo se evalúa en todos y cada uno de esos ciclos.
- Trata el propio fallo del reflejo como inseguro: si `check()` o `mitigate()` lanzan una
  excepción, en vez de asumir "no hay peligro" (fail-open), se aplica `stop_robot()` como
  mitigación por defecto (fail-safe). Esta es una decisión de seguridad explícita que no estaba
  detallada así en `recoveries_easynav.md` — queda documentada aquí porque es exactamente el
  tipo de matiz que solo aparece al implementar: para un componente cuyo único trabajo es decidir
  si algo es seguro, que ese propio componente falle no puede tratarse igual que "todo va bien".

**Extracción de la geometría a `CollisionChecker`** (`easynav_core/include/easynav_core/CollisionChecker.hpp`).
En vez de mover literalmente `is_inminent_collision`/`on_inminent_collision`/
`publish_collision_zone_marker` fuera de `ControllerMethodBase` (lo que habría exigido decidir,
sin datos, si retirar de golpe el chequeo de colisión que hoy usan varios controladores vía el
parámetro `colision_checker.active`), se extrajo la implementación a una clase reutilizable,
`CollisionChecker`, y **ambos caminos la comparten**:

- `ControllerMethodBase` sigue exponiendo exactamente el mismo comportamiento y los mismos
  nombres de parámetro (`colision_checker.active`, `.robot_radius`, `.brake_acc`,
  `.safety_margin`, etc.) que tenía antes — internamente delegando en un `CollisionChecker`
  propio. Cero cambios de configuración para quien ya lo usaba.
- El nuevo plugin `CollisionSafetyReflex` (`easynav_controller`) tiene su propia instancia de
  `CollisionChecker`, inicializada con sus propios parámetros bajo el id de instancia que se le
  dé en `safety_reflex_types` (p. ej. `collision.robot_radius`).

Esto evita duplicar ~150 líneas de código geométrico de seguridad (una sola implementación que
auditar) y, sobre todo, **no retira nada existente**: los dos mecanismos pueden convivir. Tener
el mismo chequeo corriendo dos veces (una vez por controlador vía `ControllerMethodBase`, otra
uniforme para todo el sistema vía `CollisionSafetyReflex`) no es un error en un mecanismo de
seguridad — es redundancia, y es intencional dejarlo así hasta que se audite qué despliegues
usan `colision_checker.active` y se decida si migrarlos y retirarlo (pregunta abierta, ver
"Decisión pendiente" más abajo).

**`CollisionSafetyReflex`** (`easynav_controller/include|src/easynav_controller/CollisionSafetyReflex.hpp|.cpp`,
manifiesto `easynav_safety_reflexes_plugins.xml`). Plugin de referencia: `check()` delega en
`CollisionChecker::check()`, `mitigate()` llama a `stop_robot()` (heredado de
`SafetyReflexBase`). Se coloca en `easynav_controller` (no en un nuevo paquete `easynav_recovery`)
siguiendo la alternativa que el propio diseño dejaba anotada en §5.3: "o `easynav_controller`,
ya que hoy vive ahí".

**`SystemNode`** (`easynav_system`). Cambios:

- Nuevo `pluginlib::ClassLoader<SafetyReflexBase>` (contra `easynav_core`, igual que el resto de
  categorías), cargado en el constructor.
- `on_configure()` lee `safety_reflex_types` (lista, **vacía por defecto** — activar el reflejo
  es opt-in, ver "Decisión pendiente") y, por cada id, `<id>.plugin`, igual que
  `ControllerNode::on_configure()` — pero sin la restricción de "como mucho uno": los reflejos
  son compuestos por diseño (§5.2), así que se cargan todos en un `std::vector`.
- `system_cycle_rt()`: justo después de `controller_node_->cycle_rt(...)`, se recorren todos los
  reflejos cargados con `internal_check_and_mitigate(*nav_state_)`. Se añade una variable
  `reflex_intervened` y la condición de publicación pasa de
  `if (trigger_controller)` a `if (trigger_controller || reflex_intervened)` — así, si un
  reflejo modifica `cmd_vel` en un ciclo en el que el controlador activo no ha corrido (porque su
  `rt_freq` es más baja que la del sistema), el parón se publica igualmente. Sin este cambio, el
  reflejo podría "ganar" la carrera por escribir en `NavState` pero perder la de publicarse a
  tiempo — un fallo sutil que solo aparece al mirar el punto exacto de publicación.
- Destructor: descarga las librerías de los reflejos cargados, mismo patrón que
  `ControllerNode`/`PlannerNode`.

**Regresión real encontrada y corregida.** Antes de dar la fase por cerrada se compiló
`--packages-above easynav_core` (los 29 paquetes que dependen, directa o transitivamente, de
`easynav_core`) para comprobar que retirar `is_inminent_collision`/`publish_collision_zone_marker`
y sus campos de `ControllerMethodBase` no rompía nada fuera de este paquete. Encontró un caso
real: `easynav_plugins/controllers/easynav_regulated_pp_controller/.../RegulatedPurePursuitController.cpp`
leía directamente `robot_radius_`, `safety_margin_`, `z_min_filter_` y `robot_height_` —
miembros protegidos que existían en `ControllerMethodBase` y que el refactor movía dentro de
`CollisionChecker`, donde son privados. Se resolvió añadiendo *getters* públicos a
`CollisionChecker` (`robot_radius()`, `safety_margin()`, `z_min_filter()`, `robot_height()`,
`brake_acc()`, `downsample_leaf_size()`) y actualizando ese controlador para usarlos a través del
miembro `collision_checker_` (protegido, heredado de `ControllerMethodBase`) en vez de acceder a
campos que ya no existen. Sin este paso, la Fase 1 habría roto la compilación de un controlador
en producción — es la razón concreta por la que conviene compilar "hacia arriba" (todo lo que
depende del paquete tocado) y no solo el paquete que se está modificando.

**Verificación.**

```
pixi run -e rolling build --packages-above easynav_core   # 29 paquetes, todos compilan
pixi run -e rolling colcon test --packages-select \
  easynav_core easynav_controller easynav_system easynav_regulated_pp_controller
pixi run -e rolling colcon test-result --all               # 0 fallos nuevos en los 4 paquetes
```

Se añadieron tests en `easynav_core/test/core_method_test.cpp` (comportamiento de
`SafetyReflexBase`: no mitiga si `check()` da falso, mitiga si da verdadero, falla seguro si
`check()`/`mitigate()` lanzan) y en `easynav_system/tests/system_safety_reflex_tests.cpp`
(`on_configure()` sigue funcionando sin reflejos configurados — compatibilidad hacia atrás—,
carga correctamente `easynav_controller/CollisionSafetyReflex` cuando se configura, y falla de
forma controlada con un nombre de plugin inexistente). Todos los tests, linters incluidos,
pasan.

### Decisión tomada: migración completa (Sesión 3)

La pregunta abierta de la sesión anterior —coexistencia o migración— se resolvió por migración:
se retiró por completo el mecanismo de `ControllerMethodBase` y toda la configuración real de
despliegue encontrada en el repositorio se movió al reflejo de `SystemNode`.

**Retirada en `easynav_core`.** `ControllerMethodBase` pierde `collision_checker_active_`,
`collision_checker_` (la instancia de `CollisionChecker`), `on_inminent_collision()` y su propio
`initialize()` (que ya no hacía nada más que reenviar a `MethodBase::initialize()`, así que se
elimina en vez de dejar un envoltorio vacío). `CollisionChecker` y `SafetyReflexBase` se quedan
donde estaban — siguen siendo la base de `CollisionSafetyReflex` — pero ya nada en
`ControllerMethodBase` los usa.

**Dos plugins tenían un uso propio y distinto, no solo heredado, y se desacoplaron en vez de
romperse:**

- **`RegulatedPurePursuitController`** leía `robot_radius_`/`safety_margin_`/`z_min_filter_`/
  `robot_height_` directamente de la clase base para su *propia* heurística de regulación de
  velocidad por proximidad a obstáculos (`computeMinObstacleDistance()`) — algo conceptualmente
  distinto del reflejo de parada (es una regulación continua de velocidad, no un parón). Ahora
  declara sus cuatro parámetros propios bajo su namespace habitual (`<instancia>.robot_radius`,
  etc., mismos valores por defecto que tenía `CollisionChecker`), en vez de depender de un
  mecanismo que ya no existe.
- **`MPCController`** leía `ControllerMethodBase::collision_checker_active_` como interruptor de
  su propia restricción de evitación de obstáculos *dentro* de la optimización MPC
  (`collision_checker()`, código enteramente propio, sin relación con `CollisionChecker`). Ahora
  declara su propio `bool collision_checker_active_` y su propio parámetro
  `<instancia>.colision_checker.active`, desacoplado de la clase base.

**Efecto colateral encontrado al recompilar todo el árbol** (de nuevo, `--packages-above
easynav_core`): `VffController.hpp` usaba `pcl::PointXYZ` sin incluir `pcl/point_types.h`
directamente — funcionaba porque `ControllerMethodBase.hpp` lo incluía transitivamente (primero
directamente, luego a través de `CollisionChecker.hpp`). Al retirar esa inclusión de
`ControllerMethodBase.hpp` el include transitivo desaparece y la compilación de
`easynav_vff_controller` falla. Se corrigió añadiendo el include que le faltaba directamente en
`VffController.hpp` — la solución correcta (no depender de includes transitivos ajenos), no un
parche. Es la segunda vez en esta migración que compilar "hacia arriba" del paquete tocado
encuentra un problema real; confirma que es un paso imprescindible, no opcional, cuando se toca
`easynav_core`.

**Migración de la configuración de despliegue real.** Se encontraron 7 ficheros YAML reales del
repositorio (`easynav_indoor_testcase`, `easynav_experiments`) que configuraban
`controller_node.colision_checker.*` — es decir, esto no era solo una API interna, tenía
despliegues reales detrás:

| Fichero | Antes | Después |
|---|---|---|
| `costmap.rpp.params.yaml`, `easynav.tb4.params.yaml` (controlador `rpp`) | `controller_node.colision_checker.{active:true, debug_markers:true, downsample_leaf_size:0.05, robot_radius:0.30, brake_acc:1.0, safety_margin:0.05}` | Bloque retirado; `rpp.{robot_radius:0.30, safety_margin:0.05}` (su propia heurística) + `system_node.safety_reflex_types:[collision]` con el resto de valores |
| `costmap.serest.params.yaml`, `sim-easynav.params.mppi.yaml` (controlador `serest`, sin uso propio) | mismo bloque que arriba | Bloque retirado sin más; mismo `system_node.safety_reflex_types:[collision]` |
| `costmap.mpc.params.yaml` (controlador `mpc`) | `controller_node.colision_checker.{active:true, debug_markers:true}` | `mpc.colision_checker.active: true` (ahora es un parámetro propio de `MPCController`, ya no de la base) + `system_node.safety_reflex_types:[collision]` con `debug_markers:true` |
| `bonxai.amcl.params.urjc.yaml`, `bonxai.amcl.params.urjc_alt_imu.yaml` (`active: false`) | bloque completo declarado pero desactivado | Bloque retirado sin más; nada añadido a `system_node` (equivalente exacto a estar desactivado) |

En todos los casos donde `active: true`, el reflejo se migra con los **mismos valores
numéricos** que tenía declarados el mecanismo antiguo, para que el comportamiento efectivo del
robot no cambie por la migración en sí — solo cambia dónde vive la configuración.

**Verificación.** Se repitió el mismo procedimiento que en la sesión anterior:

```
pixi run -e rolling build --packages-above easynav_core     # 29 paquetes, todos compilan
pixi run -e rolling colcon test --packages-select \
  easynav_core easynav_controller easynav_system \
  easynav_regulated_pp_controller easynav_mpc_controller easynav_vff_controller \
  easynav_serest_controller easynav_mppi_controller easynav_simple_controller
pixi run -e rolling colcon test-result --all
```

0 fallos nuevos. Los únicos fallos presentes en el árbol (`robotnik_sensors`, un paquete Python
no relacionado) son preexistentes y no se tocaron en esta sesión. Se actualizó también
`core_method_test.cpp`, retirando el test `ControllerInitializeDeclaresCollisionParams` (verificaba
parámetros que ya no declara `ControllerMethodBase`).

### Sesión 8 — consolidar `CollisionChecker` dentro de `CollisionSafetyReflex`

**Por qué.** `CollisionChecker` se extrajo como clase propia de `easynav_core` en la Sesión 2
porque en ese momento tenía **dos** consumidores: `ControllerMethodBase` (el mecanismo antiguo)
y el nuevo plugin `CollisionSafetyReflex`. La Sesión 3 retiró por completo el primero (migración
completa, ver más arriba) y con él los *getters* públicos que exponía para
`RegulatedPurePursuitController`. Desde entonces `CollisionChecker` tiene un único consumidor,
`CollisionSafetyReflex`, y vivir en un paquete distinto (`easynav_core`) del que lo usa
(`easynav_controller`) ya no aporta nada: es indirección sin reutilización real.

**Cambio aplicado.** Se aplanó `CollisionChecker` dentro de `CollisionSafetyReflex`: los
miembros y métodos (`initialize()` → `on_initialize()`, `check()`,
`publish_collision_zone_marker()`) pasan a ser parte directa de la clase
`CollisionSafetyReflex` (`easynav_controller`), sin una instancia intermedia. Se eliminan
`easynav_core/include/easynav_core/CollisionChecker.hpp` y
`easynav_core/src/easynav_core/CollisionChecker.cpp`. Como consecuencia, `PCL` y
`visualization_msgs` — que ya no los usa ningún otro fichero de `easynav_core` (comprobado por
grep antes de tocar nada) — se mueven de las dependencias de `easynav_core` a las de
`easynav_controller`, junto con `easynav_sensors` (ya usado transitivamente vía `easynav_core`,
pero ahora incluido directamente en `CollisionSafetyReflex.cpp`, así que se declara también
como dependencia directa). No se ha tocado `tf2_ros`/`tf2_geometry_msgs` en `easynav_core`
aunque un grep no encuentra ya ningún uso directo: parece deuda preexistente no relacionada con
este cambio, fuera de alcance de esta sesión.

**No afecta a ningún otro fichero.** `ObstacleProximity.hpp` (Fase 3) solo mencionaba
`CollisionChecker` en un comentario explicativo ("unlike `CollisionChecker::check()`..."),
actualizado a `CollisionSafetyReflex::check()`. Ningún test del árbol (ni en `easynav_core` ni
en `easynav_controller`) instanciaba `CollisionChecker` directamente, así que no hubo tests que
migrar.

### Sesión 9 — los reflejos también publican en `"diagnostics"`

**El hueco.** §5.2 del diseño es explícito: "Cuando un reflejo se dispara... escribe (con la
misma primitiva barata `set_group()` que ya usan los evaluadores) una entrada en el grupo
`"diagnostics"` de `NavState`", precisamente para que un futuro evaluador de nivel 1 (el
`RepeatedReflexEvaluator` que el propio diseño pone como ejemplo) pueda notar que un reflejo se
dispara repetidamente y escalar a una mitigación deliberativa, sin que la reacción inmediata del
reflejo dependa nunca de ese ciclo lento. Al revisar el código se confirmó que esto nunca se
implementó en la Sesión 2: ni `SafetyReflexBase` ni `CollisionSafetyReflex` escribían nada en
`"diagnostics"` — solo había un `RCLCPP_ERROR_THROTTLE` (log). No estaba anotado como pendiente
en ningún sitio de este documento; era simplemente un trozo del diseño que no se llegó a hacer.

**Cambio aplicado.** `SafetyReflexBase::internal_check_and_mitigate()` ahora reporta su propio
nivel de severidad bajo `"diagnostics.<plugin_name>"`, con la misma clave/convención de grupo
que `RecoveryEvaluatorBase::publish_diagnostic()` (mismo `"diagnostics.<plugin_name>"`,
pertenencia al grupo `"diagnostics"`):

- `OK` cuando `check()` devuelve `false` (no disparado).
- `WARN` cuando `check()` devuelve `true` y `mitigate()` se ejecuta con éxito.
- `ERROR` cuando `check()` o `mitigate()` lanzan una excepción (mismo camino de fallo-seguro que
  ya existía para `cmd_vel`).

**Decisión de diseño no explícita en el texto original: reportar solo en las transiciones, no
en cada ciclo.** El texto del diseño no precisa la frecuencia de publicación, y esto corre en el
ciclo RT (hasta 200 Hz) — publicar de forma incondicional en cada ciclo, como sí hacen los
evaluadores de nivel 1 en su ciclo no-RT (documentado explícitamente en
`RecoveryEvaluatorBase`: "every call to update() is expected to call publish_diagnostic()"),
contradiría la propia exigencia de la clase de ser "barato y determinista". En su lugar,
`SafetyReflexBase` guarda el último nivel reportado (`std::optional<uint8_t>`) y solo escribe
cuando cambia — el primer ciclo siempre publica (para que el reflejo aparezca en `"diagnostics"`
en cuanto arranca, aunque nunca se haya disparado), y a partir de ahí solo en las transiciones
OK↔WARN↔ERROR. Importante: la comparación es por **nivel**, no por el booleano "disparado", para
cubrir correctamente el caso "disparado, `mitigate()` funciona" (`WARN`) → "disparado,
`mitigate()` empieza a lanzar" (`ERROR`) — ambos son "disparado = true", así que una detección de
flanco basada solo en ese booleano se habría perdido esa transición; el propio test
`ReflexDiagnosticTracksLevelAcrossCycles` reproduce exactamente esta secuencia.

**Verificación.**

```
pixi run -e rolling build --packages-select easynav_core
pixi run -e rolling colcon test --packages-select easynav_core
pixi run -e rolling build --packages-above easynav_core   # 37 paquetes, todos compilan
pixi run -e rolling colcon test --packages-select <los mismos 17 de siempre>
pixi run -e rolling colcon test-result --all
```

0 fallos nuevos (mismos 28 fallos preexistentes de `robotnik_sensors`). 5 tests añadidos en
`core_method_test.cpp` (`ReflexNotTriggeredPublishesOkDiagnostic`,
`ReflexTriggeredPublishesWarnDiagnostic`, `ReflexCheckThrowPublishesErrorDiagnostic`,
`ReflexMitigateThrowPublishesErrorDiagnostic`, `ReflexDiagnosticTracksLevelAcrossCycles`).
Linters limpios.

---

## Fase 2 — Nivel 1: evaluación deliberativa (no-RT)

Objetivo (recoveries_easynav.md §5.12, punto 3): paquete `easynav_recovery`,
`RecoveryEvaluatorBase`, `RecoveryManagerNode` cargado por `SystemNode` en el ciclo no-RT, con
`CycleOverrunEvaluator`, `BlackboardFaultEvaluator`, `NoPathEvaluator` como primeros plugins
(solo diagnostican, todavía no mitigan).

| Tarea | Estado |
|---|---|
| `NavState::get_group_keys()` (pieza que faltaba, detectada en la Fase 0) | ✅ Hecho |
| `RecoveryEvaluatorBase` en `easynav_core`, con `diagnostic_msgs` | ✅ Hecho |
| Paquete `easynav_recovery` con `RecoveryManagerNode` (carga múltiples evaluadores) | ✅ Hecho |
| Wiring en `SystemNode` (sexto subsistema, ciclo no-RT) | ✅ Hecho |
| `NoPathEvaluator` (primer evaluador real, en `easynav_plugins`) | ✅ Hecho — corregido en la Fase 5, Sesión 16: no debe diagnosticar nada sin un goal activo |
| `CycleOverrunEvaluator` | ⬜ Pendiente — ver "Próximos pasos" |
| `BlackboardFaultEvaluator` | ⬜ Pendiente — ver "Próximos pasos" |

### Sesión 4 — `RecoveryEvaluatorBase`, `RecoveryManagerNode` y `NoPathEvaluator`

**El hueco anotado en la Fase 0 se cierra primero.** Antes de poder escribir un solo
diagnóstico había que resolver cómo un evaluador añade su propia entrada al grupo compartido
`"diagnostics"` sin pisar las de los demás. `NavState::set_group()` solo permite *escribir* la
lista completa de claves de un grupo, no leerla — así que se añadió
`NavState::get_group_keys(const std::string &)` (nuevo, puramente aditivo, con sus propios
tests en `easynav_common/tests/navstate_tests.cpp`), que devuelve la lista de claves actuales
de un grupo (vacía si no existe). Con esto, `RecoveryEvaluatorBase::publish_diagnostic()` puede:
leer la lista actual, comprobar si su propia clave ya está, y si no añadirla antes de volver a
llamar a `set_group()`.

**`RecoveryEvaluatorBase`** (`easynav_core`). Mismo patrón que el resto de `*MethodBase`:
`internal_update()` respeta `isTime2Run()`/`setRun()` y envuelve `update()` en un `try/catch`
que no deja escapar la excepción (igual que la Fase 0 hizo con controller/localizer/planner/maps
manager). Lo nuevo es `publish_diagnostic(nav_state, status)`, protegido, que:

1. Guarda el `diagnostic_msgs::msg::DiagnosticStatus` bajo `"diagnostics.<plugin_name>"`.
2. Añade esa clave al grupo `"diagnostics"` si es la primera vez que este plugin publica.

Se documenta explícitamente en la clase (y se verifica con un test,
`PublishDiagnosticOverwritesInsteadOfDuplicating`) la convención de diseño ya acordada: un
evaluador debe llamar a `publish_diagnostic()` en cada `update()` con su valoración *actual*,
incluyendo `OK` cuando lo que reportaba se ha resuelto — así una entrada nunca queda obsoleta
para siempre, se sobrescribe.

**`easynav_recovery` (paquete nuevo).** `RecoveryManagerNode` sigue el mismo patrón de
`ControllerNode`/`PlannerNode` (carga por pluginlib contra `easynav_core`, parámetro
`<id>.plugin`), pero **sin** la restricción de "como mucho una instancia" — cualquier número de
evaluadores en `evaluator_types` se carga y se ejecuta. Incluye `DummyEvaluator` (siempre
publica `OK`) como plugin de referencia, siguiendo el mismo patrón que `DummyController`/
`DummyPlanner`: sirve de ejemplo y también de plugin real y cargable para los propios tests de
`RecoveryManagerNode` (`configure_loads_multiple_evaluators`, `cycle_runs_loaded_evaluators`),
sin depender de un evaluador de dominio concreto.

**Wiring en `SystemNode`.** `recovery_node_` se añade como sexto subsistema: mismo patrón de
`SystemNodeInfo`/`get_system_nodes()` que el resto (propagación de lifecycle automática), y su
`cycle(nav_state_)` se llama al final de `system_cycle()` (ciclo no-RT), después de que
sensores, localizador, mapas, `GoalManager` y planner hayan escrito lo suyo — así los
evaluadores de ese ciclo ven el estado más reciente posible.

**`NoPathEvaluator` (primer evaluador real, `easynav_plugins/recovery_evaluators/`).**
Deliberadamente genérico (§5.9 del diseño: no necesita saber nada de un planner concreto, solo
de la clave `"path"` que cualquier `PlannerMethodBase` produce): `WARN` si `"path"` no existe
todavía, `ERROR` si existe pero está vacío, `OK` en otro caso. Usa `nav_state.get<...>("path")`
en vez de `get_safe<...>()` deliberadamente: el planner y `RecoveryManagerNode` corren en el
mismo hilo no-RT de `SystemNode`, así que no hay cruce de hilos que justifique pagar la copia
que exige `get_safe()` — se documenta en el propio código por qué, para que quede claro que es
una decisión informada y no un descuido de la guía de uso de `NavState`.

**Verificación.**

```
pixi run -e rolling build --packages-above easynav_core   # 31 paquetes (2 nuevos), todos compilan
pixi run -e rolling colcon test --packages-select \
  easynav_common easynav_core easynav_recovery easynav_system \
  easynav_no_path_evaluator easynav_controller
pixi run -e rolling colcon test-result --all
```

0 fallos nuevos (los mismos fallos preexistentes de `robotnik_sensors`, no tocado). Tests
añadidos: 3 en `navstate_tests.cpp` (`get_group_keys`), 8 en `core_method_test.cpp`
(`RecoveryEvaluatorBase`: rate-gating, no propaga excepciones, publica/sobrescribe/agrupa
diagnósticos), 5 en `easynav_recovery` (`RecoveryManagerNode`: nombre, configurar sin
evaluadores, configurar con varios, fallo con plugin inexistente, `cycle()` ejecuta lo
cargado), 3 en `easynav_no_path_evaluator` (los tres niveles de severidad). Todos los linters
(uncrustify, cppcheck, lint_cmake, xmllint) pasan en los paquetes nuevos y modificados.

### Sesión 5 — printer de `DiagnosticStatus` para el TUI/`easynav_navstate`

Detalle señalado tras la Fase 2: sin un printer registrado, `diagnostic_msgs::msg::DiagnosticStatus`
caía en el formato genérico de `NavState::debug_string()` (puntero + hash de tipo), ilegible en
el tópico `easynav_navstate` o en el TUI de EasyNav. Se registra un printer en el constructor de
`RecoveryManagerNode` — mismo patrón que `ControllerNode` usa para `TwistStamped` y `SystemNode`
para `Goals`: el nodo dueño del concepto registra el printer para ese tipo. Formato:
`NIVEL [nombre] (hardware_id): mensaje {clave=valor, ...}` (nivel textual —OK/WARN/ERROR/
STALE—, `hardware_id` y `values` solo si no están vacíos). Se añadió
`diagnostic_msgs` como dependencia explícita de `easynav_recovery` (antes solo transitiva vía
`easynav_core`) y un test (`DiagnosticStatusIsHumanReadableInDebugString`) que comprueba que el
nivel, el nombre, el `hardware_id` y el mensaje aparecen en `debug_string()`. Build y tests
verificados de nuevo con `--packages-above easynav_core` (31 paquetes) sin fallos nuevos.

### Próximos pasos dentro de la Fase 2

- **`CycleOverrunEvaluator` y `BlackboardFaultEvaluator`.** Quedan pendientes, no por
  dificultad sino por alcance de sesión. `BlackboardFaultEvaluator` en particular no es solo
  "un evaluador más": para que tenga algo que leer, los `catch` de la Fase 0 (que hoy solo hacen
  `RCLCPP_ERROR_THROTTLE` y siguen) tendrían que además dejar constancia en `NavState` (p. ej.
  `nav_state.set("<plugin_name>.last_exception", info)`) de que una excepción ocurrió, con
  cuándo y qué decía — ahora mismo esa información solo va al log. Es un cambio pequeño pero
  toca de nuevo los cuatro `*MethodBase` de la Fase 0, así que se ha preferido dejarlo como su
  propia tarea explícita en vez de mezclarlo con la introducción de `RecoveryEvaluatorBase`.
- **`GoalManager::set_failed/set_error`.** Sigue pendiente de la Fase 0. Con
  `RecoveryManagerNode` ya en marcha, el candidato natural es que un evaluador (o el propio
  `RecoveryManagerNode`) observe un diagnóstico `ERROR` persistente y sea quien llame a
  `GoalManager::set_failed(...)` — pero eso ya empieza a ser una decisión de *mitigación*
  (Fase 3: `RecoveryMitigationBase`), no solo de evaluación, así que se deja explícitamente
  para entonces en vez de adelantarla a medias aquí.

---

## Fase 3 — Mitigación y arbitraje de control

Objetivo (recoveries_easynav.md §5.12, punto 4): `RecoveryMitigationBase`, clave
`control_owner`, hook `cycle_rt()` en `RecoveryManagerNode`, y un primer mitigador de
movimiento con la tabla de prioridad por parámetros. Esta fase se disparó directamente por una
petición concreta: implementar `SafeRetreatRecovery` (el mitigador de "alejarse de un obstáculo
hasta una distancia segura" de §5.13), lo que obligó a construir primero toda la maquinaria de
la que dependía y que aún no existía.

| Tarea | Estado |
|---|---|
| `RecoveryMitigationBase` en `easynav_core`, con fallo-seguro ante excepción | ✅ Hecho |
| `compute_nearest_obstacle()` (utilidad de proximidad estática, independiente de `CollisionChecker`) | ✅ Hecho |
| Arbitraje en `RecoveryManagerNode` (selección, `cycle()`/`cycle_rt()` separados) | ✅ Hecho (versión simplificada, sin tabla de prioridad/cooldown — ver más abajo) |
| Clave `control_owner` y enrutado en `SystemNode::system_cycle_rt()` | ✅ Hecho |
| `ObstacleTooCloseEvaluator` (evaluador disparador, condición compuesta) | ✅ Hecho — *debounce* de la condición "parado" añadido en la Sesión 10, ver más abajo |
| `SafeRetreatRecovery` (el mitigador pedido) | ✅ Hecho (simplificado a "retroceder en línea recta", ver más abajo) |
| Tabla de prioridad/reintento/cooldown por código de diagnóstico (§5.6 completo) | 🟨 Parcial — orden de prioridad implementado en la Fase 5, Sesión 17; sigue sin *cooldown* real |

### Sesión 6 — de cero a `SafeRetreatRecovery` funcionando

**`RecoveryMitigationBase`** (`easynav_core`). Mismo patrón exacto que `SafetyReflexBase` y
`RecoveryEvaluatorBase`: `internal_start()`/`internal_cycle()`/`internal_stop()` envuelven
`on_start()`/`on_cycle()`/`on_stop()` en `try/catch`; una excepción en `on_cycle()` se trata
como fallo-seguro (`stop_robot()` + `RecoveryStatus::FAILED`), no como "sigue corriendo". Un
mitigador declara `can_handle(diagnostic)` (qué diagnósticos sabe atender) y
`requires_control()` (si necesita `control_owner` para mover el robot).

**`compute_nearest_obstacle()`** (`easynav_core/ObstacleProximity.hpp/.cpp`, nuevo). Antes de
escribir `ObstacleTooCloseEvaluator`/`SafeRetreatRecovery` hizo falta decidir cómo consultar
"a qué distancia está el obstáculo más cercano" sin acoplarlos a `CollisionChecker`, que
resuelve una pregunta distinta (¿la velocidad *comandada* llevaría a colisión?, no "¿qué tan
cerca está algo ahora mismo, en reposo?"). Se añadió una función libre, independiente de
`CollisionChecker` y sin necesitar inicialización con parámetros, que devuelve distancia y
*bearing* (ángulo) del punto más cercano en el frame del robot — reutilizable tanto por el
evaluador como por el mitigador. Tiene sus propios tests (`obstacle_proximity_tests.cpp`).

**Arbitraje en `RecoveryManagerNode`.** Se añadió `mitigation_types` (mismo patrón de carga que
`evaluator_types`, sin límite de uno). `cycle()` (no-RT) hace, en orden: evaluadores →
si hay una mitigación activa que no requiere control, se cicla aquí → **solo si no había
ninguna activa al empezar este ciclo**, se intenta seleccionar una nueva. `cycle_rt()` (nuevo)
cicla la mitigación activa *solo* si `requires_control() == true`; al terminar
(`SUCCEEDED`/`FAILED`), la para y devuelve `control_owner` a `"controller"`.

**Un bug real de flapping, encontrado por los propios tests.** La primera versión de `cycle()`
intentaba seleccionar una mitigación nueva en el *mismo* ciclo en el que otra acababa de
resolverse. Como nada garantiza que el diagnóstico que la disparó ya esté en `OK` en ese
instante (el evaluador que lo escribió no ha vuelto a ejecutarse todavía), esto re-seleccionaba
la misma mitigación una y otra vez dentro de la misma llamada — exactamente el problema de
*flapping* que la tabla de reintento/cooldown de §5.6 está pensada para evitar, solo que este
caso concreto ni siquiera necesitaba esa tabla: bastaba con no re-seleccionar en el mismo ciclo
en que algo acaba de pararse, dejando que el evaluador tenga su propio ciclo para actualizar el
diagnóstico antes de que se reconsidere. Corregido y verificado con un test que reproduce
exactamente esa secuencia (`cycle_selects_and_resolves_non_control_mitigation`).

**`control_owner` en `SystemNode`.** `system_cycle_rt()` lee `control_owner` de `NavState`
(por defecto, si no existe, `"controller"`) y decide entre `controller_node_->cycle_rt(...)` y
`recovery_node_->cycle_rt(...)` — un cambio de cinco líneas, pero es exactamente el punto que
todo el diseño de la Fase 1/2 preparaba: el reflejo de nivel 0 (justo debajo, sin cambios) sigue
vigilando el `cmd_vel` resultante sea quien sea quien lo haya escrito.

**`ObstacleTooCloseEvaluator`** (`easynav_plugins/recovery_evaluators/`). Implementa la
condición compuesta discutida en el diseño (§5.2/§5.13): solo publica `ERROR` si el robot está
**parado** (velocidad lineal y angular medidas, de `robot_pose`, por debajo de un umbral) *y*
el obstáculo más cercano está por debajo de `safe_distance`. La condición de "parado" se
verifica con una medición física independiente (la odometría), no preguntándole al reflejo si
se disparó — así funciona correctamente sea cual sea la razón por la que el robot está parado.
Verificado explícitamente con un test que reproduce el caso que motivó esta regla:
`OkWhileStillMovingEvenIfObstacleIsClose`.

**`SafeRetreatRecovery`** (`easynav_plugins/recovery_mitigations/`), el plugin pedido.
`can_handle()` casa por convención de cadena (`hardware_id == "obstacle_proximity"`), sin
dependencia de compilación entre los dos paquetes — evaluador y mitigador solo se conocen a
través del vocabulario de `DiagnosticStatus`. `on_cycle()` recalcula la distancia cada ciclo con
`compute_nearest_obstacle()` y:

- Si ya no hay percepción o la distancia alcanzó `safe_distance` → para el robot y `SUCCEEDED`.
- Si el obstáculo más cercano está detrás del robot (`|bearing| > 90°`) → **no** retrocede hacia
  él; para el robot y devuelve `FAILED`. Esta comprobación de seguridad no estaba explícita en
  el diseño original y surgió al implementar: retroceder en línea recta solo es seguro si el
  obstáculo está delante.
- En otro caso, comanda `linear.x = -retreat_speed` y devuelve `RUNNING`.

**Simplificación consciente frente al diseño**: `recoveries_easynav.md` hablaba de "alejarse
del lado del obstáculo" en un sentido más general; esta implementación solo retrocede en línea
recta. Es una simplificación deliberada, no un descuido: coincide con cómo Nav2 implementa su
propio `BackUp` (también solo-reversa), y los robots que configuran este workspace
(TurtleBot2/Kobuki, ver los YAML de `easynav_indoor_testcase`) son de tracción diferencial —no
pueden desplazarse lateralmente hacia una dirección de escape arbitraria aunque quisiéramos.

**Cómo probarlo en un robot/simulación real.** Con `CollisionSafetyReflex` ya configurado
(Fase 1), basta añadir al `system_node`:

```yaml
system_node:
  ros__parameters:
    safety_reflex_types: [collision]
    collision:
      plugin: easynav_controller/CollisionSafetyReflex
    evaluator_types: [obstacle_close]
    obstacle_close:
      plugin: easynav_obstacle_too_close_evaluator/ObstacleTooCloseEvaluator
    mitigation_types: [retreat]
    retreat:
      plugin: easynav_safe_retreat_recovery/SafeRetreatRecovery
```

**Verificación.**

```
pixi run -e rolling build --packages-above easynav_core   # 33 paquetes (2 nuevos), todos compilan
pixi run -e rolling colcon test --packages-select \
  easynav_common easynav_core easynav_recovery easynav_system \
  easynav_no_path_evaluator easynav_obstacle_too_close_evaluator \
  easynav_safe_retreat_recovery easynav_controller
pixi run -e rolling colcon test-result --all
```

0 fallos nuevos (mismos fallos preexistentes de `robotnik_sensors`, no tocado). Tests añadidos:
2 en `core_method_test.cpp` (ciclo de vida de `RecoveryMitigationBase`, fallo-seguro ante
excepción), 3 en `obstacle_proximity_tests.cpp`, 4 nuevos en `recovery_manager_node_tests.cpp`
(carga de mitigadores, selección/resolución no-RT, `cycle_rt` con `control_owner`, `cycle_rt`
sin mitigación activa), 4 en `obstacle_too_close_evaluator_tests.cpp` (incluida la condición
compuesta), 6 en `safe_retreat_recovery_tests.cpp` (incluido el caso "obstáculo detrás"). Todos
los linters pasan en los paquetes nuevos y modificados.

### Sesión 10 — el hueco del *debounce* en `ObstacleTooCloseEvaluator`

**Cómo se encontró.** Al repasar §5.13 (el ejemplo de principio a fin) para comprobar si ya se
podía reproducir completo con el código actual, el propio texto de §5.2 señala una condición que
`ObstacleTooCloseEvaluator` no implementaba: la condición "parado" debe sostenerse durante una
pequeña ventana de *debounce* (150-300 ms) antes de confiar en ella — si no, como el ciclo RT y
el no-RT corren en paralelo, una sola muestra de velocidad baja tomada en mitad de una frenada
podría hacer que `SafeRetreatRecovery` tomara el control **antes** de que el robot hubiera
terminado de parar, sustituyendo un perfil de frenada seguro por un movimiento a destiempo. A
diferencia de la simplificación de no consultar el reflejo (Sesión 6, deliberada y ya anotada),
esta ausencia del *debounce* no estaba anotada en ningún sitio — un hueco real, no una
simplificación consciente.

**Cambio aplicado** (`easynav_plugins/recovery_evaluators/easynav_obstacle_too_close_evaluator`).
Nuevo parámetro `<instancia>.debounce_duration` (por defecto 0.2 s). El evaluador guarda el
instante (`rclcpp::Time`) desde el que el robot lleva parado de forma continua
(`stopped_since_`), reiniciado en cuanto la velocidad vuelve a superar el umbral o
`"robot_pose"` deja de existir. Mientras el tiempo transcurrido desde ese instante no alcance
`debounce_duration`, publica `OK` (informativo — ningún `can_handle()` mitigador casa con `OK`,
así que no hace falta un tercer nivel de severidad para esto); solo una vez cumplida la ventana
se evalúa de verdad la distancia al obstáculo.

**Verificación.**

```
pixi run -e rolling build --packages-select easynav_obstacle_too_close_evaluator
pixi run -e rolling colcon test --packages-select easynav_obstacle_too_close_evaluator
pixi run -e rolling build --packages-above easynav_core   # 37 paquetes, todos compilan
pixi run -e rolling colcon test --packages-select <los mismos 17 de siempre> --parallel-workers 1
pixi run -e rolling colcon test-result --all
```

0 fallos nuevos (mismos 28 fallos preexistentes de `robotnik_sensors`). El test existente
`ErrorWhenStoppedTooCloseToAnObstacle` se ajustó para forzar `debounce_duration: 0.0` (un
único ciclo ya cuenta como "sostenido") y así seguir verificando el caso base sin ventana. 3
tests nuevos: `RemainsOkWithinDebounceWindowEvenIfObstacleIsClose` (una sola muestra recién
parado no dispara `ERROR`), `ErrorOnceDebounceWindowElapses` (con *debounce* corto, sí dispara
tras esperarlo), `DebounceResetsIfRobotMovesAgain` (moverse a medio camino de la ventana reinicia
el contador, no lo conserva). Linters limpios. (Nota: al correr los 17 paquetes juntos con el
planificador por defecto de `colcon test` aparecieron un par de fallos de contención de
recursos —no deterministas, cambiaban de paquete entre ejecuciones—; con `--parallel-workers 1`
desaparecen. No relacionado con este cambio.)

Con esto, el escenario de §5.13 ya se reproduce íntegro con el código actual (salvo la tabla de
prioridad de §5.6 y los `values`/`hardware_id` exactos del ejemplo del diagnóstico del reflejo,
ambos ya anotados como pendientes).

### Próximos pasos dentro de la Fase 3

- **Tabla de prioridad/reintento/cooldown (§5.6 completo).** La selección actual es "primer
  mitigador que acepte el primer diagnóstico no-`OK` que aparezca" — sin orden de prioridad
  configurable por parámetros, sin contador de reintentos por par (diagnóstico, mitigador), y
  sin *cooldown*. Suficiente para demostrar el mecanismo con un único par evaluador/mitigador,
  pero con más de un mitigador candidato para el mismo código, o con un mitigador que falla
  repetidamente, hace falta esa tabla para evitar reintentos indefinidos. **Actualización
  (Fase 4, Sesión 7):** se implementó la parte de *contador de intentos* (sin *cooldown* ni
  orden de prioridad configurable todavía) porque `ForceReplanRecovery`/`ClearMapRecovery`
  hacían el problema real por primera vez — ver esa sección para el detalle (nota: ese mecanismo
  se revirtió en la Sesión 11 junto con el resto de la Fase 4). **Actualización (Fase 5,
  Sesión 17):** el orden de prioridad configurable por parámetros sí se implementó, de forma más
  sencilla que el contador retirado — ver esa sesión. Sigue sin existir *cooldown* real por
  tiempo ni razonamiento que cruce distintos códigos de diagnóstico.
- **Prueba de integración a nivel de `SystemNode`.** Se verificó `RecoveryManagerNode::cycle_rt()`
  de forma aislada y que `SystemNode::system_cycle_rt()` lee `control_owner` correctamente por
  inspección de código, pero falta un test que ejercite el ciclo RT completo de `SystemNode`
  (sensores + localizador + reflejo + mitigador) de extremo a extremo — requiere una fixture de
  TF/percepciones más elaborada que las usadas hasta ahora.
- **Histéresis entre el umbral del evaluador y el del mitigador.** Ambos usan por defecto el
  mismo `safe_distance` (0.6 m); en la práctica convendría que el del mitigador fuera algo más
  exigente (mayor) que el del evaluador, para no resolver la mitigación justo en el punto en
  que el evaluador podría volver a dispararse.

---

## Fase 4 — Mitigación de mapa/planner y de misión

Objetivo (recoveries_easynav.md §5.12, punto 5): `ClearMapRecovery`, `ForceReplanRecovery`,
`SafeWaypointRecovery`, `NotifyAndHoldRecovery`.

**Estado actual (Sesión 11): revertida.** La tabla y el relato de la Sesión 7 se dejan como
registro de lo que se construyó y por qué, pero **ya no reflejan el código actual** — ver la
Sesión 11, al final de esta sección, para qué se retiró, qué se mantuvo y por qué.

| Tarea | Estado (en su momento, Sesión 7) |
|---|---|
| `ForceReplanRecovery` (`easynav_plugins/recovery_mitigations/easynav_force_replan_recovery`) | ✅ Hecho → 🗑️ retirado en la Sesión 11 |
| `ClearMapRecovery` (`.../easynav_clear_map_recovery`), incluido `MapsManagerBase::reset()` | ✅ Hecho → 🗑️ retirado en la Sesión 11 |
| `SafeWaypointRecovery` (`.../easynav_safe_waypoint_recovery`) | ✅ Hecho (desviación respecto al diseño, ver más abajo) → 🗑️ retirado en la Sesión 11 |
| `NotifyAndHoldRecovery` (`.../easynav_notify_and_hold_recovery`) | ✅ Hecho, cerraba el pendiente de la Fase 0 → 🗑️ retirado en la Sesión 11 (pendiente de la Fase 0 reabierto) |
| Contador de intentos por (diagnóstico, mitigador) en `RecoveryManagerNode` | ✅ Hecho (parcial) → 🗑️ revertido en la Sesión 11 (ya no hacía falta) |

### Sesión 7 — de "quién puede tocar qué" a los cuatro mitigadores

**El problema real que disparó esta fase.** A diferencia de `SafeRetreatRecovery` (Fase 3), que
solo necesitaba escribir `cmd_vel`/`control_owner` en `NavState` — algo que un
`RecoveryMitigationBase` ya podía hacer sin ayuda—, los cuatro mitigadores de esta fase
necesitan actuar sobre subsistemas que un mitigador **no tiene forma de alcanzar**: forzar un
replan es un método de `PlannerMethodBase` que solo `PlannerNode` invoca; limpiar el mapa
requeriría un método que `MapsManagerBase` ni siquiera tenía; y reportar el fallo de la misión
es una llamada a `GoalManager`, un objeto que ni `RecoveryManagerNode` ni ninguno de sus plugins
conocen (el mismo hueco que la Fase 0 dejó anotado y nunca se cerró). Antes de escribir un solo
mitigador hizo falta generalizar el patrón que ya resolvía exactamente este problema para
`cmd_vel`: `control_owner`, escrito por `RecoveryManagerNode` y leído/consumido por
`SystemNode`, que es quien sí tiene una referencia real a todos los subsistemas.

**Tres canales nuevos en `NavState`, mismo patrón que `control_owner`, pero de "un solo
disparo" en vez de cesión continua** (un mitigador escribe una petición; `SystemNode` la
consume — la aplica y la vuelve a poner en su valor neutro — en el punto de
`system_cycle()`/`system_cycle_rt()` donde tiene sentido, sin que el mitigador necesite saber
nada de `PlannerNode`, `MapsManagerNode` ni `GoalManager`):

| Clave en `NavState` | Tipo | Escrita por | Consumida por | Efecto |
|---|---|---|---|---|
| `force_replan_requested` | `bool` | `ForceReplanRecovery::on_start()` | `SystemNode::system_cycle()`, justo antes de `planner_node_->cycle(...)` | Se OR-ea con la condición existente "llegó un goal más nuevo" para decidir el `trigger` de `PlannerNode::cycle()` |
| `maps_manager_reset_requested` | `bool` | `ClearMapRecovery::on_start()` | `SystemNode::system_cycle()`, justo después de `maps_manager_node_->cycle(...)` | Llama a la nueva `MapsManagerNode::reset()` |
| `goal_manager_request` (+ `goal_manager_reason`) | `std::string` | `NotifyAndHoldRecovery::on_start()` (también pensado para que `SafeWaypointRecovery` lo use si algún día necesita `set_failed`) | `SystemNode::system_cycle()`, justo después de `recovery_node_->cycle(...)` | `"failed"`/`"error"` → llama a `goal_manager_->set_failed(reason)`/`set_error(reason)`; se repone a `"none"` |

Cada mitigador que usa uno de los dos canales booleanos reporta `RecoveryStatus::RUNNING`
mientras el flag siga en `true` (todavía no procesado) y `SUCCEEDED` en cuanto lo ve en `false`
de nuevo — un ciclo no-RT de latencia, nunca una garantía de que la acción "funcionó" (por
ejemplo, `ForceReplanRecovery` no comprueba si el nuevo intento produjo un `path` no vacío; para
eso ya está `NoPathEvaluator` en el siguiente ciclo).

**`MapsManagerBase::reset()` (nuevo, `easynav_core`).** No existía ningún hook de "limpiar
datos" — solo `update()`. Se añadió como el resto de hooks de esta familia: virtual, protegido,
con implementación por defecto no-op que devuelve `false` ("no soportado"), envuelto por un
`internal_reset()` público que aplica el mismo `try/catch` de fallo-seguro que `internal_update()`
ya usa. `MapsManagerNode::reset()` lo llama sobre **todos** los `MapsManagerBase` cargados (a
diferencia de `PlannerNode`, que solo admite una instancia, `MapsManagerNode` ya admitía varias)
y devuelve `true` si alguno limpió algo de verdad. Ningún `MapsManagerBase` existente en este
repositorio sobrescribe `reset()` todavía — `ClearMapRecovery` es, por ahora, una petición que
cualquier `MapsManagerBase` puede elegir ignorar sin romper nada; el primer map manager que de
verdad mantenga una ventana/capa recortable es quien debería empezar a sobrescribirlo.

**`ForceReplanRecovery` y `ClearMapRecovery` reaccionan al mismo diagnóstico — simplificación
deliberada, no un descuido.** El catálogo del diseño (§5.11) sugiere un mitigador por síntoma,
pero hoy solo existe un evaluador genérico de "el planner no encuentra camino"
(`NoPathEvaluator`, Fase 2, `hardware_id == "planner"`); no hay todavía un evaluador dedicado a
"el mapa/costmap está sucio" (sería candidato para una Fase futura, en la línea de
`SensorDropoutEvaluator`/`ControllerStuckEvaluator` que el diseño ya lista en §5.5 sin
implementar). En vez de inventar un evaluador solo para justificar un mitigador distinto, ambos
`can_handle()` el mismo `hardware_id == "planner"` a nivel `ERROR`, confiando en el orden de
`mitigation_types` (`force_replan` antes que `clear_map`) y en el contador de intentos descrito
más abajo para que `ClearMapRecovery` solo entre en juego después de que `ForceReplanRecovery`
ya lo haya intentado y el diagnóstico siga sin resolverse.

**Eso obligó a construir, aunque fuera en versión mínima, parte de la tabla de reintento que la
Fase 3 había dejado pendiente (§5.6 punto 4).** Sin ningún mecanismo de reintento,
`RecoveryManagerNode::try_select_mitigation()` habría escogido siempre el primer candidato de la
lista (`force_replan`) para el mismo diagnóstico persistente, sin que `clear_map` llegara a
ejecutarse nunca — los dos mitigadores habrían quedado escritos pero, en la práctica,
inalcanzables. Se añadió un contador `attempt_counts_` (`std::unordered_map<clave_diagnóstico,
std::unordered_map<nombre_mitigador, int>>`) y un parámetro nuevo,
`max_attempts_per_mitigation` (entero, por defecto `1`, a nivel de `RecoveryManagerNode`, no por
mitigador): al escanear los candidatos para un diagnóstico, se salta cualquiera que ya haya
agotado sus intentos para *esa* aparición del diagnóstico, dejando paso al siguiente candidato
de la lista. El contador de un diagnóstico se olvida en cuanto se le observa en `OK` (para que
una futura reaparición vuelva a empezar la escalada desde el primer candidato). **Lo que
deliberadamente sigue faltando** de §5.6 punto 4: orden de prioridad configurable por
parámetros (hoy es solo el orden de `mitigation_types`), *cooldown* real por tiempo, y cualquier
razonamiento que cruce distintos códigos de diagnóstico — sigue siendo, con razón, trabajo
pendiente de la Fase 3/6, solo que ahora con una pieza menos por construir.

**`SafeWaypointRecovery`: desviación deliberada respecto al texto del diseño.**
`recoveries_easynav.md` §5.8 dice literalmente "usa `GoalManagerClient`". Al implementarlo,
`GoalManagerClient` resultó no encajar: su constructor exige un `rclcpp::Node::SharedPtr` (un
`RecoveryMitigationBase` solo tiene acceso, vía `get_node()`, a un
`rclcpp_lifecycle::LifecycleNode::SharedPtr` — tipos no relacionados en `rclcpp`), y su protocolo
de aceptación (`SENT_GOAL`/`SENT_PREEMPT` → `ACCEPTED_AND_NAVIGATING`) está pensado para un
cliente externo que sondea varios ciclos, no para una acción de un solo disparo dentro de
`on_start()`. En vez de forzar ese ajuste, se reutilizó el otro canal que `GoalManager` ya
expone para exactamente este caso de uso — un pose único, inyectado externamente
(`GoalManager::comanded_pose_callback()`, el mismo que usa RViz): `SafeWaypointRecovery` crea un
publicador normal (`create_publisher`, igual que hace el propio `GoalManager` en su
constructor) sobre el topic `"goal_pose"` y publica ahí el `PoseStamped` configurado, dejando
que `GoalManager` se encargue de la preempción del objetivo activo exactamente igual que si
alguien hubiera hecho clic en RViz. No cambia nada del diseño (`recoveries_easynav.md` ya
reconocía en §5.3/§5.9 que los detalles de "cómo" pueden refinarse al implementar) — se anota
aquí en vez de en el documento de diseño porque es un detalle de qué mecanismo de EasyNav se
reutiliza, no una decisión de arquitectura.

Opt-in por diseño propio: `can_handle()` solo devuelve `true` si el parámetro
`<instancia>.enabled` es `true` (por defecto `false`). Sin esto, una instancia sin waypoint
configurado sería un candidato más para cualquier diagnóstico `ERROR`, y si estuviera listada
antes de `NotifyAndHoldRecovery` le robaría el turno sin tener nada útil que hacer; con
`can_handle()` devolviendo `false` cuando no está configurada, `RecoveryManagerNode` la salta
sin más, cayendo directamente al siguiente candidato.

**`NotifyAndHoldRecovery`: corrección sobre la tabla del diseño, no solo una implementación.**
El catálogo (§5.11) no marca este mitigador como "(toma `control_owner`)", a diferencia de
`SpinRecovery`/`SafeRetreatRecovery`/`HumanAssistanceRecovery`. Al implementarlo se vio que eso
no es seguro: pararse una vez escribiendo `cmd_vel = 0` desde el ciclo no-RT no impide que el
controlador nominal (que sigue siendo `control_owner` si nadie se lo quita) sobrescriba
`cmd_vel` en su siguiente ciclo RT persiguiendo el `path` de un objetivo que `GoalManager` ya
dio por fallido — y si el propio controlador lanzara una excepción con ese `path` ya obsoleto,
el `try/catch` fail-*safe* de la Fase 0 dejaría `cmd_vel` sin cambios en vez de a cero, no
garantizando la parada. Por eso `requires_control()` se implementó en `true`: `on_cycle()`
corre en el ciclo RT vía `RecoveryManagerNode::cycle_rt()`, re-afirma `cmd_vel = 0` (con
`stop_robot()`) en cada ciclo, y devuelve `RecoveryStatus::RUNNING` de forma indefinida — de ahí
el "*Hold*" del nombre — hasta que observa que `NavState`/`"goals"` (que `GoalManager::update()`
ya mantiene sincronizado, sin necesitar depender de `easynav_system` para leer el enum
`GoalManager::State`) vuelve a tener un objetivo no vacío, señal de que alguien envió una
misión nueva tras el fallo; solo entonces devuelve `SUCCEEDED` y libera `control_owner`.

`can_handle()` es un catch-all incondicional (`level >= ERROR`), pensado para ser el último de
`mitigation_types`: solo se alcanza para un diagnóstico que ningún candidato anterior aceptó,
que es justo la semántica de "último recurso" de §5.8 sin necesitar ningún seguimiento extra de
"se agotaron los reintentos" — el propio orden de la lista ya lo expresa.

**Verificación.**

```
pixi run -e rolling build --packages-above easynav_core   # 37 paquetes (4 nuevos), todos compilan
pixi run -e rolling colcon test --packages-select \
  easynav_common easynav_core easynav_recovery easynav_system \
  easynav_no_path_evaluator easynav_obstacle_too_close_evaluator \
  easynav_safe_retreat_recovery easynav_controller easynav_maps_manager \
  easynav_force_replan_recovery easynav_clear_map_recovery \
  easynav_safe_waypoint_recovery easynav_notify_and_hold_recovery \
  easynav_regulated_pp_controller easynav_mpc_controller easynav_vff_controller \
  easynav_planner
pixi run -e rolling colcon test-result --all
```

0 fallos nuevos. Tests añadidos: 3 en `recovery_manager_node_tests.cpp`
(`EscalatesToNextMitigationAfterMaxAttempts`, `AttemptCountResetsOnceDiagnosticIsOk`,
`MaxAttemptsPerMitigationParameterIsHonored`), 4 en `force_replan_recovery_tests.cpp`, 3 en
`clear_map_recovery_tests.cpp`, 4 en `safe_waypoint_recovery_tests.cpp` (incluida una prueba de
publicación real sobre `"goal_pose"` con un `SingleThreadedExecutor`), 7 en
`notify_and_hold_recovery_tests.cpp`. Todos los linters pasan en los paquetes nuevos y
modificados.

### Próximos pasos dentro de la Fase 4

- **Evaluador dedicado a salud de mapa/costmap.** Mientras no exista, `ClearMapRecovery` sigue
  compartiendo diagnóstico con `ForceReplanRecovery` (ver más arriba) en vez de tener su propia
  señal de disparo.
- **`SafeWaypointRecovery` no verifica si el waypoint seguro es alcanzable ni si el robot llega
  a él.** Es, deliberadamente, un traspaso de un solo disparo (§5.8): la navegación ordinaria se
  encarga del resto. Si el propio waypoint estuviera bloqueado, el ciclo normal de evaluación
  (`NoPathEvaluator`, etc.) volvería a diagnosticarlo como cualquier otro fallo de navegación.
- **`GoalManagerClient` sigue sin un uso real dentro del propio proceso de `easynav_system`** —
  la desviación de `SafeWaypointRecovery` (ver más arriba) hace que, por ahora, solo lo usen
  clientes externos como `PatrollingNode`. No es un problema a resolver, solo una nota para no
  repetir la misma exploración si una fase futura sí necesita replicar su protocolo de
  aceptación/*feedback* desde dentro del proceso.

### Sesión 11 — Fase 4 revertida: reconsiderando el mecanismo y descartando tres mitigadores

**La decisión, y por qué.** Al releer lo construido en la Fase 4, dos cosas dejaron de encajar:

1. **El mecanismo de comunicación mitigador→subsistema vía `NavState`** (los tres canales de
   "un solo disparo" de la tabla de la Sesión 7: `force_replan_requested`,
   `maps_manager_reset_requested`, `goal_manager_request`/`goal_manager_reason`) no convenció
   como sitio correcto para este tipo de comunicación — a diferencia de `control_owner`
   (Fase 3), que sí modela algo que genuinamente vive en `NavState` (quién es dueño de
   `cmd_vel` ahora mismo, un dato que el propio reflejo de nivel 0 también necesita leer), estos
   tres canales eran, en el fondo, una llamada a un método de otro subsistema disfrazada de
   escritura en el blackboard — sin necesidad real de que ese dato viva en `NavState` ni de que
   nadie más lo lea. Queda como pregunta abierta para más adelante **cómo** debe hacerse esa
   comunicación entre un mitigador y los demás subsistemas; este documento no prejuzga la
   respuesta.
2. **Tres de los cuatro mitigadores dejaron de verse necesarios para este proyecto en concreto**,
   no porque estuvieran mal construidos sino porque las condiciones que los motivaban no
   aplican aquí:
   - `ForceReplanRecovery`: no hace falta forzar un replan porque el sistema ya replanifica
     continuamente (`continuous_replan: true` en la configuración real, ver
     `costmap.rpp.params.yaml`).
   - `ClearMapRecovery`: no hace falta limpiar el mapa porque este proyecto no deja marcados
     obstáculos en el costmap una vez dejan de percibirse — el problema que el mitigador
     resolvía no se da aquí.
   - `SafeWaypointRecovery`: no encaja en el caso de uso de este proyecto.
   - `NotifyAndHoldRecovery` es la excepción: sigue viéndose útil, pero se aborda más adelante,
     no ahora — se retira junto con los otros tres para no dejar a medias un mitigador que
     depende del mismo mecanismo de comunicación que se está reconsiderando (canal
     `goal_manager_request`).

**Qué se retiró.**

- Los cuatro paquetes de mitigadores en `easynav_plugins/recovery_mitigations/`:
  `easynav_force_replan_recovery`, `easynav_clear_map_recovery`,
  `easynav_safe_waypoint_recovery`, `easynav_notify_and_hold_recovery` — eliminados por
  completo (`git rm -r`), comprobado antes por *grep* en todo el workspace que ningún YAML de
  despliegue los referenciaba.
- Los tres canales de `NavState` y su consumo en `SystemNode::system_cycle()`.
- `MapsManagerBase::reset()`/`internal_reset()` y `MapsManagerNode::reset()` (`easynav_core`,
  `easynav_maps_manager`) — sin ningún mitigador que los invoque, quedaban sin llamador.
- El contador de intentos (`attempt_counts_`, parámetro `max_attempts_per_mitigation`) de
  `RecoveryManagerNode` (Sesión 7) — existía únicamente para que `ClearMapRecovery` pudiera
  entrar en juego después de que `ForceReplanRecovery` agotara sus intentos sobre el mismo
  diagnóstico; sin esos dos mitigadores, dos candidatos compitiendo por el mismo diagnóstico ya
  no es un caso real, así que la pieza construida para resolverlo se retira con ellos en vez de
  dejarla sin motivo.

En la práctica, estos ocho ficheros de `EasyNavigation` se revirtieron exactamente a como
estaban al cierre de la Fase 3 (`git checkout 56adcb9 --`, comprobado con `git diff` vacío
contra ese commit): `easynav_core/{include,src}/easynav_core/MapsManagerBase.{hpp,cpp}`,
`easynav_maps_manager/{include,src}/easynav_maps_manager/MapsManagerNode.{hpp,cpp}`,
`easynav_recovery/{include,src}/easynav_recovery/RecoveryManagerNode.{hpp,cpp}`,
`easynav_recovery/tests/recovery_manager_node_tests.cpp` y
`easynav_system/src/easynav_system/SystemNode.cpp`.

**Qué queda de la Fase 3/4.** Solo `SafeRetreatRecovery` (Fase 3) — el único mitigador que no
necesitaba ninguno de los canales retirados, porque su comunicación con el resto del sistema es
exactamente `control_owner`/`cmd_vel`, que sigue existiendo sin cambios. `ObstacleTooCloseEvaluator`
→ `SafeRetreatRecovery` sigue siendo, por tanto, el único par evaluador+mitigador completo del
sistema. El pendiente de la Fase 0 (`GoalManager::set_failed/set_error` conectado a un fallo
real) vuelve a estar abierto — ver la nota en esa sección.

**Verificación tras la reversión.**

```
pixi run -e rolling build --packages-above easynav_core   # 33 paquetes (4 menos), todos compilan
pixi run -e rolling colcon test --packages-select \
  easynav_common easynav_core easynav_recovery easynav_system \
  easynav_no_path_evaluator easynav_obstacle_too_close_evaluator \
  easynav_safe_retreat_recovery easynav_controller easynav_maps_manager \
  easynav_regulated_pp_controller easynav_mpc_controller easynav_vff_controller \
  easynav_planner --parallel-workers 1
pixi run -e rolling colcon test-result --all
```

0 fallos nuevos (mismos 28 fallos preexistentes de `robotnik_sensors`). También se limpiaron los
directorios `build/`/`install/` obsoletos de los cuatro paquetes retirados, para que no queden
registrados en el índice de `ament` como si siguieran disponibles.

---

## Fase 5 — Recuperación especializada y pulido

Objetivo (recoveries_easynav.md §5.12, punto 6): recuperación específica de componente,
co-localizada con el plugin que la origina — el patrón que §5.9 ya describía en detalle con un
ejemplo hipotético (`AmclConvergenceEvaluator`/`AmclRelocalizeMitigation` en un imaginario
`easynav_amcl_localizer`) pero que hasta ahora no tenía ninguna instancia real en el código.

| Tarea | Estado |
|---|---|
| `AmclConvergenceEvaluator`/`AmclRelocalizeMitigation` en `easynav_costmap_localizer` | ✅ Hecho |
| Exclusión mínima de mitigadores fallidos en `RecoveryManagerNode` | ✅ Hecho |

### Sesión 14 — la primera recuperación especializada de verdad: pérdida de localización en AMCL

**El pedido.** El usuario quería un recovery para cuando `AMCLLocalizer` se deslocaliza
(partículas dispersas → covarianza por encima de un umbral): pararse y girar lentamente hasta
relocalizarse, y si pasados unos segundos configurables no se ha resuelto, abandonar y escalar.
Preguntaba explícitamente si esto obligaba a replantear el diseño — no fue así: es, literalmente,
el ejemplo insignia de §5.9, solo que llevado del paquete hipotético (`easynav_amcl_localizer`) al
real (`easynav_costmap_localizer`, clase `AMCLLocalizer`).

**La señal.** `AMCLLocalizer::get_pose()` (ya existente) calculaba `computeCovariance()`/
`computeYawVariance()` sobre las partículas para rellenar `Odometry.pose.covariance`, pero no
exponía nada de eso como una señal reutilizable. Se envuelven las dos llamadas existentes a
`get_pose()` (en `update_rt()` y en `update()`) para, además de `nav_state.set("robot_pose",
...)`, escribir `nav_state.set("localizer.amcl.covariance_trace", var_x + var_y + var_yaw)` —
clave fija, no parametrizada por instancia, exactamente igual que `"robot_pose"`: el evaluador
que la lee vive en el mismo paquete y la conoce por convención acordada entre ambos, no por
configuración.

**`AmclConvergenceEvaluator`.** `RecoveryEvaluatorBase` mínimo: si `covariance_trace` supera
`covariance_threshold` (parámetro, valor de partida sin calibrar en robot real), `ERROR` con
`hardware_id = "localizer.amcl"`; si no, `OK`. Usa `get_safe<double>(...)`, no `get()`: la clave
se escribe también desde `update_rt()` (hilo RT), y este evaluador corre en el hilo no-RT de
`RecoveryManagerNode` — mismo cruce de hilos que ya documenta `ObstacleTooCloseEvaluator` para
`"robot_pose"`.

**`AmclRelocalizeMitigation`.** `RecoveryMitigationBase` con `requires_control() == true`:
mientras `covariance_trace` siga por encima de `covariance_threshold`, comanda un giro in situ
lento (`rotation_speed`); en cuanto baja del umbral, para y `SUCCEEDED`; si pasa `timeout`
segundos (5 s por defecto) sin resolverse, para y `FAILED`. Mismo umbral por defecto que el
evaluador — la misma simplificación de histéresis ya anotada como pendiente para
`ObstacleTooCloseEvaluator`/`SafeRetreatRecovery` en la Fase 3, no un descuido nuevo.

**El hueco real que esto sacó a la luz: nada impedía reintentar en bucle.** Al diseñar el
"escalar de alguna manera" del pedido original, se confirmó que `RecoveryManagerNode`, tras la
reversión de la Fase 4 (Sesión 11), no tenía ningún mecanismo que evitara reseleccionar
inmediatamente el mismo mitigador que acababa de fallar: si `AmclRelocalizeMitigation` agotaba su
`timeout` y devolvía `FAILED`, y el diagnóstico seguía en `ERROR` en el ciclo siguiente,
`try_select_mitigation()` lo volvía a seleccionar sin más, en un bucle apretado de "gira 5s, para
un instante, gira 5s otra vez". El usuario, tras comparar dos alternativas (autocontenerlo dentro
del propio mitigador, o una exclusión mínima en `RecoveryManagerNode`), eligió la segunda —
además cumple literalmente lo que el propio comentario de `RecoveryStatus::FAILED` ya prometía
("RecoveryManagerNode may try the next applicable mitigator") sin que el código lo implementara.

**Exclusión mínima en `RecoveryManagerNode`.** Nuevo `active_diagnostic_key_` (qué clave de
`"diagnostics"` disparó la mitigación activa — antes no se guardaba) y
`excluded_mitigations_` (`unordered_map<clave_diagnóstico, unordered_set<nombre_mitigador>>`).
Cuando una mitigación termina en `FAILED` (en `cycle()` o en `cycle_rt()`), su nombre se añade a
`excluded_mitigations_[active_diagnostic_key_]`; `try_select_mitigation()` salta cualquier
candidato ya excluido para la clave actual. En `SUCCEEDED` no se excluye nada. En cuanto
`try_select_mitigation()` observa una clave en `OK`, borra su entrada de `excluded_mitigations_`
por completo — si el mismo problema reaparece más tarde, el mitigador excluido vuelve a ser
candidato desde cero, no queda vetado para siempre. Deliberadamente más simple que el contador de
intentos (`max_attempts_per_mitigation`) retirado en la Sesión 11: aquí basta un único fallo para
excluir, sin reintentos configurables — coincide exactamente con la semántica que el propio
`RecoveryStatus::FAILED` ya documentaba ("gave up").

Para poder probarlo, `DummyMitigation` (plugin de referencia de `easynav_recovery`) gana un
parámetro nuevo, `<instancia>.should_fail` (por defecto `false`), que hace que `on_cycle()`
devuelva `FAILED` en vez de `SUCCEEDED` — necesario para poder construir en test el escenario
"un mitigador falla, el siguiente candidato toma el relevo".

**Límite conocido, documentado, no resuelto ahora.** Al no existir ya ningún mitigador de último
recurso (`NotifyAndHoldRecovery` se retiró en la Sesión 11), en cuanto se excluye
`AmclRelocalizeMitigation` no queda ningún candidato para `"localizer.amcl"` — `control_owner`
vuelve a `"controller"` y el controlador nominal recupera el mando pese a la mala localización.
El diagnóstico `ERROR` sigue visible (topic `easynav_navstate`/TUI) para que un humano lo note, y
`CollisionSafetyReflex` (nivel 0, independiente de la localización global) sigue protegiendo
contra colisiones inminentes por percepción local mientras tanto. Cerrar esto de verdad —
escalado real a nivel de misión— es, otra vez, el trabajo pendiente de
`NotifyAndHoldRecovery`/`GoalManager::set_failed` ya anotado como futuro.

**Configuración real.** Añadido a `easynav_indoor_testcase/robots_params/costmap.rpp.params.yaml`:
`amcl_convergence` a `evaluator_types` y `amcl_relocalize` a `mitigation_types` de `system_node`,
con los parámetros de partida (`covariance_threshold: 1.0`, `rotation_speed: 0.3`, `timeout: 5.0`)
— todos a calibrar en el robot real.

**Verificación.**

```
pixi run -e rolling build --packages-select easynav_costmap_localizer easynav_recovery
pixi run -e rolling colcon test --packages-select easynav_costmap_localizer easynav_recovery
pixi run -e rolling build --packages-above easynav_core   # 33 paquetes, todos compilan
pixi run -e rolling colcon test --packages-select <los mismos 15 de siempre> --parallel-workers 1
pixi run -e rolling colcon test-result --all
```

0 fallos nuevos (mismos 28 fallos preexistentes de `robotnik_sensors`). Tests añadidos: 3 en
`amcl_convergence_evaluator_tests.cpp`, 5 en `amcl_relocalize_mitigation_tests.cpp`, 2 en
`recovery_manager_node_tests.cpp` (`failed_mitigation_is_excluded_and_next_candidate_takes_over`,
`exclusion_is_forgotten_once_diagnostic_is_ok_again`). Linters limpios.

### Sesión 15 — probándolo de verdad: el `recovery_node` nunca se estaba configurando

**El síntoma.** Al mirar el `NavState` real en la TUI tras lanzar `easynav_costmap_rpp.launch.py`
con la config de la Sesión 14, solo aparecía `diagnostics.collision` (el reflejo) — ni
`diagnostics.no_path`, ni `diagnostics.obstacle_close`, ni `diagnostics.amcl_convergence`, pese a
que `localizer.amcl.covariance_trace` sí estaba presente y con un valor coherente (confirmando
que el código nuevo de `AMCLLocalizer` sí corría).

**La causa.** `RecoveryManagerNode` es un nodo ROS propio, llamado `"recovery_node"` (no
`"system_node"`) — `evaluator_types`/`mitigation_types` y sus bloques de parámetros los declara
sobre sí mismo. Llevaban todo este tiempo anidados dentro de `system_node:` en
`costmap.rpp.params.yaml` (error introducido varias sesiones atrás, cuando se añadió por primera
vez `evaluator_types`), así que `recovery_node` se configuraba con las listas vacías por defecto
— sin ningún error, `on_configure()` devuelve `SUCCESS` igualmente con cero evaluadores/
mitigadores cargados — y por tanto nunca ejecutaba nada. `safety_reflex_types`/`collision` sí
funcionaban porque esos los declara `SystemNode` sobre sí mismo, y ahí sí estaban bien ubicados.

**Corrección.** Nueva clave de nivel superior `recovery_node:` en el YAML (hermana de
`system_node:`, `controller_node:`, etc.), con `use_sim_time: true` (para que su reloj coincida
con el resto del sistema — importante porque `AmclRelocalizeMitigation` hace aritmética de
`rclcpp::Time` para el *timeout*, y restar tiempos de tipos de reloj distintos lanza excepción) y
todo lo que antes estaba mal anidado. Es un fallo de despliegue, no de código — ningún test
unitario lo detecta porque todos construyen `RecoveryManagerNode` con `NodeOptions` explícitas en
memoria, nunca cargando el YAML real de principio a fin; solo se ve lanzando el sistema completo,
que es exactamente cómo se encontró.

**`debug_string()` ordena las claves alfabéticamente.** De paso, para hacer legible la TUI: `values_`
es un `std::unordered_map`, así que `debug_string()` iteraba en orden de *hash*, no alfabético.
Se recogen las claves en un `std::vector`, se ordenan (`std::sort`) y se itera en ese orden —
cambio puramente de presentación, no afecta a `NavState::get()`/`set()` ni a ningún otro
consumidor.

**Verificación.**

```
pixi run -e rolling build --packages-select easynav_common
pixi run -e rolling colcon test --packages-select easynav_common
pixi run -e rolling build --packages-above easynav_common   # 39 paquetes, todos compilan
pixi run -e rolling colcon test --packages-select <los mismos 15 de la Fase 5> --parallel-workers 1
```

0 fallos nuevos (mismos 28 fallos preexistentes de `robotnik_sensors`). 1 test nuevo,
`DebugStringListsKeysInAlphabeticalOrder` (`easynav_common/tests/navstate_tests.cpp`).

### Sesión 16 — `NoPathEvaluator` no debe diagnosticar nada sin un goal activo

**El síntoma.** Con `recovery_node` ya configurándose de verdad (Sesión 15), el usuario observó
`diagnostics.no_path` en `ERROR` incluso cuando el robot no tenía ninguna misión asignada — un
falso positivo: no hay nada roto en no tener `path` cuando no hay a dónde ir.

**La causa.** `NoPathEvaluator` (Fase 2, Sesión 4) nunca miró si había un objetivo activo — solo
si `"path"` existía y no estaba vacío. Sin goal, el planner no produce (o deja de producir) un
`path` con poses, y eso se leía como `WARN`/`ERROR` exactamente igual que si hubiera un goal y el
planner hubiera fallado en encontrarle camino.

**Corrección.** Se añade una comprobación previa usando `"goals"` (`nav_msgs::msg::Goals`,
mantenido por `GoalManager` en el mismo hilo no-RT que este evaluador — `get()` normal, no
`get_safe()`, mismo razonamiento que ya usa este evaluador para `"path"`): si no hay clave
`"goals"` o su campo `goals` está vacío, se publica `OK` ("no active goal") sin mirar `"path"` en
absoluto. Solo cuando hay al menos un goal activo se aplica la lógica original
(`WARN`/`ERROR`/`OK` según `"path"`). No hace falta ninguna dependencia nueva:
`nav_msgs::msg::Goals` vive en el mismo paquete `nav_msgs` que `nav_msgs::msg::Path`, ya usado
aquí.

**Verificación.**

```
pixi run -e rolling build --packages-select easynav_no_path_evaluator
pixi run -e rolling colcon test --packages-select easynav_no_path_evaluator
pixi run -e rolling build --packages-above easynav_core   # 33 paquetes, todos compilan
pixi run -e rolling colcon test --packages-select <los mismos 15 de la Fase 5> --parallel-workers 1
```

0 fallos nuevos (mismos 28 fallos preexistentes de `robotnik_sensors`). Los 3 tests existentes se
ajustaron para fijar un goal activo (ya que antes no lo hacían y ahora eso importa) y se añadieron
2 nuevos: `OkWithoutAnActiveGoal` (sin clave `"goals"`), `OkWithEmptyGoalsListEvenWithNoPath`
(clave presente pero vacía) — ambos deben dar `OK` pase lo que pase con `"path"`.

### Sesión 17 — prioridad configurable entre mitigadores, y `HumanAssistanceRecovery` genérico

**El pedido, y una aclaración que no era un cambio de diseño.** El usuario quería un mitigador
de último recurso que pidiera ayuda a un humano, y de paso una forma de priorizar mitigadores
cuando varios `can_handle()` el mismo diagnóstico. También pensaba que quería cambiar algo del
diseño: que al pedir ayuda humana y resolverse, la misión debía continuar (no fallar). Revisando
`recoveries_easynav.md` §5.15 (`HumanAssistanceRecovery`), esa semántica **ya estaba
especificada así** — quien fallaba la misión era `NotifyAndHoldRecovery` (§5.11, retirado en la
Sesión 11), un mitigador distinto. No hizo falta tocar el documento de diseño.

**Prioridad: parámetro `<instancia>.priority`, sin tocar `RecoveryMitigationBase`.** Primera
idea descartada: añadirlo como propiedad del propio plugin (método virtual o parámetro que cada
mitigador declara). Se optó por algo más simple — es `RecoveryManagerNode` quien lo declara y
lee por instancia, exactamente donde ya declara `<mitigation_type>.plugin`
(`on_configure()`), porque la prioridad es un detalle de **arbitraje** que pertenece al gestor,
no una propiedad intrínseca del mitigador. Tras cargar todos los mitigadores de
`mitigation_types` se ordenan una vez con `std::stable_sort` (ascendente — menor número, más
prioritario), y `stable_sort` conserva el orden de la lista como desempate: quien no configure
`priority` obtiene exactamente el comportamiento de antes. `try_select_mitigation()` no cambia
ni una línea — ya elegía "el primero no excluido en `mitigations_`"; con la lista pre-ordenada,
ese "primero" ya es el más prioritario. Ningún mitigador existente (`SafeRetreatRecovery`,
`AmclRelocalizeMitigation`) necesitó cambios.

**`HumanAssistanceRecovery` (nuevo paquete, `easynav_plugins/recovery_mitigations/easynav_human_assistance_recovery`)
— deliberadamente genérico, no de AMCL.** Primera versión de este trabajo lo puso, por error,
dentro de `easynav_costmap_localizer` con `hardware_id == "localizer.amcl"` — el usuario lo
corrigió: el catálogo (§5.11) lo define como mitigador general, no ligado a ningún componente.
Versión simplificada de §5.15 (sin modo `teleop`, sin `ack` con id de episodio — el "ack" es
físico: el propio diagnóstico volviendo a `OK`):

- `can_handle()`: `level >= ERROR`, sin mirar `hardware_id` — catch-all incondicional, igual que
  tenía `NotifyAndHoldRecovery` en su día. Se descartó `true` sin más (aceptar también `WARN`):
  hay diagnósticos `WARN` transitorios y normales (p. ej. `NoPathEvaluator` justo después de
  fijar un goal, antes de que el planner corra su primer ciclo) que no ameritan parar el robot y
  avisar a un humano.
- `on_start()` (corre en el hilo no-RT, como toda selección): recorre el grupo `"diagnostics"`
  con `get()` normal y hace `RCLCPP_ERROR` con qué claves siguen en `ERROR` — la señal
  observable mínima que pide §5.15 (ya sale además en `easynav_navstate`/TUI vía esos mismos
  diagnósticos).
- `on_cycle()` (corre en el hilo RT, `requires_control() == true`): repite el escaneo con
  `get_safe<DiagnosticStatus>(...)` (cruce de hilos real, mismo razonamiento que
  `AmclRelocalizeMitigation` para `covariance_trace`); si ya no queda ningún `ERROR` →
  `stop_robot()`, `SUCCEEDED` — **sin tocar `GoalManager`**, la misión nunca falló, solo se
  pausó. Si sigue habiendo alguno → se queda parado (pasivo, no gira ni avanza) y `RUNNING`
  indefinido, sin *timeout* — es el último recurso.

**Configuración real.** En `costmap.rpp.params.yaml`, `mitigation_types: [retreat,
amcl_relocalize, human_assistance]`, con `amcl_relocalize.priority: 10` y
`human_assistance.priority: 1000` — la prioridad alta garantiza que `human_assistance` nunca le
robe el turno a `retreat`/`amcl_relocalize` para sus propios diagnósticos, sea cual sea el orden
en que aparezcan en la lista en el futuro. Escalada resultante para AMCL: reflejo de colisión
(nivel 0) → `amcl_relocalize` (gira, 5 s) → excluido tras fallar (Sesión 14) →
`human_assistance` (genérico, para y espera) → el operador publica una pose correcta en
`"initialpose"` (`AMCLLocalizer` ya la escucha, `init_pose_callback`, sin *plumbing* nuevo) →
`covariance_trace` baja → `human_assistance` resuelve → opera con normalidad.

**Verificación.**

```
pixi run -e rolling build --packages-select easynav_recovery easynav_human_assistance_recovery
pixi run -e rolling colcon test --packages-select easynav_recovery easynav_human_assistance_recovery
pixi run -e rolling build --packages-above easynav_core   # 34 paquetes, todos compilan
pixi run -e rolling colcon test --packages-select <los mismos 16 de la Fase 5> --parallel-workers 1
pixi run -e rolling colcon test-result --all
```

0 fallos nuevos (mismos 28 fallos preexistentes de `robotnik_sensors`). Tests añadidos: 2 en
`recovery_manager_node_tests.cpp` (prioridad gana pese a ir después en la lista; empate cae al
orden de lista), 6 en `human_assistance_recovery_tests.cpp`. Linters limpios.

### Sesión 18 — "sin progreso": `ControllerStuckEvaluator` + `AdvanceRecovery`

**El pedido.** Recuperación para cuando el robot se atora (se le comanda avanzar pero no
avanza): si no hay obstáculo, avanzar una distancia configurada. Ya estaba en el catálogo del
diseño §5.5 (`ControllerStuckEvaluator`, análogo a `IsStuckCondition` de Nav2) desde el
principio, nunca implementado.

**Tres precondiciones, las tres ya resolubles con mecanismos existentes de sesiones
anteriores — ninguna requirió construir nada nuevo en `easynav_core`:**

1. **No disparar con el reflejo de colisión activo.** `SafetyReflexBase` (Sesión 9) ya publica
   `hardware_id = "safety_reflex"` — genérico, no atado a `CollisionSafetyReflex`, así que sirve
   para cualquier reflejo presente o futuro sin que este evaluador necesite conocerlo.
2. **No disparar mientras retrocede o espera a un humano.** Ambos casos (y el propio
   `AdvanceRecovery` mientras está activo) ocupan `control_owner`. La regla ya estaba en el
   diseño, §5.7 punto 5, nunca aplicada hasta ahora: *"todo evaluador que observe `cmd_vel` o el
   comportamiento del controlador debe comprobar primero `control_owner`"* — evita que la propia
   recuperación se autodiagnostique como un fallo nuevo.
3. **Sin goal activo, no hay expectativa de avance** (matiz añadido por el usuario a mitad del
   diseño) — mismo patrón que `NoPathEvaluator` desde la Sesión 16 (`"goals"` no vacío).

**`ControllerStuckEvaluator`** (`easynav_plugins/recovery_evaluators/easynav_controller_stuck_evaluator`).
Compara `robot_pose` contra una posición de referencia; si no se supera
`progress_distance_threshold` durante `stuck_time_threshold` segundos **mientras** `cmd_vel`
comanda una velocidad por encima de `linear_velocity_threshold`, publica `ERROR`,
`hardware_id = "controller_stuck"`, con `values: [{"stuck_duration", segundos}]`. Las tres
condiciones de arriba, si no se cumplen, devuelven `OK` sin tocar la posición de referencia —
queda "congelada" hasta que las condiciones normales vuelven (así, el primer ciclo tras
recuperar `control_owner` compara contra una referencia ya desfasada, y cualquier movimiento
real ocurrido durante la mitigación cuenta como progreso de inmediato).

**`AdvanceRecovery`** (`easynav_plugins/recovery_mitigations/easynav_advance_recovery`).
Comanda un avance recto y lento (`advance_speed`) hasta cubrir `advance_distance` — pasa por
`CollisionSafetyReflex` antes de publicarse, igual que cualquier otro mitigador: es esa puerta
única de nivel 0 la que cubre "si no hay obstáculo", sin duplicar el chequeo aquí (mismo
razonamiento que `SafeRetreatRecovery`).

**El punto explícito del usuario: avanzar no significa "arreglado".** Cada activación, al
completar la distancia, devuelve `SUCCEEDED` — pero deliberadamente **no** como "problema
resuelto": si el atasco persiste, `ControllerStuckEvaluator` simplemente lo vuelve a diagnosticar
en el siguiente ciclo y `AdvanceRecovery` se reactiva, avanzando otro poco. Es intencional que se
dispare repetidamente mientras el problema siga ahí.

**El matiz de diseño que obligó a repensar el escalado.** El propio avance del mitigador mueve
al robot lo suficiente como para que `ControllerStuckEvaluator` lo cuente como "progreso" y
reinicie *su* temporizador — así que ese temporizador del evaluador no sirve para decidir cuándo
escalar (con él, prácticamente nunca acumularía tiempo suficiente). Tras plantear la disyuntiva
al usuario (contador de intentos vs. tiempo total acumulado), se eligió que **el propio
mitigador** acumule el tiempo total transcurrido desde la primera activación de un mismo
"episodio" — independientemente de cuántos avances haya dado — y escale (`FAILED`) al superar
`escalate_after`. Para no dejar ese reloj corriendo para siempre tras un episodio ya resuelto sin
escalar (lo que haría que un atasco *futuro* y sin relación escalase de inmediato), se añadió
`on_stop()` — su primer uso real en el catálogo — para registrar cuándo paró por última vez: un
hueco mayor que `episode_gap` desde entonces hace que el siguiente `on_start()` trate la
activación como un episodio nuevo, reiniciando el reloj.

**Configuración real.** `costmap.rpp.params.yaml`, `evaluator_types` gana `controller_stuck`;
`mitigation_types` gana `advance` (`priority: 10`, igual que `amcl_relocalize` — no compiten por
el mismo `hardware_id`, así que el valor relativo entre ellos es indiferente; solo importa ser
menor que `human_assistance`, 1000). Escalada resultante: `ControllerStuckEvaluator` diagnostica
→ `AdvanceRecovery` avanza repetidamente → si el episodio supera `escalate_after` (15 s) en
total → excluido (Sesión 14) → `human_assistance` (último recurso ya configurado, Sesión 17).

**Verificación.**

```
pixi run -e rolling build --packages-select easynav_controller_stuck_evaluator easynav_advance_recovery
pixi run -e rolling colcon test --packages-select easynav_controller_stuck_evaluator easynav_advance_recovery
pixi run -e rolling build --packages-above easynav_core   # 36 paquetes, todos compilan
pixi run -e rolling colcon test --packages-select <los mismos 18 de la Fase 5> --parallel-workers 1
pixi run -e rolling colcon test-result --all
```

0 fallos nuevos (mismos 28 fallos preexistentes de `robotnik_sensors`). Tests añadidos: 6 en
`controller_stuck_evaluator_tests.cpp` (incluidas las tres condiciones que deben inhibir el
diagnóstico), 7 en `advance_recovery_tests.cpp` (incluidos los dos casos que distinguen
acumulación vs. reinicio del reloj de episodio). Linters limpios.

### Sesión 19 — el último escalón: cancelar la misión, `/diagnostics` real, y un bug de verdad

**Cuatro piezas pedidas juntas.** (1) `timeout` opcional en `HumanAssistanceRecovery`. (2) un
mitigador nuevo que cancele la misión activa informando del error. (3) publicar los
`diagnostic_msgs::msg::DiagnosticStatus` (hasta ahora solo dentro de `NavState`) en un topic
`/diagnostics` real. (4) un cuadro en la TUI con esos diagnósticos, coloreado por criticidad.

**`timeout` en `HumanAssistanceRecovery`.** Parámetro `<instancia>.timeout` (s, por defecto
`0.0` = espera infinita — la convención se documenta explícitamente, no es un valor "mágico").
Mismo patrón que el *timeout* de `AmclRelocalizeMitigation`: si se agota sin que ningún
diagnóstico vuelva a `OK`, `stop_robot()` + `FAILED` — excluye este mitigador (Sesión 14) y deja
paso al siguiente candidato.

**`CancelMissionRecovery` (nuevo paquete `easynav_cancel_mission_recovery`).** Catch-all
idéntico a `HumanAssistanceRecovery` (`level >= ERROR`, cualquier `hardware_id`), pensado como
el escalón final — mayor número de prioridad de todos. A diferencia de todo lo anterior, no
mueve el robot ni necesita `control_owner`: solo escribe `nav_state.set("mission_cancel_requested",
true)` — el mismo patrón de "una sola señal, escrita por un mitigador, consumida y reseteada por
el único subsistema que puede actuar sobre ella" que ya usa `control_owner`, aquí para
`GoalManager`, al que ningún mitigador tiene forma de referenciar directamente (solo
`SystemNode` la tiene). `GoalManager::update()` (`GoalManager.cpp`, al principio del método) lee
la señal, construye el motivo recorriendo el grupo `"diagnostics"` (`get_safe`, por si alguna
entrada la escribió un reflejo desde el hilo RT) concatenando las que estén en `ERROR`, llama a
`set_error(reason)` (ya existente: `state_ = IDLE`, `goals_.goals.clear()`, publica
`NavigationControl::ERROR`), y resetea la señal.

**Aclaración, no un cambio de diseño**: el usuario pensaba que esto contradecía "lo que dijimos
de otra manera" en el diseño — no era así. `recoveries_easynav.md` §5.15
(`HumanAssistanceRecovery`) ya especificaba "control devuelto, la misión sigue" para el caso
humano; era `NotifyAndHoldRecovery` (retirado, Sesión 11) quien trataba su disparo como fallo de
misión. Son dos mitigadores distintos del catálogo (§5.11); `CancelMissionRecovery` reintroduce,
a propósito y de forma acotada, el mecanismo de señalización vía `NavState` que se retiró junto
con el resto de la Fase 4 — no por descuido, sino porque es exactamente la comunicación
mitigador→`GoalManager` que no tiene otro camino.

**El bug real que esto sacó a la luz.** El usuario preguntó, con toda la razón, cómo sabe el
sistema que debe parar el robot con `goals` vacío — y la respuesta reveló un fallo genuino,
preexistente, en `CostmapPlanner` (`easynav_plugins/planners/easynav_costmap_planner/.../CostmapPlanner.cpp:147-150`):

```cpp
if (goals.goals.empty()) {
  nav_state.set("path", current_path_);   // current_path_ nunca se vaciaba aquí
  return;
}
```

`current_path_` es una variable miembro que solo se vacía cuando hay un goal nuevo y se
recalcula (línea 230) — con `goals` vacío, el planner republicaba indefinidamente el **último**
path calculado. El controlador (`RegulatedPurePursuitController.cpp:427`) solo para cuando ve
`path.poses.empty()` — nunca lo veía, así que seguiría intentando llegar a un destino ya
cancelado, con solo `CollisionSafetyReflex` de red de seguridad (evita chocar, no evita
circular). No se notaba en el caso normal (misión completada con éxito) porque ahí el robot ya
está físicamente en el destino del path — la propia lógica de distancia-al-último-punto del
controlador da velocidad ~0 sin que el path se vaciara, por casualidad, no por diseño. Para
`CancelMissionRecovery` sí importa: se dispara precisamente cuando el robot **no** ha llegado a
ningún sitio.

**Corrección** (mínima, en la causa raíz, sin tocar `CancelMissionRecovery` ni darle
`control_owner`): cuando `goals.goals.empty()`, `CostmapPlanner` vacía `current_path_.poses`
antes de publicarlo, en vez de republicar el path viejo. Se revisaron también los otros dos
planners del árbol (`SimplePlanner`, `AStarPlanner` de `easynav_navmap_planner`) — ambos ya
hacían `current_path_.poses.clear()` **incondicional**, la primera línea de `update()`, así que
nunca tuvieron este fallo; es un problema aislado de `CostmapPlanner`, no un patrón repetido.
`easynav_costmap_planner` no tiene infraestructura de tests (`# add_subdirectory(tests)`
comentado en su `CMakeLists.txt` desde siempre) — construir esa infraestructura desde cero
queda fuera de alcance de esta sesión; el fix se verificó por inspección de código y por la
batería de tests del resto del árbol, no con un test dedicado a este paquete.

**Publicación en `/diagnostics`.** `RecoveryManagerNode` (ya escaneaba el grupo `"diagnostics"`
en varios sitios) gana un publicador `diagnostic_msgs::msg::DiagnosticArray` — nombre de topic
**relativo** (`"diagnostics"`, no `"/diagnostics"`; el resto de topics del proyecto, `cmd_vel`,
`easynav_navstate`, etc., también lo son, para respetar el *namespace* del nodo en despliegues
multi-robot como el de `easynav_playground_kobuki`). Se publica al final de `cycle()`, tras
evaluadores y arbitraje (así incluye también la entrada del reflejo, que llega desde el ciclo
RT), y solo si hay algún suscriptor.

**Panel de diagnósticos en la TUI.** `DiagnosticsProcessor` nuevo en
`easynav_tools/easynav_tools/controller/ros_controllers.py` (mismo patrón que
`GoalManagerInfoProcessor`/`NavStateProcessor`), coloreando cada línea con el marcado Rich que
la TUI ya usa (`[green]OK[/green]`, `[yellow]WARN[/yellow]`, `[red]ERROR[/red]`/`STALE`). Nuevo
cuadro "Diagnostics" en `easynav_tools/easynav_tools/tui/app.py`, entre "NavState" y "Time
stats", con su propio interruptor on/off — mismo patrón exacto que esos dos bloques.

**Configuración real.** `costmap.rpp.params.yaml`: `mitigation_types` gana `cancel_mission`
(`priority: 2000`, el mayor de todos); `human_assistance` gana `timeout` (ajustado por el
usuario a `10.0` s). Escalada completa para AMCL: `amcl_relocalize` (gira, 5 s) → excluido →
`human_assistance` (espera a un humano, 10 s) → excluido → `cancel_mission` (cancela la misión,
informando de qué diagnósticos seguían en `ERROR`).

**Verificación.**

```
pixi run -e rolling build --packages-select easynav_human_assistance_recovery easynav_cancel_mission_recovery easynav_recovery easynav_system easynav_costmap_planner
pixi run -e rolling colcon test --packages-select easynav_human_assistance_recovery easynav_cancel_mission_recovery easynav_recovery easynav_system easynav_costmap_planner
pixi run -e rolling build --packages-above easynav_core   # 37 paquetes, todos compilan
pixi run -e rolling colcon test --packages-select <los mismos 20 de la Fase 5> --parallel-workers 1
pixi run -e rolling colcon test-result --all
```

0 fallos nuevos (mismos 28 fallos preexistentes de `robotnik_sensors`). Tests añadidos: 3 en
`human_assistance_recovery_tests.cpp` (sin *timeout* espera indefinidamente; con él, falla tras
agotarlo), 5 en `cancel_mission_recovery_tests.cpp`, 1 en `recovery_manager_node_tests.cpp`
(publicación real en el topic, verificada con un suscriptor y un `SingleThreadedExecutor`). La
TUI se verifica lanzándola (no tiene suite automatizada en este repo). Linters limpios.

### Sesión 20 — un falso positivo real de `ControllerStuckEvaluator` al pausar, y cierre de fases

**La pregunta que destapó el bug.** El usuario preguntó si `GoalManagerClient::pause()` podía
hacer que `ControllerStuckEvaluator` (Sesión 18) diagnosticara "atascado" por error. La
respuesta fue que sí, y era un fallo real, no hipotético — se confirmó rastreando el código, no
solo por inspección superficial:

1. `GoalManager::update()` refleja `paused_` en `nav_state.set("navigation_paused", ...)`
   (`GoalManager.cpp`).
2. Pero **el controlador no se entera de la pausa**: `RegulatedPurePursuitController::update_rt()`
   sigue escribiendo un `"cmd_vel"` no nulo en `NavState` mientras haya un `path`/goal activo,
   ajeno a `navigation_paused` (`RegulatedPurePursuitController.cpp:536-539`).
3. `SystemNode::cycle_rt()` solo pone a cero la copia local que se **publica** de verdad al
   robot (`SystemNode.cpp:296-311`) — nunca reescribe la entrada `"cmd_vel"` de `NavState`.
4. `ControllerStuckEvaluator` lee ese `"cmd_vel"` (no el publicado), ve velocidad comandada por
   encima del umbral, compara contra `"robot_pose"` (que no avanza, porque la actuación real fue
   puesta a cero) → tras `stuck_time_threshold_` segundos de pausa, dispara `ERROR` y puede
   escalar hasta `AdvanceRecovery`, pese a que la pausa fue intencionada.

Se comprobó además que ningún evaluador/mitigador del árbol miraba `navigation_paused` — no era
un descuido aislado de este plugin, pero es el único evaluador realmente afectado, al ser el
único que compara "se manda mover" contra "se mueve de verdad".

**Por qué el patrón ya usado para `control_owner` no basta para `navigation_paused`.** La
comprobación de `control_owner` (Sesión 18) *congela* `reference_position_`/`reference_time_`
confiando en que, si una mitigación mueve el robot de verdad, la posición habrá cambiado al
recuperar el control y el contador se reinicia solo. Con la pausa el robot **no se mueve en
absoluto**, así que congelar el reloj habría dejado `reference_time_` estancado desde antes de
pausar — al reanudar, `stuck_for = now - reference_time_` ya habría incluido todo el tiempo de
pausa, disparando `ERROR` de inmediato, peor que no tratarlo.

**Corrección** (`ControllerStuckEvaluator.cpp`, nueva comprobación, la primera de todas).
Mientras `navigation_paused` sea `true`: se informa `OK` ("navigation paused") y se **reactualiza**
`reference_position_`/`reference_time_` a la pose y el instante actuales en cada ciclo (si
`"robot_pose"` no está disponible, solo se resetea `reference_position_` para que el primer
muestreo lo reinicialice cuando lo esté) — a diferencia de `control_owner`, que congela. Así, al
reanudar, se exige una ventana completa de `stuck_time_threshold_` de no-progreso real, no
arrastrada de antes de pausar. `navigation_paused` se lee con `get<bool>` normal, no
`get_safe()`: lo escribe `GoalManager::update()` en el mismo ciclo no-RT y antes que
`recovery_node_->cycle()` (`SystemNode::system_cycle()`), igual que `control_owner`.

**Verificación.**

```
pixi run -e rolling build --packages-select easynav_controller_stuck_evaluator
pixi run -e rolling colcon test --packages-select easynav_controller_stuck_evaluator
pixi run -e rolling build --packages-above easynav_core   # 37 paquetes, todos compilan
```

2 tests nuevos en `controller_stuck_evaluator_tests.cpp`: `OkWhileNavigationPaused` (permanece
`OK` pasado el `stuck_time_threshold_` mientras dura la pausa) y
`OkImmediatelyAfterResumingFromPause` (no dispara `ERROR` en el primer ciclo tras reanudar,
aunque el robot siga sin moverse). 8/8 tests del paquete en verde.

**Aparte, verificación de que el merge con `rolling` (previo a esta sesión) no rompió nada.**
`pixi run -e rolling build --packages-above easynav_core` (37 paquetes) y
`colcon test --packages-above easynav_core --parallel-workers 1`: 1499 tests, 0 errores, 28
fallos — los mismos 28 preexistentes de `robotnik_sensors` (flake8) de siempre, 0 nuevos.

**Segundo bug real, encontrado probando el fix anterior en real.** El usuario provocó que el
robot se perdiera, esperó la escalada completa (`amcl_relocalize` → `human_assistance` →
`cancel_mission`), vio que la misión se cancelaba correctamente informando del error — y a partir
de ahí la terminal se llenó, sin parar, de:

```
[recovery_node]: Selecting mitigation [cancel_mission] for diagnostic [diagnostics.amcl_convergence]
[recovery_node]: CancelMissionRecovery [cancel_mission]: nothing else resolved this — cancelling the active mission
```

repetido en cada ciclo no-RT. Causa: `CancelMissionRecovery::on_cycle()` devolvía
`RecoveryStatus::SUCCEEDED` en cuanto `GoalManager` consumía la señal — pero
`RecoveryManagerNode::try_select_mitigation()` (`easynav_recovery/src/.../RecoveryManagerNode.cpp:342-350`)
solo añade a `excluded_mitigations_` (Sesión 14) cuando el resultado es `FAILED`, nunca cuando es
`SUCCEEDED`. Cancelar la misión no arregla el diagnóstico que la disparó (`amcl_convergence`
sigue divergido — cancelar la misión no relocaliza al robot), así que ese diagnóstico nunca
vuelve a `OK`, y al no estar excluido, `cancel_mission` volvía a ser el candidato elegido en el
siguiente ciclo, indefinidamente: repetía `on_start()` (con su `RCLCPP_ERROR` incluido) y
`on_cycle()` en cada ciclo, para siempre.

**Por qué es un bug de `CancelMissionRecovery`, no de `RecoveryManagerNode`.** El propio
comentario de `RecoveryStatus::SUCCEEDED` en `RecoveryMitigationBase.hpp` es explícito: "the
diagnostic that triggered this mitigation is considered resolved". `CancelMissionRecovery` no
cumple ese contrato — nunca resuelve el diagnóstico, solo contiene el daño cancelando la misión.
El resto de mitigadores existentes sí lo cumplen (cuando `SafeRetreatRecovery`/
`AmclRelocalizeMitigation`/`HumanAssistanceRecovery`/`AdvanceRecovery` informan `SUCCEEDED`, es
precisamente porque el diagnóstico está a punto de leerse `OK` en el siguiente ciclo del
evaluador), así que generalizar la exclusión de `RecoveryManagerNode` a "cualquier estado
terminal, no solo `FAILED`" habría sido un cambio innecesariamente amplio para un fallo aislado
de un solo mitigador.

**Corrección** (`CancelMissionRecovery.cpp`, `on_cycle()`): cuando la señal ya ha sido consumida,
devuelve `RecoveryStatus::FAILED` en vez de `SUCCEEDED` — semánticamente correcto ("gave up"),
y con el efecto colateral exacto que se necesita: al estar configurado con el mayor número de
prioridad de todos (Sesión 19, el último candidato), quedar excluido termina la escalada en vez
de repetirla, hasta que el diagnóstico se resuelva de verdad (por ejemplo, un operador
relocalizando AMCL a mano) y una futura recurrencia reinicie la escalada desde el principio.

**Verificación.**

```
pixi run -e rolling build --packages-select easynav_cancel_mission_recovery
pixi run -e rolling colcon test --packages-select easynav_cancel_mission_recovery
pixi run -e rolling build --packages-above easynav_core   # 37 paquetes, todos compilan
pixi run -e rolling colcon test --packages-select easynav_recovery easynav_system easynav_cancel_mission_recovery easynav_human_assistance_recovery easynav_costmap_localizer --parallel-workers 1
```

Test existente `SucceedsOnceGoalManagerResetsTheFlag` renombrado a
`ReportsFailedOnceGoalManagerResetsTheFlag` y actualizado a la nueva expectativa. 5/5 tests del
paquete en verde; 0 fallos nuevos en los otros cuatro paquetes tocados por el cambio de contrato.

**Cierre de fases, a petición del usuario.** Revisando el roadmap (§5.12) contra lo realmente
implementado:

- **Fase 5 → ✅ Hecho, con matices.** De los 4 puntos de §5.12.6, tres tienen huecos frente a la
  letra del diseño, aceptados aquí en vez de cerrados con más código:
  - *"entradas en `easynav_tools plugins`"*: el verbo CLI (`easynav_tools/cli/plugins.py`) no
    lista `RecoveryEvaluatorBase`/`RecoveryMitigationBase`/`SafetyReflexBase` — solo las
    categorías de navegación "normal". No se ha tocado.
  - *"observabilidad (`GoalManagerInfo` extendido)"*: el .msg no cambió, pero el objetivo
    (observabilidad del subsistema de recuperación) sí se cumplió, por otra vía ya construida:
    `/diagnostics` real + panel de la TUI (Sesión 19).
  - *"documentación del punto de extensión `score()`"*: no existe tal método virtual — Sesión 17
    ya lo descartó a propósito por un parámetro `priority` leído por `RecoveryManagerNode`, más
    simple que una propiedad del plugin. No hay nada que documentar porque el mecanismo del
    diseño no es el que se construyó.
- **Fase 6 → ⬜ Descartada.** Detección de mala configuración
  (`RecurringPatternEvaluator`/`ParameterAdjustmentMitigation`, histórico de diagnósticos,
  mitigadores en paralelo) queda fuera de alcance: es más ambiciosa que lo que se necesita
  funcionando ahora mismo. No se ha empezado nada de esta fase.
- **Fase 7 → ✅ Hecho, descopeada.** El `HumanAssistanceRecovery` genérico de la Sesión 17 se da
  por suficiente: cubre el espíritu (parar y esperar a que un humano resuelva el problema) sin
  el modo `teleop` ni el mensaje de petición/`ack` con id de episodio que pedía §5.15 literalmente.
  Si en el futuro hiciera falta teletransporte real del robot durante la asistencia, `teleop`
  queda como extensión posterior, no como pendiente de esta fase.

### Sesión 21 — topic `"mitigation"`, un path que se quedaba pegado en RViz, y limpieza de comentarios

**Lo que pedía el usuario.** Los mitigadores ya informaban de lo que hacían por rosout, pero el
usuario quería además un topic dedicado (`"mitigation"`) para poder verlo en la TUI sin filtrar
el `/rosout` de todo el sistema, con un cuadro nuevo debajo de "Diagnostics" que fuera acumulando
líneas con *scroll* hasta que la situación se mitigara, momento en el que se limpiaría.

**Elección del mensaje: `rcl_interfaces/msg/Log`, no algo nuevo.** Es literalmente el mensaje que
ya usa `/rosout` — reutilizarlo evita definir un mensaje propio. `diagnostic_msgs/DiagnosticStatus`
no encajaba: modela un estado *actual* por clave (se sobrescribe cada ciclo), no un registro de
eventos que se van acumulando.

**`RecoveryMitigationBase::report(nav_state, level, msg)`** (nuevo, `easynav_core`). Sustituye a
las llamadas `RCLCPP_INFO/WARN/ERROR` sueltas que ya tenían `SafeRetreatRecovery`,
`AmclRelocalizeMitigation`, `HumanAssistanceRecovery`, `AdvanceRecovery` y `CancelMissionRecovery`:
un único punto de llamada que loguea a rosout exactamente igual que antes y además encola la
misma entrada (`MitigationReport`: un `seq` global + un `rcl_interfaces::msg::Log`) en NavState
para que `RecoveryManagerNode` la publique. Solo se guarda la última entrada, no una cola — un
`seq` monótono (compartido por todas las instancias del proceso) le basta a
`RecoveryManagerNode` para saber si hay algo nuevo con una sola lectura atómica, sin necesitar
una cola *thread-safe* en `NavState`. En la práctica no se pierde nada relevante: cada
`on_start()`/transición terminal de `on_cycle()` informa como mucho una vez por episodio; el
único caso que informaba en cada ciclo (`HumanAssistanceRecovery`, "still waiting", antes con
`RCLCPP_ERROR_THROTTLE`) pasó a un *throttle* manual explícito para no inundar ese único hueco.

**`RecoveryManagerNode` centraliza la publicación real**, igual que ya hacía con `/diagnostics`:
`mitigation_pub_` (topic relativo `"mitigation"`) se publica al final de `cycle()` (nunca desde
`cycle_rt()`, para no meter una llamada de publicación en el hilo RT). Además publica un
*sentinel* de "resuelto" (nivel `DEBUG`, nunca usado por `report()` en condiciones normales, así
que sirve de marca inequívoca) cuando un diagnóstico que sí llegó a tener una mitigación activa
vuelve a `OK` — para eso, un `keys_with_mitigation_history_` nuevo, porque una mitigación que
tiene éxito a la primera nunca queda registrada en `excluded_mitigations_` (que solo trackea
`FAILED`).

**TUI.** `MitigationProcessor` nuevo (mismo patrón que `DiagnosticsProcessor`), y un cuadro
"Mitigation" nuevo bajo "Diagnostics" usando `RichLog` (soporta *append* + scroll nativo, a
diferencia del `Static` que sobrescribe usado en el resto de cuadros), con su propio interruptor
on/off. Al llegar el *sentinel* de "resuelto", el cuadro se limpia. Se aprovechó para repartir
mejor la altura entre los cuadros de la columna derecha (`NavState` 2fr, el resto 1fr cada uno),
en vez de los tres a partes iguales de antes.

**El path que se quedaba pegado en RViz.** Aparte, el usuario probó el escenario completo (perder
localización, escalar hasta cancelar la misión) y notó que, aunque el robot paraba bien, RViz
seguía mostrando el último `path` calculado. Causa: el fix de la Sesión 19 en
`CostmapPlanner::update()` vacía `current_path_` cuando no hay `goals` y lo escribe en
`nav_state.set("path", ...)` (lo que lee el controlador), pero nunca llama a
`path_pub_->publish(...)` (el topic que ve RViz) — así que el último path no vacío seguía
"congelado" visualmente. Corrección de una línea: publicar el path ya vaciado, una vez, en esa
misma transición.

**Limpieza de comentarios, a petición del usuario.** Repaso de todo el subsistema de
recuperación (no solo lo de hoy): se quitaron todas las referencias a
`docs/recoveries_easynav*.md`/`§X.Y`/"Sesión N" dentro de comentarios de código (quedan, como es
lógico, en este propio documento) y se recortó verbosidad en general, en `easynav_core`,
`easynav_recovery`, `easynav_system`, `easynav_controller`, `easynav_tools` y los paquetes de
`easynav_plugins` tocados por el desarrollo de recuperación. Ningún cambio de lógica — solo
comentarios — verificado con la misma batería de tests de después.

**Verificación.**

```
pixi run -e rolling build --packages-select easynav_core easynav_recovery <5 mitigadores> easynav_costmap_planner easynav_tools
pixi run -e rolling colcon test --packages-select <los mismos>
pixi run -e rolling build --packages-above easynav_core   # 37-38 paquetes, todos compilan
pixi run -e rolling colcon test --packages-above easynav_core easynav_tools --parallel-workers 1
```

1501 tests, 0 fallos nuevos (mismos 28 preexistentes de `robotnik_sensors`). La TUI se verifica
lanzándola (no tiene suite automatizada).

### Pendientes generales (no bloquean el cierre de ninguna fase)

- **Calibrar `covariance_threshold`/`rotation_speed`/`timeout` en el robot real** — los valores
  actuales son puntos de partida razonables, no medidos.
- **`hardware_id`/`values` del reflejo de nivel 0** (§5.13: `"controller.safety_reflex"` con
  `distancia_obstaculo`/`activo_desde`) siguen sin ese detalle — `SafetyReflexBase` (Sesión 9)
  publica un diagnóstico genérico, no estos campos específicos.
- **Tabla de prioridad/cooldown real (§5.6)** sigue sin existir como tal — el parámetro
  `priority` (Sesión 17) resuelve el orden de arbitraje, y la exclusión (Sesión 14) resuelve "no
  reintentes lo que ya falló para este mismo problema", pero no hay *cooldown* por tiempo.
- **Entradas de recuperación en `easynav_tools plugins`** (ver Fase 5 más arriba) — quedaría como
  tres categorías más en `BASE_CLASSES`/`CATEGORY_ORDER` (`plugins.py`), trabajo pequeño y
  aislado si en algún momento hace falta.

---

## Nota fuera de fase — ruido de log de TF al arrancar (encontrado probando `CollisionSafetyReflex`)

No es parte del roadmap de recuperación (el fallo vive en `easynav_sensors`, un paquete de
percepción de propósito general), pero se encontró y corrigió al probar en real el reflejo de
colisión, así que se deja constancia aquí en vez de en un documento aparte.

**Síntoma.** Al arrancar `easynav_costmap_rpp.launch.py`, el log mostraba warnings repetidos de
`PointPerceptionsOpsView`: `"TF lookup failed in fuse(): ... base_link ... does not exist"`,
cada uno con un *backtrace* completo. Se confirmó que es benigno — es el TF de `base_link`
todavía no publicado (localización/`robot_state_publisher` arrancan un instante después), la
excepción se captura dentro de `fuse()` (la percepción se marca inválida y se sigue, sin afectar
a `CollisionSafetyReflex`), y el propio *backtrace* es una utilidad de depuración deliberada
(`backtrace()`/`execinfo.h`, no una señal ni un `abort()`), ya limitada a una vez cada 5 s.

**El problema real, no obstante.** El mensaje de warning en sí (`RCLCPP_WARN`, no
`_THROTTLE`) no tenía ningún límite de frecuencia. Como `PointPerceptionsOpsView::fuse()` se
llama desde `CollisionSafetyReflex::check()` en cada ciclo RT (`SafetyReflexBase::internal_check_and_mitigate()`,
hasta 200 Hz) y por cada percepción de nube de puntos, durante la ventana de arranque en la que
TF aún no está poblado esto podía inundar el log a 200×N líneas/segundo — con el coste de
formateo/logging correspondiente cayendo justo en el camino RT de seguridad.

**Corrección** (`easynav_sensors/src/easynav_sensors/types/PointPerception.cpp`,
`PointPerceptionsOpsView::fuse()`). Los tres `RCLCPP_WARN` de la rama de captura de
`tf2::TransformException` pasan a `RCLCPP_WARN_THROTTLE` con un `rclcpp::Clock` local `static`
(`RCL_STEADY_TIME` — no depende de que `/clock` de simulación esté ya publicando, que sería
exactamente el mismo tipo de problema de "arranque" que se está intentando evitar) y un período
de 2000 ms, mismo valor que ya usan otros `*_THROTTLE` de este árbol
(`MethodBase`, `RecoveryEvaluatorBase`, etc.). No se tocó la lógica de reintento de TF en sí
(la rama que ya intenta *fallback* a la última TF conocida cuando la extrapolación pide un
instante futuro) ni el propio *backtrace*, que sigue disponible pero ahora acompaña a un log ya
limitado.

**Verificación.**

```
pixi run -e rolling build --packages-select easynav_sensors
pixi run -e rolling colcon test --packages-select easynav_sensors
pixi run -e rolling build --packages-above easynav_sensors   # 35 paquetes, todos compilan
```

0 fallos nuevos en `easynav_sensors` (144 tests, linters incluidos) ni en el resto de paquetes
afectados por la Fase 1-3 (probados uno a uno para evitar los falsos positivos por contención de
recursos ya vistos en sesiones anteriores al lanzar muchos binarios `gtest`/`rclcpp` en
paralelo).
