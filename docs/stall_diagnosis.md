# Stall-Diagnose (A/B) – ohne GUI, mit steady-clock Logs

Ziel: sporadische ~0.7–1.2s „Hänger“ eindeutig klassifizieren:

- **A: Sim/Transport/Executor-Freeze**
  - `/clock` stoppt **und/oder** `/joint_states` stoppt
  - es kommen zeitweise **keine** Callbacks / Messages an
- **B: Control/Physics-Stall**
  - `/clock` **und** `/joint_states` laufen weiter
  - Joint-Positionen ändern sich in der Zeit **nicht** (keine Bewegung)

Wichtig: Alle Diagnose-Logs sind **wall/steady-clock** (monotonic), nicht ROS-Time.

---

## 0) WSL2-Hinweis

Auf WSL2 (erkennbar an `/proc/version` mit „Microsoft“) können Timer-/Scheduler-Jitter sichtbarer werden.
Die unten genutzten Diagnosen vermeiden deshalb:
- ausschließlich `/clock`-getriebene ROS-Timer
- busy-spin loops

---

## 1) Empfohlen: unabhängige Diagnose (A/B) laufen lassen

In einem Terminal (mit gesourceter ROS2-Umgebung):

- `python3 scripts/diagnose_stalls.py`

Was du im Log erwartest:
- Periodisch: `Status: /clock hz≈... age=... | /joint_states hz≈... age=... | motion_age=...`
- Bei Stalls:
  - `STALL CLASS=A: ...` wenn `/clock` und/oder `/joint_states` „silent“ werden
  - `STALL CLASS=B: ...` wenn Streams leben, aber `motion_age` hochläuft

Optional (Controller-Introspection):
- Das Script ruft (wenn verfügbar) `/controller_manager/list_controllers` periodisch auf und loggt aktive Controller.

---

## 2) Parallel: bestehender Lag-Monitor

- `python3 scripts/monitor_lag.py`

Der Monitor hat bereits einen **wall-clock Heartbeat**. Wenn der Monitor während eines Hängers still wird, ist das ein starker Hinweis auf **A** (kein Thread/Callback kommt mehr dran oder Prozess hängt).

---

## 3) Zusätzliche ROS2-CLI Checks (optional)

Die ROS2-CLI ist praktisch, aber `ros2 topic hz` selbst ist nicht „steady-clock“ geloggt.
Für rohe Frequenz-Anzeige kannst du dennoch parallel laufen lassen:

- `ros2 topic hz /clock`
- `ros2 topic hz /joint_states`

Wenn `diagnose_stalls.py` **A** meldet, solltest du in der Regel auch hier eine Pause sehen.

---

## 4) go_to_pose_node: Stall→Resume Evidence

Im Action-Node gibt es in der EXECUTING-Phase zusätzliche Logs, die **explizit** das „Wieder-Anlaufen“ melden:
- `ROS time (/clock) resumed after ... wall (during EXECUTING)`
- `/joint_states reception resumed ... (during EXECUTING)`
- `Joint motion resumed after ... wall (during EXECUTING)`

Diese Logs sind rein diagnostisch (keine Verhaltensänderung).

---

## 5) Wie du A/B aus Logs ableitest

Minimaler Entscheidungsbaum:

- Wenn `/clock age` **und** `/joint_states age` hochlaufen → **A** (Sim/Bridge/Executor/Transport Freeze)
- Wenn `/clock age` klein bleibt, `/joint_states age` klein bleibt, aber `motion_age` hochläuft → **B** (Control/Physics Stall)
- Sonderfall: `/clock` lebt, `/joint_states` nicht → **A**, aber eher controller/transport/executor (nicht pure sim-time)

---

## 6) Nächste Schritte nach Klassifikation

- **Wenn A dominiert:** Fokus auf Gazebo/bridge/DDS/WSL2 scheduling (z.B. Gazebo server load, bridge, RMW, CPU spikes). Fixes sollten hier außerhalb des IK/Trajectory-Publishing ansetzen.
- **Wenn B dominiert:** Fokus auf Controller/ros2_control update-loop, Gazebo physics, controller_manager responsiveness.

(Änderungen/Fixes werden erst abgeleitet, wenn A/B im Log eindeutig belegt ist.)
