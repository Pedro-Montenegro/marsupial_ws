#!/usr/bin/env python3
"""
MissionOrchestrator SIMPLE (Nav2-native)

- Usa Nav2 NavigateToPose (BT Navigator). Nada de ComputePathToPose + ExecutePath.
- Acompanha o trajeto assinando um tópico de Path (ex: /plan).
- Publica progresso (remaining distance estimada) usando TF + path atual.
- PAUSE: cancela o goal atual e guarda o último goal para poder RESUME.
- RESUME: reenvia o último goal guardado.
- CANCEL: cancela a missão inteira (e descarta goal guardado).

Tópicos:
  Sub:
    /ui/start_mission (String JSON)  -> {"id":"recepcao"} por exemplo
    /mission_ctrl (String)          -> "PAUSE" | "RESUME" | "CANCEL" | "RETURN"
    <plan_topic> (nav_msgs/Path)    -> padrão /plan (configurável)

  Pub:
    /ui/progress (String JSON)      -> eventos: state / progress / arrived
    /tts/say (String)               -> fala

Requisitos:
  - Nav2 rodando e action server de NavigateToPose disponível
  - TF map->base_link ok
  - Um tópico de Path do Nav2 publicado (configure plan_topic)
"""

import math
import time
import json
import os
from dataclasses import dataclass
from typing import Optional, List, Tuple, Union

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener

from nav2_msgs.action import NavigateToPose


# ==============================
# DSL de passos (mantém seu estilo)
# ==============================
@dataclass
class GoTo:
    x: float
    y: float
    frame: str = "map"
    yaw: Optional[float] = None
    tag: Optional[str] = None

@dataclass
class Tour:
    waypoints: List[Tuple[float, float]]
    frame: str = "map"

@dataclass
class Speak:
    text: str

@dataclass
class Wait:
    seconds: float

@dataclass
class ReturnHome:
    pass

@dataclass
class AwaitCommand:
    cmd: str
    label: Optional[str] = None

Step = Union[GoTo, Tour, Speak, Wait, ReturnHome, AwaitCommand]


# ==============================
# Utils
# ==============================
def quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def dist_xy(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])

def path_to_xy_list(path: Path) -> List[Tuple[float, float]]:
    return [(ps.pose.position.x, ps.pose.position.y) for ps in path.poses]

def nearest_index(points: List[Tuple[float, float]], p: Tuple[float, float]) -> int:
    if not points:
        return 0
    best_i, best_d = 0, 1e18
    for i, xy in enumerate(points):
        d = dist_xy(xy, p)
        if d < best_d:
            best_i, best_d = i, d
    return best_i

def remaining_distance(points: List[Tuple[float, float]], k: int, p: Tuple[float, float]) -> float:
    if not points:
        return 0.0
    k = max(0, min(k, len(points) - 1))
    d = dist_xy(p, points[k])
    for i in range(k + 1, len(points)):
        d += dist_xy(points[i - 1], points[i])
    return d

def pose_xy(x: float, y: float, frame_id: str = "map", yaw: Optional[float] = None) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    if yaw is None:
        ps.pose.orientation.w = 1.0
    else:
        ps.pose.orientation = quat_from_yaw(float(yaw))
    return ps

def yaw_from_direction(src: Tuple[float, float], dst: Tuple[float, float]) -> float:
    """Calcula o yaw apontando de src para dst."""
    return math.atan2(dst[1] - src[1], dst[0] - src[0])




# ==============================
# MissionOrchestrator (Nav2 Simple)
# ==============================
class MissionOrchestratorNav2(Node):
    def __init__(self):
        super().__init__("mission_orchestrator_nav2")

        # --- params ---
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_link_frame", "base_link")

        # Nav2 action name:
        self.declare_parameter("navigate_action", "/navigate_to_pose")

        # Path topic do Nav2 (ajuste para o seu):
        self.declare_parameter("plan_topic", "/plan")

        # Progresso
        self.declare_parameter("progress_hz", 5.0)
        self.declare_parameter("reach_tol", 0.25)  # para "arrived" (se você quiser complementar)
        self.declare_parameter("lookahead_idx_cap", 5)

        # --- Carregar Missões do JSON ---
        self.declare_parameter(
            "missions_file",
            os.path.join(
                get_package_share_directory("marsupial_robot_navigation"),
                "config",
                "missions.json",
            ),
        )
        self.load_missions_from_json()

        # Missão
        self.declare_parameter("complete_requires_return", False)
        self.declare_parameter("return_reach_tol", 0.60)

        self.global_frame = self.get_parameter("global_frame").value
        self.base_link_frame = self.get_parameter("base_link_frame").value
        self.navigate_action = self.get_parameter("navigate_action").value
        self.plan_topic = self.get_parameter("plan_topic").value
        self.progress_hz = float(self.get_parameter("progress_hz").value)
        self.reach_tol = float(self.get_parameter("reach_tol").value)
        self.lookahead_idx_cap = int(self.get_parameter("lookahead_idx_cap").value)
        self.complete_requires_return = bool(self.get_parameter("complete_requires_return").value)
        self.return_reach_tol = float(self.get_parameter("return_reach_tol").value)

        # --- TF ---
        self.tf = Buffer(cache_time=Duration(seconds=10.0))
        self.tfl = TransformListener(self.tf, self)

        # --- action client Nav2 ---
        self.nav_ac = ActionClient(self, NavigateToPose, self.navigate_action)
        self._gh_nav = None

        # --- UI / controle ---
        self.create_subscription(String, "/ui/start_mission", self._on_ui_start, 10)
        self.create_subscription(String, "/mission_ctrl", self._on_mission_ctrl, 10)

        self.pub_ui = self.create_publisher(String, "/ui/progress", 10)
        self.tts_pub = self.create_publisher(String, "/tts/say", 10)

        # --- Path monitor (trajeto do Nav2) ---
        self._last_plan: Optional[Path] = None
        self._points: List[Tuple[float, float]] = []
        self._total_points = 0
        self._last_idx = 0

        self.create_subscription(Path, self.plan_topic, self._on_plan_path, 10)

        # --- timer progresso ---
        self._progress_timer = self.create_timer(1.0 / max(1e-3, self.progress_hz), self._on_progress_tick)

        # --- estado de missão ---
        self._mission_id = 0
        self._blocked_until_nav_done = False
        self._awaiting: Optional[AwaitCommand] = None

        self.current_steps: List[Step] = []
        self.current_step_idx: int = -1
        self._tour_queue: List[Tuple[float, float]] = []
        self._last_executed_step: Optional[Step] = None

        self.home_pose: Optional[PoseStamped] = None
        self._current_goal_pose: Optional[PoseStamped] = None  # goal em execução
        self._paused_goal_pose: Optional[PoseStamped] = None   # guardado ao pausar

        self._paused = False
        self._pause_requested = False

        self.current_quat: Optional[Quaternion] = None

        # presets (ajuste livre)
        """ self.missions: dict[str, List[Step]] = {
            "poi1": [
                Speak("Olá! Iniciando rota para a recepção."),
                GoTo(9.0, 7.0, "map", tag="recepcao"),
                Speak("Cheguei à recepção. Posso ajudar?"),
                # AwaitCommand("RETURN", label="AT_RECEPTION"),
            ],
            "poi3": [
                Speak("Iniciando patrulha do Laboratório."),
                # Tour([(2.0, 2.0), (11.0, 0.0), (14.0, -2.0), (17.0, 0.0), (16.0, 5.0), (6.0, 6.0), (2.0, 2.0)]),
                Tour([(4.0, 4.0), (4.5, 3.0), (4.5, 7.0), (2.0, 7.0), (11.0, 3.0)]),
                Speak("Patrulha concluída."),
                ReturnHome(),
            ],
            "poi2": [
                Speak("Olá! Iniciando rota para a Estação de Trabalho."),
                GoTo(5.0, 16.0, "map", tag="estacao"),
                Speak("Cheguei à Estação de Trabalho. Posso ajudar?"),
                AwaitCommand("RETURN", label="AT_STATION"),
            ],
        } """

        self.get_logger().info(
            f"[mission_nav2] up | action={self.navigate_action} | plan_topic={self.plan_topic} | TF {self.global_frame}->{self.base_link_frame}"
        )
        self._publish_state("IDLE")

        

    def load_missions_from_json(self):
        file_path = self.get_parameter("missions_file").value
        try:
            with open(file_path, 'r') as f:
                raw_data = json.load(f)
            
            self.missions = {}
            for m_id, steps in raw_data.items():
                self.missions[m_id] = []
                for s in steps:
                    stype = s.get("type")
                    if stype == "Speak":
                        self.missions[m_id].append(Speak(text=s["text"]))
                    elif stype == "GoTo":
                        self.missions[m_id].append(GoTo(x=s["x"], y=s["y"], frame=s.get("frame", "map"), tag=s.get("tag")))
                    elif stype == "Wait":
                        self.missions[m_id].append(Wait(seconds=s["seconds"]))
                    elif stype == "Tour":
                        self.missions[m_id].append(Tour(waypoints=[tuple(x) for x in s["waypoints"]]))
                    elif stype == "ReturnHome":
                        self.missions[m_id].append(ReturnHome())
                    elif stype == "AwaitCommand":
                        self.missions[m_id].append(AwaitCommand(cmd=s["cmd"], label=s.get("label")))
            
            self.get_logger().info(f"✅ {len(self.missions)} missões carregadas de {file_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Erro ao carregar missões: {str(e)}")
            self.missions = {}

    # ---------------- UI events ----------------
    def _publish_state(self, status: str):
        msg = String()
        msg.data = json.dumps({"event": "state", "status": status})
        self.pub_ui.publish(msg)

    def _publish_progress(self, current_idx: int, total: int, remaining_m: float):
        msg = String()
        msg.data = json.dumps({
            "event": "progress",
            "current": int(current_idx),
            "total": int(total),
            "remaining_m": round(float(remaining_m), 3),
        })
        self.pub_ui.publish(msg)

    def _publish_arrived(self, place: Optional[str]):
        msg = String()
        msg.data = json.dumps({"event": "arrived", "place": place or ""})
        self.pub_ui.publish(msg)

    # ---------------- TF ----------------
    def _tf_xy_now(self) -> Optional[Tuple[float, float]]:
        try:
            tr = self.tf.lookup_transform(self.global_frame, self.base_link_frame, rclpy.time.Time())
        except Exception:
            return None
        return float(tr.transform.translation.x), float(tr.transform.translation.y)

    def _dist_home(self) -> Optional[float]:
        if self.home_pose is None:
            return None
        cur = self._tf_xy_now()
        if cur is None:
            return None
        hx = float(self.home_pose.pose.position.x)
        hy = float(self.home_pose.pose.position.y)
        return dist_xy(cur, (hx, hy))

    # ---------------- Path callback ----------------
    def _on_plan_path(self, msg: Path):
        # guarda o plano atual (trajeto do Nav2)
        if not msg.poses:
            return
        self._last_plan = msg
        self._points = path_to_xy_list(msg)
        self._total_points = len(self._points)
        # não reseta _last_idx agressivamente (pra evitar "pular para trás" quando replana)
        # mas se o path mudou muito / primeiro path do goal, deixe ele coerente
        if self._last_idx >= self._total_points:
            self._last_idx = max(0, self._total_points - 1)

    # ---------------- Start mission ----------------
    def _on_ui_start(self, msg: String):
        print(f"DEBUG: Tópico recebido no script! Conteúdo: {msg.data}") # Adicione isso
        self.get_logger().info(f"[mission_nav2] Iniciando missão")
        try:
            data = json.loads(msg.data or "{}")
        except Exception:
            self.get_logger().error("[mission_nav2] /ui/start_mission JSON inválido")
            return
        pid = (data.get("id") or "").strip()
        if pid not in self.missions:
            self.get_logger().error(f"[mission_nav2] preset desconhecido: '{pid}'")
            return

        # preempção limpa
        self._cancel_everything()

        # nova geração
        self._mission_id += 1
        self._blocked_until_nav_done = False
        self._awaiting = None

        self.current_steps = list(self.missions[pid])
        self.current_step_idx = -1
        self._tour_queue.clear()
        self._last_executed_step = None

        # seta home
        xy = self._tf_xy_now()
        if xy is None:
            self.get_logger().error("[mission_nav2] TF indisponível — abortado.")
            self._publish_state("IDLE")
            return
        self.home_pose = pose_xy(xy[0], xy[1], self.global_frame, yaw=None)

        self._publish_state("RUNNING")
        self.get_logger().info(f"[mission_nav2] Iniciando missão '{pid}' ({len(self.current_steps)} passos).")
        self._run_next_step()

    # ---------------- Mission control ----------------
    def _on_mission_ctrl(self, msg: String):
        raw = msg.data or ""
        cmd = raw.strip().upper()

        self.get_logger().info(
            f"[mission_nav2] /mission_ctrl recebido: raw='{raw}' -> cmd='{cmd}' "
            f"paused={self._paused} blocked={self._blocked_until_nav_done} "
            f"has_goal={self._gh_nav is not None} "
            f"has_current_goal={self._current_goal_pose is not None} "
            f"has_paused_goal={self._paused_goal_pose is not None}"
        )

        # AwaitCommand gate
        if self._awaiting is not None:
            expected = (self._awaiting.cmd or "").upper()
            if cmd == expected:
                if cmd == "RETURN" and self.home_pose is not None:
                    self.get_logger().info("[mission_nav2] RETURN aceito (await). Indo para casa.")
                    self._awaiting = None
                    self._go_home()
                    return
                self.get_logger().info(f"[mission_nav2] Comando aguardado '{expected}' recebido — prosseguindo.")
                self._awaiting = None
                self._run_next_step()
                return

        if cmd == "PAUSE":
            self._pause()
        elif cmd == "RESUME":
            self._resume()
        elif cmd == "CANCEL":
            self.get_logger().warn("[mission_nav2] CANCEL recebido via /mission_ctrl.")
            self._cancel_everything()
            self._publish_state("ABORTED")
        elif cmd == "RETURN":
            self._go_home()

    def _pause(self):
        # PAUSE = congelar execução e cancelar goal quando possível
        if self._current_goal_pose is None:
            self.get_logger().info("[mission_nav2] PAUSE ignorado: sem goal ativo.")
            return

        self.get_logger().info("[mission_nav2] PAUSE: armando pausa e cancelando goal quando possível.")
        self._paused = True
        self._pause_requested = True

        # guarda goal para RESUME (mesmo que ainda não tenha sido aceito)
        self._paused_goal_pose = self._current_goal_pose

        # se já existe goal handle, cancela agora
        if self._gh_nav is not None:
            try:
                self._gh_nav.cancel_goal_async()
            except Exception:
                pass

        # IMPORTANTe: não deixa o motor avançar passos enquanto pausado
        self._blocked_until_nav_done = False

        self._publish_state("PAUSED")


    def _resume(self):
        if self._paused_goal_pose is None:
            self.get_logger().info("[mission_nav2] RESUME ignorado: nenhum goal pausado guardado.")
            self._paused = False
            self._pause_requested = False
            self._publish_state("RUNNING")
            return

        self.get_logger().info("[mission_nav2] RESUME: reenviando goal pausado.")
        self._paused = False
        self._pause_requested = False
        self._publish_state("RUNNING")

        self._blocked_until_nav_done = True
        self._send_nav_goal(self._paused_goal_pose, reason="RESUME")
        self._paused_goal_pose = None

    def _go_home(self):
        if self.home_pose is None:
            self.get_logger().warn("[mission_nav2] RETURN ignorado: sem home_pose.")
            return
        self.get_logger().info("[mission_nav2] RETURN: indo para casa.")
        self._publish_state("RETURNING")
        self._publish_log("warn", "Retornando ao ponto de chamada (home).")
        self._blocked_until_nav_done = True
        self._send_nav_goal(self.home_pose, reason="RETURN_HOME")

    def _cancel_everything(self):
        self._mission_id += 1  # invalida callbacks antigos
        self._blocked_until_nav_done = False
        self._awaiting = None
        self._tour_queue.clear()

        self._paused_goal_pose = None
        self._current_goal_pose = None

        if self._gh_nav is not None:
            try:
                self._gh_nav.cancel_goal_async()
            except Exception:
                pass
        self._gh_nav = None

        self._last_plan = None
        self._points = []
        self._total_points = 0
        self._last_idx = 0

        self.get_logger().info("[mission_nav2] Cancelado tudo (missão/goals).")

    # ---------------- Step engine ----------------
    def _run_next_step(self):
        if self._paused:
            return
        if self._blocked_until_nav_done:
            return

        self.current_step_idx += 1
        if self.current_step_idx >= len(self.current_steps):
            # terminou: se exigir retorno, manda pra casa
            if self.complete_requires_return and self.home_pose is not None:
                d = self._dist_home()
                if d is None or d > self.return_reach_tol:
                    self.get_logger().info("[mission_nav2] Roteiro terminou; retornando para casa antes de concluir.")
                    self._go_home()
                    return

            self.get_logger().info("✅ [mission_nav2] Missão concluída.")
            self._publish_state("READY")
            return

        step = self.current_steps[self.current_step_idx]
        self.get_logger().info(f"[mission_nav2] Passo {self.current_step_idx+1}/{len(self.current_steps)} -> {step}")

        step = self.current_steps[self.current_step_idx]
        total = len(self.current_steps)

        # Emite evento de passo (sempre)
        self._publish_step(
            idx=self.current_step_idx + 1,
            total=total,
            kind=type(step).__name__,
            text=(step.text if isinstance(step, Speak) else ""),
            meta=(
                {"x": step.x, "y": step.y, "tag": step.tag, "frame": step.frame}
                if isinstance(step, GoTo) else
                {"cmd": step.cmd, "label": step.label}
                if isinstance(step, AwaitCommand) else
                {}
            )
        )

        if isinstance(step, Speak):
            s = String()
            s.data = step.text
            self.tts_pub.publish(s)
            self._publish_log("info", step.text)
            self._blocked_until_nav_done = False
            self._one_shot(0.1, self._run_next_step)

        elif isinstance(step, Wait):
            self._blocked_until_nav_done = False
            self._one_shot(step.seconds, self._run_next_step)

        elif isinstance(step, GoTo):
            self._last_executed_step = step
            goal = pose_xy(step.x, step.y, step.frame, yaw=step.yaw)
            self._blocked_until_nav_done = True
            self._send_nav_goal(goal, reason="GOTO")

        elif isinstance(step, Tour):
            self._tour_queue = list(step.waypoints)
            self._blocked_until_nav_done = True
            self._advance_tour(frame=step.frame)

        elif isinstance(step, ReturnHome):
            self._go_home()

        elif isinstance(step, AwaitCommand):
            self._awaiting = step
            self._publish_state(step.label or "WAITING")
            self._publish_log("info", f"Aguardando comando: {step.cmd}")
            # fica parado aqui até receber o cmd esperado
            return

    def _advance_tour(self, frame: str = "map"):
        if not self._tour_queue:
            self._blocked_until_nav_done = False
            self._run_next_step()
            return
        
        x, y = self._tour_queue.pop(0)
        self._last_executed_step = GoTo(x, y, frame, tag=None)

        # ← calcula yaw apontando para o próximo waypoint depois deste (se existir)
        #   assim o robô já chega orientado para onde vai continuar
        if self._tour_queue:
            nx, ny = self._tour_queue[0]
            auto_yaw = yaw_from_direction((x, y), (nx, ny))
        else:
            auto_yaw = None  # último waypoint: mantém orientação livre

        goal = pose_xy(x, y, frame, yaw=auto_yaw)
        self._blocked_until_nav_done = True
        self._send_nav_goal(goal, reason="TOUR_WP")

    # ---------------- Nav2 goal send + result ----------------
    def _send_nav_goal(self, goal_pose: PoseStamped, reason: str = ""):
        gen = self._mission_id

        if not self.nav_ac.wait_for_server(timeout_sec=5.0):
            if gen != self._mission_id:
                return
            self.get_logger().error("[mission_nav2] NavigateToPose indisponível.")
            self._publish_state("ABORTED")
            self._blocked_until_nav_done = False
            return

        # prepare action goal
        g = NavigateToPose.Goal()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        if not goal_pose.header.frame_id:
            goal_pose.header.frame_id = self.global_frame
        g.pose = goal_pose

        self._current_goal_pose = goal_pose
        self._last_idx = 0  # reseta progresso por goal (pode ajustar se quiser manter)
        # não zera path, porque o Nav2 pode demorar a publicar novo plan (mas ok zerar também)
        # self._points = []; self._total_points = 0; self._last_plan = None

        self.get_logger().info(
            f"[mission_nav2] NAV goal ({reason}): ({goal_pose.pose.position.x:.2f},{goal_pose.pose.position.y:.2f}) frame={goal_pose.header.frame_id}"
        )

        fut = self.nav_ac.send_goal_async(g, feedback_callback=self._on_nav_feedback)
        fut.add_done_callback(lambda f: self._on_nav_goal_response(f, gen))

    def _on_nav_goal_response(self, future, gen: int):
        if gen != self._mission_id:
            return
        try:
            self._gh_nav = future.result()
        except Exception as e:
            self.get_logger().error(f"[mission_nav2] Exceção ao enviar goal: {e}")
            self._publish_state("ABORTED")
            self._blocked_until_nav_done = False
            return

        if self._gh_nav is None or not self._gh_nav.accepted:
            print(f"self._gh_nav: {self._gh_nav} / self._gh_nav.accepted: {self._gh_nav.accepted}")
            self.get_logger().error("[mission_nav2] Goal rejeitado pelo Nav2.")
            self._publish_state("ABORTED")
            self._blocked_until_nav_done = False
            return

        self.get_logger().info("[mission_nav2] Goal aceito. Aguardando conclusão...")
        res_fut = self._gh_nav.get_result_async()
        res_fut.add_done_callback(lambda f: self._on_nav_result(f, gen))

    def _on_nav_result(self, future, gen: int):
        if gen != self._mission_id:
            return
        ok = False
        status = None
        try:
            result = future.result()
            status = result.status
            # NavigateToPose.Result tem campos variando por versão;
            # em geral não precisamos deles aqui: usamos status.
            ok = (status == 4)  # STATUS_SUCCEEDED geralmente = 4
        except Exception as e:
            self.get_logger().error(f"[mission_nav2] Exceção no resultado: {e}")
            ok = False

        self.get_logger().info(f"[mission_nav2] NAV terminou: ok={ok} status={status}")

        self._gh_nav = None
        self._current_goal_pose = None
        self._blocked_until_nav_done = False

        if not ok:
            # Se estava "pausando", isso vai vir como cancel; não mata a missão.
            # Heurística: se existe paused_goal_pose, considere que foi PAUSE.
            if self._paused_goal_pose is not None:
                self.get_logger().info("[mission_nav2] Goal cancelado (provável PAUSE). Mantendo missão em pausa.")
                return

            # Cancel manual (CANCEL) também chega aqui.
            self.get_logger().warn("[mission_nav2] Navegação falhou/cancelada.")
            self._publish_state("ABORTED")
            return

        # sucesso: se o passo foi GoTo com tag, emite arrived
        if isinstance(self._last_executed_step, GoTo) and self._last_executed_step.tag:
            self._publish_arrived(self._last_executed_step.tag)
            self._last_executed_step = None

        # se era tour, avança
        if self._tour_queue:
            self._advance_tour(frame=self.global_frame)
            return

        # se era retorno, decide concluir
        # (aqui simplificamos: se o roteiro já acabou, _run_next_step fecha)
        self._run_next_step()

    def _on_nav_feedback(self, fb_msg):
        # Feedback do NavigateToPose varia (distance_remaining nem sempre existe).
        # A gente usa nosso próprio progresso pelo Path + TF no timer.
        pass

    # ---------------- Progress tick ----------------
    def _on_progress_tick(self):
        if not self._blocked_until_nav_done:
            return

        cur = self._tf_xy_now()
        if cur is None:
            return
        if not self._points:
            # ainda não chegou um plan novo (ou seu plan_topic está errado)
            return

        idx_now = nearest_index(self._points, cur)

        # avanço suave do índice (evita oscilações)
        if idx_now > self._last_idx:
            step = min(idx_now - self._last_idx, self.lookahead_idx_cap)
            self._last_idx += step

        rem = remaining_distance(self._points, self._last_idx, cur)
        self._publish_progress(self._last_idx, self._total_points, rem)

        # opcional: complementar "chegada" por tolerância no fim do path
        # (Nav2 já decide; isso aqui é só pra UI ficar esperta)
        if self._points and dist_xy(cur, self._points[-1]) <= self.reach_tol:
            # não finaliza aqui (deixa o Nav2 finalizar), mas poderia emitir um evento
            pass

    # ---------------- One-shot helper ----------------
    def _one_shot(self, delay_sec: float, fn):
        gen = self._mission_id
        holder = {"t": None}

        def _wrap():
            try:
                if gen == self._mission_id:
                    fn()
            finally:
                t = holder["t"]
                if t is not None:
                    t.cancel()

        holder["t"] = self.create_timer(float(delay_sec), _wrap)
        
    def _publish_log(self, level: str, msg_txt: str):
        msg = String()
        msg.data = json.dumps({"event": "log", "level": level, "msg": msg_txt})
        self.pub_ui.publish(msg)

    def _publish_step(self, idx: int, total: int, kind: str, text: str = "", meta: dict = None):
        pkt = {
            "event": "step",
            "i": int(idx),
            "n": int(total),
            "kind": kind,   # "Speak" | "GoTo" | "Tour" | "Wait" | "AwaitCommand" | "ReturnHome"
            "text": text or "",
            "meta": meta or {},
        }
        msg = String()
        msg.data = json.dumps(pkt)
        self.pub_ui.publish(msg)

# ---------------- main ----------------
def main():
    rclpy.init()
    node = MissionOrchestratorNav2()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
