# docker_ubuntu24

## Workspace rules

This workspace is developed through a Docker container. The container image and runtime are defined by the Dockerfile, compose.yaml, and .env in this outer docker_ubuntu24 directory.

Edit files on the host filesystem or container, but run ROS2/PX4/build/test/runtime commands inside the container.

Use this command pattern:

```bash
docker compose exec ros2_jazzy bash -lc "cd /home/ncrl/docker_ubuntu24 && <command>"
```

The ROS 2 workspace is `px4_ws` inside this outer workspace. Put project ROS 2 packages under `px4_ws/src/`, and run `colcon`, `ros2 interface`, ROS 2 package tests, and ROS 2 launch commands from the container path `/home/ncrl/docker_ubuntu24/px4_ws`:

```bash
docker compose exec ros2_jazzy bash -lc "cd /home/ncrl/docker_ubuntu24/px4_ws && source /opt/ros/jazzy/setup.bash && <command>"
```

Do not assume ROS2, PX4, or Jazzy tools are installed on the host.

Do not edit `PX4-Autopilot/CLAUDE.md` unless explicitly requested.

QGC is an optional monitoring and manual safety observation tool during development. Do not make QGC the first-version control entrypoint; swarm commands should flow through ROS 2 actions/topics unless explicitly requested.

## Agent skills

### Issue tracker

Issues are tracked as local markdown files under `.scratch/`. See `docs/agents/issue-tracker.md`.

### Triage labels

Triage uses the default five canonical labels: `needs-triage`, `needs-info`, `ready-for-agent`, `ready-for-human`, `wontfix`. See `docs/agents/triage-labels.md`.

### Domain docs

Domain docs use a single-context layout with `CONTEXT.md` and `docs/adr/` at the workspace root. See `docs/agents/domain.md`.
