#!/usr/bin/env bash
# Orchestrator helper: start or resume a sub-agent in its worktree, outside the process tree AND
# the cgroup of the calling session, as a transient systemd user unit. Observed twice on
# 2026-09-21: the orchestrator session runs inside the cgroup of a system service; when that
# service restarts, every process of the cgroup is killed, `setsid nohup` children included.
# A `systemd-run --user` unit lives under user@<uid>.service (needs `loginctl enable-linger`) and
# survives. Falls back to `setsid nohup` where there is no systemd user manager.
# Repository-level orchestration lives in glam-cairo's docs/ORCHESTRATOR.md.
#
# usage: scripts/agent.sh <task> <claude|codex> <model> <new|resume> <prompt> [codex-session-id] [effort]
#        scripts/agent.sh status            # one line per known task
#        scripts/agent.sh wait <task>       # block until the agent of <task> has exited
#   worktree : <main checkout>/.claude/worktrees/cli-<task>   (create it first with git worktree add)
#   log      : <main checkout>/.claude/worktrees/logs/<task>.log   (+ <task>.unit or <task>.pid,
#              <task>.last.md for codex)
set -euo pipefail
main=$(dirname "$(git rev-parse --path-format=absolute --git-common-dir)")
W=$main/.claude/worktrees
L=$W/logs
mkdir -p "$L"

running() { # <task>
  if [ -f "$L/$1.unit" ]; then
    systemctl --user is-active -q "$(cat "$L/$1.unit")"
  elif [ -f "$L/$1.pid" ]; then
    kill -0 "$(cat "$L/$1.pid")" 2> /dev/null
  else
    return 1
  fi
}

case "${1:-}" in
  status)
    for f in "$L"/*.log; do
      t=$(basename "$f" .log)
      printf '%-20s %-8s log %9s B, last write %s\n' "$t" \
        "$(running "$t" && echo running || echo stopped)" "$(wc -c < "$f")" \
        "$(date -u -r "$f" +%H:%M:%SZ)"
    done
    exit 0 ;;
  wait)
    while running "$2"; do sleep 20; done
    exit 0 ;;
esac

task=$1 cli=$2 model=$3 mode=$4 prompt=$5 sid=${6:-} effort=${7:-high}
wt=$W/cli-$task
[ -d "$wt" ] || { echo "no worktree $wt" >&2; exit 2; }
if running "$task"; then echo "$task: already running" >&2; exit 1; fi
case "$cli:$mode" in
  claude:new)
    cmd=(claude -p "$prompt" --model "$model" --dangerously-skip-permissions --name "$task"
      --output-format text) ;;
  claude:resume)
    cmd=(claude --continue -p "$prompt" --model "$model" --dangerously-skip-permissions
      --output-format text) ;;
  codex:new)
    cmd=(codex exec -C "$wt" -m "$model" -c "model_reasoning_effort=$effort"
      --dangerously-bypass-approvals-and-sandbox -o "$L/$task.last.md" "$prompt") ;;
  codex:resume)
    cmd=(codex exec resume "$sid" -m "$model" -c "model_reasoning_effort=$effort"
      --dangerously-bypass-approvals-and-sandbox -o "$L/$task.last.md" "$prompt") ;;
  *) echo "usage: agent.sh <task> <claude|codex> <model> <new|resume> <prompt> [sid] [effort]" >&2
    exit 2 ;;
esac
echo "--- $(date -u +%FT%TZ) $cli $model $mode" >> "$L/$task.log"
# $0 of the inner shell is the log file, "$@" the agent command line.
inner='"$@" < /dev/null >> "$0" 2>&1; echo "exit=$? $(date -u +%FT%TZ)" >> "$0"'
rm -f "$L/$task.unit" "$L/$task.pid"
if systemctl --user is-system-running > /dev/null 2>&1 || systemctl --user list-units > /dev/null 2>&1; then
  unit="glam-agent-$task-$(date -u +%H%M%S)"
  systemd-run --user --unit="$unit" --collect --quiet --working-directory="$wt" \
    --setenv=PATH="$PATH" bash -c "$inner" "$L/$task.log" "${cmd[@]}"
  echo "$unit" > "$L/$task.unit"
  echo "$task: started as systemd user unit $unit, log $L/$task.log"
else
  cd "$wt"
  setsid nohup bash -c "$inner" "$L/$task.log" "${cmd[@]}" > /dev/null 2>&1 &
  echo "$!" > "$L/$task.pid"
  echo "$task: started detached (no systemd user manager), pid $!, log $L/$task.log"
fi
