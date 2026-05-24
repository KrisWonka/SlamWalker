#!/usr/bin/env bash
set -euo pipefail

# Auto-commit loop:
# - checks tracked/untracked changes every N seconds
# - commits only when there are real changes
# - pushes to current branch if remote exists

INTERVAL_SECONDS="${1:-10}"

commit_repo_if_dirty() {
  local repo_path="$1"
  local label="$2"
  local ts="$3"

  if [[ -z "$(git -C "${repo_path}" status --porcelain 2>/dev/null || true)" ]]; then
    return 0
  fi

  local msg="auto: snapshot ${ts}"
  git -C "${repo_path}" add -A
  if git -C "${repo_path}" commit -m "${msg}" >/dev/null 2>&1; then
    echo "[auto-commit] committed ${label}: ${msg}"
    if git -C "${repo_path}" rev-parse --abbrev-ref --symbolic-full-name '@{u}' >/dev/null 2>&1; then
      git -C "${repo_path}" push >/dev/null 2>&1 || true
    fi
  fi
}

while true; do
  if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo "[auto-commit] not in a git repo, retrying..."
    sleep "${INTERVAL_SECONDS}"
    continue
  fi

  ts="$(date +"%Y-%m-%d %H:%M:%S")"

  # 1) Commit nested repos under src/ first (these are embedded git repos).
  for d in src/*; do
    if [[ -e "${d}/.git" ]] && git -C "${d}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
      commit_repo_if_dirty "${d}" "${d}" "${ts}"
    fi
  done

  # 2) Commit/push outer repo (records script/docs/maps and gitlink updates).
  if [[ -n "$(git status --porcelain)" ]]; then
    msg="auto: snapshot ${ts}"
    git add -A
    if git commit -m "${msg}" >/dev/null 2>&1; then
      echo "[auto-commit] committed root: ${msg}"
      if git rev-parse --abbrev-ref --symbolic-full-name '@{u}' >/dev/null 2>&1; then
        git push >/dev/null 2>&1 || true
      fi
    fi
  fi

  sleep "${INTERVAL_SECONDS}"
done

