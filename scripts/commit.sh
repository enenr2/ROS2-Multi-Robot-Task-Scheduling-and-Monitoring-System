#!/usr/bin/env bash
# 提交包装器：用固定身份提交，并过滤掉协作标记。
#
# 为什么需要它：本机有外部工具注入的 GIT_COMMITTER_* 环境变量，其优先级
# 高于 git config —— 连 `git -c user.email=... commit` 也压不住。只要哪次
# 忘了前缀覆盖，提交者就会被写成那个外部工具的标识，在 GitHub 贡献者
# 列表里多出一个陌生人。把身份注入固化在脚本里，避免依赖人的记性。
#
# 用法：
#   scripts/commit.sh "feat(scope): 说明"
#   scripts/commit.sh -a "fix: 说明"      # 先暂存已跟踪文件的改动
#   scripts/commit.sh --amend "新说明"    # 修正上一条（仅本地未推送时）
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IDENT_NAME="enenr2"
IDENT_EMAIL="3336180061@qq.com"

cd "$REPO_ROOT"

STAGE_ALL=0
AMEND=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    -a|--all) STAGE_ALL=1; shift ;;
    --amend)  AMEND=1; shift ;;
    -*) echo "未知选项: $1" >&2; exit 2 ;;
    *) break ;;
  esac
done

if [[ $# -lt 1 || -z "$1" ]]; then
  echo "用法: scripts/commit.sh [-a] [--amend] \"提交信息\"" >&2
  exit 2
fi
MSG="$1"

# 兜底过滤：即便调用方把协作标记写进了信息，也不让它进历史。
# 只删结构性标记行（GitHub 据此识别共同作者），不做泛化关键词过滤 ——
# 否则正常的说明文字也会被误删。
CLEAN_MSG="$(printf '%s\n' "$MSG" \
  | sed '/^[[:space:]]*[Cc]o-[Aa]uthored-[Bb]y[[:space:]]*:/d' \
  | sed -e :a -e '/^\n*$/{$d;N;};/\n$/ba')"

# 环境变量优先级高于 git config，必须在这里显式覆盖
export GIT_AUTHOR_NAME="$IDENT_NAME"
export GIT_AUTHOR_EMAIL="$IDENT_EMAIL"
export GIT_COMMITTER_NAME="$IDENT_NAME"
export GIT_COMMITTER_EMAIL="$IDENT_EMAIL"

if [[ $STAGE_ALL -eq 1 ]]; then
  git add -u
fi

MSG_FILE="$(mktemp)"
trap 'rm -f "$MSG_FILE"' EXIT
printf '%s\n' "$CLEAN_MSG" > "$MSG_FILE"

ARGS=(-F "$MSG_FILE")
[[ $AMEND -eq 1 ]] && ARGS+=(--amend)

git -c user.name="$IDENT_NAME" -c user.email="$IDENT_EMAIL" commit "${ARGS[@]}"

# 提交后自检：身份不符或仍有结构性标记就立刻失败，避免脏历史被推送
FAILED=0
LAST_AUTHOR="$(git log -1 --format='%an <%ae>')"
LAST_COMMITTER="$(git log -1 --format='%cn <%ce>')"
if [[ "$LAST_AUTHOR" != "$IDENT_NAME <$IDENT_EMAIL>" ]]; then
  echo "[EE] 作者身份异常: $LAST_AUTHOR" >&2
  FAILED=1
fi
if [[ "$LAST_COMMITTER" != "$IDENT_NAME <$IDENT_EMAIL>" ]]; then
  echo "[EE] 提交者身份异常: $LAST_COMMITTER" >&2
  FAILED=1
fi
if git log -1 --format='%B' | grep -qiE '^[[:space:]]*co-authored-by[[:space:]]*:'; then
  echo "[EE] 提交信息仍含共同作者标记" >&2
  FAILED=1
fi

if [[ $FAILED -ne 0 ]]; then
  echo "    可用 scripts/commit.sh --amend \"信息\" 修正后再推送" >&2
  exit 1
fi

echo "[OK] 已提交: $(git log -1 --format='%h %an <%ae> | %s')"
