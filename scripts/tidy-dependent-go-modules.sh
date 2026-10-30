#!/usr/bin/env bash
set -euo pipefail

repo_root=$(git rev-parse --show-toplevel)
while IFS= read -r -d '' modfile; do
	module_dir=${modfile%/go.mod}
	printf 'Tidying %s\n' "$module_dir"
	(cd "$repo_root/$module_dir" && go mod tidy)
done < <(cd "$repo_root" && find . -mindepth 2 -maxdepth 2 -type f -name go.mod \
	-not -path './.git/*' \
	-not -path './.tmp/*' \
	-not -path './.worktrees/*' \
	-print0 | sort -z)
