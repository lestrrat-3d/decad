#!/usr/bin/env bash
set -euo pipefail

repo_root=$(git rev-parse --show-toplevel)
root_requirements=$(go mod edit -json "$repo_root/go.mod" | jq -c '[(.Require // [])[] | select(.Indirect != true)]')
while IFS= read -r -d '' modfile; do
	module_dir=${modfile%/go.mod}
	printf 'Tidying %s\n' "$module_dir"
	updates=$(go mod edit -json "$modfile" | jq -r --argjson root "$root_requirements" '
		[(.Require // [])[] as $dependent |
		 $root[] |
		 select(.Path == $dependent.Path) |
		 "-require=\(.Path)@\(.Version)"] | .[]')
	if [[ -n "$updates" ]]; then
		# The root module is the source of truth for dependencies shared with
		# nested tools. Update existing requirements before tidying each module.
		readarray -t update_args <<< "$updates"
		go mod edit "${update_args[@]}" "$repo_root/$modfile"
	fi
	(cd "$repo_root/$module_dir" && go mod tidy)
done < <(cd "$repo_root" && find . -mindepth 2 -maxdepth 2 -type f -name go.mod \
	-not -path './.git/*' \
	-not -path './.tmp/*' \
	-not -path './.worktrees/*' \
	-print0 | sort -z)
