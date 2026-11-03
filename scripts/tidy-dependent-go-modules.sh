#!/usr/bin/env bash
# Updates every nested module's requirements to the root module's versions and
# tidies each one in place. With --check, changes nothing: it reports the
# modules whose go.mod or go.sum would change and exits 1 when there are any.
set -euo pipefail

usage() {
	printf 'usage: %s [--check]\n' "${0##*/}" >&2
}

check=false
case "$#" in
0) ;;
1)
	if [[ "$1" == "--check" ]]; then
		check=true
	else
		usage
		exit 2
	fi
	;;
*)
	usage
	exit 2
	;;
esac

repo_root=$(git rev-parse --show-toplevel)

list_modules() {
	(cd "$repo_root" && find . -mindepth 2 -maxdepth 2 -type f -name go.mod \
		-not -path './.git/*' \
		-not -path './.tmp/*' \
		-not -path './.worktrees/*' \
		-print0 | sort -z)
}

snapshot=
restore_snapshot() {
	local modfile module_dir name
	while IFS= read -r -d '' modfile; do
		module_dir=${modfile%/go.mod}
		for name in go.mod go.sum; do
			if [[ -e "$snapshot/$module_dir/$name" ]]; then
				cp -p "$snapshot/$module_dir/$name" "$repo_root/$module_dir/$name"
			else
				rm -f "$repo_root/$module_dir/$name"
			fi
		done
	done < <(list_modules)
}
cleanup() {
	restore_snapshot
	rm -rf "$snapshot"
}

if $check; then
	# Snapshot the on-disk files so uncommitted edits survive the check.
	snapshot=$(mktemp -d)
	trap cleanup EXIT
	while IFS= read -r -d '' modfile; do
		module_dir=${modfile%/go.mod}
		mkdir -p "$snapshot/$module_dir"
		for name in go.mod go.sum; do
			if [[ -e "$repo_root/$module_dir/$name" ]]; then
				cp -p "$repo_root/$module_dir/$name" "$snapshot/$module_dir/$name"
			fi
		done
	done < <(list_modules)
fi

root_requirements=$(go mod edit -json "$repo_root/go.mod" | jq -c '[(.Require // [])[] | select(.Indirect != true)]')
drifted=()
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
	if $check; then
		for name in go.mod go.sum; do
			if [[ -e "$snapshot/$module_dir/$name" ]]; then
				cmp -s "$snapshot/$module_dir/$name" "$repo_root/$module_dir/$name" || drifted+=("$module_dir/$name")
			elif [[ -e "$repo_root/$module_dir/$name" ]]; then
				drifted+=("$module_dir/$name")
			fi
		done
	fi
done < <(cd "$repo_root" && list_modules)

if $check && ((${#drifted[@]} > 0)); then
	printf '\nNested Go modules are out of sync with the root module:\n' >&2
	printf '  %s\n' "${drifted[@]}" >&2
	printf '\nRun scripts/tidy-dependent-go-modules.sh and commit the result.\n' >&2
	exit 1
fi
