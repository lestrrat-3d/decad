package decad_test

import (
	"io/fs"
	"maps"
	"os"
	"os/exec"
	"path/filepath"
	"regexp"
	"slices"
	"strconv"
	"strings"
	"testing"

	"github.com/stretchr/testify/require"
)

// Regression guard for the race-detector shards in .github/workflows/ci.yml.
//
// The shards exist to bound the race job's wall time, and they split the ROOT
// package's test names between two runners. The hazard the split creates is
// silent: a `-run` regex built from one package's names matches NOTHING in
// any other package, so a shard handed `./...` runs the root package's half
// and zero tests everywhere else, then reports success. No `go test` exit
// status and no non-empty-variable guard can see that, because the root
// package's own name list is never empty.
//
// This test is the mechanical check that class of loss cannot come back. It
// reads the workflow as text — the repository has no YAML parser among its
// approved modules (CLAUDE.md) and gains none for a guard — and asserts five
// properties, each of which fails loudly rather than degrading quietly:
//
//   - no `go test` command combines a `-run` filter with `./...`;
//   - every Go package directory on disk outside the root is named by some
//     shard's own `packages:` operand, so adding a package fails this test
//     until the workflow covers it;
//   - the shard enumeration selects Examples and Fuzz targets, not Test
//     functions alone, since `-run` gates all three;
//   - every test on disk is named by exactly one shard in
//     .github/test-shards.txt, and that file names no test that has gone away,
//     so a test can be neither dropped nor run twice;
//   - the file uses exactly the shards the matrix declares, so adding a shard
//     entry without regenerating the file cannot leave a runner with nothing,
//     nor a bucket with no runner;
//   - the unfiltered whole-suite step is gated on a runner having neither a
//     shard nor a package operand, so the runner that owns the non-root
//     packages cannot also re-run everything.
//
// The assignment is a recorded FILE rather than a rule computed from a test's
// position. A positional rule re-rolls the shards every time the suite grows:
// adding one test shifts every name after it, so half of them change shard, and
// a balance that held yesterday is decided afresh by the next commit — which is
// how a shard ends up against its timeout with nobody having touched it. A
// recorded assignment packed by measured cost moves nothing when a test is
// added, and the shard step's own soft budget reports drift long before the
// timeout would.
//
// Every enumeration below refuses an empty result, so a rename or a
// reorganisation that stops the scanner from matching fails the test instead
// of passing it vacuously.
const ciWorkflowPath = ".github/workflows/ci.yml"

var (
	// ciMatrixShardRe and ciMatrixPackagesRe read the two matrix columns the
	// shard steps key on.
	ciMatrixShardRe    = regexp.MustCompile(`(?m)^\s*shard:\s*"([^"]*)"\s*$`)
	ciMatrixPackagesRe = regexp.MustCompile(`(?m)^\s*packages:\s*"([^"]*)"\s*$`)
	// ciListPatternRe reads the regexp handed to `go test -list`, and
	// ciSelectPatternRe the awk pattern that keeps a listed name. Both gate
	// which names a shard can ever run.
	ciListPatternRe   = regexp.MustCompile(`-list '([^']*)'`)
	ciSelectPatternRe = regexp.MustCompile(`(?m)^\s*/([^/]+)/\s*\{\s*$`)
)

// ciShardFilePath records which shard runs each root-package test.
const ciShardFilePath = ".github/test-shards.txt"

func TestCIWorkflowRaceShardsCoverEveryPackage(t *testing.T) {
	t.Parallel()
	raw, err := os.ReadFile(ciWorkflowPath)
	require.NoError(t, err, "the CI workflow must be readable from the package directory")
	workflow := string(raw)

	t.Run("no -run filter is applied to ./...", func(t *testing.T) {
		var filtered int
		for line := range strings.SplitSeq(workflow, "\n") {
			if !strings.Contains(line, "go test ") || !strings.Contains(line, "-run ") {
				continue
			}
			filtered++
			require.NotContainsf(t, line, "./...",
				"a -run regex derived from one package matches no name in another, so this command runs nothing outside the package it was derived from: %s", strings.TrimSpace(line))
		}
		require.Positive(t, filtered,
			"no `go test` command in %s carries -run, so this check asserted nothing — the shard step was renamed or removed", ciWorkflowPath)
	})

	t.Run("every non-root package is named by a shard", func(t *testing.T) {
		dirs := goPackageDirs(t)
		require.NotEmpty(t, dirs,
			"no Go package directory outside the root was found, so this check asserted nothing — the packages moved or this test no longer runs from the repository root")

		covered := make(map[string]struct{})
		for _, m := range ciMatrixPackagesRe.FindAllStringSubmatch(workflow, -1) {
			for operand := range strings.FieldsSeq(m[1]) {
				require.NotContainsf(t, operand, "...",
					"a `packages:` operand must name packages explicitly; %q would re-run the root package unfiltered and undo the sharding", operand)
				covered[strings.Trim(strings.TrimPrefix(operand, "./"), "/")] = struct{}{}
			}
		}

		for _, dir := range dirs {
			t.Run(dir, func(t *testing.T) {
				_, ok := covered[dir]
				require.Truef(t, ok,
					"%s runs in no race shard: add ./%s/ to a shard's `packages:` operand in %s", dir, dir, ciWorkflowPath)
			})
		}
	})

	t.Run("the shard enumeration selects Examples and Fuzz targets", func(t *testing.T) {
		list := ciListPatternRe.FindAllStringSubmatch(workflow, -1)
		require.Len(t, list, 1, "expected exactly one `go test -list` pattern in %s", ciWorkflowPath)
		selects := ciSelectPatternRe.FindAllStringSubmatch(workflow, -1)
		require.Len(t, selects, 1, "expected exactly one awk selection pattern in %s", ciWorkflowPath)

		// -run gates Examples and Fuzz seed corpora exactly as it gates Test
		// functions, so a shard that enumerates only Test names drops both
		// from the race build entirely.
		for _, pattern := range []string{list[0][1], selects[0][1]} {
			re, err := regexp.Compile(pattern)
			require.NoErrorf(t, err, "the shard pattern %q must be a valid regexp", pattern)
			for _, name := range []string{"TestSomething", "FuzzSomething", "ExampleSomething"} {
				require.Truef(t, re.MatchString(name),
					"the shard pattern %q drops %s from every race shard", pattern, name)
			}
		}
	})

	t.Run("the whole-suite step excludes every specialised runner", func(t *testing.T) {
		// The unfiltered `go test ./...` leg belongs to the runners that carry
		// no shard AND no package operand. Gating it on the shard alone would
		// hand the whole suite to the runner that owns ./examples/ as well,
		// which re-runs everything under -race for no coverage at all.
		// Match the COMMAND, never the prose: a step's own comment may discuss
		// `go test ./...` while running something else entirely.
		unfiltered := func(block string) bool {
			for line := range strings.SplitSeq(block, "\n") {
				trimmed := strings.TrimSpace(line)
				if strings.HasPrefix(trimmed, "#") {
					continue
				}
				if strings.Contains(trimmed, "run:") && strings.Contains(trimmed, "go test ") && strings.Contains(trimmed, "./...") {
					return true
				}
			}
			return false
		}

		var found bool
		for block := range strings.SplitSeq(workflow, "- name:") {
			if !unfiltered(block) {
				continue
			}
			found = true
			require.Containsf(t, block, "matrix.shard == ''",
				"the whole-suite step must exclude the sharded runners: %s", strings.TrimSpace(block))
			require.Containsf(t, block, "matrix.packages == ''",
				"the whole-suite step must also exclude the runner that owns the non-root packages, or it re-runs the entire suite there: %s", strings.TrimSpace(block))
		}
		require.True(t, found,
			"no `go test ./...` step was found in %s, so this check asserted nothing", ciWorkflowPath)
	})

	t.Run("every test is named by exactly one shard", func(t *testing.T) {
		assigned := shardAssignment(t)
		listed := listedTestNames(t)

		// The two directions are reported as whole lists rather than one
		// subtest per name: a regeneration is a single action, and seeing every
		// name it would fix at once is what makes the failure actionable.
		var unassigned []string
		for name := range listed {
			if _, ok := assigned[name]; !ok {
				unassigned = append(unassigned, name)
			}
		}
		slices.Sort(unassigned)
		require.Emptyf(t, unassigned,
			"these tests run in no race shard: regenerate %s (see _shardgen/main.go)", ciShardFilePath)

		// The other direction: a renamed or deleted test left behind in the
		// file would quietly shrink a shard's -run regex to names that match
		// nothing, and no exit status would show it.
		var stale []string
		for name := range assigned {
			if _, ok := listed[name]; !ok {
				stale = append(stale, name)
			}
		}
		slices.Sort(stale)
		require.Emptyf(t, stale,
			"%s names these tests, which no longer exist: regenerate it (see _shardgen/main.go)", ciShardFilePath)
	})

	t.Run("the file uses exactly the shards the matrix declares", func(t *testing.T) {
		declared := make(map[string]struct{})
		for _, m := range ciMatrixShardRe.FindAllStringSubmatch(workflow, -1) {
			if m[1] != "" {
				declared[m[1]] = struct{}{}
			}
		}
		require.NotEmpty(t, declared, "no non-empty `shard:` entry was found in %s", ciWorkflowPath)

		used := make(map[string]struct{})
		for _, shard := range shardAssignment(t) {
			used[shard] = struct{}{}
		}

		for shard := range declared {
			_, ok := used[shard]
			require.Truef(t, ok,
				"the matrix declares shard %s but %s assigns it no test, so that runner would run nothing",
				shard, ciShardFilePath)
		}
		for shard := range used {
			_, ok := declared[shard]
			require.Truef(t, ok,
				"%s assigns tests to shard %s, which the matrix in %s declares no runner for, so those tests never run",
				ciShardFilePath, shard, ciWorkflowPath)
		}
	})
}

// shardAssignment reads the recorded test-to-shard mapping. It refuses an empty
// or malformed file rather than returning a partial map, since every check
// above would then pass vacuously.
func shardAssignment(t *testing.T) map[string]string {
	t.Helper()
	raw, err := os.ReadFile(ciShardFilePath)
	require.NoErrorf(t, err, "%s must be readable from the package directory", ciShardFilePath)

	assigned := make(map[string]string)
	for line := range strings.SplitSeq(string(raw), "\n") {
		if line == "" || strings.HasPrefix(line, "#") {
			continue
		}
		fields := strings.Split(line, "\t")
		require.Lenf(t, fields, 3, "malformed line in %s: %q", ciShardFilePath, line)
		_, err := strconv.Atoi(fields[0])
		require.NoErrorf(t, err, "shard column is not a number in %s: %q", ciShardFilePath, line)
		_, seen := assigned[fields[2]]
		require.Falsef(t, seen, "%s assigns %s to more than one shard, so it would run twice", ciShardFilePath, fields[2])
		assigned[fields[2]] = fields[0]
	}
	require.NotEmptyf(t, assigned, "%s assigns no test, so the shards would run nothing", ciShardFilePath)
	return assigned
}

// listedTestNames enumerates the root package's own Test, Fuzz and Example
// names — the same set `-run` gates and the same set the workflow lists.
func listedTestNames(t *testing.T) map[string]struct{} {
	t.Helper()
	cmd := exec.CommandContext(t.Context(), "go", "test", "-list", ".*", ".")
	out, err := cmd.Output()
	require.NoError(t, err, "enumerating the root package's test names")

	names := make(map[string]struct{})
	for line := range strings.SplitSeq(string(out), "\n") {
		line = strings.TrimSpace(line)
		if strings.HasPrefix(line, "Test") || strings.HasPrefix(line, "Fuzz") || strings.HasPrefix(line, "Example") {
			names[line] = struct{}{}
		}
	}
	require.NotEmpty(t, names, "the root package enumerated no test names, so this check asserted nothing")
	return names
}

// goPackageDirs lists every directory holding a .go file, relative to the
// repository root and excluding the root itself. Hidden and underscore-
// prefixed directories are skipped: they hold the git checkout, nested
// worktrees, scratch output and the toolchain's own excluded trees, none of
// which `go list ./...` reaches either.
func goPackageDirs(t *testing.T) []string {
	t.Helper()
	seen := make(map[string]struct{})
	err := filepath.WalkDir(".", func(path string, d fs.DirEntry, err error) error {
		if err != nil {
			return err
		}
		if d.IsDir() {
			if path == "." {
				return nil
			}
			name := d.Name()
			if strings.HasPrefix(name, ".") || strings.HasPrefix(name, "_") || name == "testdata" {
				return fs.SkipDir
			}
			return nil
		}
		if filepath.Ext(path) != ".go" {
			return nil
		}
		dir := filepath.ToSlash(filepath.Dir(path))
		if dir == "." {
			return nil
		}
		seen[dir] = struct{}{}
		return nil
	})
	require.NoError(t, err)
	return slices.Sorted(maps.Keys(seen))
}
