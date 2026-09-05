package decad_test

import (
	"io/fs"
	"maps"
	"os"
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
// approved modules (CLAUDE.md) and gains none for a guard — and asserts four
// properties, each of which fails loudly rather than degrading quietly:
//
//   - no `go test` command combines a `-run` filter with `./...`;
//   - every Go package directory on disk outside the root is named by some
//     shard's own `packages:` operand, so adding a package fails this test
//     until the workflow covers it;
//   - the shard enumeration selects Examples and Fuzz targets, not Test
//     functions alone, since `-run` gates all three;
//   - the shard modulus equals the number of shard entries in the matrix, so
//     adding a third shard without widening the modulus cannot silently drop
//     a third of the suite.
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
	// ciShardModulusRe reads the awk modulus the shard index is taken over.
	ciShardModulusRe = regexp.MustCompile(`count % (\d+) == shard`)
)

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

	t.Run("the shard modulus equals the shard count", func(t *testing.T) {
		var shards int
		for _, m := range ciMatrixShardRe.FindAllStringSubmatch(workflow, -1) {
			if m[1] != "" {
				shards++
			}
		}
		require.Positive(t, shards, "no non-empty `shard:` entry was found in %s", ciWorkflowPath)

		modulus := ciShardModulusRe.FindAllStringSubmatch(workflow, -1)
		require.Len(t, modulus, 1, "expected exactly one shard modulus in %s", ciWorkflowPath)
		n, err := strconv.Atoi(modulus[0][1])
		require.NoError(t, err)
		require.Equalf(t, shards, n,
			"%d shard entries split their names over %d buckets, so %d bucket(s) run on no runner", shards, n, shards-n)
	})
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
