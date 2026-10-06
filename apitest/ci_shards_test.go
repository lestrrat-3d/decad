package apitest_test

import (
	"os"
	"os/exec"
	"regexp"
	"slices"
	"strconv"
	"strings"
	"testing"

	"github.com/stretchr/testify/require"
)

// Regression guard for apitest's race shards in .github/workflows/ci.yml.
//
// Each shard runner runs its shard of this package's test binary, picking the
// names .github/test-shards-apitest.txt assigns to it. The root package's
// ci_workflow_test.go checks the workflow itself and the root package's own
// assignment. It cannot list this package's tests, because it runs inside the
// root package's binary, so this test checks this package's assignment from
// this package's binary:
//
//   - the shard step runs apitest.test from apitest/ against this file;
//   - every test here is named by exactly one shard, so none is dropped;
//   - the file names no test that has gone away, so no shard's -run regex
//     shrinks to names that match nothing.
//
// The parsing below repeats what ci_workflow_test.go does for the root
// package's file. The two test packages cannot share a helper without a
// non-test package for it.
const (
	apitestShardFilePath = "../.github/test-shards-apitest.txt"
	apitestWorkflowPath  = "../.github/workflows/ci.yml"
)

func TestCIShardsCoverEveryAPITest(t *testing.T) {
	t.Parallel()

	t.Run("the shard step runs this package against its file", func(t *testing.T) {
		raw, err := os.ReadFile(apitestWorkflowPath)
		require.NoError(t, err, "the CI workflow must be readable from the package directory")
		re := regexp.MustCompile(`(?m)^\s*run_shard apitest\.test \.github/test-shards-apitest\.txt apitest\s*$`)
		require.Len(t, re.FindAllString(string(raw), -1), 1,
			"%s must run apitest.test from apitest/ against .github/test-shards-apitest.txt exactly once", apitestWorkflowPath)
	})

	t.Run("every test is named by exactly one shard", func(t *testing.T) {
		assigned := apitestShardAssignment(t)
		listed := apitestListedNames(t)

		var unassigned []string
		for name := range listed {
			if _, ok := assigned[name]; !ok {
				unassigned = append(unassigned, name)
			}
		}
		slices.Sort(unassigned)
		require.Emptyf(t, unassigned,
			"these tests run in no race shard: regenerate %s (see _shardgen/main.go)", apitestShardFilePath)

		var stale []string
		for name := range assigned {
			if _, ok := listed[name]; !ok {
				stale = append(stale, name)
			}
		}
		slices.Sort(stale)
		require.Emptyf(t, stale,
			"%s names these tests, which no longer exist: regenerate it (see _shardgen/main.go)", apitestShardFilePath)
	})
}

// apitestShardAssignment reads the recorded test-to-shard mapping. It refuses
// an empty or malformed file rather than returning a partial map.
func apitestShardAssignment(t *testing.T) map[string]string {
	t.Helper()
	raw, err := os.ReadFile(apitestShardFilePath)
	require.NoErrorf(t, err, "%s must be readable from the package directory", apitestShardFilePath)

	assigned := make(map[string]string)
	for line := range strings.SplitSeq(string(raw), "\n") {
		if line == "" || strings.HasPrefix(line, "#") {
			continue
		}
		fields := strings.Split(line, "\t")
		require.Lenf(t, fields, 3, "malformed line in %s: %q", apitestShardFilePath, line)
		_, err := strconv.Atoi(fields[0])
		require.NoErrorf(t, err, "shard column is not a number in %s: %q", apitestShardFilePath, line)
		_, seen := assigned[fields[2]]
		require.Falsef(t, seen, "%s assigns %s to more than one shard, so it would run twice", apitestShardFilePath, fields[2])
		assigned[fields[2]] = fields[0]
	}
	require.NotEmptyf(t, assigned, "%s assigns no test, so the shards would run nothing", apitestShardFilePath)
	return assigned
}

// apitestListedNames enumerates this package's Test, Fuzz and Example names
// from this test binary, as the workflow does.
func apitestListedNames(t *testing.T) map[string]struct{} {
	t.Helper()
	executable, err := os.Executable()
	require.NoError(t, err, "finding the running test binary")
	out, err := exec.CommandContext(t.Context(), executable, "-test.list", ".*").Output()
	require.NoError(t, err, "enumerating this package's test names")

	names := make(map[string]struct{})
	for line := range strings.SplitSeq(string(out), "\n") {
		line = strings.TrimSpace(line)
		if strings.HasPrefix(line, "Test") || strings.HasPrefix(line, "Fuzz") || strings.HasPrefix(line, "Example") {
			names[line] = struct{}{}
		}
	}
	require.NotEmpty(t, names, "this package enumerated no test names, so this check asserted nothing")
	return names
}
