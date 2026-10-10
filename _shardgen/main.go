// Command shardgen writes the race-shard assignment CI reads.
//
// The shards exist to bound the race job's wall time, and how a test is
// assigned to one decides whether that works. Assigning by a test's POSITION in
// the enumerated list does not: adding a test shifts every name after it, so
// half of them change shard, and a suite whose shards were balanced yesterday
// is re-rolled by the next commit that adds an odd number of tests. That is how
// one shard reached 564s of its 600s timeout while its partner sat at 425s.
//
// This packs by MEASURED cost instead. Every test is named explicitly in the
// output. Tests sharing the costly chordSweepTable fixture are packed as one
// unit, so one test binary builds that table. The other tests are packed
// individually by the combined cost of both binaries on each shard.
//
// CI shards two test binaries, the root package's and apitest's, and each has
// its own assignment file. Both files use the same shard numbers, and every
// shard runner runs its shard of both binaries. Measure both packages, then
// regenerate both assignment files together:
//
//	GOMAXPROCS=4 go test -race -parallel 1 -timeout 40m -count=1 -json . > .tmp/costs-root.jsonl
//	GOMAXPROCS=4 go test -race -parallel 1 -timeout 40m -count=1 -json ./apitest/ > .tmp/costs-apitest.jsonl
//	go -C _shardgen run . -root-costs ../.tmp/costs-root.jsonl -apitest-costs ../.tmp/costs-apitest.jsonl \
//		-root-out ../.github/test-shards.txt -apitest-out ../.github/test-shards-apitest.txt
//
// Both flags matter. GOMAXPROCS=4 matches the CI runner. -parallel 1 is what
// makes the figures comparable: the suite runs its tests concurrently, and a
// test measured while three others share the machine records their contention
// as its own cost. Serialising the measurement gives each test its own weight,
// which is what the fill needs to balance them.
//
// A test the cost file does not mention is recorded at zero and packed anyway,
// so a newly added test lands in the lightest shard rather than being dropped;
// re-measure to give it its real weight.
package main

import (
	"bufio"
	"cmp"
	"encoding/json"
	"flag"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"slices"
	"strings"
)

// shardCount is how many race shards the workflow declares. ci_workflow_test.go
// asserts this equals the number of shard entries in the matrix, so the two
// cannot drift apart silently.
const shardCount = 10

// These tests read the same costly sync.OnceValue table. Putting them in one
// shard lets the test binary build that table once.
var chordSweepReaders = []string{
	"TestChordedWallAndTwistLegsEncloseTheMeasuredGap",
	"TestChordedWallLegIsLoadBearing",
}

func main() {
	if err := run(); err != nil {
		fmt.Fprintf(os.Stderr, "shardgen: %s\n", err)
		os.Exit(1)
	}
}

func run() error {
	rootCostsPath := flag.String("root-costs", "", "root package go test -json output")
	apitestCostsPath := flag.String("apitest-costs", "", "apitest go test -json output")
	rootOutPath := flag.String("root-out", "", "root package assignment file")
	apitestOutPath := flag.String("apitest-out", "", "apitest assignment file")
	flag.Parse()
	if *rootCostsPath == "" || *apitestCostsPath == "" || *rootOutPath == "" || *apitestOutPath == "" {
		return fmt.Errorf("root and apitest cost and output paths are required")
	}

	rootCosts, err := readCosts(*rootCostsPath)
	if err != nil {
		return err
	}
	apitestCosts, err := readCosts(*apitestCostsPath)
	if err != nil {
		return err
	}
	rootNames, err := listTests("..")
	if err != nil {
		return err
	}
	apitestNames, err := listTests("../apitest")
	if err != nil {
		return err
	}
	if len(rootNames) == 0 || len(apitestNames) == 0 {
		return fmt.Errorf("both packages must enumerate tests")
	}

	assigned, totals, err := pack(rootNames, rootCosts, apitestNames, apitestCosts)
	if err != nil {
		return err
	}
	if err := write(*rootOutPath, "..", assigned[0], totals[0], rootCosts); err != nil {
		return err
	}
	return write(*apitestOutPath, "../apitest", assigned[1], totals[1], apitestCosts)
}

// readCosts reads each top-level test's cost from `go test -json` output.
// A parent's elapsed time includes sequential subtests but excludes children
// paused by t.Parallel. Add each direct parallel child's cost recursively;
// adding a parallel descendant through a sequential child would count it twice.
func readCosts(path string) (map[string]float64, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	type testCost struct {
		children []*testCost
		elapsed  float64
		paused   bool
	}
	nodes := make(map[string]*testCost)
	roots := make(map[string]*testCost)
	var active []*testCost
	sc := bufio.NewScanner(f)
	sc.Buffer(make([]byte, 0, 1<<20), 1<<24)
	for sc.Scan() {
		var ev struct {
			Action  string
			Test    string
			Elapsed float64
		}
		if err := json.Unmarshal(sc.Bytes(), &ev); err != nil {
			continue // a non-JSON line is build output, not a test event
		}
		if ev.Test == "" {
			continue
		}
		switch ev.Action {
		case "run":
			node := &testCost{}
			nodes[ev.Test] = node
			if len(active) == 0 {
				roots[ev.Test] = node
			} else {
				parent := active[len(active)-1]
				parent.children = append(parent.children, node)
			}
			active = append(active, node)
		case "pause":
			if node := nodes[ev.Test]; node != nil {
				node.paused = true
			}
			if len(active) != 0 {
				active = active[:len(active)-1]
			}
		case "cont":
			if node := nodes[ev.Test]; node != nil {
				active = append(active, node)
			}
		case "pass", "fail", "skip":
			if node := nodes[ev.Test]; node != nil {
				node.elapsed = ev.Elapsed
			} else if !strings.Contains(ev.Test, "/") {
				// Keep accepting a top-level result from a partial event log.
				roots[ev.Test] = &testCost{elapsed: ev.Elapsed}
			}
			if len(active) != 0 {
				active = active[:len(active)-1]
			}
		}
	}
	if err := sc.Err(); err != nil {
		return nil, err
	}

	var cost func(*testCost) float64
	cost = func(node *testCost) float64 {
		total := node.elapsed
		for _, child := range node.children {
			if child.paused {
				total += cost(child)
			}
		}
		return total
	}
	costs := make(map[string]float64, len(roots))
	for name, root := range roots {
		costs[name] = cost(root)
	}
	return costs, nil
}

// listTests enumerates the package's Test, Fuzz and Example names, which is the
// same set -run gates. It shells out to `go test -list` so the enumeration can
// never disagree with what the workflow itself lists.
func listTests(dir string) ([]string, error) {
	cmd := exec.Command("go", "test", "-list", ".*", ".")
	cmd.Dir = dir
	cmd.Stderr = os.Stderr
	out, err := cmd.Output()
	if err != nil {
		return nil, fmt.Errorf("listing tests in %s: %w", dir, err)
	}
	var names []string
	for line := range strings.SplitSeq(string(out), "\n") {
		line = strings.TrimSpace(line)
		if strings.HasPrefix(line, "Test") || strings.HasPrefix(line, "Fuzz") || strings.HasPrefix(line, "Example") {
			names = append(names, line)
		}
	}
	return names, nil
}

type testUnit struct {
	pkg   int
	names []string
	cost  float64
}

func testUnits(pkg int, names []string, costs map[string]float64) []testUnit {
	var grouped testUnit
	var units []testUnit
	for _, name := range names {
		if pkg == 0 && slices.Contains(chordSweepReaders, name) {
			grouped.pkg = pkg
			grouped.names = append(grouped.names, name)
			grouped.cost += costs[name]
			continue
		}
		units = append(units, testUnit{pkg: pkg, names: []string{name}, cost: costs[name]})
	}
	if len(grouped.names) != 0 {
		slices.Sort(grouped.names)
		units = append(units, grouped)
	}
	return units
}

// pack balances the cost of both binaries on each shard. A single test whose
// cost exceeds half the equal-share target gets a shard with only one cheap
// test from the other package. This leaves room for slower CI runners. Every
// shard receives at least one test from each package because CI runs both
// binaries on every shard. Name ties keep the output byte-stable.
func pack(rootNames []string, rootCosts map[string]float64, apitestNames []string, apitestCosts map[string]float64) (
	[2][][]string, [2][]float64, error,
) {
	units := append(testUnits(0, rootNames, rootCosts), testUnits(1, apitestNames, apitestCosts)...)
	slices.SortFunc(units, func(a, b testUnit) int {
		if c := cmp.Compare(b.cost, a.cost); c != 0 {
			return c
		}
		if c := cmp.Compare(a.pkg, b.pkg); c != 0 {
			return c
		}
		return cmp.Compare(a.names[0], b.names[0])
	})
	var total float64
	for _, u := range units {
		total += u.cost
	}
	assigned := [2][][]string{make([][]string, shardCount), make([][]string, shardCount)}
	totals := [2][]float64{make([]float64, shardCount), make([]float64, shardCount)}
	assign := func(shard int, u testUnit) {
		assigned[u.pkg][shard] = append(assigned[u.pkg][shard], u.names...)
		totals[u.pkg][shard] += u.cost
	}

	firstRegularShard := 0
	if len(units) != 0 && units[0].cost > total/(2*shardCount) {
		assign(0, units[0])
		units = units[1:]
		firstRegularShard = 1
	}

	// Seed both binaries on every shard with their cheapest remaining tests.
	// A reserved shard receives no further work after this step.
	for shard := range shardCount {
		for pkg := range 2 {
			if len(assigned[pkg][shard]) != 0 {
				continue
			}
			index := -1
			for i := len(units) - 1; i >= 0; i-- {
				if units[i].pkg == pkg {
					index = i
					break
				}
			}
			if index < 0 {
				return assigned, totals, fmt.Errorf("package %d has fewer than %d test groups", pkg, shardCount)
			}
			assign(shard, units[index])
			units = slices.Delete(units, index, index+1)
		}
	}

	for _, u := range units {
		lightest := firstRegularShard
		for shard := firstRegularShard + 1; shard < shardCount; shard++ {
			// Rounded zero-second tests still consume runner overhead.
			if u.cost == 0 {
				count := len(assigned[0][shard]) + len(assigned[1][shard])
				leastCount := len(assigned[0][lightest]) + len(assigned[1][lightest])
				if count < leastCount {
					lightest = shard
				}
				continue
			}
			if totals[0][shard]+totals[1][shard] < totals[0][lightest]+totals[1][lightest] {
				lightest = shard
			}
		}
		assign(lightest, u)
	}
	for pkg := range 2 {
		for shard := range shardCount {
			slices.Sort(assigned[pkg][shard])
		}
	}
	return assigned, totals, nil
}

// write records the assignment for the package in pkgDir. The header names the
// package by its directory relative to the repository root, which is pkgDir
// relative to _shardgen. It mentions chordSweepTable only for a package whose
// tests read it.
func write(path, pkgDir string, assigned [][]string, totals []float64, costs map[string]float64) error {
	pkg := "the root package"
	if rel := strings.Trim(strings.TrimPrefix(filepath.ToSlash(pkgDir), ".."), "/"); rel != "" {
		pkg = rel + "/"
	}
	var chord bool
	for _, names := range assigned {
		for _, name := range names {
			chord = chord || slices.Contains(chordSweepReaders, name)
		}
	}
	var b strings.Builder
	if chord {
		fmt.Fprintf(&b, "# Race-shard assignment for %s. Generated by _shardgen;\n", pkg)
		b.WriteString("# keep tests sharing chordSweepTable in one shard.\n")
	} else {
		fmt.Fprintf(&b, "# Race-shard assignment for %s. Generated by _shardgen.\n", pkg)
	}
	b.WriteString("#\n")
	b.WriteString("# Each line is: <shard> <measured seconds> <test name>. The assignment is\n")
	b.WriteString("# explicit so that adding a test moves no test already here, and it is packed\n")
	if chord {
		b.WriteString("# by measured cost so the shards stay level as the suite grows. Tests\n")
		b.WriteString("# sharing chordSweepTable are packed together. Regenerate\n")
	} else {
		b.WriteString("# by measured cost so the shards stay level as the suite grows. Regenerate\n")
	}
	b.WriteString("# with the command in _shardgen/main.go's doc comment.\n")
	b.WriteString("#\n")
	b.WriteString("# Shard totals at the last measurement:\n")
	for i, t := range totals {
		fmt.Fprintf(&b, "#   shard %d: %7.2fs over %d tests\n", i, t, len(assigned[i]))
	}
	b.WriteString("#\n")
	for i, names := range assigned {
		for _, name := range names {
			fmt.Fprintf(&b, "%d\t%.2f\t%s\n", i, costs[name], name)
		}
	}
	return os.WriteFile(path, []byte(b.String()), 0o644)
}
