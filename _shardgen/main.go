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
// output, so adding one moves nothing that is already there, and the shards are
// filled longest-first into whichever is currently lightest — the standard
// greedy fill, which on this suite lands the four shards within a hundredth of
// a second of each other.
//
// Usage:
//
//	GOMAXPROCS=4 go test -race -parallel 1 -timeout 40m -count=1 -json . > costs.jsonl
//	go -C _shardgen run . -costs ../costs.jsonl -out ../.github/test-shards.txt
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
	"slices"
	"strings"
)

// shardCount is how many race shards the workflow declares. ci_workflow_test.go
// asserts this equals the number of shard entries in the matrix, so the two
// cannot drift apart silently.
const shardCount = 4

func main() {
	if err := run(); err != nil {
		fmt.Fprintf(os.Stderr, "shardgen: %s\n", err)
		os.Exit(1)
	}
}

func run() error {
	costsPath := flag.String("costs", "", "go test -json output to read per-test elapsed times from")
	outPath := flag.String("out", "", "shard assignment file to write")
	pkgDir := flag.String("pkg", "..", "directory of the package whose tests are sharded")
	flag.Parse()
	if *costsPath == "" || *outPath == "" {
		return fmt.Errorf("both -costs and -out are required")
	}

	costs, err := readCosts(*costsPath)
	if err != nil {
		return err
	}
	names, err := listTests(*pkgDir)
	if err != nil {
		return err
	}
	if len(names) == 0 {
		return fmt.Errorf("%s enumerates no tests", *pkgDir)
	}

	assigned, totals := pack(names, costs)
	return write(*outPath, assigned, totals, costs)
}

// readCosts reads each top-level test's elapsed time from `go test -json`
// output. Subtest events carry a "/" in their name and are skipped: their time
// is already inside their parent's, and -run selects the parent.
func readCosts(path string) (map[string]float64, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	costs := make(map[string]float64)
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
		if ev.Test == "" || strings.Contains(ev.Test, "/") {
			continue
		}
		switch ev.Action {
		case "pass", "fail", "skip":
			costs[ev.Test] = ev.Elapsed
		}
	}
	return costs, sc.Err()
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

// pack fills the shards longest-first, each test going to whichever shard is
// lightest so far. Ties break on the name so the output is byte-stable: the
// same inputs must always produce the same file, or every regeneration would
// churn the diff.
func pack(names []string, costs map[string]float64) ([][]string, []float64) {
	ordered := slices.Clone(names)
	slices.SortFunc(ordered, func(a, b string) int {
		if c := cmp.Compare(costs[b], costs[a]); c != 0 {
			return c
		}
		return cmp.Compare(a, b)
	})

	assigned := make([][]string, shardCount)
	totals := make([]float64, shardCount)
	for _, name := range ordered {
		lightest := 0
		for i := 1; i < shardCount; i++ {
			if totals[i] < totals[lightest] {
				lightest = i
			}
		}
		assigned[lightest] = append(assigned[lightest], name)
		totals[lightest] += costs[name]
	}
	for i := range assigned {
		slices.Sort(assigned[i])
	}
	return assigned, totals
}

func write(path string, assigned [][]string, totals []float64, costs map[string]float64) error {
	var b strings.Builder
	b.WriteString("# Race-shard assignment for the root package. Generated by _shardgen;\n")
	b.WriteString("# do not hand-edit beyond adding a new test's line.\n")
	b.WriteString("#\n")
	b.WriteString("# Each line is: <shard> <measured seconds> <test name>. The assignment is\n")
	b.WriteString("# explicit so that adding a test moves no test already here, and it is packed\n")
	b.WriteString("# by measured cost so the shards stay level as the suite grows. Regenerate\n")
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
