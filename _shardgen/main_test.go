package main

import (
	"bytes"
	"encoding/json"
	"io"
	"math"
	"os"
	"os/exec"
	"path/filepath"
	"testing"
)

func TestReadCostsAccountsForParallelChildren(t *testing.T) {
	cmd := exec.Command("go", "test", "-parallel", "1", "-count=1", "-json", ".")
	cmd.Dir = filepath.Join("testdata", "parallel_cost")
	output, err := cmd.CombinedOutput()
	if err != nil {
		t.Fatalf("go test -json: %v\n%s", err, output)
	}

	path := filepath.Join(t.TempDir(), "costs.jsonl")
	if err := os.WriteFile(path, output, 0o600); err != nil {
		t.Fatal(err)
	}
	costs, err := readCosts(path)
	if err != nil {
		t.Fatal(err)
	}

	elapsed := make(map[string]float64)
	dec := json.NewDecoder(bytes.NewReader(output))
	for {
		var ev struct {
			Action  string
			Test    string
			Elapsed float64
		}
		if err := dec.Decode(&ev); err != nil {
			if err == io.EOF {
				break
			}
			t.Fatal(err)
		}
		if ev.Action == "pass" {
			elapsed[ev.Test] = ev.Elapsed
		}
	}

	const root = "TestCostHierarchy"
	direct := root + "/direct/parallel"
	grandchild := direct + "/parallel/grandchild"
	want := elapsed[root] + elapsed[direct] + elapsed[grandchild]
	if got := costs[root]; math.Abs(got-want) > 1e-9 {
		t.Fatalf("cost = %.2fs, want %.2fs from the parent and its directly paused descendants", got, want)
	}
	if want <= elapsed[root] {
		t.Fatalf("parallel children did not add cost: parent %.2fs, combined %.2fs", elapsed[root], want)
	}
	if len(costs) != 1 {
		t.Fatalf("got %d top-level costs, want 1: %v", len(costs), costs)
	}
}

func TestPackKeepsChordSweepReadersTogether(t *testing.T) {
	names := []string{
		"TestChordedBoundaryVolumeAllowEnclosesTheMeasuredGap",
		"TestChordedBoundaryVolumeAllowSeamLegDeletionSearch",
		"TestChordedBoundaryVolumeAllowWallLegDeletionSearch",
		"TestOtherA", "TestOtherB", "TestOtherC",
	}
	costs := map[string]float64{
		names[0]: 7,
		names[1]: 6,
		names[2]: 5,
		names[3]: 10,
		names[4]: 9,
		names[5]: 8,
	}

	assigned, totals := pack(names, costs)
	groupedShards := 0
	for shard, tests := range assigned {
		found := 0
		for _, name := range tests {
			for _, reader := range chordSweepReaders {
				if name == reader {
					found++
				}
			}
		}
		if found != 0 && found != len(chordSweepReaders) {
			t.Fatalf("shard %d has %d of %d chord sweep readers: %v", shard, found, len(chordSweepReaders), tests)
		}
		if found == len(chordSweepReaders) {
			groupedShards++
			if totals[shard] != 18 {
				t.Fatalf("grouped shard cost = %g, want 18", totals[shard])
			}
		}
	}
	if groupedShards != 1 {
		t.Fatalf("%d shards contain the chord sweep readers, want 1", groupedShards)
	}
}
