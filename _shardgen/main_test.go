package main

import (
	"bytes"
	"encoding/json"
	"fmt"
	"io"
	"math"
	"os"
	"os/exec"
	"path/filepath"
	"reflect"
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
		"TestChordedWallAndTwistLegsEncloseTheMeasuredGap",
		"TestChordedWallLegIsLoadBearing",
	}
	costs := map[string]float64{
		names[0]: 7,
		names[1]: 6,
	}
	var apiNames []string
	apiCosts := make(map[string]float64)
	for i := range 10 {
		name := fmt.Sprintf("TestOther%02d", i)
		names = append(names, name)
		costs[name] = float64(10 - i)
		apiName := fmt.Sprintf("TestAPI%02d", i)
		apiNames = append(apiNames, apiName)
		apiCosts[apiName] = 1
	}

	assigned, totals, err := pack(names, costs, apiNames, apiCosts)
	if err != nil {
		t.Fatal(err)
	}
	groupedShards := 0
	for shard, tests := range assigned[0] {
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
			if totals[0][shard] < 13 {
				t.Fatalf("grouped shard cost = %g, want at least 13", totals[0][shard])
			}
		}
	}
	if groupedShards != 1 {
		t.Fatalf("%d shards contain the chord sweep readers, want 1", groupedShards)
	}
}

func TestPackIsolatesCostlyTestAndCoversBothPackages(t *testing.T) {
	rootCosts := make(map[string]float64)
	apiCosts := make(map[string]float64)
	var rootNames, apiNames []string
	for i := range 12 {
		rootName := fmt.Sprintf("TestRoot%02d", i)
		apiName := fmt.Sprintf("TestAPI%02d", i)
		rootNames = append(rootNames, rootName)
		apiNames = append(apiNames, apiName)
		rootCosts[rootName] = 2
		apiCosts[apiName] = 2
	}
	apiNames = append(apiNames, "TestCostlyGear")
	apiCosts["TestCostlyGear"] = 100

	assigned, totals, err := pack(rootNames, rootCosts, apiNames, apiCosts)
	if err != nil {
		t.Fatal(err)
	}
	if len(assigned[0][0]) != 1 || len(assigned[1][0]) != 1 || assigned[1][0][0] != "TestCostlyGear" {
		t.Fatalf("costly test's shard contains extra work: root %v, api %v", assigned[0][0], assigned[1][0])
	}
	if totals[0][0] != 2 || totals[1][0] != 100 {
		t.Fatalf("costly shard totals = %g + %g, want 2 + 100", totals[0][0], totals[1][0])
	}
	for pkg, names := range [2][]string{rootNames, apiNames} {
		seen := make(map[string]int)
		for shard, tests := range assigned[pkg] {
			if len(tests) == 0 {
				t.Fatalf("package %d has no test on shard %d", pkg, shard)
			}
			for _, name := range tests {
				seen[name]++
			}
		}
		for _, name := range names {
			if seen[name] != 1 {
				t.Fatalf("package %d test %q appears %d times", pkg, name, seen[name])
			}
		}
	}

	again, _, err := pack(rootNames, rootCosts, apiNames, apiCosts)
	if err != nil {
		t.Fatal(err)
	}
	if !reflect.DeepEqual(assigned, again) {
		t.Fatalf("same costs gave different assignments")
	}
}

func TestPackSpreadsZeroCostTests(t *testing.T) {
	var rootNames, apiNames []string
	for i := range 60 {
		rootNames = append(rootNames, fmt.Sprintf("TestRoot%02d", i))
		apiNames = append(apiNames, fmt.Sprintf("TestAPI%02d", i))
	}
	assigned, _, err := pack(rootNames, nil, apiNames, nil)
	if err != nil {
		t.Fatal(err)
	}
	for shard := range shardCount {
		count := len(assigned[0][shard]) + len(assigned[1][shard])
		want := (len(rootNames) + len(apiNames)) / shardCount
		if count != want {
			t.Fatalf("shard %d has %d zero-cost tests, want %d", shard, count, want)
		}
	}
}
