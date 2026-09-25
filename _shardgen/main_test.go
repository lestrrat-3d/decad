package main

import "testing"

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
