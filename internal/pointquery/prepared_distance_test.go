package pointquery

// CI: isolated-tests

import (
	"context"
	"errors"
	"math/rand/v2"
	"sync"
	"testing"
	"time"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// maxReported bounds how many failing cases a test names.
const maxReported = 5

// cancelAfterContext reports context.Canceled once left calls of Err have
// returned nil.
type cancelAfterContext struct {
	left int
}

func (c *cancelAfterContext) Deadline() (time.Time, bool) { return time.Time{}, false }
func (c *cancelAfterContext) Done() <-chan struct{}       { return nil }
func (c *cancelAfterContext) Value(any) any               { return nil }

func (c *cancelAfterContext) Err() error {
	if c.left <= 0 {
		return context.Canceled
	}
	c.left--
	return nil
}

func TestPreparedDistanceMatchesMeshDistanceSquared(t *testing.T) {
	rng := rand.New(rand.NewPCG(21, 22))
	for _, f := range cleanDistanceFixtures() {
		d := buildDistanceIndex(f.verts, f.tris)
		require.True(t, d.clean, f.name)
		points := distancePoints(f, 300, rng)
		var bad []int
		for i, p := range points {
			want, err := MeshDistanceSquared(t.Context(), p, f.verts, f.tris)
			require.NoError(t, err, f.name)
			got, err := d.distanceSquared(t.Context(), p, f.tris)
			require.NoError(t, err, f.name)
			if want.Cmp(got) != 0 {
				bad = append(bad, i)
			}
		}
		require.Empty(t, bad[:min(len(bad), maxReported)], "%s: %d of %d points differ; first indices", f.name, len(bad), len(points))
	}
}

func TestPreparedDistanceBuildsOnlyForCleanMeshes(t *testing.T) {
	for _, f := range cleanDistanceFixtures() {
		require.NotNil(t, NewPreparedMesh(f.verts, f.tris).cleanDistanceIndex(), f.name)
	}
	for _, f := range uncleanDistanceFixtures() {
		require.Nil(t, NewPreparedMesh(f.verts, f.tris).cleanDistanceIndex(), f.name)
	}
}

func TestPreparedDistanceLargeMeshIsTreeBacked(t *testing.T) {
	f := cleanDistanceFixtures()[2]
	d := buildDistanceIndex(f.verts, f.tris)
	require.Greater(t, len(f.tris), 4*distanceLeafSize)
	require.Greater(t, len(d.nodes), 1)
	require.Len(t, d.order, len(f.tris))
}

func TestPreparedDistanceCancelledContext(t *testing.T) {
	f := cleanDistanceFixtures()[2]
	d := buildDistanceIndex(f.verts, f.tris)
	p := r3.NewVec(20, 20, 20)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	got, err := d.distanceSquared(ctx, p, f.tris)
	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, got)
}

func TestPreparedDistanceCancelDuringSearch(t *testing.T) {
	f := cleanDistanceFixtures()[2]
	d := buildDistanceIndex(f.verts, f.tris)
	// The context stays live for the check before the search, then fails at
	// the first poll after distancePollInterval heap pops. The sphere's centre
	// is equally near every triangle, so the search makes many pops.
	ctx := &cancelAfterContext{left: 1}
	got, err := d.distanceSquared(ctx, r3.NewVec(0.5, 0.25, -1), f.tris)
	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, got)
}

func TestLocatePreparedMatchesLocate(t *testing.T) {
	rng := rand.New(rand.NewPCG(31, 32))
	bounds := [][2]float64{{0, 0}, {0.001, 0}, {0, 0.001}, {0.001, 0.001}, {1e-6, 1e-9}}
	fixtures := append(cleanDistanceFixtures(), parityAcceptedUncleanFixtures()...)
	for _, f := range fixtures {
		prepared := NewPreparedMesh(f.verts, f.tris)
		points := distancePoints(f, 100, rng)
		compared, bad, errored := 0, 0, 0
		seen := map[Location]int{}
		var first []int
		for i, p := range points {
			for _, b := range bounds {
				wantLoc, wantErr := Locate(t.Context(), p, f.verts, f.tris, b[0], b[1])
				gotLoc, gotErr := LocatePrepared(t.Context(), p, prepared, b[0], b[1])
				compared++
				seen[wantLoc]++
				if wantErr != nil {
					errored++
				}
				if wantLoc == gotLoc && errorText(wantErr) == errorText(gotErr) {
					continue
				}
				bad++
				if len(first) < maxReported {
					first = append(first, i)
				}
			}
		}
		require.Zero(t, bad, "%s: %d of %d queries differ; first point indices %v", f.name, bad, compared, first)
		t.Logf("%s: %d queries, %d with an error, results %v", f.name, compared, errored, seen)
	}
}

func TestLocatePreparedCancelledContext(t *testing.T) {
	f := cleanDistanceFixtures()[2]
	prepared := NewPreparedMesh(f.verts, f.tris)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	loc, err := LocatePrepared(ctx, r3.NewVec(20, 20, 20), prepared, 0.001, 0.001)
	require.True(t, errors.Is(err, context.Canceled), "error %v", err)
	require.Equal(t, Undecided, loc)
}

func TestLocatePreparedConcurrent(t *testing.T) {
	f := cleanDistanceFixtures()[2]
	prepared := NewPreparedMesh(f.verts, f.tris)
	points := distancePoints(f, 40, rand.New(rand.NewPCG(41, 42)))
	want := make([]Location, len(points))
	for i, p := range points {
		loc, err := Locate(t.Context(), p, f.verts, f.tris, 0.001, 0.001)
		require.NoError(t, err)
		want[i] = loc
	}
	var wg sync.WaitGroup
	mismatches := make([]int, 4)
	for w := range mismatches {
		wg.Go(func() {
			for i, p := range points {
				loc, err := LocatePrepared(t.Context(), p, prepared, 0.001, 0.001)
				if err != nil || loc != want[i] {
					mismatches[w]++
				}
			}
		})
	}
	wg.Wait()
	require.Equal(t, make([]int, 4), mismatches)
}

func errorText(err error) string {
	if err == nil {
		return ""
	}
	return err.Error()
}
