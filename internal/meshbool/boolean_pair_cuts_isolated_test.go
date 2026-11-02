package meshbool

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// referenceEnds replays the contacts of every facet pair and returns the key
// set of the segment endpoints (the old `ends` map) and the point contacts'
// keys with their facet pairs, so a test can restate the old isolated-point
// check independently of pairCuts.
func referenceEnds(t *testing.T, ma, mb *BoolMesh, memo *ContactMemo) (map[string]struct{}, []string) {
	t.Helper()
	keyOf := newKeyMemo()
	ends := map[string]struct{}{}
	var points []string
	for i := range ma.Tris {
		for j := range mb.Tris {
			c, err := memo.Classify(i, j)
			require.NoError(t, err)
			switch c.Kind {
			case ContactPoint:
				points = append(points, keyOf(c.P0))
			case ContactSegment:
				ends[keyOf(c.P0)] = struct{}{}
				ends[keyOf(c.P1)] = struct{}{}
			}
		}
	}
	return ends, points
}

// TestPairCutsIsolatedPointMatchesSegmentEnds pins that reading the rim map
// answers the isolated-point question exactly as the old rebuilt endpoint set
// did: every segment endpoint is a rim key under the same key function.
func TestPairCutsIsolatedPointMatchesSegmentEnds(t *testing.T) {
	identity := func(v r3.Vec) r3.Vec { return v }
	// tilt turns a unit box by integer 3-4-5 rotations about z then x, scaled
	// by 25, so every vertex stays an exact integer.
	tilt := func(v r3.Vec) r3.Vec {
		x, y, z := 3*v.X-4*v.Y, 4*v.X+3*v.Y, v.Z*5
		return r3.Vec{X: x * 5, Y: 3*y - 4*z, Z: 4*y + 3*z}
	}
	cases := []struct {
		name         string
		half         float64
		at           r3.Vec
		rot          func(r3.Vec) r3.Vec
		wantIsolated bool
		wantShared   bool
	}{
		{"isolated corner touch", 1, r3.Vec{X: 2, Y: 2, Z: 2}, identity, true, false},
		{"point on a crossing segment", 10, r3.Vec{X: -10, Y: -10, Z: 10}, tilt, false, true},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			a := boxMesh(t, r3.Vec{X: tc.half, Y: tc.half, Z: tc.half}, identity, r3.Vec{})
			b := boxMesh(t, r3.Vec{X: 1, Y: 1, Z: 1}, tc.rot, tc.at)
			memo := NewContactMemo(a, b)
			ends, points := referenceEnds(t, a, b, memo)

			isolated, shared := 0, 0
			for _, k := range points {
				if _, ok := ends[k]; ok {
					shared++
				} else {
					isolated++
				}
			}
			require.Equal(t, tc.wantIsolated, isolated > 0, "reference isolated count %d", isolated)
			require.Equal(t, tc.wantShared, shared > 0, "reference shared count %d", shared)

			res, err := pairCuts(t.Context(), a, b, NewContactMemo(a, b), newKeyMemo())
			if isolated > 0 {
				touch, ok := errors.AsType[*TouchContactError](err)
				require.True(t, ok, "want a point refusal, got %v", err)
				require.Equal(t, touchPoint, touch.What)
				require.Len(t, touch.Contacts, min(isolated, maxTouchContacts))
				return
			}
			require.NoError(t, err)
			rimKeys := map[string]struct{}{}
			for k := range res.Rims {
				rimKeys[k] = struct{}{}
			}
			require.Equal(t, ends, rimKeys)
		})
	}
}

// TestMeshBooleanJoinsBothSides pins that the result is operand A's kept
// facets followed by operand B's, in order, and that joining them leaves the
// per-side lists a caller recomputes unchanged.
func TestMeshBooleanJoinsBothSides(t *testing.T) {
	identity := func(v r3.Vec) r3.Vec { return v }
	a := boxMesh(t, r3.Vec{X: 1, Y: 1, Z: 1}, identity, r3.Vec{})
	b := boxMesh(t, r3.Vec{X: 1, Y: 1, Z: 1}, identity, r3.Vec{X: .5, Y: .5, Z: .5})

	keyOf := newKeyMemo()
	cuts, err := pairCuts(t.Context(), a, b, NewContactMemo(a, b), keyOf)
	require.NoError(t, err)
	keepA, err := keepSide(t.Context(), a, b, cuts.CutsA, cuts.BlockedA, cuts.Rims, false, false, keyOf)
	require.NoError(t, err)
	keepB, err := keepSide(t.Context(), b, a, cuts.CutsB, cuts.BlockedB, cuts.Rims, false, false, keyOf)
	require.NoError(t, err)
	require.NotEmpty(t, keepA)
	require.NotEmpty(t, keepB)
	srcA := make([]int, len(keepA))
	for i, k := range keepA {
		srcA[i] = k.Src
	}

	got, _, err := meshBoolean(t.Context(), OpUnion, a, b, NewContactMemo(a, b), newKeyMemo)
	require.NoError(t, err)
	require.Len(t, got, len(keepA)+len(keepB))
	for i, k := range keepA {
		require.Equal(t, k.Keys, got[i].Keys, "A facet %d", i)
		require.Equal(t, k.Src, got[i].Src)
		require.Equal(t, srcA[i], keepA[i].Src, "keepA changed at %d", i)
	}
	for i, k := range keepB {
		require.Equal(t, k.Keys, got[len(keepA)+i].Keys, "B facet %d", i)
		require.Equal(t, k.Src, got[len(keepA)+i].Src)
	}
}
