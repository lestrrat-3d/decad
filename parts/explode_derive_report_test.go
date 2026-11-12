package parts_test

import (
	"math"
	"sync/atomic"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/parts"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// cylinderAt extrudes a circle about the vertical line through (cx, cy).
func cylinderAt(t testing.TB, doc *decad.Document, cx, cy, radius, z0, height float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z0)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(cx, cy)
	s.Fix(c)
	s.CreateCircle(c, radius)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	b, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	require.NoError(t, err)
	return b
}

func findBlocked(ex *parts.Exploded, mover, blocker string) (parts.BlockedPair, bool) {
	for _, b := range ex.Blocked {
		if b.Mover == mover && b.Blocker == blocker {
			return b, true
		}
	}
	return parts.BlockedPair{}, false
}

func TestDerivedReportFields(t *testing.T) {
	ex, err := collarSet(t).Explode(t.Context(), parts.WithDerivedExplode(), parts.WithExplodeGround(nameCarrier))
	require.NoError(t, err)
	require.True(t, ex.Derived)
	bearing, shaft, carrier := partByName(t, ex, nameBearing), partByName(t, ex, nameShaft), partByName(t, ex, nameCarrier)
	require.Equal(t, [2]int{1, 1}, [2]int{bearing.Order, bearing.Round})
	require.Equal(t, [2]int{2, 2}, [2]int{shaft.Order, shaft.Round})
	require.Equal(t, [2]int{0, 0}, [2]int{carrier.Order, carrier.Round})
	// The joints' cylinder faces give the axis both parts leave by.
	require.Equal(t, parts.AxisFromFaces, bearing.Pull)
	require.Equal(t, parts.AxisFromFaces, shaft.Pull)

	// The collar stands over the bearing: the shaft is blocked by the bearing, and by nothing else.
	require.Len(t, ex.Blocked, 1)
	pair, ok := findBlocked(ex, nameShaft, nameBearing)
	require.True(t, ok)
	require.InDelta(t, -1, pair.Dir.Z, 1e-9)
	require.Greater(t, pair.Height.Base(), 0.0)
	require.InDelta(t, 12, pair.Point.Z, 1.0, "the witness is on the collar's underside")
	require.Greater(t, math.Hypot(pair.Point.X-10, pair.Point.Y-10), 2.0, "the witness is outside the stem")
}

func TestDerivedReportSources(t *testing.T) {
	// No joints: a free part leaves along a world axis.
	ex, err := pressFitSet(t).Explode(t.Context(), parts.WithDerivedExplode(), parts.WithExplodeGround(namePlate))
	require.NoError(t, err)
	pin := partByName(t, ex, namePin)
	require.Equal(t, parts.WorldAxis, pin.Pull)
	require.Equal(t, 1, pin.Order)
	// The gap is below twice the mesh bound, so the pair is read as free and reported.
	require.Contains(t, ex.Undecided, parts.UndecidedPair{Mover: namePin, Other: namePlate, Dir: pin.Dir})
	require.Empty(t, ex.Blocked)

	// A stated direction is reported as such.
	ex, err = collarSet(t).Explode(t.Context(), parts.WithDerivedExplode(), parts.WithExplodeGround(nameCarrier),
		parts.WithRemoval(nameBearing, parts.Removal{Direction: r3.Vec{Z: -1}}))
	require.NoError(t, err)
	require.Equal(t, parts.RemovalDirection, partByName(t, ex, nameBearing).Pull)
	require.Equal(t, parts.AxisFromFaces, partByName(t, ex, nameShaft).Pull)
}

func TestDerivedReportZeroWithoutDerived(t *testing.T) {
	ex, err := collarSet(t).Explode(t.Context(), parts.WithExplodeGround(nameCarrier))
	require.NoError(t, err)
	require.False(t, ex.Derived)
	require.Empty(t, ex.Blocked)
	require.Empty(t, ex.Undecided)
	for _, p := range ex.Parts {
		require.Equal(t, [2]int{0, 0}, [2]int{p.Order, p.Round}, p.Name)
	}
}

// boltRingSet is a plate with n bolt holes on a ring and one identical bolt in each.
func boltRingSet(t testing.TB, n int) *parts.Set {
	t.Helper()
	doc := decad.New()
	plate := block(t, doc, -30, -30, 30, 30, 0, 6)
	s := parts.NewSet(doc)
	var bolts []parts.Part
	for k := range n {
		a := 2 * math.Pi * float64(k) / float64(n)
		x, y := 20*math.Cos(a), 20*math.Sin(a)
		plate = must(decad.Cut(t.Context(), plate, cylinderAt(t, doc, x, y, 2.5, -1, 8)))
		head := cylinderAt(t, doc, x, y, 4, 6, 3)
		shank := cylinderAt(t, doc, x, y, 2, -4, 10)
		bolts = append(bolts, parts.Part{Name: boltName(k), Kind: parts.Purchased, Body: must(decad.Union(t.Context(), head, shank))})
	}
	mustAdd(t, s, parts.Part{Name: namePlate, Kind: parts.Manufactured, Body: plate})
	mustAdd(t, s, bolts...)
	return s
}

func boltName(k int) string { return "bolt" + string(rune('A'+k)) }

func TestDerivedSharesProjectionsOfIdenticalBodies(t *testing.T) {
	const n = 8
	var built atomic.Int64
	ex, err := boltRingSet(t, n).Explode(t.Context(), parts.WithDerivedExplode(), parts.WithExplodeGround(namePlate),
		parts.CountProjections(&built))
	require.NoError(t, err)
	// Two shapes (plate, bolt) in each of the six world directions; unshared it would be (n+1) x 6.
	require.Equal(t, int64(2*6), built.Load())
	// Sharing changes no answer: every bolt leaves up in round 1.
	for k := range n {
		b := partByName(t, ex, boltName(k))
		require.Equal(t, 1, b.Round, b.Name)
		require.InDelta(t, 1, b.Dir.Z, 1e-9, b.Name)
		require.Greater(t, b.Offset.Z, 0.0, b.Name)
		require.InDelta(t, 0, b.Offset.X, 1e-9, b.Name)
	}
	require.Empty(t, ex.Blocked)
}
