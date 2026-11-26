package apitest_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures pin the partial-loop fillet of a root chain
// (agent-docs/modify-design.md Part V §2): a block stands on a plate's top
// flush with one or two of the plate's sides, so the plate top's outer loop
// runs down the plate's side and up the block's. Filleting the block's root
// edges is an open chain of that loop whose selected walls all rise. The
// unselected walls beside the loop lie on both sides of the top, which the
// selection does not read.

// requireMeasurementHoldsPiSquared asserts the published reading's interval
// holds a + b·π + c·π² at both ends of a 40-digit π bracket.
func requireMeasurementHoldsPiSquared(t *testing.T, m decad.Measurement, a, b, c *big.Rat) {
	t.Helper()
	lo, _ := new(big.Rat).SetString("3.141592653589793238462643383279502884197")
	hi, _ := new(big.Rat).SetString("3.141592653589793238462643383279502884198")
	held := new(big.Rat).SetFloat64(m.Value.Base())
	bound := new(big.Rat).SetFloat64(m.Bound.Base())
	for _, p := range []*big.Rat{lo, hi} {
		want := new(big.Rat).Add(a, new(big.Rat).Mul(b, p))
		want.Add(want, new(big.Rat).Mul(c, new(big.Rat).Mul(p, p)))
		gap := new(big.Rat).Abs(new(big.Rat).Sub(held, want))
		require.LessOrEqual(t, gap.Cmp(bound), 0, "%s ± %s misses %s", m.Value, m.Bound, want.FloatString(15))
	}
}

// rootEdge names one straight edge of the plate's top (z = 10) by an end and
// its direction.
func rootEdge(q *decad.EdgeQuery, x, y float64, dir r3.Vec) *decad.EdgeQuery {
	preds := []decad.EdgePredicate{decad.EndpointAt(r3.NewVec(x, y, 10)), decad.ParallelTo(dir)}
	if q == nil {
		return decad.Edges(preds...)
	}
	return q.Or(preds...)
}

// requireSoundRootFillet checks the filleted body's topology, mesh and
// verification, and returns its volume.
func requireSoundRootFillet(t *testing.T, doc *decad.Document, got *decad.Body, horns int) decad.Measurement {
	t.Helper()
	requireEveryEdgeOnTwoFaces(t, got)
	var tori int
	for _, f := range got.Faces() {
		if tor, ok := f.Surface().(decad.Torus); ok {
			require.Equal(t, 1.0, tor.Minor.Base())
			require.Equal(t, 1.0, tor.Major.Base(), "a sharp block corner's root patch is a horn torus")
			tori++
		}
	}
	require.Equal(t, horns, tori)
	mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.05))
	require.NoError(t, err)
	require.NotEmpty(t, mesh.Triangles())
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	reading, err := report.ForBody(got)
	require.NoError(t, err)
	require.Equal(t, decad.Sound, reading.Status)
	volume, err := got.Volume()
	require.NoError(t, err)
	return volume
}

// TestBrepPartialRootFilletFlushSide fillets the three root edges of a
// 10 × 6 × 5 block standing on a 40 × 20 × 10 plate flush with its side
// y = 0, at r = 1. The chain's walls rise; its two ends lie in the shared
// side wall, which also descends down the plate. Each root adds (1 − π/4)·r²
// per millimetre over 6 + 10 + 6 mm, and each of the block's two corners on
// the plate a horn-torus quarter turn of (π/2)·r³·(5/6 − π/4), so the volume
// is 8300 + 22 − (14/3)π − π²/4.
func TestBrepPartialRootFilletFlushSide(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 40, 20, 10)
	block := boxBodyAtZ(t, doc, 10, 0, 20, 6, 10, 5)
	body, err := decad.Union(plate, block).Do(t.Context())
	require.NoError(t, err)
	require.False(t, tookMeshPath(body))
	var sel *decad.EdgeQuery
	sel = rootEdge(sel, 10, 0, r3.NewVec(0, 1, 0))
	sel = rootEdge(sel, 10, 6, r3.NewVec(1, 0, 0))
	sel = rootEdge(sel, 20, 0, r3.NewVec(0, 1, 0))
	got, err := decad.Fillet(t.Context(), body, sel.Exactly(3), units.Millimeters(1))
	require.NoError(t, err)
	volume := requireSoundRootFillet(t, doc, got, 2)
	requireMeasurementHoldsPiSquared(t, volume, big.NewRat(8322, 1), big.NewRat(-14, 3), big.NewRat(-1, 4))
}

// TestBrepPartialRootFilletFlushCorner fillets the two root edges of a
// 10 × 6 × 5 block standing in the plate's corner, flush with its sides
// y = 0 and x = 40, at r = 1. The chain ends in two different shared walls.
// The roots add (1 − π/4)·r² over 6 + 10 mm and the block's one corner on the
// plate a horn-torus quarter turn: 8300 + 16 − (43/12)π − π²/8.
func TestBrepPartialRootFilletFlushCorner(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 40, 20, 10)
	block := boxBodyAtZ(t, doc, 30, 0, 40, 6, 10, 5)
	body, err := decad.Union(plate, block).Do(t.Context())
	require.NoError(t, err)
	require.False(t, tookMeshPath(body))
	var sel *decad.EdgeQuery
	sel = rootEdge(sel, 30, 0, r3.NewVec(0, 1, 0))
	sel = rootEdge(sel, 40, 6, r3.NewVec(1, 0, 0))
	got, err := decad.Fillet(t.Context(), body, sel.Exactly(2), units.Millimeters(1))
	require.NoError(t, err)
	volume := requireSoundRootFillet(t, doc, got, 1)
	requireMeasurementHoldsPiSquared(t, volume, big.NewRat(8316, 1), big.NewRat(-43, 12), big.NewRat(-1, 8))
}

// TestBrepPartialRootFilletMixedSidesRefuses selects the flush block's root
// chain together with the plate top's edge x = 0, whose wall descends. The
// selected walls then lie on both sides of the top, which one partial band
// cannot build (modify-general SL2); the refusal leaves the receiver live.
func TestBrepPartialRootFilletMixedSidesRefuses(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 40, 20, 10)
	block := boxBodyAtZ(t, doc, 10, 0, 20, 6, 10, 5)
	body, err := decad.Union(plate, block).Do(t.Context())
	require.NoError(t, err)
	var sel *decad.EdgeQuery
	sel = rootEdge(sel, 10, 0, r3.NewVec(0, 1, 0))
	sel = rootEdge(sel, 10, 6, r3.NewVec(1, 0, 0))
	sel = rootEdge(sel, 20, 0, r3.NewVec(0, 1, 0))
	sel = rootEdge(sel, 0, 0, r3.NewVec(0, 1, 0))
	_, err = decad.Fillet(t.Context(), body, sel.Exactly(4), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Len(t, doc.Bodies(), 1, "the receiver stays live")
}
