package apitest_test

import (
	"fmt"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file holds a line-only region to the region sketch arranged when its
// corners are crossings of long lines: every fragment is cut at both ends, so
// each corner joins two recorded parameters that are each off the exact
// crossing by about ulp(t)·|line|. The region closes every such corner
// through the exact crossing of the two supporting lines
// (docs/evaluator-design.md §4), so its area and centroid are the exact
// crossing polygon's, rounded once.
//
// Shown-to-fail: closing these corners with the straight chord between the
// two recorded ends instead (removing the corner arm of momentinput's
// chargeLoopJunctions) publishes the chord-closed polygon's exact rational.
// It differs from the crossing polygon by a sliver at each corner: four of
// the five triangles read their Area 2.6e-21 to 8.5e-20 mm² outside its
// single-rounding bound, and the square reads Approximate instead of Exact.

// ratPt is an exact plane point.
type ratPt struct{ u, v *big.Rat }

// supportCrossing is the exact crossing of the lines through a0–a1 and b0–b1.
func supportCrossing(t *testing.T, a0, a1, b0, b1 [2]float64) ratPt {
	t.Helper()
	au, av := ratOf(a0[0]), ratOf(a0[1])
	du := new(big.Rat).Sub(ratOf(a1[0]), au)
	dv := new(big.Rat).Sub(ratOf(a1[1]), av)
	bu, bv := ratOf(b0[0]), ratOf(b0[1])
	eu := new(big.Rat).Sub(ratOf(b1[0]), bu)
	ev := new(big.Rat).Sub(ratOf(b1[1]), bv)
	det := new(big.Rat).Sub(new(big.Rat).Mul(du, ev), new(big.Rat).Mul(dv, eu))
	require.NotZero(t, det.Sign(), `the fixture's supports cross`)
	// a + s·d = b + r·e, s = ((b − a) × e) / (d × e).
	wu, wv := new(big.Rat).Sub(bu, au), new(big.Rat).Sub(bv, av)
	s := new(big.Rat).Sub(new(big.Rat).Mul(wu, ev), new(big.Rat).Mul(wv, eu))
	s.Quo(s, det)
	return ratPt{
		u: new(big.Rat).Add(au, new(big.Rat).Mul(s, du)),
		v: new(big.Rat).Add(av, new(big.Rat).Mul(s, dv)),
	}
}

// crossingPolygon is the exact area and centroid of the polygon whose k-th
// vertex is the crossing of line k with line k+1.
func crossingPolygon(t *testing.T, lines [][2][2]float64) (*big.Rat, ratPt) {
	t.Helper()
	n := len(lines)
	verts := make([]ratPt, n)
	for k := range n {
		a, b := lines[k], lines[(k+1)%n]
		verts[k] = supportCrossing(t, a[0], a[1], b[0], b[1])
	}
	area2 := new(big.Rat)
	cu, cv := new(big.Rat), new(big.Rat)
	for k := range n {
		p, q := verts[k], verts[(k+1)%n]
		cross := new(big.Rat).Sub(new(big.Rat).Mul(p.u, q.v), new(big.Rat).Mul(q.u, p.v))
		area2.Add(area2, cross)
		cu.Add(cu, new(big.Rat).Mul(new(big.Rat).Add(p.u, q.u), cross))
		cv.Add(cv, new(big.Rat).Mul(new(big.Rat).Add(p.v, q.v), cross))
	}
	six := new(big.Rat).Mul(area2, big.NewRat(3, 1))
	area := new(big.Rat).Quo(area2, big.NewRat(2, 1))
	area.Abs(area)
	return area, ratPt{u: cu.Quo(cu, six), v: cv.Quo(cv, six)}
}

// requireRatEncloses checks |value − truth| ≤ bound over exact rationals.
func requireRatEncloses(t *testing.T, what string, value, bound float64, truth *big.Rat) {
	t.Helper()
	diff := new(big.Rat).Sub(ratOf(value), truth)
	diff.Abs(diff)
	f, _ := diff.Float64()
	excess, _ := new(big.Rat).Sub(diff, ratOf(bound)).Float64()
	require.LessOrEqualf(t, diff.Cmp(ratOf(bound)), 0, "%s: published %.17g is %.3e from the exact crossing polygon, %.3e outside its bound %.3e", what, value, f, excess, bound)
}

// recordLongLines draws each line as given and records the sketch's one
// region.
func recordLongLines(t *testing.T, lines [][2][2]float64) momentinput.Profile {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	for _, l := range lines {
		s.CreateLine(s.CreatePoint(l[0][0], l[0][1]), s.CreatePoint(l[1][0], l[1][1]))
	}
	profiles := s.Profiles()
	require.Len(t, profiles, 1, `the long lines bound one region`)
	cut := 0
	for _, edge := range profiles[0].Outer {
		require.True(t, edge.Partial, `every side is a fragment of a longer line`)
		require.True(t, edge.TExact)
		cut++
	}
	require.Equal(t, len(lines), cut)
	record, _, err := momentinput.RecordProfile(s, profiles[0])
	require.NoError(t, err)
	return record
}

// requireCrossingPolygon holds a record's area and centroid to the exact
// crossing polygon of its lines.
func requireCrossingPolygon(t *testing.T, record momentinput.Profile, lines [][2][2]float64) decad.Measurement {
	t.Helper()
	wantArea, wantCentroid := crossingPolygon(t, lines)
	area, err := record.Area()
	require.NoError(t, err)
	value, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	bound, err := area.Bound.In(units.SquareMillimeter)
	require.NoError(t, err)
	requireRatEncloses(t, `area`, value, bound, wantArea)

	centroid, err := record.Centroid()
	require.NoError(t, err)
	cb, err := centroid.Bound.In(units.Millimeter)
	require.NoError(t, err)
	requireRatEncloses(t, `centroid u`, centroid.Value.X, cb, wantCentroid.u)
	requireRatEncloses(t, `centroid v`, centroid.Value.Y, cb, wantCentroid.v)
	return area
}

// longLinesThrough extends each side of a polygon to a line reaching far past
// both of its corners, so sketch cuts every side at both ends.
func longLinesThrough(corners [][2]float64, reach float64) [][2][2]float64 {
	n := len(corners)
	lines := make([][2][2]float64, n)
	for k := range n {
		a, b := corners[k], corners[(k+1)%n]
		du, dv := b[0]-a[0], b[1]-a[1]
		lines[k] = [2][2]float64{
			{a[0] - reach*du, a[1] - reach*dv},
			{b[0] + reach*du, b[1] + reach*dv},
		}
	}
	return lines
}

// TestProfileRecordClosesLongLineTriangleAtCrossings records five oblique
// triangles, each bounded by three lines about 2e6 mm long, and reads their
// area and centroid against the exact crossing polygon in math/big.
func TestProfileRecordClosesLongLineTriangleAtCrossings(t *testing.T) {
	t.Parallel()
	triangles := [][][2]float64{
		{{0, 0}, {7.3, 1.1}, {2.9, 6.7}},
		{{-3.1, -0.7}, {5.5, -2.2}, {1.3, 4.9}},
		{{0.37, 0.21}, {9.83, 0.92}, {4.41, 13.7}},
		{{-11.3, 4.2}, {6.1, -3.3}, {2.2, 8.8}},
		{{100.3, 200.7}, {113.9, 203.1}, {104.4, 219.6}},
	}
	for i, corners := range triangles {
		t.Run(fmt.Sprint(i), func(t *testing.T) {
			t.Parallel()
			lines := longLinesThrough(corners, 1e5)
			requireCrossingPolygon(t, recordLongLines(t, lines), lines)
		})
	}
}

// TestProfileRecordClosesLongCutSquareExactly records the 10 mm square whose
// four sides are lines 2e6 mm long crossing at its corners. Every corner is
// an exact crossing of two axis-aligned supports, so the region is exactly
// the square: area exactly 100 and centroid exactly (5, 5), both Exact.
func TestProfileRecordClosesLongCutSquareExactly(t *testing.T) {
	t.Parallel()
	lines := [][2][2]float64{
		{{-1e6, 0}, {1e6, 0}},
		{{10, -1e6}, {10, 1e6}},
		{{1e6, 10}, {-1e6, 10}},
		{{0, 1e6}, {0, -1e6}},
	}
	record := recordLongLines(t, lines)
	area := requireCrossingPolygon(t, record, lines)
	require.Equal(t, decad.Exact, area.Exactness)
	value, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.Equal(t, 100.0, value)
	centroid, err := record.Centroid()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, centroid.Exactness)
	require.Equal(t, 5.0, centroid.Value.X)
	require.Equal(t, 5.0, centroid.Value.Y)
}
