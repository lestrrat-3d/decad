package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The faceted-cap tests stack a ruled boss on the cap plane z = 0 of a lower body whose top outline is the boss's own
// base: the hull of a disc of radius facetedCapDiscR about facetedCapDisc and the circle of radius facetedCapJoinR
// about the z axis. The boss lofts that hull to the hull of the axis circle and a disc shrunk towards it, on
// z = sharedCapHeight.
const (
	facetedCapDiscR = 2.1
	facetedCapJoinR = 0.9922
	facetedCapTau   = 0.9
)

var facetedCapDisc = [2]float64{-1.09, -1.556}

// facetedCapHull sketches on the plane z the hull of the circle of radius r1 about c1 and the circle of radius r2 about
// the z axis, r1 > r2: an arc of each joined by their two outer tangents.
func facetedCapHull(t *testing.T, w *sketch.World, z float64, c1 [2]float64, r1, r2 float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	plane, err := w.CreateOffsetPlane(w.XY(), z)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	th := math.Atan2(-c1[1], -c1[0])
	bt := math.Acos((r1 - r2) / math.Hypot(c1[0], c1[1]))
	pt := func(c [2]float64, r, a float64) *sketch.Point {
		return s.CreatePoint(c[0]+r*math.Cos(a), c[1]+r*math.Sin(a))
	}
	t1p, t1m := pt(c1, r1, th+bt), pt(c1, r1, th-bt)
	t2p, t2m := pt([2]float64{}, r2, th+bt), pt([2]float64{}, r2, th-bt)
	k1, k2 := s.CreatePoint(c1[0], c1[1]), s.CreatePoint(0, 0)
	s.Fix(k1)
	s.Fix(k2)
	a1 := s.CreateArc(k1, t1p, t1m)
	l1 := s.CreateLine(t1m, t2m)
	a2 := s.CreateArc(k2, t2m, t2p)
	l2 := s.CreateLine(t2p, t1p)
	s.AddConstraint(sketch.NewRadius(a1, r1), sketch.NewRadius(a2, r2), sketch.NewTangent(l1, a1),
		sketch.NewTangent(l1, a2), sketch.NewTangent(l2, a2), sketch.NewTangent(l2, a1))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	return s, s.Profiles()[0]
}

// facetedCapPrism extrudes the base hull from z = bottom up to z = top.
func facetedCapPrism(t *testing.T, doc *Document, bottom, top float64) *Body {
	t.Helper()
	s, p := facetedCapHull(t, sketch.NewWorld(), bottom, facetedCapDisc, facetedCapDiscR, facetedCapJoinR)
	body, err := Extrude(doc, s, p, Distance{D: units.Millimeters(top - bottom), Dir: Along}).Do(t.Context())
	require.NoError(t, err)
	return body
}

// facetedCapCutPrism is the base hull extruded from z = -2.4 to z = 1 and cut back to z = 0 by a box: a mesh Cut result,
// whose top is the box's floor and whose walls are the prism's, trimmed there.
func facetedCapCutPrism(t *testing.T, doc *Document) *Body {
	t.Helper()
	prism := facetedCapPrism(t, doc, -2.4, 1)
	cut, err := Cut(prism, internalBoxBodyAtZ(t, doc, -4, -4, 2, 2, 0, 2)).Do(t.Context())
	require.NoError(t, err)
	return cut
}

// facetedCapBoss lofts the base hull on z = 0 to the top hull on z = sharedCapHeight.
func facetedCapBoss(t *testing.T, doc *Document) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s0, p0 := facetedCapHull(t, w, 0, facetedCapDisc, facetedCapDiscR, facetedCapJoinR)
	c := [2]float64{(1 - facetedCapTau) * facetedCapDisc[0], (1 - facetedCapTau) * facetedCapDisc[1]}
	s1, p1 := facetedCapHull(t, w, sharedCapHeight, c, (1-facetedCapTau)*facetedCapDiscR+facetedCapTau*facetedCapJoinR,
		facetedCapJoinR)
	body, err := Loft(doc, s0, p0, s1, p1).Do(t.Context())
	require.NoError(t, err)
	return body
}

// facetedCapDrive is the boss with a cone standing on its top: a mesh Union result.
func facetedCapDrive(t *testing.T, doc *Document) *Body {
	t.Helper()
	drive, err := Union(facetedCapBoss(t, doc), sharedCapCone(t, doc, [2]float64{}, facetedCapJoinR, 0.409, 4.15)).Do(t.Context())
	require.NoError(t, err)
	return drive
}

// facetedCapPost is a 0.3 mm square post about (x, y) from z = -2.4 up through the cap plane to z = 1.
func facetedCapPost(t *testing.T, doc *Document, x, y float64) *Body {
	t.Helper()
	return internalBoxBodyAtZ(t, doc, x-0.15, y-0.15, x+0.15, y+0.15, -2.4, 3.4)
}

// facetedCapMaxBound is the boundary bound every faceted-cap Union must stay under: the outline radii of §4.3 of
// agent-docs/flush-contact-design.md alone, without the seam radius or a kept outline, give 0.08 mm and more.
const facetedCapMaxBound = 0.03

// requireStackedUnion asserts that joined, the Union of lower and upper, has that many lumps and no gap shift, a volume
// equal to the sum of the operands' within the three bounds, and a boundary bound under facetedCapMaxBound.
func requireStackedUnion(t *testing.T, lower, upper, joined *Body, lumps int) {
	t.Helper()
	require.True(t, joined.IsSolid())
	require.Len(t, joined.Lumps(), lumps)
	requireNoGapShift(t, joined)
	lv, err := lower.Volume()
	require.NoError(t, err)
	uv, err := upper.Volume()
	require.NoError(t, err)
	jv, err := joined.Volume()
	require.NoError(t, err)
	slack := jv.Bound.Base() + lv.Bound.Base() + uv.Bound.Base()
	require.InDelta(t, lv.Value.Base()+uv.Value.Base(), jv.Value.Base(), slack, "volume of the union vs the sum of its operands")
	jb, err := joined.Bounds()
	require.NoError(t, err)
	require.Less(t, jb.Bound.Base(), facetedCapMaxBound, "boundary bound of the union")
}
