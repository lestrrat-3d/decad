package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// internalBrepBody commits the brep body a prism's or a stacked prism's face
// view builds, beside its source.
func internalBrepBody(t *testing.T, src *Body) *Body {
	t.Helper()
	var bp brepPayload
	var err error
	switch p := src.n.Payload.(type) {
	case prismPayload:
		bp, err = brepOfPrism(p)
	case stackedPrismPayload:
		bp, err = brepOfStacked(t.Context(), p)
	default:
		t.Fatalf("no face view for %T", src.n.Payload)
	}
	require.NoError(t, err)
	return internalCommitBrep(t, documentHandle(src.n.Doc), bp)
}

func internalCommitBrep(t *testing.T, doc *Document, bp brepPayload) *Body {
	t.Helper()
	body, err := evalBrepContext(t.Context(), doc, doc.nextProducerID(), bp)
	require.NoError(t, err)
	doc.commit(body)
	return body
}

// internalCrossDrilledBrep is the result general-boolean §9's S1 names, built
// by hand: a 40×20×20 box with a Ø6 hole along y through (20, ·, 10). The caps
// lie in the XY frame, the x walls in a YZ frame, and the y walls and the
// hole's wall in an XZ frame whose normal is −y, so every frame is a signed
// permutation of the first.
func internalCrossDrilledBrep(t *testing.T) brepPayload {
	t.Helper()
	frame := func(u, v r3.Vec) r3.Frame {
		f, err := r3.NewFrame(r3.Vec{}, u, v)
		require.NoError(t, err)
		return f
	}
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	xy, yz, xz := frame(x, y), frame(y, z), frame(x, z)
	rect := func(u0, v0, u1, v1 float64) loopRecord {
		pts := []planePoint{{U: u0, V: v0}, {U: u1, V: v0}, {U: u1, V: v1}, {U: u0, V: v1}}
		var loop loopRecord
		for i, p := range pts {
			loop.Segments = append(loop.Segments, lineSeg{Start: p, End: pts[(i+1)%len(pts)], TStart: 0, TEnd: 1})
		}
		return loop
	}
	hole := circleSeg{Center: planePoint{U: 20, V: 10}, Radius: units.Millimeters(3), CCW: false, TStart: 1, TEnd: 0}
	plate := profileRecord{Outer: rect(0, 0, 40, 20)}
	side := profileRecord{Outer: rect(0, 0, 20, 20)}
	drilled := profileRecord{Outer: rect(0, 0, 40, 20), Holes: []loopRecord{{Segments: []curveSegment{hole}}}}
	bp := brepPayload{Xform: r3.Identity(), Faces: []brepFace{
		{Frame: xy, Region: &plate},
		{Frame: xy, Region: &plate, Outward: true, Z0: 20, Z1: 20},
		{Frame: yz, Region: &side},
		{Frame: yz, Region: &side, Outward: true, Z0: 40, Z1: 40},
		{Frame: xz, Region: &drilled, Outward: true},
		{Frame: xz, Region: &drilled, Z0: -20, Z1: -20},
		{Frame: xz, Wall: hole, Z0: -20, Z1: 0},
	}}
	bp.AssignRoles()
	return bp
}

// piEnclosed evaluates an expression a + b·π at both ends of the proven π
// enclosure, low end first for b ≥ 0.
func piEnclosed(a, b *big.Rat) (*big.Rat, *big.Rat) {
	lo := new(big.Rat).Add(a, new(big.Rat).Mul(b, proofbound.PiLower))
	hi := new(big.Rat).Add(a, new(big.Rat).Mul(b, proofbound.PiUpper))
	if lo.Cmp(hi) > 0 {
		lo, hi = hi, lo
	}
	return lo, hi
}

// requireCoversInterval asserts the published reading's interval contains
// every value in [lo, hi], the proven enclosure of the closed form.
func requireCoversInterval(t *testing.T, reading Measurement, lo, hi *big.Rat) {
	t.Helper()
	held := new(big.Rat).SetFloat64(reading.Value.Base())
	bound := new(big.Rat).SetFloat64(reading.Bound.Base())
	require.LessOrEqual(t, new(big.Rat).Sub(held, bound).Cmp(lo), 0, "%s ± %s misses %s", reading.Value, reading.Bound, lo.FloatString(20))
	require.GreaterOrEqual(t, new(big.Rat).Add(held, bound).Cmp(hi), 0, "%s ± %s misses %s", reading.Value, reading.Bound, hi.FloatString(20))
}

// requireCentroidCovers asserts the published centroid ball contains the
// exact point.
func requireCentroidCovers(t *testing.T, reading VecMeasurement, exact [3]*big.Rat) {
	t.Helper()
	held := [3]float64{reading.Value.X, reading.Value.Y, reading.Value.Z}
	sum := new(big.Rat)
	for i, e := range exact {
		d := new(big.Rat).Sub(new(big.Rat).SetFloat64(held[i]), e)
		sum.Add(sum, d.Mul(d, d))
	}
	bound := new(big.Rat).SetFloat64(reading.Bound.Base())
	require.LessOrEqual(t, sum.Cmp(new(big.Rat).Mul(bound, bound)), 0, "%v ± %s misses the exact centroid", reading.Value, reading.Bound)
}

// requireClosedTopology asserts every edge bounds exactly two faces of the
// body and the body is one lump.
func requireClosedTopology(t *testing.T, body *Body) {
	t.Helper()
	require.Len(t, body.Lumps(), 1)
	for _, e := range body.Edges() {
		require.Len(t, e.Faces(), 2)
	}
	require.True(t, auditBoundary(body))
}

// requireSameConvexity asserts two bodies carry as many convex edges, as many
// concave ones, and as many of each edge length.
func requireSameConvexity(t *testing.T, want, got *Body) {
	t.Helper()
	count := func(b *Body) map[[2]float64]int {
		out := map[[2]float64]int{}
		for _, e := range b.Edges() {
			convex := 0.0
			if e.IsConvex() {
				convex = 1
			}
			out[[2]float64{convex, e.n.Length}]++
		}
		return out
	}
	require.Equal(t, count(want), count(got))
}

// internalConvexityByEnds maps every straight edge of a body, keyed by its
// two end positions in lexicographic order, to Edge.IsConvex. A whole
// circle, whose ends coincide, is left out.
func internalConvexityByEnds(t *testing.T, body *Body) map[[2]r3.Vec]bool {
	t.Helper()
	out := map[[2]r3.Vec]bool{}
	for _, e := range body.Edges() {
		if _, line := e.n.Curve.(surfacegeom.Line3); !line {
			continue
		}
		a, b := e.Start().Position().Value, e.End().Position().Value
		if b.X < a.X || (b.X == a.X && (b.Y < a.Y || (b.Y == a.Y && b.Z < a.Z))) {
			a, b = b, a
		}
		key := [2]r3.Vec{a, b}
		_, dup := out[key]
		require.False(t, dup, "one edge runs %v–%v", a, b)
		out[key] = e.IsConvex()
	}
	return out
}

// internalMeshVolumeRat is the mesh's exact signed enclosed volume over its
// held vertex floats.
func internalMeshVolumeRat(m *Mesh) *big.Rat {
	total := new(big.Rat)
	for _, tri := range m.n.Triangles {
		var r [3][3]*big.Rat
		for k, vi := range tri {
			v := m.n.Vertices[vi]
			r[k] = [3]*big.Rat{new(big.Rat).SetFloat64(v.X), new(big.Rat).SetFloat64(v.Y), new(big.Rat).SetFloat64(v.Z)}
		}
		det := new(big.Rat)
		for i := range 3 {
			j, k := (i+1)%3, (i+2)%3
			term := new(big.Rat).Sub(new(big.Rat).Mul(r[1][j], r[2][k]), new(big.Rat).Mul(r[1][k], r[2][j]))
			det.Add(det, term.Mul(term, r[0][i]))
		}
		total.Add(total, det.Quo(det, big.NewRat(6, 1)))
	}
	return total
}

func TestBrepFaceViewOfAllLinePrismMatchesItsMeasurements(t *testing.T) {
	t.Parallel()
	doc := New()
	for _, prism := range []*Body{
		internalPolyPrismBody(t, doc, [][2]float64{{0, 0}, {30, 0}, {30, 10}, {10, 10}, {10, 25}, {0, 25}}, 12),
		internalOffsetBox(t, doc, -4, 3, 16, 9, 5, Distance{D: units.Millimeters(7.5), Dir: Along}),
	} {
		brep := internalBrepBody(t, prism)
		requireClosedTopology(t, brep)
		require.Len(t, brep.Faces(), len(prism.Faces()))
		require.Len(t, brep.Edges(), len(prism.Edges()))
		require.Len(t, brep.Vertices(), len(prism.Vertices()))
		requireSameConvexity(t, prism, brep)
		require.Equal(t, measurementFromInternal(prism.n.Volume), measurementFromInternal(brep.n.Volume))
		require.Equal(t, Exact, Exactness(brep.n.Volume.Exactness))
		require.Equal(t, measurementFromInternal(prism.n.Area), measurementFromInternal(brep.n.Area))
		// The centroid is the same rounded rational. Its bound is that
		// rounding's own exact error, which no valid bound on the same held
		// value undercuts.
		require.Equal(t, prism.n.Centroid.Value, brep.n.Centroid.Value)
		require.Equal(t, Exactness(prism.n.Centroid.Exactness), Exactness(brep.n.Centroid.Exactness))
		require.LessOrEqual(t, brep.n.Centroid.Bound.Base(), prism.n.Centroid.Bound.Base())
		require.Equal(t, boxFromInternal(prism.n.Bounds), boxFromInternal(brep.n.Bounds))
		for _, f := range brep.Faces() {
			_, isPlane := f.n.Surface.(surfacegeom.Plane)
			require.True(t, isPlane)
		}
		g := r3.NewVec(0.6, 0, 0.8)
		plo, phi, pb, err := prism.n.Payload.(prismPayload).ExtentAlong(g)
		require.NoError(t, err)
		blo, bhi, bb, err := brep.n.Payload.(brepPayload).ExtentAlong(g)
		require.NoError(t, err)
		require.Equal(t, [3]float64{plo, phi, pb}, [3]float64{blo, bhi, bb})
	}
}

func TestBrepFaceViewOfCircularPrismCoversItsMeasurements(t *testing.T) {
	t.Parallel()
	doc := New()
	prism := internalHoledPlateBody(t, doc)
	brep := internalBrepBody(t, prism)
	requireClosedTopology(t, brep)
	require.Len(t, brep.Faces(), len(prism.Faces()))
	requireSameConvexity(t, prism, brep)
	cylinders := 0
	for _, f := range brep.Faces() {
		if _, ok := f.n.Surface.(surfacegeom.Cylinder); ok {
			cylinders++
			require.True(t, f.n.Reversed, `the hole's wall faces into the hole`)
		}
	}
	require.Equal(t, 1, cylinders)

	// 100×60×8 plate less a radius-10 hole: 48000 − 800π.
	lo, hi := piEnclosed(big.NewRat(48000, 1), big.NewRat(-800, 1))
	requireCoversInterval(t, measurementFromInternal(brep.n.Volume), lo, hi)
	requireCoversInterval(t, measurementFromInternal(prism.n.Volume), lo, hi)
	require.Equal(t, Approximate, Exactness(brep.n.Volume.Exactness))
	require.Less(t, brep.n.Volume.Bound.Base(), 1e-9*brep.n.Volume.Value.Base())
	// Area: two caps of 6000 − 100π, the outline's 320·8, the hole's 20π·8.
	lo, hi = piEnclosed(big.NewRat(2*6000+320*8, 1), big.NewRat(-200+160, 1))
	requireCoversInterval(t, measurementFromInternal(brep.n.Area), lo, hi)
	// Centroid: x = (6000·50 − 100π·70)/(6000 − 100π), y = 30, z = 4.
	for _, pi := range []*big.Rat{proofbound.PiLower, proofbound.PiUpper} {
		num := new(big.Rat).Sub(big.NewRat(300000, 1), new(big.Rat).Mul(big.NewRat(7000, 1), pi))
		den := new(big.Rat).Sub(big.NewRat(6000, 1), new(big.Rat).Mul(big.NewRat(100, 1), pi))
		exact := [3]*big.Rat{num.Quo(num, den), big.NewRat(30, 1), big.NewRat(4, 1)}
		requireCentroidCovers(t, vecMeasurementFromInternal(brep.n.Centroid), exact)
	}
	require.Equal(t, prism.n.Bounds.Min, brep.n.Bounds.Min)
	require.Equal(t, prism.n.Bounds.Max, brep.n.Bounds.Max)
}

func TestBrepFaceViewOfStackedPrismMatchesItsMeasurements(t *testing.T) {
	t.Parallel()
	_, pocket := internalPocket(t)
	brep := internalBrepBody(t, pocket)
	requireClosedTopology(t, brep)
	require.Len(t, brep.Faces(), len(pocket.Faces()))
	require.Len(t, brep.Edges(), len(pocket.Edges()))
	requireSameConvexity(t, pocket, brep)
	// A 10×10×10 plate with a 4×4 pocket 4 deep: 1000 − 64, exactly.
	require.Equal(t, measurementFromInternal(pocket.n.Volume), measurementFromInternal(brep.n.Volume))
	require.Equal(t, Exact, Exactness(brep.n.Volume.Exactness))
	require.Equal(t, 936.0, brep.n.Volume.Value.Base())
	require.Equal(t, measurementFromInternal(pocket.n.Area), measurementFromInternal(brep.n.Area))
	// Slab moments: 600 at z = 3 and 336 at z = 8, over 936.
	exact := [3]*big.Rat{big.NewRat(5, 1), big.NewRat(5, 1), big.NewRat(600*3+336*8, 936)}
	requireCentroidCovers(t, vecMeasurementFromInternal(brep.n.Centroid), exact)
	requireCentroidCovers(t, vecMeasurementFromInternal(pocket.n.Centroid), exact)
	require.Equal(t, boxFromInternal(pocket.n.Bounds), boxFromInternal(brep.n.Bounds))
	floors := 0
	for _, f := range brep.Faces() {
		if f.n.HasAxialDelta && f.n.Surface.(surfacegeom.Plane).Frame.Origin().Z == 6 {
			floors++
		}
	}
	require.Equal(t, 1, floors, `the pocket floor is the one planar face at the interface`)
}

func TestBrepCrossDrilledBoxMeasuresAcrossPermutedFrames(t *testing.T) {
	t.Parallel()
	doc := New()
	brep := internalCommitBrep(t, doc, internalCrossDrilledBrep(t))
	requireClosedTopology(t, brep)
	require.Len(t, brep.Faces(), 7)
	require.Len(t, brep.Edges(), 14)
	cylinders := 0
	for _, f := range brep.Faces() {
		if _, ok := f.n.Surface.(surfacegeom.Cylinder); ok {
			cylinders++
		}
	}
	require.Equal(t, 1, cylinders)
	// 16000 − 9π·20, with a bound below 1e-9 mm³ (general-boolean §9).
	lo, hi := piEnclosed(big.NewRat(16000, 1), big.NewRat(-180, 1))
	requireCoversInterval(t, measurementFromInternal(brep.n.Volume), lo, hi)
	require.Positive(t, brep.n.Volume.Bound.Base())
	require.Less(t, brep.n.Volume.Bound.Base(), 1e-9)
	// Area: caps 2·800, x walls 2·400, y walls 2·(800 − 9π), the hole 6π·20.
	lo, hi = piEnclosed(big.NewRat(4000, 1), big.NewRat(102, 1))
	requireCoversInterval(t, measurementFromInternal(brep.n.Area), lo, hi)
	requireCentroidCovers(t, vecMeasurementFromInternal(brep.n.Centroid), [3]*big.Rat{big.NewRat(20, 1), big.NewRat(10, 1), big.NewRat(10, 1)})
	require.Equal(t, r3.NewVec(0, 0, 0), brep.n.Bounds.Min)
	require.Equal(t, r3.NewVec(40, 20, 20), brep.n.Bounds.Max)
	require.Equal(t, Exact, Exactness(brep.n.Bounds.Exactness))
	// Each y wall carries the hole as its one inner loop, and the hole's two
	// rims are whole circles of radius 3 about the drill axis.
	circles := 0
	for _, e := range brep.Edges() {
		c, ok := e.n.Curve.(surfacegeom.Circle3)
		if !ok {
			continue
		}
		circles++
		require.Equal(t, 3.0, c.Radius.Base())
		require.Equal(t, 20.0, c.Center.X)
		require.Equal(t, 10.0, c.Center.Z)
		require.Equal(t, 1.0, math.Abs(c.Axis.Y))
	}
	require.Equal(t, 2, circles)
}

func TestBrepTessellationProvesOccupiedVolume(t *testing.T) {
	t.Parallel()
	doc := New()
	cases := []struct {
		name   string
		body   *Body
		lo, hi *big.Rat
	}{
		{name: "cross-drilled box", body: internalCommitBrep(t, doc, internalCrossDrilledBrep(t))},
		{name: "holed plate", body: internalBrepBody(t, internalHoledPlateBody(t, doc))},
	}
	cases[0].lo, cases[0].hi = piEnclosed(big.NewRat(16000, 1), big.NewRat(-180, 1))
	cases[1].lo, cases[1].hi = piEnclosed(big.NewRat(48000, 1), big.NewRat(-800, 1))
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			tol := 0.05
			mesh, err := tessellateContext(t.Context(), tc.body, units.Millimeters(tol), VerifyAll)
			require.NoError(t, err)
			require.True(t, mesh.n.SymDiffOK)
			require.LessOrEqual(t, mesh.n.Bound, tol)
			for _, f := range tc.body.Faces() {
				_, ok := mesh.sourceBound(faceNode(f))
				require.True(t, ok, `every face is a source of the mesh`)
			}
			// The mesh's own exact volume differs from the analytic one by at
			// most the published occupied volume.
			held := internalMeshVolumeRat(mesh)
			require.Positive(t, held.Sign(), `every facet faces outward`)
			bound := new(big.Rat).SetFloat64(mesh.n.VolSymDiff)
			require.LessOrEqual(t, new(big.Rat).Abs(new(big.Rat).Sub(held, tc.lo)).Cmp(bound), 0)
			require.LessOrEqual(t, new(big.Rat).Abs(new(big.Rat).Sub(held, tc.hi)).Cmp(bound), 0)
		})
	}
}

func TestBrepPlacementReEvaluatesUnderMotion(t *testing.T) {
	t.Parallel()
	doc := New()
	prism := internalHoledPlateBody(t, doc)
	brep := internalBrepBody(t, prism)
	turn, err := r3.RotationAround(r3.NewVec(1, 2, 3), r3.NewVec(0, 0, 1), units.Degrees(30))
	require.NoError(t, err)
	plane, err := r3.NewFrame(r3.NewVec(5, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	mirror, err := r3.Reflection(plane)
	require.NoError(t, err)
	for _, motion := range []r3.Transform{turn, mirror} {
		placedPrism, err := Copy(t.Context(), prism, WithCopyPlacement(motion))
		require.NoError(t, err)
		placed, err := Copy(t.Context(), brep, WithCopyPlacement(motion))
		require.NoError(t, err)
		_, ok := placed.n.Payload.(brepPayload)
		require.True(t, ok)
		requireClosedTopology(t, placed)
		require.Len(t, placed.Faces(), len(brep.Faces()))
		// Volume is read in the record's own frame; its bound also charges the
		// motion's own departure from orthonormal (docs/evaluator-design.md §5.1).
		require.Equal(t, brep.n.Volume.Value, placed.n.Volume.Value, `volume is read in the record's own frame`)
		require.GreaterOrEqual(t, placed.n.Volume.Bound.Base(), brep.n.Volume.Bound.Base())
		// The two placed centroids each lie within their bound of the truth,
		// so they lie within the sum of the bounds of each other.
		gap := placed.n.Centroid.Value.Sub(placedPrism.n.Centroid.Value).Len()
		require.LessOrEqual(t, gap, placed.n.Centroid.Bound.Base()+placedPrism.n.Centroid.Bound.Base())
		mesh, err := tessellateContext(t.Context(), placed, units.Millimeters(0.05), VerifyAll)
		require.NoError(t, err)
		require.Positive(t, internalMeshVolumeRat(mesh).Sign(), `a placed mesh still faces outward`)
	}
}

func TestBrepVerifyIsSoundWithAGateDiameter(t *testing.T) {
	t.Parallel()
	doc := New()
	brep := internalCommitBrep(t, doc, internalCrossDrilledBrep(t))
	d, ok, err := bodyGateDiameter(t.Context(), brep)
	require.NoError(t, err)
	require.True(t, ok)
	// The box's corners are witnesses: the diameter is its diagonal √2400,
	// read at or below it.
	require.LessOrEqual(t, new(big.Rat).Mul(new(big.Rat).SetFloat64(d), new(big.Rat).SetFloat64(d)).Cmp(big.NewRat(2400, 1)), 0)
	require.Greater(t, d, math.Nextafter(math.Sqrt(2400), 0)-1e-9)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	br, err := report.ForBody(brep)
	require.NoError(t, err)
	require.Equal(t, Sound, br.Status)
	require.Equal(t, ValidityValid, br.Validity.Outcome)
	require.NotNil(t, br.Region)
	require.Equal(t, ToleranceSatisfied, br.Region.Volume.Tolerance.State,
		`an Approximate volume passes the gate only against a reference diameter`)
}

func TestBrepPayloadAuditRefusesBrokenRecords(t *testing.T) {
	t.Parallel()
	base := internalCrossDrilledBrep(t)
	require.NoError(t, falsifyBrepPayload(t.Context(), base))
	tilted, err := r3.NewFrame(r3.Vec{}, r3.NewVec(0.6, 0.8, 0), r3.NewVec(-0.8, 0.6, 0))
	require.NoError(t, err)
	shifted, err := r3.NewFrame(r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	cases := []struct {
		name   string
		change func(*brepPayload)
		want   error
	}{
		{"no face", func(bp *brepPayload) { bp.Faces = nil }, ErrDegenerate},
		{"region and wall", func(bp *brepPayload) { bp.Faces[0].Wall = bp.Faces[6].Wall }, ErrDegenerate},
		{"planar face with two levels", func(bp *brepPayload) { bp.Faces[0].Z1 = 1 }, ErrDegenerate},
		{"empty sweep", func(bp *brepPayload) { bp.Faces[6].Z1 = bp.Faces[6].Z0 }, ErrDegenerate},
		{"negative displacement", func(bp *brepPayload) { bp.Faces[6].Delta = -1 }, ErrDegenerate},
		{"repeated role", func(bp *brepPayload) { bp.Faces[1].Role = bp.Faces[0].Role }, ErrDegenerate},
		{"elliptical wall", func(bp *brepPayload) {
			bp.Faces[6].Wall = ellipseSeg{Center: planePoint{U: 20, V: 10}, Rx: units.Millimeters(3), Ry: units.Millimeters(2), CCW: true, TStart: 0, TEnd: 1}
		}, ErrUnsupported},
		{"missing wall", func(bp *brepPayload) { bp.Faces = bp.Faces[:6] }, ErrUnsupported},
		{"missing planar face", func(bp *brepPayload) { bp.Faces = append(bp.Faces[:3:3], bp.Faces[4:]...) }, ErrUnsupported},
		{"tilted frame", func(bp *brepPayload) { bp.Faces[1].Frame = tilted }, ErrUnsupported},
		{"shifted origin", func(bp *brepPayload) { bp.Faces[1].Frame = shifted }, ErrUnsupported},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			bp := brepPayload{Xform: base.Xform, Faces: append([]brepFace(nil), base.Faces...)}
			tc.change(&bp)
			_, err := brepTopologyContext(t.Context(), bp)
			require.ErrorIs(t, err, tc.want)
		})
	}
}

func TestBrepMeshPathCutTakesABrepOperand(t *testing.T) {
	t.Parallel()
	doc := New()
	brep := internalCommitBrep(t, doc, internalCrossDrilledBrep(t))
	tool := internalBoxBodyAtZ(t, doc, 30, -1, 50, 21, -1, 22)
	result, err := Cut(t.Context(), brep, tool)
	require.NoError(t, err)
	// The tool removes x ≥ 30 of the box, clear of the hole: 30·20·20 less
	// the hole's 9π·20.
	lo, hi := piEnclosed(big.NewRat(12000, 1), big.NewRat(-180, 1))
	requireCoversInterval(t, measurementFromInternal(result.n.Volume), lo, hi)
}

func TestBrepFaceViewChargesSectionDisplacement(t *testing.T) {
	t.Parallel()
	doc := New()
	plate := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	const shift = 1e9
	tool := internalBoxBody(t, doc, 3-shift, 3, 7-shift, 7, 4)
	move, err := r3.Translation(r3.NewVec(shift, 0, 0))
	require.NoError(t, err)
	tool, err = Transform(t.Context(), tool, move)
	require.NoError(t, err)
	pocket, err := Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	delta := pocket.n.Payload.(stackedPrismPayload).SectionDelta
	require.Positive(t, delta, `the placed tool's re-expression is charged`)
	brep := internalBrepBody(t, pocket)
	// The denoted pocket is exact: 1000 − 64. Every wall's band 2·δ·L over its
	// height is in the bound: the outline's 40·10 and the pocket's 16·4.
	requireCoversInterval(t, measurementFromInternal(brep.n.Volume), big.NewRat(936, 1), big.NewRat(936, 1))
	require.GreaterOrEqual(t, brep.n.Volume.Bound.Base(), 2*delta*(40*10+16*4))
	require.GreaterOrEqual(t, brep.n.Area.Bound.Base(), 2*delta*(40+40+16+16))
	_, err = tessellateContext(t.Context(), brep, units.Millimeters(delta/2), VerifyAll)
	require.ErrorIs(t, err, ErrUnsupported, `a tolerance below the displacement leaves no chord budget`)
	mesh, err := tessellateContext(t.Context(), brep, units.Millimeters(0.05), VerifyAll)
	require.NoError(t, err)
	held := internalMeshVolumeRat(mesh)
	require.LessOrEqual(t, new(big.Rat).Abs(new(big.Rat).Sub(held, big.NewRat(936, 1))).Cmp(new(big.Rat).SetFloat64(mesh.n.VolSymDiff)), 0)
	require.GreaterOrEqual(t, mesh.n.VolSymDiff, 2*delta*(40*10+16*4))
}

func TestBrepMeasurementsChargeFaceDisplacements(t *testing.T) {
	t.Parallel()
	bp := internalCrossDrilledBrep(t)
	// The x = 40 wall's level and the hole wall's section are each held only
	// within a displacement of what they denote.
	const level, section = 0.5, 0.01
	bp.Faces[3].Z0Delta, bp.Faces[3].Z1Delta = level, level
	bp.Faces[6].Delta = section
	brep := internalCommitBrep(t, New(), bp)
	// Moving the 20×20 wall by its level sweeps 400·level; the hole's band is
	// 2·section times its 6π circumference over its 20 mm length.
	band := 20 * 2 * section * 6 * math.Pi
	require.GreaterOrEqual(t, brep.n.Volume.Bound.Base(), 400*level+band)
	require.GreaterOrEqual(t, brep.n.Bounds.Bound.Base(), level+section)
	lo, hi := piEnclosed(big.NewRat(16000, 1), big.NewRat(-180, 1))
	requireCoversInterval(t, measurementFromInternal(brep.n.Volume), lo, hi)
	mesh, err := tessellateContext(t.Context(), brep, units.Millimeters(0.05), VerifyAll)
	require.NoError(t, err)
	require.GreaterOrEqual(t, mesh.n.VolSymDiff, 400*level+band)
	require.GreaterOrEqual(t, mesh.n.Bound, level)
}

func TestBrepTessellationRefusesWallsCloserThanTheirChords(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	var outline loopRecord
	pts := []planePoint{{U: 0, V: 0}, {U: 20, V: 0}, {U: 20, V: 20}, {U: 0, V: 20}}
	for i, p := range pts {
		outline.Segments = append(outline.Segments, lineSeg{Start: p, End: pts[(i+1)%4], TStart: 0, TEnd: 1})
	}
	// A through hole beside an enclosed void 0.002 mm away: no planar face
	// carries both loops, so only the two walls can meet each other's chords.
	hole := circleSeg{Center: planePoint{U: 8, V: 10}, Radius: units.Millimeters(3), CCW: false, TStart: 1, TEnd: 0}
	void := circleSeg{Center: planePoint{U: 14.002, V: 10}, Radius: units.Millimeters(3), CCW: false, TStart: 1, TEnd: 0}
	voidFace := circleSeg{Center: void.Center, Radius: void.Radius, CCW: true, TStart: 0, TEnd: 1}
	plate := profileRecord{Outer: outline, Holes: []loopRecord{{Segments: []curveSegment{hole}}}}
	disc := profileRecord{Outer: loopRecord{Segments: []curveSegment{voidFace}}}
	bp := brepPayload{Xform: r3.Identity(), Faces: []brepFace{
		{Frame: frame, Region: &plate},
		{Frame: frame, Region: &plate, Outward: true, Z0: 10, Z1: 10},
		{Frame: frame, Wall: hole, Z1: 10},
		{Frame: frame, Wall: void, Z0: 3, Z1: 7},
		{Frame: frame, Region: &disc, Outward: true, Z0: 3, Z1: 3},
		{Frame: frame, Region: &disc, Z0: 7, Z1: 7},
	}}
	for _, seg := range outline.Segments {
		bp.Faces = append(bp.Faces, brepFace{Frame: frame, Wall: seg, Z1: 10})
	}
	bp.AssignRoles()
	body := internalCommitBrep(t, New(), bp)
	_, err = tessellateContext(t.Context(), body, units.Millimeters(0.05), VerifyAll)
	require.ErrorIs(t, err, ErrDegenerate)
}

func TestBrepFaceViewOfStackedUnionCoversItsMeasurements(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name         string
		bossZ, bossH float64
		slabs        int
	}{
		{name: "boss on plate", bossZ: 10, bossH: 15, slabs: 2},
		{name: "rooted boss", bossZ: 5, bossH: 20, slabs: 3},
	} {
		t.Run(tc.name, func(t *testing.T) {
			plate, boss := internalBossOnPlate(t, 0, tc.bossZ, tc.bossH)
			union, err := Union(t.Context(), plate, boss)
			require.NoError(t, err)
			sp, ok := union.n.Payload.(stackedPrismPayload)
			require.True(t, ok)
			require.Len(t, sp.Slabs, tc.slabs)
			brep := internalBrepBody(t, union)
			requireClosedTopology(t, brep)
			require.Len(t, brep.Faces(), len(union.Faces()))
			require.Len(t, brep.Edges(), len(union.Edges()))
			requireSameConvexity(t, union, brep)
			// The outer wall changes at the plate's top: the boss's outline is
			// its own column above it, and the plate's floor carries the
			// boss as its hole.
			cylinders := 0
			for _, f := range brep.Faces() {
				if _, ok := f.n.Surface.(surfacegeom.Cylinder); ok {
					cylinders++
					require.False(t, f.n.Reversed, `the boss's wall faces out`)
				}
			}
			require.Equal(t, 1, cylinders)
			// 40×40×10 plate plus a radius-5 boss 15 above it: 16000 + 375π.
			lo, hi := piEnclosed(big.NewRat(16000, 1), big.NewRat(375, 1))
			requireCoversInterval(t, measurementFromInternal(brep.n.Volume), lo, hi)
			requireCoversInterval(t, measurementFromInternal(union.n.Volume), lo, hi)
			require.Less(t, brep.n.Volume.Bound.Base(), 1e-9)
			// Area: plate 2·1600 + 160·10, less the boss's footprint, plus the
			// boss's top 25π and wall 10π·15.
			lo, hi = piEnclosed(big.NewRat(4800, 1), big.NewRat(150, 1))
			requireCoversInterval(t, measurementFromInternal(brep.n.Area), lo, hi)
			require.Equal(t, union.n.Bounds.Min, brep.n.Bounds.Min)
			require.Equal(t, union.n.Bounds.Max, brep.n.Bounds.Max)
			mesh, err := tessellateContext(t.Context(), brep, units.Millimeters(0.05), VerifyAll)
			require.NoError(t, err)
			lo, hi = piEnclosed(big.NewRat(16000, 1), big.NewRat(375, 1))
			held := internalMeshVolumeRat(mesh)
			bound := new(big.Rat).SetFloat64(mesh.n.VolSymDiff)
			require.LessOrEqual(t, new(big.Rat).Abs(new(big.Rat).Sub(held, lo)).Cmp(bound), 0)
			require.LessOrEqual(t, new(big.Rat).Abs(new(big.Rat).Sub(held, hi)).Cmp(bound), 0)
		})
	}
}

func TestBrepFaceViewOfSquareBossUnionMatchesItsMeasurements(t *testing.T) {
	t.Parallel()
	doc := New()
	plate := internalBoxBody(t, doc, -20, -20, 20, 20, 10)
	boss := internalOffsetBox(t, doc, -5, -5, 5, 5, 10, Distance{D: units.Millimeters(15), Dir: Along})
	union, err := Union(t.Context(), plate, boss)
	require.NoError(t, err)
	_, ok := union.n.Payload.(stackedPrismPayload)
	require.True(t, ok)
	brep := internalBrepBody(t, union)
	requireClosedTopology(t, brep)
	require.Len(t, brep.Faces(), len(union.Faces()))
	// The floor walks the boss's straight walls reversed, as its one hole:
	// those rims keep the boss wall's own outer-loop convexity.
	requireSameConvexity(t, union, brep)
	// 16000 + 10·10·15, exactly.
	require.Equal(t, measurementFromInternal(union.n.Volume), measurementFromInternal(brep.n.Volume))
	require.Equal(t, 17500.0, brep.n.Volume.Value.Base())
	require.Equal(t, Exact, Exactness(brep.n.Volume.Exactness))
	require.Equal(t, measurementFromInternal(union.n.Area), measurementFromInternal(brep.n.Area))
	require.Equal(t, boxFromInternal(union.n.Bounds), boxFromInternal(brep.n.Bounds))
}

func TestBrepFaceViewJoinsACrossingBuiltPrism(t *testing.T) {
	t.Parallel()
	doc := New()
	box := internalBoxBody(t, doc, 0, 0, 40, 20, 10)
	bite := internalCircleBody(t, doc, 40, 5, 0, Distance{D: units.Millimeters(10), Dir: Along})
	prism, err := Cut(t.Context(), box, bite)
	require.NoError(t, err)
	pp, ok := prism.n.Payload.(prismPayload)
	require.True(t, ok)
	// The premise: the cut's two fragments walk to their shared corner at
	// two different floats.
	joined, _, err := brepJoinProfile(pp.Profile)
	require.NoError(t, err)
	require.NotEqual(t, pp.Profile, joined)
	brep := internalBrepBody(t, prism)
	requireClosedTopology(t, brep)
	require.Len(t, brep.Faces(), len(prism.Faces()))
	// 40·20·10 less a quarter of a radius-5 disc over the 10 mm height.
	lo, hi := piEnclosed(big.NewRat(8000, 1), big.NewRat(-125, 2))
	requireCoversInterval(t, measurementFromInternal(brep.n.Volume), lo, hi)
	requireCoversInterval(t, measurementFromInternal(prism.n.Volume), lo, hi)
	for _, f := range brep.n.Payload.(brepPayload).Faces {
		require.GreaterOrEqual(t, f.Delta, pp.SectionDelta, `every face keeps the record's own displacement`)
	}
}

// TestBrepCoordinateEnvelopeReadsArcsTightly pins the brep topology's
// coordinate envelope (brepgeom.Build through momentinput.WalkCoordinateUpper)
// on the brep view of F3's slot, semicircles of radius 5 at u = ±15 swept
// 10 mm: it covers a dense sample of |u| + |v| over both arcs and sits within
// 1e-12 of 15 + 5√2.
//
// Shown to fail first: read through the walks' own CoordUpper, which charge
// an ArcSeg its coordinates' L1 sizes as a radius, the envelope was 85.
func TestBrepCoordinateEnvelopeReadsArcsTightly(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	slot, err := s.CreateSlot(-15, 0, 15, 0, 5)
	require.NoError(t, err)
	s.Fix(slot.C1)
	s.Fix(slot.C2)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	prism, err := Extrude(t.Context(), New(), s, s.Profiles()[0], Distance{D: units.Millimeters(10), Dir: Along})
	require.NoError(t, err)
	bp, err := brepOfPrism(prism.n.Payload.(prismPayload))
	require.NoError(t, err)
	btopo, err := brepTopologyContext(t.Context(), bp)
	require.NoError(t, err)
	worst := 0.0
	for i := range 4096 {
		th := 2 * math.Pi * float64(i) / 4096
		worst = math.Max(worst, 15+5*math.Abs(math.Cos(th))+5*math.Abs(math.Sin(th)))
	}
	require.LessOrEqual(t, worst, btopo.CoordUpper)
	require.InDelta(t, 15+5*math.Sqrt2, btopo.CoordUpper, 1e-12)
}

// TestBrepFaceReversedFlipsTheOutwardSide pins the cavity rule of
// docs/modify-general-design.md §3.3 step 5 on the cross-drilled box: a
// planar face flips outward and keeps its region, levels and frame; the hole's
// swept wall walks its circle the other way over the same levels, and a swept
// face exchanges its start-line and end-line splits. Reversing twice restores
// the face.
func TestBrepFaceReversedFlipsTheOutwardSide(t *testing.T) {
	t.Parallel()
	bp := internalCrossDrilledBrep(t)

	top := bp.Faces[1]
	flipped := top.Reversed()
	require.Equal(t, !top.Outward, flipped.Outward)
	require.Same(t, top.Region, flipped.Region)
	require.Equal(t, top.Frame, flipped.Frame)
	require.Equal(t, [2]float64{top.Z0, top.Z1}, [2]float64{flipped.Z0, flipped.Z1})
	require.Equal(t, top, flipped.Reversed())

	wall := bp.Faces[6]
	wall.Side0 = []brepSplit{{Z: -15}}
	wall.Side1 = []brepSplit{{Z: -10}, {Z: -5}}
	back := wall.Reversed()
	circle, ok := back.Wall.(circleSeg)
	require.True(t, ok)
	want := wall.Wall.(circleSeg)
	require.Equal(t, want.Center, circle.Center)
	require.Equal(t, want.Radius, circle.Radius)
	require.Equal(t, !want.CCW, circle.CCW)
	require.Equal(t, [2]float64{want.TEnd, want.TStart}, [2]float64{circle.TStart, circle.TEnd})
	require.Equal(t, wall.Side1, back.Side0)
	require.Equal(t, wall.Side0, back.Side1)
	require.Equal(t, [2]float64{wall.Z0, wall.Z1}, [2]float64{back.Z0, back.Z1})
	require.Equal(t, wall, back.Reversed())
}
