package decad

import (
	"math"
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file holds docs/loft-design.md §12 PR 4's acceptance tests: a
// same-kind Tier A free-form pair builds through §5.1's free-form arm, its
// readings enclose closed-form or densely sampled references, and the
// refusals beside it (S3, S17) keep their sentinels. wedgePlanes,
// wedgeSplineSketch and wedgeHeight are loft_chord_calibration_internal_test.go's
// own A10b fixtures.

// TestLoftFitSplineWedgeMatchesExtrude lofts the A10b fit-spline wedge to its
// own pure translate. That body is the extrude of the section, so Extrude on
// the same profile is an independent oracle: Volume, Centroid and Area must
// each overlap Extrude's interval. Every station on the top section sits
// exactly above a station on the bottom one, which pins the shared dyadic
// correspondence on built coordinates.
//
// Shown to fail first: with FreeformCellPoints' MatchedDelta replaced by
// zeros, the chord-to-curve volume legs vanish and the Volume overlap fails
// (gap 5.4e-3 against a 1.5e-13 bound).
func TestLoftFitSplineWedgeMatchesExtrude(t *testing.T) {
	t.Parallel()
	w, base, top := wedgePlanes(t)
	s0, p0 := wedgeSplineSketch(t, w, base)
	s1, p1 := wedgeSplineSketch(t, w, top)
	doc := New()
	body, err := doc.Loft(t.Context(), s0, p0, s1, p1)
	require.NoError(t, err)

	extDoc := New()
	ext, err := extDoc.Extrude(s0, p0, Distance{D: units.Millimeters(wedgeHeight), Dir: Along})
	require.NoError(t, err)

	vol, err := body.Volume()
	require.NoError(t, err)
	extVol, err := ext.Volume()
	require.NoError(t, err)
	gap := math.Abs(vol.Value.Base() - extVol.Value.Base())
	t.Logf("Volume: loft=%.12g+/-%.3e extrude=%.12g+/-%.3e gap=%.3e",
		vol.Value.Base(), vol.Bound.Base(), extVol.Value.Base(), extVol.Bound.Base(), gap)
	require.LessOrEqual(t, gap, vol.Bound.Base()+extVol.Bound.Base(), "the loft and extrude volume intervals must overlap")

	cen, err := body.Centroid()
	require.NoError(t, err)
	extCen, err := ext.Centroid()
	require.NoError(t, err)
	dist := cen.Value.Sub(extCen.Value).Len()
	require.LessOrEqual(t, dist, cen.Bound.Base()+extCen.Bound.Base(), "the loft and extrude centroid intervals must overlap")

	area, err := body.Area()
	require.NoError(t, err)
	extArea, err := ext.Area()
	require.NoError(t, err)
	areaGap := math.Abs(area.Value.Base() - extArea.Value.Base())
	t.Logf("Area: loft=%.10g+/-%.3e extrude=%.10g+/-%.3e gap=%.3e",
		area.Value.Base(), area.Bound.Base(), extArea.Value.Base(), extArea.Bound.Base(), areaGap)
	require.LessOrEqual(t, areaGap, area.Bound.Base()+extArea.Bound.Base(), "the loft and extrude area intervals must overlap")

	bounds, err := body.Bounds()
	require.NoError(t, err)
	for _, m := range []Measurement{vol, area} {
		require.Equal(t, Approximate, m.Exactness)
		require.Positive(t, m.Bound.Base())
	}
	require.Equal(t, Approximate, cen.Exactness)
	require.Positive(t, cen.Bound.Base())
	require.Equal(t, Approximate, bounds.Exactness)
	require.Positive(t, bounds.Bound.Base())

	lp, ok := body.payload.(loftPayload)
	require.True(t, ok)
	require.Positive(t, lp.sectionDelta, "a chorded free-form pair publishes its measured sagitta")
	require.Greater(t, lp.walls, 2*3, "the free-form pair must chord into more than one cell")

	lower := map[[2]float64]struct{}{}
	upper := map[[2]float64]struct{}{}
	for _, v := range lp.verts {
		switch v.Z {
		case 0:
			lower[[2]float64{v.X, v.Y}] = struct{}{}
		case wedgeHeight:
			upper[[2]float64{v.X, v.Y}] = struct{}{}
		default:
			t.Fatalf("a held vertex %v sits on neither section plane", v)
		}
	}
	require.Equal(t, lower, upper, "each top station must sit exactly above its paired bottom station")
}

// TestLoftFitSplineWedgeVerifiesSound is docs/loft-design.md §13's A10b
// Verify line: the fit-spline wedge reads Sound at the default tolerance, with
// the achieved margin asserted as a number.
//
// The wedge's two sections are the same curve, so every chorded wall cell is
// untwisted (T = 0) with rung G = (0, 0, 10). There the sharp arm of
// proofbound.CellChordCurveAreaAllow reduces to
//
//	2md·(c + max(Ia, Ib)) + ((eB + 2md)²·(Ja + Jb) + 2·(2md·c)²) / (2·eB·c)
//
// against the premise-free (eB + 2md)·(ia + ib)/2 + 2md·c, with eB = 10, c the
// cell's chord and Ia = min(ia, √Ja). Each cell's Ja is measured
// independently of freeform's exact path: a finite-difference integral of
// |C'(s) − Δ|² over the dense samples between the cell's two stations, which
// sit at uniform native parameter. Each published energy must agree with it.
// The test then recomputes the Area reading through evalLoft's own steps
// twice — once as built, once with every free-form energy at +Inf — checks
// the first against the published Area, and asserts the Area bound shrinks by
// the sum of the per-cell differences the measured energies predict. The
// shrunk bound must still enclose a densely sampled reference: 2·(cap area) +
// 10·(perimeter), the wedge being a translate.
//
// Shown to fail first: with LoopPair.appendFreeform passing +Inf energies,
// Verify reads Suspect (an Area bound of 157.3 mm² on a 217.5 mm² reading);
// with it passing 0
// energies, the per-cell energy comparison fails.
func TestLoftFitSplineWedgeVerifiesSound(t *testing.T) {
	t.Parallel()
	w, base, top := wedgePlanes(t)
	s0, p0 := wedgeSplineSketch(t, w, base)
	s1, p1 := wedgeSplineSketch(t, w, top)
	doc := New()
	body, err := doc.Loft(t.Context(), s0, p0, s1, p1)
	require.NoError(t, err)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, Sound, report.Status, "diagnostics: %+v", report.Diagnostics)
	ratio, reading := loftBodyBindingRatio(t, t.Context(), body)
	margin := toleranceRel / ratio
	t.Logf("A10b wedge Verify margin: binding=%s ratio=%.6g margin=%.3gx", reading, ratio, margin)
	require.Greater(t, margin, 1.0)
	// Volume binds at a measured ~14.3x once Area no longer does,
	// Centroid's bound is the shift form (docs/loft-gear-bounds-design.md
	// §3) and Volume's is the per-cell chain (§2). Pinned with generous
	// slack, the arc wedge's own rule, so host rounding never flips it.
	require.Equal(t, "Volume", reading)
	require.InEpsilon(t, 14.3, margin, 0.25)

	area, err := body.Area()
	require.NoError(t, err)
	lp := body.payload.(loftPayload)
	built, pairs, a := loftWedgeAreaRebuild(t, lp, false)
	require.Equal(t, area.Bound.Base(), built.Bound.Base(), "the rebuild must reproduce the published Area")
	unsharp, _, _ := loftWedgeAreaRebuild(t, lp, true)

	require.Len(t, pairs, 1)
	samples := wedgeLoopSamples(t, lp.profile0, 1<<12)
	predicted, freeTotal, sharpTotal := 0.0, 0.0, 0.0
	cells := 0
	for i, p := range pairs {
		n := len(p.v)
		for j := range n {
			if p.matchedDelta[j] <= 0 {
				continue
			}
			jn := (j + 1) % n
			vLo, vHi := a.verts[a.vIdx[i][j]], a.verts[a.vIdx[i][jn]]
			wLo, wHi := a.verts[a.wIdx[i][j]], a.verts[a.wIdx[i][jn]]
			require.Equal(t, vHi.Sub(vLo), wHi.Sub(wLo), "cell %d/%d must be untwisted for the closed form", i, j)
			require.Equal(t, r3.NewVec(0, 0, wedgeHeight), wLo.Sub(vLo))
			measured := sampledTangentEnergy(t, samples, p.v[j], p.v[jn])
			require.InEpsilon(t, measured, p.tangentEnergyV[j], 1e-3, "cell %d: side 0's energy", j)
			require.InEpsilon(t, measured, p.tangentEnergyW[j], 1e-3, "cell %d: side 1's energy", j)
			md := chordCellDeltaUpper(p.matchedDelta[j], a.delta)
			c := vHi.Sub(vLo).Len()
			energyRuled := untwistedRuledLeg(c, wedgeHeight, md, p.arcUpperV[j], p.arcUpperW[j], measured, measured)
			infRuled := untwistedRuledLeg(c, wedgeHeight, md, p.arcUpperV[j], p.arcUpperW[j], math.Inf(1), math.Inf(1))
			predicted += infRuled - energyRuled
			freeTotal += infRuled
			sharpTotal += energyRuled
			cells++
		}
	}
	require.Positive(t, cells)
	shrink := unsharp.Bound.Base() - built.Bound.Base()
	t.Logf("A10b Area bound: %.6g without the energy, %.6g with it (%.0fx); ruled leg %.6g -> %.6g over %d cells; shrink %.9g predicted %.9g",
		unsharp.Bound.Base(), built.Bound.Base(), unsharp.Bound.Base()/built.Bound.Base(), freeTotal, sharpTotal, cells, shrink, predicted)
	require.InEpsilon(t, predicted, shrink, 1e-7, "the Area bound must shrink by the derivation's own per-cell difference")

	ref := wedgeDenseArea(t, lp.profile0, wedgeHeight)
	t.Logf("A10b Area: value=%.10g bound=%.3e reference=%.10g residual=%.3e", area.Value.Base(), area.Bound.Base(), ref, math.Abs(area.Value.Base()-ref))
	require.LessOrEqual(t, math.Abs(area.Value.Base()-ref), area.Bound.Base())
}

// untwistedRuledLeg is proofbound.CellChordCurveAreaAllow's published value on
// a cell whose four corner normals all equal da × G for a rung G of length h
// perpendicular to the chord, so Nmin = h·c and the twist terms vanish.
func untwistedRuledLeg(c, h, md, arcA, arcB, energyA, energyB float64) float64 {
	ia, ib := arcA+c, arcB+c
	ja, jb := ia*ia, ib*ib
	if !math.IsInf(energyA, 1) {
		ia, ja = math.Min(ia, math.Sqrt(energyA)), math.Min(ja, energyA)
	}
	if !math.IsInf(energyB, 1) {
		ib, jb = math.Min(ib, math.Sqrt(energyB)), math.Min(jb, energyB)
	}
	free := (h+2*md)*(arcA+c+arcB+c)/2 + 2*md*c
	beta, gamma := h+2*md, 2*md*c
	sharp := 2*md*(c+math.Max(ia, ib)) + (beta*beta*(ja+jb)+2*gamma*gamma)/(2*h*c)
	return math.Min(free, sharp)
}

// loftWedgeAreaRebuild replays evalLoft's own steps from the records to the
// Area reading. With dropEnergy set, every cell's tangent energy is +Inf, the
// reading a build with no energy proof publishes.
func loftWedgeAreaRebuild(t *testing.T, pl loftPayload, dropEnergy bool) (Measurement, []loftLoopPair, loftAssembly) {
	t.Helper()
	work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	offsets, walks0, walks1, target, err := validateLoftRecords(pl.profile0, pl.profile1, pl.plane0, pl.plane1, pl.alignment, loftRecordAreas(t, pl.profile0, pl.profile1), work0, work1)
	require.NoError(t, err)
	pairs, sectionDelta, sectionMatchedDelta, stationRound, err := loftPairings(pl.profile0, pl.profile1, offsets, walks0, walks1, target, work0, work1)
	require.NoError(t, err)
	if dropEnergy {
		for i := range pairs {
			for j := range pairs[i].tangentEnergyV {
				pairs[i].tangentEnergyV[j] = math.Inf(1)
				pairs[i].tangentEnergyW[j] = math.Inf(1)
			}
		}
	}
	a, err := assembleLoft(t.Context(), pairs, pl.frame0, pl.frame1, pl.plane0, pl.xform, stationRound)
	require.NoError(t, err)
	mass := buildLoftMass(pl, a, pairs, sectionDelta, sectionMatchedDelta)
	return mass.area(capPolygonAreaRat(a.pts0, a.loopIdx0), capPolygonAreaRat(a.pts1, a.loopIdx1)), pairs, a
}

// wedgeLoopSamples samples the recorded outer loop in walk order: a LineSeg
// contributes its walk start, a free-form segment its own perSpan samples per
// span at uniform native parameter (denseWalkSamples).
func wedgeLoopSamples(t *testing.T, p ProfileRecord, perSpan int) []Point2 {
	t.Helper()
	var loop []Point2
	for _, seg := range p.Outer.Segments {
		switch seg := seg.(type) {
		case LineSeg:
			start := seg.Start
			if seg.TStart > seg.TEnd {
				start = seg.End
			}
			loop = append(loop, start)
		default:
			loop = append(loop, denseWalkSamples(t, seg, perSpan)...)
		}
	}
	return loop
}

// sampledTangentEnergy measures one free-form cell's ∫|C'(s) − Δ|² ds from
// the loop samples between the cell's two held stations. A dyadic cell's ends
// fall on the sample grid, and the samples between them sit at uniform cell
// parameter, so N·(x_{i+1} − x_i) is C' at each midpoint to O(1/N²) and the
// midpoint sum integrates the square.
func sampledTangentEnergy(t *testing.T, loop []Point2, lo, hi Point2) float64 {
	t.Helper()
	nearest := func(p Point2) int {
		best, bestD := 0, math.Inf(1)
		for i, q := range loop {
			if d := math.Hypot(q.U-p.U, q.V-p.V); d < bestD {
				best, bestD = i, d
			}
		}
		require.Less(t, bestD, 1e-9, "a station must sit on the sample grid")
		return best
	}
	iLo, iHi := nearest(lo), nearest(hi)
	require.Greater(t, iHi, iLo+64, "a cell needs enough samples to measure")
	n := float64(iHi - iLo)
	du, dv := loop[iHi].U-loop[iLo].U, loop[iHi].V-loop[iLo].V
	energy := 0.0
	for i := iLo; i < iHi; i++ {
		eu := n*(loop[i+1].U-loop[i].U) - du
		ev := n*(loop[i+1].V-loop[i].V) - dv
		energy += (eu*eu + ev*ev) / n
	}
	return energy
}

// wedgeDenseArea is a translate's surface area, 2·(cap area) + height·
// (perimeter), over the recorded loop sampled densely (wedgeLoopSamples).
func wedgeDenseArea(t *testing.T, p ProfileRecord, height float64) float64 {
	t.Helper()
	loop := wedgeLoopSamples(t, p, 1<<14)
	shoelace, perimeter := 0.0, 0.0
	for i, a := range loop {
		b := loop[(i+1)%len(loop)]
		shoelace += a.U*b.V - b.U*a.V
		perimeter += math.Hypot(b.U-a.U, b.V-a.V)
	}
	return math.Abs(shoelace) + height*perimeter
}

// TestLoftFreeformReversedRangeBuildsTheSameStations records the same curve
// shape twice: once forward, and once with its control points reversed and its
// range recorded as [1, 0], which is how the seam records a walk that runs
// against the entity. Both name the same walk, so both build bit-identical
// vertices: slot 0 of the shared dyadic coordinate is each side's own walk
// START (docs/loft-design.md §5.1). A natural-order mapping lands the reversed
// side's slot 0 at its far end and fails here.
//
// The spline has five control points, so its clamped knots are 0 and 1/2 and
// reversing the control points reverses the curve exactly.
//
// Shown to fail first: with walkOrderSpans returning the natural chain for a
// reversed walk, the opposite-sense build no longer succeeds.
func TestLoftFreeformReversedRangeBuildsTheSameStations(t *testing.T) {
	t.Parallel()
	control := []Point2{pt(4, 0), pt(4.5, 2), pt(2, 3.5), pt(-0.5, 2), pt(0, 0)}
	forward := SplineSeg{Control: control, TStart: 0, TEnd: 1}
	reversed := SplineSeg{Control: slices.Clone(control), TStart: 1, TEnd: 0}
	slices.Reverse(reversed.Control)
	loopOf := func(spline CurveSegment) ProfileRecord {
		return ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
			LineSeg{Start: pt(0, 0), End: pt(4, 0), TStart: 0, TEnd: 1},
			spline,
		}}}
	}

	sameSense := evalLoftFixture(t, loftPayloadFor(t, loopOf(forward), loopOf(forward), r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 3)))
	opposite := evalLoftFixture(t, loftPayloadFor(t, loopOf(forward), loopOf(reversed), r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 3)))

	a := sameSense.payload.(loftPayload)
	b := opposite.payload.(loftPayload)
	require.Greater(t, a.walls, 4, "the spline must chord into several cells")
	require.Equal(t, a.verts, b.verts, "the reversed record must build the identical station coordinates")
	require.Equal(t, a.tris, b.tris)
}

// TestLoftFreeformTwistedClosedSplineBracketsDenseReference lofts a closed
// spline to the same local curve on a plane turned 20 degrees about the z
// axis. The true body is ruled by straight lines between points at the same
// span-native parameter, so its cross-section at height fraction λ is the loop
// (1-λ)·C0(s) + λ·C1(s): that loop's area is quadratic and its first moments
// cubic in λ, so Simpson's rule over three heights integrates the volume and
// centroid exactly from densely sampled loops. The wall area integrates the
// densely sampled ruled surface directly. Each published reading must enclose
// its reference, and the box widened by its own bound must contain both curves.
//
// Shown to fail first: zeroing FreeformCellPoints' MatchedDelta fails the
// Volume enclosure, and zeroing its Sagitta leaves the box short of the curve.
func TestLoftFreeformTwistedClosedSplineBracketsDenseReference(t *testing.T) {
	t.Parallel()
	const height = 10.0
	const twist = 20 * math.Pi / 180
	w := sketch.NewWorld()
	frame0, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	frame1, err := r3.NewFrame(r3.NewVec(0, 0, height), r3.NewVec(math.Cos(twist), math.Sin(twist), 0), r3.NewVec(-math.Sin(twist), math.Cos(twist), 0))
	require.NoError(t, err)
	plane0, err := w.CreatePlaneFromFrame(frame0)
	require.NoError(t, err)
	plane1, err := w.CreatePlaneFromFrame(frame1)
	require.NoError(t, err)
	s0, p0 := closedSplineSketch(t, w, plane0)
	s1, p1 := closedSplineSketch(t, w, plane1)

	doc := New()
	body, err := doc.Loft(t.Context(), s0, p0, s1, p1)
	require.NoError(t, err)
	lp := body.payload.(loftPayload)
	require.Positive(t, lp.sectionDelta)

	c0 := denseWalkSamples(t, lp.profile0.Outer.Segments[0], 2048)
	c1 := denseWalkSamples(t, lp.profile1.Outer.Segments[0], 2048)
	require.Len(t, c1, len(c0))
	x0 := liftSamples(c0, lp.frame0)
	x1 := liftSamples(c1, lp.frame1)
	ref := ruledReference(x0, x1, height)

	vol, err := body.Volume()
	require.NoError(t, err)
	t.Logf("Volume: value=%.12g bound=%.3e reference=%.12g residual=%.3e",
		vol.Value.Base(), vol.Bound.Base(), ref.volume, math.Abs(vol.Value.Base()-ref.volume))
	require.LessOrEqual(t, math.Abs(vol.Value.Base()-ref.volume), vol.Bound.Base())

	cen, err := body.Centroid()
	require.NoError(t, err)
	t.Logf("Centroid: value=%v bound=%.3e reference=%v", cen.Value, cen.Bound.Base(), ref.centroid)
	require.LessOrEqual(t, cen.Value.Sub(ref.centroid).Len(), cen.Bound.Base())

	area, err := body.Area()
	require.NoError(t, err)
	t.Logf("Area: value=%.10g bound=%.3e reference=%.10g", area.Value.Base(), area.Bound.Base(), ref.area)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-ref.area), area.Bound.Base())

	box, err := body.Bounds()
	require.NoError(t, err)
	b := box.Bound.Base()
	require.Positive(t, b)
	for _, p := range append(slices.Clone(x0), x1...) {
		require.GreaterOrEqual(t, p.X, box.Min.X-b)
		require.LessOrEqual(t, p.X, box.Max.X+b)
		require.GreaterOrEqual(t, p.Y, box.Min.Y-b)
		require.LessOrEqual(t, p.Y, box.Max.Y+b)
		require.GreaterOrEqual(t, p.Z, box.Min.Z-b)
		require.LessOrEqual(t, p.Z, box.Max.Z+b)
	}
}

// TestLoftFreeformWedgePlacedAndTessellated places the A10b wedge loft and
// tessellates both bodies. The placed body re-lifts the records under the
// motion, so its volume still encloses Extrude's and its centroid moves with
// the motion inside both bounds. Each mesh restates the held triangle set,
// publishes the payload's facet departure as its Bound, and its own signed
// volume sits within its occupied-volume proof of Extrude's interval.
//
// Shown to fail first: zeroing FreeformCellPoints' MatchedDelta fails the
// mesh volume check.
func TestLoftFreeformWedgePlacedAndTessellated(t *testing.T) {
	t.Parallel()
	w, base, top := wedgePlanes(t)
	s0, p0 := wedgeSplineSketch(t, w, base)
	s1, p1 := wedgeSplineSketch(t, w, top)
	doc := New()
	body, err := doc.Loft(t.Context(), s0, p0, s1, p1)
	require.NoError(t, err)
	ext, err := New().Extrude(s0, p0, Distance{D: units.Millimeters(wedgeHeight), Dir: Along})
	require.NoError(t, err)
	extVol, err := ext.Volume()
	require.NoError(t, err)

	spin, err := r3.Rotation(r3.NewVec(1, 0, 0), units.Degrees(37))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(5, -3, 2))
	require.NoError(t, err)
	motion, err := spin.Then(shift)
	require.NoError(t, err)
	placed, err := body.PlacedCopy(t.Context(), motion)
	require.NoError(t, err)

	src := body.payload.(loftPayload)
	moved := placed.payload.(loftPayload)
	require.Greater(t, moved.delta, src.delta, "a placement adds its rigid rounding to every vertex")

	placedVol, err := placed.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(placedVol.Value.Base()-extVol.Value.Base()), placedVol.Bound.Base()+extVol.Bound.Base())

	srcCen, err := body.Centroid()
	require.NoError(t, err)
	placedCen, err := placed.Centroid()
	require.NoError(t, err)
	want := motion.Apply(srcCen.Value)
	require.LessOrEqual(t, placedCen.Value.Sub(want).Len(), placedCen.Bound.Base()+srcCen.Bound.Base())

	for _, b := range []*Body{body, placed} {
		lp := b.payload.(loftPayload)
		mesh, err := b.Tessellate(t.Context(), units.Millimeters(1), WithVerification(VerifyAll))
		require.NoError(t, err)
		require.Len(t, mesh.Triangles(), len(lp.tris))
		require.Equal(t, lp.proof.facetDeparture, mesh.Bound().Base())
		require.Positive(t, mesh.Bound().Base())
		require.True(t, mesh.VolumeVerified())
		meshVol := signedMeshVolume(mesh.Vertices(), mesh.Triangles())
		require.LessOrEqual(t, math.Abs(meshVol-extVol.Value.Base()), mesh.volSymDiff+extVol.Bound.Base(),
			"the mesh's own volume must sit within its occupied-volume proof of the extruded body")
	}
}

// TestLoftFreeformSpanCountMismatchRefusesS17 pairs two fit-spline wedges
// whose curves pass through five and six points, so their converted chains
// hold four and five Bézier spans. That pair has no shared station coordinate
// and refuses as S17, with ErrUnsupported, before any body is committed. The
// record-only gates alone reach the refusal, which pins it among the shape
// gates rather than in station generation (docs/loft-design.md §4).
//
// Shown to fail first: with SpanCountGate's refusal ignored, the record-only
// gates pass and the refusal comes from station generation instead.
func TestLoftFreeformSpanCountMismatchRefusesS17(t *testing.T) {
	t.Parallel()
	const s17 = "reduce to 4 and 5 Bézier spans; this evaluator chords a free-form pair only over an equal span count"
	w, base, top := wedgePlanes(t)
	s0, p0 := fitSplineWedgeSketch(t, w, base, 5)
	s1, p1 := fitSplineWedgeSketch(t, w, top, 6)
	doc := New()
	_, err := doc.Loft(t.Context(), s0, p0, s1, p1)
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, s17)
	require.Empty(t, doc.Bodies(), "a refused loft leaves the document unchanged")

	rec0, pl0, err := RecordProfile(s0, p0)
	require.NoError(t, err)
	rec1, pl1, err := RecordProfile(s1, p1)
	require.NoError(t, err)
	err = validateLoftRecordsErr(rec0, rec1, pl0, pl1, nil, freeform.NewFreeformWork(), freeform.NewFreeformWork())
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, s17)
}

// TestLoftFreeformMixedPairsRefuseS3 keeps S3 for every pairing that is not
// the same recorded type: a free-form side against an arc or a line, and two
// different free-form types. None of them reaches S17's span comparison.
func TestLoftFreeformMixedPairsRefuseS3(t *testing.T) {
	t.Parallel()
	fit := FitSplineSeg{Fit: []Point2{pt(0, 0), pt(0.3, 0.2), pt(0.6, -0.1), pt(1, 0)}, TStart: 0, TEnd: 1}
	spline := SplineSeg{Control: []Point2{pt(0, 0), pt(0.3, 0.2), pt(0.6, -0.1), pt(1, 0)}, TStart: 0, TEnd: 1}
	arc := ArcSeg{Center: pt(0.5, -1), Start: pt(0, 0), End: pt(1, 0), TStart: 0, TEnd: 1}
	line := LineSeg{Start: pt(0, 0), End: pt(1, 0), TStart: 0, TEnd: 1}
	for _, row := range []struct {
		name   string
		s0, s1 CurveSegment
	}{
		{"arc against fit spline", arc, fit},
		{"line against fit spline", line, fit},
		{"spline against fit spline", spline, fit},
	} {
		t.Run(row.name, func(t *testing.T) {
			t.Parallel()
			p0 := ProfileRecord{Outer: squareLoopWithFirstSegment(row.s0)}
			p1 := ProfileRecord{Outer: squareLoopWithFirstSegment(row.s1)}
			pl0, pl1 := planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 1))
			err := validateLoftRecordsErr(p0, p1, pl0, pl1, nil, freeform.NewFreeformWork(), freeform.NewFreeformWork())
			require.ErrorIs(t, err, ErrUnsupported)
			require.ErrorContains(t, err, "not the same admitted segment type")
		})
	}
}

// TestLoftFreeformPairPastItsShareRefusesS15 gives a free-form pair a share of
// one station: its loop holds loftmesh.StationCapCeiling paired segments, so P
// already reaches the cap and §5.1's allocation clamps every chorded pair to
// mMax = 1. The pair's
// four Bézier spans need at least one cell each, so its dyadic walk refuses
// before measuring a cell, and the refusal names the segment whose share it
// passed (Table S row S15).
//
// Shown to fail first: with the walk's ceiling left at MaxChordsPerWalk
// instead of the share, the pair chords and no refusal comes back.
func TestLoftFreeformPairPastItsShareRefusesS15(t *testing.T) {
	t.Parallel()
	fit := FitSplineSeg{Fit: []Point2{pt(0, 0), pt(1, 1), pt(2, 0), pt(3, 1), pt(4, 0)}, TStart: 0, TEnd: 1}
	segs := []CurveSegment{LineSeg{Start: pt(0, 0), End: pt(0, -1), TStart: 0, TEnd: 1}, fit}
	for len(segs) < loftmesh.StationCapCeiling {
		segs = append(segs, LineSeg{Start: pt(4, 0), End: pt(0, 0), TStart: 0, TEnd: 1})
	}
	p := ProfileRecord{Outer: LoopRecord{Segments: segs}}
	walks := resolveLoftLoopWalks(t, p)
	_, _, _, _, err := loftPairings(p, p, []int{0}, walks, walks, 1, freeform.NewFreeformWork(), freeform.NewFreeformWork()) //nolint:dogsled // only the refusal is under test.
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorIs(t, err, freeform.ErrTooManyChords)
	var capErr *loftStationCapError
	require.ErrorAs(t, err, &capErr)
	require.Equal(t, loftStationCapError{Loop: 0, Seg: 1, M: 2, MMax: 1, AtLeast: true}, *capErr)
}

// TestLoftDegreeOneTwistedPairPublishesTheRuledBody lofts the square
// (±1, ±1) at z = 0 to the diamond (±2, 0)/(0, ±2) at z = 10, one straight
// side per segment, unplaced on two XY planes. Every free-form cell's chord
// departure is zero there, since a degree-1 NURBSSeg IS its own chord.
//
// The free-form pair denotes the ruled body (docs/loft-design.md §5.2): the
// section at t = z/10 is the quadrilateral through (1 − t)·square + t·diamond,
// whose area is A(t) = 4 + 4t² (A(0) = 4, A(1/2) = 5, A(1) = 8). So the
// volume is 10·∫A = 160/3 and the centroid height is 10·∫t·A / ∫A = 45/8.
// The same section drawn as LineSegs denotes the triangulated polyhedron
// instead (§5), whose volume 40 publishes Exact.
//
// Shown to fail first: on the build that skipped zero-departure cells in
// ComputeLoftChordedAllow, and on the build whose evalLoft gate read the two
// section terms alone, the NURBS pair published Volume 40 ± 0 Exact (the
// twist correction never applied). With Area's bilinear correction keyed on
// the section terms alone, Area missed the ruled walls by 5.28 against a
// 2.1e-13 bound. With Verify's leg 4 reading sectionDelta alone, the NURBS
// sheet proved itself simple.
func TestLoftDegreeOneTwistedPairPublishesTheRuledBody(t *testing.T) {
	square := [][2]float64{{1, -1}, {1, 1}, {-1, 1}, {-1, -1}}
	diamond := [][2]float64{{2, 0}, {0, 2}, {-2, 0}, {0, -2}}
	build := func(t *testing.T, nurbs bool, opts ...LoftOption) *Body {
		t.Helper()
		w := sketch.NewWorld()
		f0, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
		require.NoError(t, err)
		f1, err := r3.NewFrame(r3.NewVec(0, 0, 10), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
		require.NoError(t, err)
		pl0, err := w.CreatePlaneFromFrame(f0)
		require.NoError(t, err)
		pl1, err := w.CreatePlaneFromFrame(f1)
		require.NoError(t, err)
		s0, p0 := straightSidedSketch(t, w, pl0, square, nurbs)
		s1, p1 := straightSidedSketch(t, w, pl1, diamond, nurbs)
		body, err := New().Loft(t.Context(), s0, p0, s1, p1, opts...)
		require.NoError(t, err)
		return body
	}

	t.Run("degree-1 NURBS pair encloses the ruled body", func(t *testing.T) {
		body := build(t, true)
		lp := body.payload.(loftPayload)
		require.Zero(t, lp.delta, "two unplaced XY planes through the origin's own axis lift exactly")
		require.Zero(t, lp.sectionDelta, "a degree-1 NURBSSeg is its own chord")

		vol, err := body.Volume()
		require.NoError(t, err)
		requireRatCovered(t, vol, big.NewRat(160, 3))

		cen, err := body.Centroid()
		require.NoError(t, err)
		requireCentroidCovers(t, cen, [3]*big.Rat{new(big.Rat), new(big.Rat), big.NewRat(45, 8)})

		// The four walls are one bilinear patch each, congruent under the
		// quarter turn, and the caps are exactly 4 and 8.
		wall := bilinearAreaMidpoint(r3.NewVec(1, -1, 0), r3.NewVec(1, 1, 0), r3.NewVec(2, 0, 10), r3.NewVec(0, 2, 10), 1024)
		area, err := body.Area()
		require.NoError(t, err)
		require.LessOrEqual(t, math.Abs(12+4*wall-area.Value.Base()), area.Bound.Base(),
			"Area must enclose the ruled walls, not the held triangle pairs")

		require.Positive(t, lp.proof.facetDeparture,
			"the held triangle pair stands for a twisted bilinear patch, so the mesh is not the boundary")
	})

	t.Run("LineSeg pair publishes the polyhedron", func(t *testing.T) {
		body := build(t, false)
		vol, err := body.Volume()
		require.NoError(t, err)
		require.Equal(t, Exact, vol.Exactness)
		require.Equal(t, 40.0, vol.Value.Base())
		requireRatCovered(t, vol, big.NewRat(40, 1))
	})

	// Verify's leg 4 admits a loft sheet only where the audited triangle set
	// IS the surface. The NURBS sheet's walls are bilinear patches the audit
	// never cleared, so the proof does not transfer.
	t.Run("only the LineSeg sheet proves itself simple", func(t *testing.T) {
		require.False(t, payloadProvesSimple(t.Context(), build(t, true, WithSurfaceResult()).payload))
		require.True(t, payloadProvesSimple(t.Context(), build(t, false, WithSurfaceResult()).payload))
	})
}

// --- fixtures and references ---

// straightSidedSketch draws one closed loop through the fixed coords, each side
// a LineSeg or, when nurbs is set, a degree-1 NURBSSeg over the same two
// points.
func straightSidedSketch(t *testing.T, w *sketch.World, plane *sketch.Plane, coords [][2]float64, nurbs bool) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	pts := make([]*sketch.Point, len(coords))
	for i, c := range coords {
		pts[i] = s.CreatePoint(c[0], c[1])
		s.Fix(pts[i])
	}
	for i := range pts {
		a, b := pts[i], pts[(i+1)%len(pts)]
		if !nurbs {
			s.CreateLine(a, b)
			continue
		}
		_, err := s.CreateNURBS(1, []*sketch.Point{a, b}, nil, []float64{0, 0, 1, 1})
		require.NoError(t, err)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

// closedSplineSketch draws one closed cubic spline over six fixed control
// points around the origin.
func closedSplineSketch(t *testing.T, w *sketch.World, plane *sketch.Plane) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	coords := [][2]float64{{6, 0}, {3, 4}, {-2, 5}, {-6, 0}, {-3, -4}, {3, -5}}
	pts := make([]*sketch.Point, len(coords))
	for i, c := range coords {
		pts[i] = s.CreatePoint(c[0], c[1])
		s.Fix(pts[i])
	}
	_, err = s.CreateClosedSpline(pts...)
	require.NoError(t, err)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

// fitSplineWedgeSketch is wedgeSplineSketch with n fit points on the quarter
// circle rather than five.
func fitSplineWedgeSketch(t *testing.T, w *sketch.World, plane *sketch.Plane, n int) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	origin := s.CreatePoint(0, 0)
	s.Fix(origin)
	pts := make([]*sketch.Point, n)
	for k := range pts {
		theta := float64(k) * math.Pi / 2 / float64(n-1)
		pts[k] = s.CreatePoint(wedgeRadius*math.Cos(theta), wedgeRadius*math.Sin(theta))
		s.Fix(pts[k])
	}
	_, err = s.CreateFitSpline(pts...)
	require.NoError(t, err)
	s.CreateLine(origin, pts[0])
	s.CreateLine(pts[n-1], origin)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

// denseWalkSamples samples one recorded free-form segment in walk order at
// perSpan evenly spaced local parameters per converted Bézier span, the span's
// own end excluded. Sample i of one side and sample i of another side with the
// same span count sit at the same span-native parameter.
func denseWalkSamples(t *testing.T, seg CurveSegment, perSpan int) []Point2 {
	t.Helper()
	w, err := walkOf(seg, freeform.NewFreeformWork())
	require.NoError(t, err)
	require.Equal(t, survey2d.WalkFreeform, w.Kind)
	spans := w.Spans
	if w.Reversed {
		spans = make([]freeform.BezierSpan, len(w.Spans))
		for i, span := range w.Spans {
			rev := slices.Clone(span)
			slices.Reverse(rev)
			spans[len(w.Spans)-1-i] = rev
		}
	}
	var out []Point2
	for _, span := range spans {
		ctrl := make([]Point2, len(span))
		for i, p := range span {
			u, _ := p.U.Float64()
			v, _ := p.V.Float64()
			ctrl[i] = pt(u, v)
		}
		for k := range perSpan {
			out = append(out, deCasteljau(ctrl, float64(k)/float64(perSpan)))
		}
	}
	return out
}

func deCasteljau(ctrl []Point2, s float64) Point2 {
	work := slices.Clone(ctrl)
	for n := len(work) - 1; n > 0; n-- {
		for i := range n {
			work[i] = pt((1-s)*work[i].U+s*work[i+1].U, (1-s)*work[i].V+s*work[i+1].V)
		}
	}
	return work[0]
}

func liftSamples(pts []Point2, f r3.Frame) []r3.Vec {
	out := make([]r3.Vec, len(pts))
	for i, p := range pts {
		out[i] = f.ToWorldUV(p.U, p.V)
	}
	return out
}

type ruledReferenceReadings struct {
	volume   float64
	centroid r3.Vec
	area     float64
}

// ruledReference integrates the solid ruled between two closed sample loops on
// the parallel planes z = 0 and z = height. A cross-section's signed area is
// quadratic and its first moments cubic in the height fraction, so Simpson's
// rule over three fractions is exact for the sampled loops. The lateral wall
// integrates each sample interval's bilinear patch by 4-point Gauss-Legendre in
// both directions; the two caps add the end sections' own areas.
func ruledReference(x0, x1 []r3.Vec, height float64) ruledReferenceReadings {
	n := len(x0)
	section := func(lambda float64) (float64, float64, float64) {
		var a, mx, my float64
		for i := range n {
			p := x0[i].Scale(1 - lambda).Add(x1[i].Scale(lambda))
			q := x0[(i+1)%n].Scale(1 - lambda).Add(x1[(i+1)%n].Scale(lambda))
			cross := p.X*q.Y - q.X*p.Y
			a += cross / 2
			mx += (p.X + q.X) * cross / 6
			my += (p.Y + q.Y) * cross / 6
		}
		return a, mx, my
	}
	simpson := func(f0, fm, f1 float64) float64 { return (f0 + 4*fm + f1) / 6 }
	a0, mx0, my0 := section(0)
	am, mxm, mym := section(0.5)
	a1, mx1, my1 := section(1)
	volume := height * simpson(a0, am, a1)
	mx := height * simpson(mx0, mxm, mx1)
	my := height * simpson(my0, mym, my1)
	mz := height * height * simpson(0, 0.5*am, a1)

	gauss := [4][2]float64{
		{0.5 - 0.3399810435848563/2, 0.6521451548625461 / 2},
		{0.5 + 0.3399810435848563/2, 0.6521451548625461 / 2},
		{0.5 - 0.8611363115940526/2, 0.3478548451374538 / 2},
		{0.5 + 0.8611363115940526/2, 0.3478548451374538 / 2},
	}
	wall := 0.0
	for i := range n {
		a, b := x0[i], x0[(i+1)%n]
		c, d := x1[i], x1[(i+1)%n]
		for _, gu := range gauss {
			for _, gl := range gauss {
				u, l := gu[0], gl[0]
				xu := b.Sub(a).Scale(1 - l).Add(d.Sub(c).Scale(l))
				xl := c.Sub(a).Scale(1 - u).Add(d.Sub(b).Scale(u))
				wall += gu[1] * gl[1] * xu.Cross(xl).Len()
			}
		}
	}
	return ruledReferenceReadings{
		volume:   math.Abs(volume),
		centroid: r3.NewVec(mx/volume, my/volume, mz/volume),
		area:     wall + math.Abs(a0) + math.Abs(a1),
	}
}

func signedMeshVolume(verts []r3.Vec, tris [][3]int) float64 {
	sum := 0.0
	for _, tri := range tris {
		a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
		sum += a.Dot(b.Cross(c))
	}
	return sum / 6
}

// TestLoftFreeformWalkBisectsOnMatched pins docs/loft-gear-bounds-design.md
// §5's matched bisection on the loft's free-form station walk
// (loftmesh.FreeformCellPoints over freeform.PairChainStations). The pair is
// zigzagHuggingSpan with itself: every control point lies on the chord, so its
// sagitta is 0 at every depth and a walk deciding on the sagitta alone accepts
// the whole span as one cell, while its parameter-matched departure is above
// 0.3 (TestSpanMatchedDeltaUpperEnclosesWhatTheSagittaMisses). The walk must
// bisect until every accepted cell's matched value is at or below the target.
//
// Shown to fail: with PairChainStations leaving SagittaStationWalk.Matched
// false, the walk accepts one cell and the cell-count assertion fails.
func TestLoftFreeformWalkBisectsOnMatched(t *testing.T) {
	t.Parallel()
	w := survey2d.SegmentWalk{Kind: survey2d.WalkFreeform, Spans: []freeform.BezierSpan{zigzagHuggingSpan()}}
	const target = 1e-2
	cell, err := loftmesh.FreeformCellPoints(w, w, target, freeform.MaxChordsPerWalk, nil, nil)
	require.NoError(t, err)

	require.Zero(t, cell.Sagitta, "the sagitta alone never asks this span for a bisection")
	require.Greater(t, len(cell.MatchedDelta), 1, "the matched departure forces the bisection the sagitta does not")
	require.Len(t, cell.Stations0, len(cell.MatchedDelta), "one station per accepted cell")
	worst := 0.0
	for k, md := range cell.MatchedDelta {
		require.LessOrEqual(t, md, target, "cell %d's matched departure must meet the target", k)
		worst = math.Max(worst, md)
	}
	require.Positive(t, worst, "the curve departs from its chords, so the recorded matched values cannot all be 0")
	t.Logf("cells=%d worst matched=%.4g target=%g", len(cell.MatchedDelta), worst, target)
}
