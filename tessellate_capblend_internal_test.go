package decad

import (
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/extent"

	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stationbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// capBlendMeshHeight is the sweep every section in this file is extruded by.
const capBlendMeshHeight = 20.0

// chamferedSectionBody extrudes one drawn section and chamfers its end cap
// loop, returning the result and its own payload — the two things every
// tessellator assertion below reads.
func chamferedSectionBody(t *testing.T, section func(*sketch.Sketch), d float64) (*Body, capBlendPayload) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	section(s)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.NotEmpty(t, s.Profiles())

	doc := New()
	prof := s.Profiles()[0]
	for _, p := range s.Profiles() {
		if len(p.Holes) > len(prof.Holes) {
			prof = p
		}
	}
	body, err := doc.Extrude(s, prof, Distance{D: units.Millimeters(capBlendMeshHeight), Dir: Along})
	require.NoError(t, err)
	chamfered, err := body.Chamfer(t.Context(), Edges(CreatedBy(CapEnd(body))), units.Millimeters(d))
	require.NoError(t, err)
	cbp, ok := chamfered.payload.(capBlendPayload)
	require.True(t, ok)
	return chamfered, cbp
}

// TestCapBlendPayloadStoresEachBandsContourDisplacement is
// docs/tessellation-reach-design.md §10's task 23: the build already computes
// each band's own contour displacement once, and the payload must carry it, so
// the tessellator charges the SAME number the band's vertices, edges and areas
// were charged rather than deriving a second one.
func TestCapBlendPayloadStoresEachBandsContourDisplacement(t *testing.T) {
	t.Parallel()
	chamfered, cbp := chamferedSectionBody(t, quarterDiskSection(10), 2)
	require.Len(t, cbp.bandDelta, 1)
	stored, ok := cbp.bandDelta[capBandKey{loop: 0, start: false}]
	require.True(t, ok, `the end cap's band must be keyed by its own loop and cap`)

	// The same value, re-derived through the build's own capBandResult.
	budget := proofbound.NewWorkBudget(t.Context())
	work := freeform.NewFreeformWork()
	cl, err := oneLoopCornerLoop(budget, cbp.loops()[0], work)
	require.NoError(t, err)
	joins, err := capOffsetJoins(budget, cl, cbp.loopOffset(0))
	require.NoError(t, err)
	want, err := capband.ContourDisplacement(cl.walks, capContourJoins(joins),
		cbp.loopOffset(0), cbp.loopSetback(0).dcDelta, shellTol)
	require.NoError(t, err)
	require.Equal(t, want, stored)
	require.NotNil(t, chamfered)
}

// TestCapBlendChordingSharesOneCountPerWalk is DX3's own answer, asserted on
// the chording itself: a chamfered circular wall carries ONE count, at least as
// fine as either of its two directrices needs on its own, and the side ring and
// the cap contour ring each hold exactly that many stations. Two independently
// chosen counts would leave the band strip's two sides at different densities,
// which is the un-watertight mesh docs/modify-reach-design.md §12 row DX3
// refuses to return.
func TestCapBlendChordingSharesOneCountPerWalk(t *testing.T) {
	t.Parallel()
	t.Run("the side directrix asks for more", func(t *testing.T) {
		// A convex outer arc offsets INWARD, so its cap directrix is both
		// shorter-swept and smaller-radius: the wall's own arc sets the count.
		_, cbp := chamferedSectionBody(t, quarterDiskSection(10), 2)
		sideN, capN := requireCapBlendSharedCount(t, cbp, 0, 0.05)
		require.Greater(t, sideN, capN)
	})

	t.Run("the cap directrix asks for more", func(t *testing.T) {
		// A HOLE offsets outward, so its cap directrix is the LARGER circle and
		// its own sagitta is the one that decides. A count taken off the wall
		// alone would chord the cap contour past the requested tolerance.
		_, cbp := chamferedSectionBody(t, holedPlateSection, 2)
		sideN, capN := requireCapBlendSharedCount(t, cbp, 1, 0.05)
		require.Greater(t, capN, sideN)
	})
}

// holedPlateSection is the 100x60 plate carrying one 10 mm-radius circular
// hole — a cornerless loop whose cap contour is the WIDER concentric circle.
func holedPlateSection(s *sketch.Sketch) {
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(50, 30), 10)
}

// requireCapBlendSharedCount asserts one loop's chording holds a single count
// per walk, shared by its side ring and its cap contour ring, at least as fine
// as either directrix needs alone. It returns the two per-directrix counts of
// the loop's own circular wall so the caller can say which one decided.
func requireCapBlendSharedCount(t *testing.T, cbp capBlendPayload, li int, tol float64) (int, int) {
	t.Helper()
	lm, err := chordCapBlendLoop(t.Context(), proofbound.NewWorkBudget(t.Context()), cbp, li, cbp.loops()[li], tol, freeform.NewFreeformWork())
	require.NoError(t, err)
	n := len(lm.walks)

	circular, wantSide, wantCap := 0, 0, 0
	for i, w := range lm.walks {
		sideLen := len(lm.sidePts) - lm.sideStart[i]
		if i+1 < n {
			sideLen = lm.sideStart[i+1] - lm.sideStart[i]
		}
		capLen := len(lm.capPts) - lm.capWallStart[i]
		if i+1 < n {
			capLen = lm.capWallStart[i+1] - lm.capWallStart[i]
		}
		require.Equal(t, lm.count[i], sideLen, `walk %d's side ring holds its own count`, i)
		require.Equal(t, lm.count[i], capLen, `walk %d's cap ring holds the SAME count`, i)
		if !w.IsCircular() {
			require.Equal(t, 1, lm.count[i])
			continue
		}
		circular++
		nSide, _, err := chordCount(w.SegmentWalk, tol, chordWalkMin(w.SegmentWalk))
		require.NoError(t, err)
		capWalk := survey2d.SegmentWalk{
			Kind: survey2d.WalkCircular, Radius: lm.capRadius[i],
			Th0: lm.capTh0[i], Th1: lm.capTh1[i], Closed: w.Closed,
		}
		nCap, _, err := chordCount(capWalk, tol, chordWalkMin(capWalk))
		require.NoError(t, err)
		require.Equal(t, max(nSide, nCap), lm.count[i],
			`the shared count is the larger of what the two directrices need`)
		require.LessOrEqual(t, lm.sideSag[i], tol, `the side ring stays inside the tolerance`)
		require.LessOrEqual(t, lm.capSag[i], tol, `and so does the cap contour ring`)
		wantSide, wantCap = nSide, nCap
	}
	require.Equal(t, 1, circular, `these sections each carry one circular wall`)
	return wantSide, wantCap
}

// TestCapBlendMeshChargesTheWindowSkew pins the term a MITERED band patch owes
// beyond its own chording. Its cap directrix is the offset arc trimmed at the
// corner feet, so the two windows genuinely differ and the ruled surface the
// build assembles is not the cone sector it is tagged as; the patch's published
// displacement must cover capRadius times that skew on top of the larger of the
// two rings' sagitta.
func TestCapBlendMeshChargesTheWindowSkew(t *testing.T) {
	t.Parallel()
	const tol = 0.05
	chamfered, cbp := chamferedSectionBody(t, quarterDiskSection(10), 2)
	mesh, err := tessellateCapBlend(t.Context(), chamfered, cbp, tol, VerifyAll)
	require.NoError(t, err)

	lm, err := chordCapBlendLoop(t.Context(), proofbound.NewWorkBudget(t.Context()), cbp, 0, cbp.loops()[0], tol, freeform.NewFreeformWork())
	require.NoError(t, err)
	sag := 0.0
	for i, w := range lm.walks {
		if w.IsCircular() {
			sag = math.Max(lm.sideSag[i], lm.capSag[i])
		}
	}
	require.Positive(t, sag)

	roles := facesByRole(chamfered)
	found := 0
	for _, patch := range cbp.patches {
		if !patch.geom.Circular {
			continue
		}
		found++
		skew := capPatchWindowSkew(patch.geom)
		require.Positive(t, skew, `a mitered arc's two windows differ`)
		skewTerm := proofbound.ProductUpper(patch.geom.CapRadius, skew)
		require.Positive(t, skewTerm)
		face := roles[patch.role]
		require.NotNil(t, face)
		bound, ok := mesh.sourceBound(face)
		require.True(t, ok, `every source face states its own bound`)
		require.GreaterOrEqual(t, bound, sag+skewTerm,
			`a mitered patch owes capRadius times its window skew ON TOP of its own chording`)
	}
	require.Equal(t, 1, found)
}

// TestCapBlendBandPatchBoundCoversItsOwnChording pins the sagitta leg on the
// one band whose two directrices sweep the SAME window — a whole closed circle
// offset into a concentric one. That patch charges no window skew and no miter
// locus at all, so its published displacement is its own chording plus the
// band's level terms, and it must cover the coarser of its two rings.
func TestCapBlendBandPatchBoundCoversItsOwnChording(t *testing.T) {
	t.Parallel()
	const tol = 0.25
	chamfered, cbp := chamferedSectionBody(t, diskSection(0, 0, 10), 2)
	mesh, err := tessellateCapBlend(t.Context(), chamfered, cbp, tol, VerifyAll)
	require.NoError(t, err)
	lm, err := chordCapBlendLoop(t.Context(), proofbound.NewWorkBudget(t.Context()), cbp, 0, cbp.loops()[0], tol, freeform.NewFreeformWork())
	require.NoError(t, err)
	require.True(t, lm.whole, `a cornerless circle is the one whole-turn band`)
	sag := math.Max(lm.sideSag[0], lm.capSag[0])
	require.Positive(t, sag)

	roles := facesByRole(chamfered)
	require.Len(t, cbp.patches, 1)
	for _, patch := range cbp.patches {
		require.Equal(t, 0.0, capPatchWindowSkew(patch.geom),
			`a whole-turn band's two windows coincide`)
		bound, ok := mesh.sourceBound(roles[patch.role])
		require.True(t, ok)
		require.GreaterOrEqual(t, bound, sag,
			`a band patch's own bound covers the chording of both its directrices`)
	}
}

func TestCapBlendHoleApexPatchBoundCoversConnectorSagitta(t *testing.T) {
	t.Parallel()
	chamfered, cbp := chamferedSectionBody(t, func(s *sketch.Sketch) {
		outer := s.CreateRectangle(0, 0, 60, 40)
		s.Fix(outer.A)
		s.CreateRectangle(15, 10, 45, 30)
	}, 1.5)
	const tol = 0.05
	lm, err := chordCapBlendLoop(t.Context(), proofbound.NewWorkBudget(t.Context()), cbp, 1, cbp.loops()[1],
		tol, freeform.NewFreeformWork())
	require.NoError(t, err)
	require.Equal(t, len(lm.capPts), lm.capArcStart[0]+lm.arcCount[0],
		"corner 0's connector ends at the sample array boundary")
	require.Positive(t, lm.arcSag[0])
	mesh, err := tessellateCapBlend(t.Context(), chamfered, cbp, tol, VerifyAll)
	require.NoError(t, err)

	roles := facesByRole(chamfered)
	apexCount := 0
	for _, patch := range cbp.patches {
		if !patch.geom.Circular || patch.geom.SideRadius != 0 {
			continue
		}
		apexCount++
		bound, ok := mesh.sourceBound(roles[patch.role])
		require.True(t, ok)
		require.GreaterOrEqual(t, bound, lm.arcSag[0],
			"each apex face's own bound covers its connector chord")
	}
	require.Equal(t, 4, apexCount)
}

// TestCapBlendCornerLocusGapIsZeroOnlyWhereBothLociAreAffine pins the
// locus-gap term: a built miter ruling is tagged Line3, and it IS the denoted
// locus only where both neighbouring offsets move affinely in the setback. A
// line-line corner and every reflex foot are that case; a corner between a line
// and a circle is not, and the ruling then stands for a conic the gap measures.
func TestCapBlendCornerLocusGapIsZeroOnlyWhereBothLociAreAffine(t *testing.T) {
	t.Parallel()
	t.Run("line-line miter charges nothing", func(t *testing.T) {
		_, cbp := chamferedSectionBody(t, func(s *sketch.Sketch) {
			rect := s.CreateRectangle(0, 0, 100, 60)
			s.Fix(rect.A)
		}, 3)
		walks, joins := capBlendCornerSetup(t, cbp)
		for i, j := range joins {
			gap, err := tessellation.CapBlendCornerLocusGap(proofbound.NewWorkBudget(t.Context()),
				capBlendLocusInput(cbp, walks, i, j))
			require.NoError(t, err)
			require.Equal(t, 0.0, gap, `corner %d joins two straight walls`, i)
		}
	})

	t.Run("line-circle miter charges its conic", func(t *testing.T) {
		_, cbp := chamferedSectionBody(t, quarterDiskSection(10), 2)
		walks, joins := capBlendCornerSetup(t, cbp)
		positive := 0
		for i, j := range joins {
			gap, err := tessellation.CapBlendCornerLocusGap(proofbound.NewWorkBudget(t.Context()),
				capBlendLocusInput(cbp, walks, i, j))
			require.NoError(t, err)
			require.False(t, proofbound.IsNonFinite(gap))
			if gap > 0 {
				positive++
			}
		}
		require.Equal(t, 2, positive, `both of the arc's own corners meet a straight wall`)
	})
}

// capBlendCornerSetup resolves one payload's outer loop into the walks and
// offset joins the corner readings take.
func capBlendCornerSetup(t *testing.T, cbp capBlendPayload) ([]survey2d.SideWalk, []cornerJoin) {
	t.Helper()
	budget := proofbound.NewWorkBudget(t.Context())
	cl, err := oneLoopCornerLoop(budget, cbp.loops()[0], freeform.NewFreeformWork())
	require.NoError(t, err)
	joins, err := capOffsetJoins(budget, cl, cbp.loopOffset(0))
	require.NoError(t, err)
	return cl.walks, joins
}

func capBlendLocusInput(cbp capBlendPayload, walks []survey2d.SideWalk, i int,
	j cornerJoin,
) tessellation.CapBlendLocusInput {
	return tessellation.CapBlendLocusInput{
		Setback: capBlendProofSetback(cbp.loopSetback(0)), Segments: cbp.loops()[0].Segments,
		Walks: walks, Corner: i, Join: capBlendSampleJoins([]cornerJoin{j})[0],
		FootDelta: cbp.loopBandDelta(0),
	}
}

// TestCapStationBoundEnclosesTheStationItDenotes pins the cap contour's own
// coordinate-construction reading: a held station's gap from the certified
// enclosure of the point at that exact angle is small but never claimed zero,
// and an angle this arithmetic cannot enclose refuses with +Inf rather than a
// silently small number.
func TestCapStationBoundEnclosesTheStationItDenotes(t *testing.T) {
	t.Parallel()
	const cU, cV, r = 3.0, -7.0, 8.0
	theta := 0.9
	held := Point2{U: cU + r*math.Cos(theta), V: cV + r*math.Sin(theta)}
	bound := tessellation.CapStationBound(cU, cV, r, theta, held.U, held.V)
	require.True(t, bound.Derivable())
	require.LessOrEqual(t, proofbound.WalkEndBoundAllow(bound), 1e-12,
		`the station's own evaluation rounds at the coordinate's scale`)

	// A held pair moved well off the circle must be caught by the same reading.
	off := tessellation.CapStationBound(cU, cV, r, theta, held.U+1e-6, held.V)
	require.Greater(t, off.U, 5e-7, `a displaced station is measured, not excused`)

	require.False(t, tessellation.CapStationBound(cU, cV, r, math.Inf(1), held.U, held.V).Derivable())
	require.False(t, tessellation.CapStationBound(cU, cV, math.NaN(), theta, held.U, held.V).Derivable())
}

// TestCapBlendMeshPublishesNoVolumeProofForAMiteredBand is the proof's own
// boundary, read off the mesh record: a quarter disk's arc meets each radius at
// a genuine miter, so every face states a bound and the area slack is finite,
// but no occupied-volume allowance is published, and the boolean's own gate
// refuses the operand with the admission's reason
// (docs/tessellation-reach-design.md §7).
func TestCapBlendMeshPublishesNoVolumeProofForAMiteredBand(t *testing.T) {
	t.Parallel()
	chamfered, cbp := chamferedSectionBody(t, quarterDiskSection(10), 2)
	mesh, err := tessellateCapBlend(t.Context(), chamfered, cbp, 0.1, VerifyAll)
	require.NoError(t, err)
	require.False(t, mesh.symDiffOK, `a mitered circular wall is not covered by the slice-wise proof`)
	require.Equal(t, 0.0, mesh.volSymDiff)
	_, err = operandSymDiff(mesh)
	require.ErrorIs(t, err, ErrUnsupported)
	refusal, err := capBlendOccupiedVolumeAdmission(proofbound.NewWorkBudget(t.Context()), cbp)
	require.NoError(t, err)
	require.ErrorContains(t, refusal, `loop 0`)
	require.ErrorContains(t, refusal, `no proof of the volume`)
	require.False(t, proofbound.IsNonFinite(mesh.areaSlack))
	require.Positive(t, mesh.areaSlack)
	for _, f := range mesh.source {
		_, ok := mesh.sourceBound(f)
		require.True(t, ok, `every source face is present in the proof record`)
	}
}

// capBlendMotionUnderTest rebuilds a payload's loops, cap motion and vertices
// exactly as tessellateCapBlend does at VerifyAll, and returns the loops and
// the per-vertex motion array the occupied-volume proof reads.
func capBlendMotionUnderTest(t *testing.T, cbp capBlendPayload, chord float64) ([]capBlendLoopMesh, []float64) {
	t.Helper()
	budget := proofbound.NewWorkBudget(t.Context())
	loops := cbp.loops()
	lms := make([]capBlendLoopMesh, len(loops))
	for li, loop := range loops {
		lm, err := chordCapBlendLoop(t.Context(), budget, cbp, li, loop, chord, freeform.NewFreeformWork())
		require.NoError(t, err)
		require.NoError(t, capBlendCapMotion(budget, cbp, &lm))
		lms[li] = lm
	}
	var mesh Mesh
	store, motion, err := capBlendVertices(budget, cbp, cbp.prismLike(0, 0), lms, &mesh, true)
	require.NoError(t, err)
	require.Len(t, motion, len(store), `motion is a parallel array, one entry per vertex`)
	return lms, motion
}

// TestCapBlendMeshPublishesVolumeProofForAnAdmittedBand reads §7's proof off
// the flange's mesh: every corner is exactly tangent and the bore a whole turn,
// so the mesh publishes a finite volSymDiff at least as large as its own
// chord-polygon term, and operandSymDiff hands that figure to the boolean.
//
// It also guards the cap leg of the per-vertex motion. The bore's seam cap
// station is held at the float full turn while B1 places it at (19, 0)
// exactly, so its motion must cover that measured gap.
func TestCapBlendMeshPublishesVolumeProofForAnAdmittedBand(t *testing.T) {
	t.Parallel()
	const tol = 0.05
	chamfered, cbp := chamferedFlange(t)
	mesh, err := tessellateCapBlend(t.Context(), chamfered, cbp, tol, VerifyAll)
	require.NoError(t, err)
	require.True(t, mesh.symDiffOK)
	require.False(t, proofbound.IsNonFinite(mesh.volSymDiff))

	lms, motion := capBlendMotionUnderTest(t, cbp, tol)
	chordVolume := capBlendChordVolume(cbp, lms)
	require.Positive(t, chordVolume, `the fillet arcs and the bore are chorded`)
	require.GreaterOrEqual(t, mesh.volSymDiff, chordVolume)
	// tessellateContext is the one writer of boundaryOK, which operandSymDiff
	// reads first; the cap-blend path runs no facet-contact audit, so it is
	// true at VerifyAll and the published figure reaches the boolean.
	viaContext, err := tessellateContext(t.Context(), chamfered, units.Millimeters(tol), VerifyAll)
	require.NoError(t, err)
	got, err := operandSymDiff(viaContext)
	require.NoError(t, err)
	require.Equal(t, mesh.volSymDiff, got)

	// Shown to fail: dropping lm.capMotion[j] from the cap vertex's motion in
	// capBlendVertices turns this assertion red.
	bore := lms[1]
	require.True(t, bore.whole)
	seg := bore.loop.Segments[bore.walks[0].Segs[0]]
	held := bore.capPts[0]
	gap := proofbound.WalkEndBoundAllow(stationbound.CapOffsetStationBound(seg, 0, bore.count[0],
		capcontour.CapWallRadiusOffset(bore.walks[0], cbp.loopOffset(bore.li)), held.U, held.V))
	require.Positive(t, gap, `the float full turn leaves the seam station off (19, 0)`)
	seam := motion[bore.capHiV[0]]
	require.Positive(t, seam)
	require.GreaterOrEqual(t, seam, gap, `the seam cap vertex's motion covers its own measured gap`)
}

// TestCapBlendMeshMotionCarriesEachLevelBound guards the level legs of the
// per-vertex motion: a pin stopped ToFace against a 1e12 mm plate holds a
// COMPUTED end level, and chamfered on that end both its side ring at
// zHi = z1 − d and its cap ring at z1 carry that level's proven bound. The
// plate is no boolean operand, so the mesh is read here directly.
func TestCapBlendMeshMotionCarriesEachLevelBound(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateRectangle(120, 0, 140, 20)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var plateProfile, pinProfile *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Area > 1000 {
			plateProfile = p
		} else {
			pinProfile = p
		}
	}
	require.NotNil(t, plateProfile)
	require.NotNil(t, pinProfile)
	doc := New()
	plate, err := doc.Extrude(s, plateProfile, Distance{D: units.Millimeters(1e12), Dir: Along})
	require.NoError(t, err)
	pin, err := doc.Extrude(s, pinProfile, ToFace{
		Body:   plate,
		Face:   Faces(FaceCreatedBy(CapEnd(plate))),
		Offset: units.Millimeters(-1e-3),
	})
	require.NoError(t, err)
	chamfered, err := pin.Chamfer(t.Context(), Edges(CreatedBy(CapEnd(pin))), units.Millimeters(1))
	require.NoError(t, err)
	cbp, ok := chamfered.payload.(capBlendPayload)
	require.True(t, ok)
	capLevel := cbp.capBandLevel(cbp.z1, -1)
	require.Positive(t, capLevel.Bound, `a ToFace stop's end level is computed, and says so`)

	const tol = 1.0
	mesh, err := tessellateCapBlend(t.Context(), chamfered, cbp, tol, VerifyAll)
	require.NoError(t, err)
	require.True(t, mesh.symDiffOK, `an all-Plane band of line-line miters is admitted`)

	lms, motion := capBlendMotionUnderTest(t, cbp, tol)
	require.Len(t, lms, 1)
	lm := lms[0]
	require.Equal(t, 0.0, capBlendChordVolume(cbp, lms), `nothing in a rectangle is chorded`)
	// Shown to fail: dropping lm.zHi.bound (the side leg) or
	// capBandLevel(...).bound (the cap leg) from capBlendVertices' motion turns
	// the matching assertion red.
	require.Positive(t, lm.zHi.Bound)
	for j := range lm.sidePts {
		require.GreaterOrEqual(t, motion[lm.sideHi[j]], lm.zHi.Bound, `side vertex %d stands at a computed level`, j)
	}
	for j := range lm.capPts {
		require.GreaterOrEqual(t, motion[lm.capHiV[j]], capLevel.Bound, `cap vertex %d stands at a computed level`, j)
	}
	// The end cap is a 20×20 square shrunk by the 1 mm setback, so its area is
	// at least 18², and a level displaced by its bound sweeps at least that
	// much volume times the bound.
	require.GreaterOrEqual(t, mesh.volSymDiff, proofbound.ProductUpper(capLevel.Bound, 18*18))
}

// TestCapBlendMeshMotionCarriesPlacementRounding guards the rounding leg: a
// placed plate's every coordinate is computed, and its only motion is the
// rounding exactPrismPointRound measures, so a zero there would publish an
// exact proof the placement never earned.
func TestCapBlendMeshMotionCarriesPlacementRounding(t *testing.T) {
	t.Parallel()
	chamfered, _ := chamferedSectionBody(t, func(s *sketch.Sketch) {
		rect := s.CreateRectangle(0, 0, 100, 60)
		s.Fix(rect.A)
	}, 5)
	rot, err := r3.Rotation(r3.Vec{X: 1, Y: 2, Z: 3}, units.Radians(0.7))
	require.NoError(t, err)
	moved, err := chamfered.Placed(t.Context(), rot)
	require.NoError(t, err)
	cbp, ok := moved.payload.(capBlendPayload)
	require.True(t, ok)
	mesh, err := tessellateCapBlend(t.Context(), moved, cbp, 1, VerifyAll)
	require.NoError(t, err)
	require.True(t, mesh.symDiffOK)
	_, motion := capBlendMotionUnderTest(t, cbp, 1)
	require.Equal(t, 0.0, capBlendChordVolume(cbp, nil))
	// Shown to fail: dropping round from capBlendVertices' motion leaves every
	// entry zero and this assertion red.
	require.Positive(t, mesh.volSymDiff, `a placed plate rounds every coordinate it writes`)
	worst := 0.0
	for _, m := range motion {
		worst = math.Max(worst, m)
	}
	require.Positive(t, worst)
}

// TestCapBlendMeshMotionCarriesTheMiterDisplacement guards the line-line miter
// leg: a hexagon's offset corners sit at irrational points, so each held miter
// foot is the exact one only within the band's contour displacement, and that
// displacement is the whole of the proof.
func TestCapBlendMeshMotionCarriesTheMiterDisplacement(t *testing.T) {
	t.Parallel()
	chamfered, cbp := chamferedSectionBody(t, func(s *sketch.Sketch) {
		points := make([]*sketch.Point, 6)
		for i := range points {
			th := 2 * math.Pi * float64(i) / 6
			points[i] = s.CreatePoint(100*math.Cos(th), 100*math.Sin(th))
		}
		for i := range points {
			s.CreateLine(points[i], points[(i+1)%len(points)])
		}
		s.Fix(points[0])
	}, 2)
	delta := cbp.bandDelta[capBandKey{loop: 0, start: false}]
	require.Positive(t, delta, `an irrational miter point is held only within the band's contour displacement`)
	mesh, err := tessellateCapBlend(t.Context(), chamfered, cbp, 1, VerifyAll)
	require.NoError(t, err)
	require.True(t, mesh.symDiffOK)
	lms, motion := capBlendMotionUnderTest(t, cbp, 1)
	require.Equal(t, 0.0, capBlendChordVolume(cbp, lms), `a hexagon chords nothing`)
	// Shown to fail: dropping the bandDelta arm of capBlendCapMotion turns
	// these assertions red.
	require.Positive(t, mesh.volSymDiff)
	for j := range lms[0].capPts {
		require.GreaterOrEqual(t, motion[lms[0].capHiV[j]], delta, `miter foot %d`, j)
	}
}

// TestCapOffsetStationBoundReadsTheExactOffsetCircle pins
// stationbound.CapOffsetStationBound: the gap between a held cap station and the point the
// ideal polyhedron places on the wall's EXACT offset circle at the exact
// fraction k/n of the wall's own window, both ends of the window included.
func TestCapOffsetStationBoundReadsTheExactOffsetCircle(t *testing.T) {
	t.Parallel()
	t.Run("a clockwise bore's seam", func(t *testing.T) {
		// The bore is a whole clockwise circle walked T = 1 → 0, and a hole rim
		// grows by d. Its seam station is (19, 0) exactly, while the held sample
		// is evaluated at the float full turn, which is not 2π: the gap is real
		// and small.
		bore := circleSeg{Center: Point2{U: 0, V: 0}, Radius: units.Millimeters(18), CCW: false, TStart: 1, TEnd: 0}
		off := big.NewRat(1, 1)
		const n = 64
		heldU, heldV := 19*math.Cos(2*math.Pi), 19*math.Sin(2*math.Pi)
		first := stationbound.CapOffsetStationBound(bore, 0, n, off, heldU, heldV)
		require.True(t, first.Derivable())
		require.Positive(t, proofbound.WalkEndBoundAllow(first), `the float full turn is not 2π, and the gap says so`)
		require.Less(t, proofbound.WalkEndBoundAllow(first), 1e-13)
		last := stationbound.CapOffsetStationBound(bore, n, n, off, heldU, heldV)
		require.Equal(t, first, last, `k == n closes the turn on the same exact point as k == 0`)
	})

	t.Run("a fillet arc's foot", func(t *testing.T) {
		fillet := arcSeg{Center: Point2{U: 36, V: -22}, Start: Point2{U: 36, V: -34}, End: Point2{U: 48, V: -22}, TStart: 0, TEnd: 1}
		off := big.NewRat(-1, 1)
		at := stationbound.CapOffsetStationBound(fillet, 0, 8, off, 36, -33)
		require.True(t, at.Derivable())
		ulp := math.Nextafter(36, math.Inf(1)) - 36
		require.LessOrEqual(t, math.Max(at.U, at.V), 4*ulp, `the foot on the shrunken circle is (36, −33) to within the enclosure's own rounding`)

		displaced := stationbound.CapOffsetStationBound(fillet, 0, 8, off, 36+1e-6, -33)
		require.Greater(t, displaced.U, 5e-7, `a displaced station is measured, not excused`)
	})

	t.Run("an index or coordinate it cannot read", func(t *testing.T) {
		fillet := arcSeg{Center: Point2{U: 36, V: -22}, Start: Point2{U: 36, V: -34}, End: Point2{U: 48, V: -22}, TStart: 0, TEnd: 1}
		off := big.NewRat(-1, 1)
		require.False(t, stationbound.CapOffsetStationBound(fillet, -1, 8, off, 36, -33).Derivable())
		require.False(t, stationbound.CapOffsetStationBound(fillet, 9, 8, off, 36, -33).Derivable())
		require.False(t, stationbound.CapOffsetStationBound(fillet, 0, 8, off, math.NaN(), -33).Derivable())
		require.False(t, stationbound.CapOffsetStationBound(fillet, 0, 8, off, 36, math.Inf(1)).Derivable())
		require.False(t, stationbound.CapOffsetStationBound(fillet, 0, 8, big.NewRat(-13, 1), 36, -33).Derivable(),
			`an offset that swallows the radius denotes no circle`)
	})
}

// asymChamferedSectionBody is chamferedSectionBody's two-distance sibling: the
// end cap face is the reference, so the cap contour sits dc in and the side
// level ds below the cap (docs/modify-reach-design.md §8.3.1).
func asymChamferedSectionBody(t *testing.T, section func(*sketch.Sketch), dc, ds float64) (*Body, capBlendPayload) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	section(s)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	doc := New()
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(capBlendMeshHeight), Dir: Along})
	require.NoError(t, err)
	chamfered, err := body.Chamfer(t.Context(), Edges(CreatedBy(CapEnd(body))), units.Millimeters(dc),
		WithAsymmetricChamfer(Faces(FaceCreatedBy(CapEnd(body))), units.Millimeters(ds)))
	require.NoError(t, err)
	cbp, ok := chamfered.payload.(capBlendPayload)
	require.True(t, ok)
	require.Equal(t, capSetback{dc: dc, ds: ds}, cbp.end)
	return chamfered, cbp
}

// TestCapBlendChordVolumeChargesEachCapsBandHeight checks the occupied-volume
// proof's chord term covers a two-distance band ds tall. The disk's mesh is
// inscribed — every station sits on the true frustum at its own azimuth — so the
// body's volume less the mesh's is the slice-wise circular-segment volume the
// term bounds, and the band's share of it is ds tall, not dc.
//
// Shown to fail: charging the band at dc's height (capBlendChordVolume's
// bandHeight) leaves the term below the measured deficit.
func TestCapBlendChordVolumeChargesEachCapsBandHeight(t *testing.T) {
	t.Parallel()
	const tol = 0.25
	chamfered, cbp := asymChamferedSectionBody(t, diskSection(0, 0, 10), 1, 8)
	mesh, err := tessellateCapBlend(t.Context(), chamfered, cbp, tol, VerifyAll)
	require.NoError(t, err)
	require.True(t, mesh.symDiffOK)

	meshVol := 0.0
	for _, tri := range mesh.triangles {
		a, b, c := mesh.vertices[tri[0]], mesh.vertices[tri[1]], mesh.vertices[tri[2]]
		meshVol += a.Dot(b.Cross(c)) / 6
	}
	bodyVol := chamfered.volume.Value.Mag()
	deficit := bodyVol - meshVol
	require.Positive(t, deficit, `an inscribed mesh holds less than the body`)

	lms, _ := capBlendMotionUnderTest(t, cbp, tol)
	chordVolume := capBlendChordVolume(cbp, lms)
	require.GreaterOrEqual(t, chordVolume, deficit, `the chord term covers the measured segment volume`)
	require.GreaterOrEqual(t, mesh.volSymDiff, deficit)
}

// TestCapBlendCornerLocusGapEnclosesTheTwoDistanceLocus checks the mesh's
// locus gap at a line-circle miter covers the denoted corner locus of a
// two-distance band. The quarter disk's corner at (r, 0) moves to
// (sqrt(r² − 2·r·dc·s), s·dc) in the plane and s·ds along the sweep; every
// sample's distance from the built ruling must stay within the gap.
//
// Shown to fail: passing dc as the locus's axial span in
// CapBlendCornerLocusGap answers a zero gap for the ds > dc rows.
func TestCapBlendCornerLocusGapEnclosesTheTwoDistanceLocus(t *testing.T) {
	t.Parallel()
	const r = 10.0
	for _, tc := range []struct{ dc, ds float64 }{{1, 4}, {2, 6}, {3, 1}} {
		t.Run(fmt.Sprintf("dc=%g,ds=%g", tc.dc, tc.ds), func(t *testing.T) {
			t.Parallel()
			_, cbp := asymChamferedSectionBody(t, quarterDiskSection(r), tc.dc, tc.ds)
			walks, joins := capBlendCornerSetup(t, cbp)
			checked := 0
			for i, j := range joins {
				gap, err := tessellation.CapBlendCornerLocusGap(proofbound.NewWorkBudget(t.Context()),
					capBlendLocusInput(cbp, walks, i, j))
				require.NoError(t, err)
				if j.vU != r || j.vV != 0 {
					continue
				}
				locus := func(s float64) r3.Vec {
					return r3.NewVec(math.Sqrt(r*r-2*r*tc.dc*s), s*tc.dc, s*tc.ds)
				}
				a, b := locus(0), r3.NewVec(j.m.U, j.m.V, tc.ds)
				axis := b.Sub(a)
				worst := 0.0
				for k := range 4097 {
					p := locus(float64(k) / 4096).Sub(a)
					along := p.Dot(axis) / axis.Dot(axis)
					worst = math.Max(worst, p.Sub(axis.Scale(along)).Len())
				}
				require.Positive(t, worst, `the locus bows off its chord`)
				require.GreaterOrEqual(t, gap, worst, `the gap covers the locus`)
				checked++
			}
			require.Equal(t, 1, checked, `the corner at (r, 0)`)
		})
	}
}

// TestCapBlendMeshCapMotionCarriesSetbackRounding checks a whole circle's cap
// sample motion charges the unit conversion of the setback across the cap. A
// radius-10 disk chamfered 0.1 in holds its cap circle at 10 − 2.54 = 7.46 mm
// with no rounding of the subtraction, so its seam sample sits exactly on the
// held offset circle and off the denoted one by the conversion alone.
//
// Shown to fail: with CapBlendMotionInput.DDelta unread, the seam sample's
// motion is zero against a 3.7e-17 mm gap.
func TestCapBlendMeshCapMotionCarriesSetbackRounding(t *testing.T) {
	t.Parallel()
	const r, h, tol = 10.0, 8.0, 0.05
	inch := units.Inches(0.1)
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	diskSection(0, 0, r)(s)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := New().Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(h), Dir: Along})
	require.NoError(t, err)
	chamfered, err := body.Chamfer(t.Context(), Edges(CreatedBy(CapEnd(body))), inch)
	require.NoError(t, err)
	cbp, ok := chamfered.payload.(capBlendPayload)
	require.True(t, ok)
	require.Positive(t, cbp.end.dcDelta, `the premise: the conversion to millimetres rounds`)

	exactDC := extent.ExactConversion(inch, units.Millimeter)
	denoted := new(big.Rat).Sub(new(big.Rat).SetFloat64(r), exactDC)
	lms, _ := capBlendMotionUnderTest(t, cbp, tol)
	lm := lms[0]
	require.True(t, lm.whole)
	seam := lm.capPts[0]
	require.Zero(t, seam.V)
	gap := new(big.Rat).Abs(new(big.Rat).Sub(new(big.Rat).SetFloat64(seam.U), denoted))
	require.Positive(t, gap.Sign(), `the premise: the seam sample is not the denoted seam`)
	require.GreaterOrEqual(t, new(big.Rat).SetFloat64(lm.capMotion[0]).Cmp(gap), 0,
		`the seam sample's motion %v covers its gap from the denoted seam`, lm.capMotion[0])
}
