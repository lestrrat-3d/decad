package decad

import (
	"math"
	"testing"

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
	chamfered, err := body.Chamfer(Edges(CreatedBy(CapEnd(body))), units.Millimeters(d))
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
	chamfered, cbp := chamferedSectionBody(t, quarterDiskSection(10), 2)
	require.Len(t, cbp.bandDelta, 1)
	stored, ok := cbp.bandDelta[capBandKey{loop: 0, start: false}]
	require.True(t, ok, `the end cap's band must be keyed by its own loop and cap`)

	// The same value, re-derived through the build's own capBandResult.
	budget := newWorkBudget(t.Context())
	work := newFreeformWork()
	cl, err := oneLoopCornerLoop(budget, cbp.loops()[0], work)
	require.NoError(t, err)
	joins, err := capOffsetJoins(budget, cl, cbp.d)
	require.NoError(t, err)
	want, err := capContourDelta(cl.walks, joins, cbp.d)
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
	lm, err := chordCapBlendLoop(t.Context(), newWorkBudget(t.Context()), cbp, li, cbp.loops()[li], tol, newFreeformWork())
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
		if !w.isCircular() {
			require.Equal(t, 1, lm.count[i])
			continue
		}
		circular++
		nSide, _, err := chordCount(w.segmentWalk, tol, chordWalkMin(w.segmentWalk))
		require.NoError(t, err)
		capWalk := segmentWalk{
			kind: walkCircular, radius: lm.capRadius[i],
			th0: lm.capTh0[i], th1: lm.capTh1[i], closed: w.closed,
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
	const tol = 0.05
	chamfered, cbp := chamferedSectionBody(t, quarterDiskSection(10), 2)
	mesh, err := tessellateCapBlend(t.Context(), chamfered, cbp, tol)
	require.NoError(t, err)

	lm, err := chordCapBlendLoop(t.Context(), newWorkBudget(t.Context()), cbp, 0, cbp.loops()[0], tol, newFreeformWork())
	require.NoError(t, err)
	sag := 0.0
	for i, w := range lm.walks {
		if w.isCircular() {
			sag = math.Max(lm.sideSag[i], lm.capSag[i])
		}
	}
	require.Positive(t, sag)

	roles := facesByRole(chamfered)
	found := 0
	for _, patch := range cbp.patches {
		if !patch.geom.circular {
			continue
		}
		found++
		skew := capPatchWindowSkew(patch.geom)
		require.Positive(t, skew, `a mitered arc's two windows differ`)
		skewTerm := productUpper(patch.geom.capRadius, skew)
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
	const tol = 0.25
	chamfered, cbp := chamferedSectionBody(t, diskSection(0, 0, 10), 2)
	mesh, err := tessellateCapBlend(t.Context(), chamfered, cbp, tol)
	require.NoError(t, err)
	lm, err := chordCapBlendLoop(t.Context(), newWorkBudget(t.Context()), cbp, 0, cbp.loops()[0], tol, newFreeformWork())
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

// TestCapBlendCornerLocusGapIsZeroOnlyWhereBothLociAreAffine pins the
// locus-gap term: a built miter ruling is tagged Line3, and it IS the denoted
// locus only where both neighbouring offsets move affinely in the setback. A
// line-line corner and every reflex foot are that case; a corner between a line
// and a circle is not, and the ruling then stands for a conic the gap measures.
func TestCapBlendCornerLocusGapIsZeroOnlyWhereBothLociAreAffine(t *testing.T) {
	t.Run("line-line miter charges nothing", func(t *testing.T) {
		_, cbp := chamferedSectionBody(t, func(s *sketch.Sketch) {
			rect := s.CreateRectangle(0, 0, 100, 60)
			s.Fix(rect.A)
		}, 3)
		walks, joins := capBlendCornerSetup(t, cbp)
		for i, j := range joins {
			gap, err := capBlendCornerLocusGap(newWorkBudget(t.Context()), cbp, walks, i, j)
			require.NoError(t, err)
			require.Equal(t, 0.0, gap, `corner %d joins two straight walls`, i)
		}
	})

	t.Run("line-circle miter charges its conic", func(t *testing.T) {
		_, cbp := chamferedSectionBody(t, quarterDiskSection(10), 2)
		walks, joins := capBlendCornerSetup(t, cbp)
		positive := 0
		for i, j := range joins {
			gap, err := capBlendCornerLocusGap(newWorkBudget(t.Context()), cbp, walks, i, j)
			require.NoError(t, err)
			require.False(t, isNonFinite(gap))
			if gap > 0 {
				positive++
			}
		}
		require.Equal(t, 2, positive, `both of the arc's own corners meet a straight wall`)
	})
}

// capBlendCornerSetup resolves one payload's outer loop into the walks and
// offset joins the corner readings take.
func capBlendCornerSetup(t *testing.T, cbp capBlendPayload) ([]sideWalk, []cornerJoin) {
	t.Helper()
	budget := newWorkBudget(t.Context())
	cl, err := oneLoopCornerLoop(budget, cbp.loops()[0], newFreeformWork())
	require.NoError(t, err)
	joins, err := capOffsetJoins(budget, cl, cbp.d)
	require.NoError(t, err)
	return cl.walks, joins
}

// TestCapStationBoundEnclosesTheStationItDenotes pins the cap contour's own
// coordinate-construction reading: a held station's gap from the certified
// enclosure of the point at that exact angle is small but never claimed zero,
// and an angle this arithmetic cannot enclose refuses with +Inf rather than a
// silently small number.
func TestCapStationBoundEnclosesTheStationItDenotes(t *testing.T) {
	const cU, cV, r = 3.0, -7.0, 8.0
	theta := 0.9
	held := Point2{U: cU + r*math.Cos(theta), V: cV + r*math.Sin(theta)}
	bound := capStationBound(cU, cV, r, theta, held.U, held.V)
	require.True(t, bound.derivable())
	require.LessOrEqual(t, walkEndBoundAllow(bound), 1e-12,
		`the station's own evaluation rounds at the coordinate's scale`)

	// A held pair moved well off the circle must be caught by the same reading.
	off := capStationBound(cU, cV, r, theta, held.U+1e-6, held.V)
	require.Greater(t, off.u, 5e-7, `a displaced station is measured, not excused`)

	require.False(t, capStationBound(cU, cV, r, math.Inf(1), held.U, held.V).derivable())
	require.False(t, capStationBound(cU, cV, math.NaN(), theta, held.U, held.V).derivable())
}

// TestCapBlendMeshPublishesNoVolumeProof is the increment's own boundary, read
// off the mesh record: every face states a bound and the area slack is finite,
// but no occupied-volume allowance is published, so the boolean's own gate
// refuses the operand (docs/tessellation-reach-design.md §7, §9).
func TestCapBlendMeshPublishesNoVolumeProof(t *testing.T) {
	chamfered, cbp := chamferedSectionBody(t, quarterDiskSection(10), 2)
	mesh, err := tessellateCapBlend(t.Context(), chamfered, cbp, 0.1)
	require.NoError(t, err)
	require.False(t, mesh.symDiffOK, `no occupied-volume homotopy is proven for a cap blend`)
	require.Equal(t, 0.0, mesh.volSymDiff)
	_, err = operandSymDiff(mesh)
	require.ErrorIs(t, err, ErrUnsupported)
	require.False(t, isNonFinite(mesh.areaSlack))
	require.Positive(t, mesh.areaSlack)
	for _, f := range mesh.source {
		_, ok := mesh.sourceBound(f)
		require.True(t, ok, `every source face is present in the proof record`)
	}
}
