package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file covers the record-level half of docs/tessellation-reach-design.md
// §4: the loftMeshProof evalLoft composes, and the restatement tessellateLoft
// publishes from it. The public half — mesh shape, boolean admission, export
// determinism — is tessellate_loft_test.go.

// loftProofOf reads the payload's own composed mesh proof off a built body.
func loftProofOf(t *testing.T, body *Body) (loftPayload, loftMeshProof) {
	t.Helper()
	lp, ok := body.payload.(loftPayload)
	require.True(t, ok, "a lofted body carries a loftPayload")
	return lp, lp.proof
}

// twistedArcWedgePayload is a same-kind ArcSeg pairing whose two wedges share
// one half-sweep but not one radius, so every wall cell is genuinely TWISTED:
// T = vLo - vHi - wLo + wHi is the difference of the two sections' own rung
// vectors, which two unequal radii make nonzero.
//
// Both wedges use arcWedgeLoopEqualRadii, whose arc records two EXACTLY equal
// radii, so neither arc end charges an arc-end radial residual
// (docs/loft-design.md §5.2) and the half-sweep atan(v/u) is small enough that
// a single chord already meets the target. The build therefore carries a zero
// delta beside a positive sectionDelta, which is what isolates the facet
// departure's chorded half from its placement half.
func twistedArcWedgePayload(t *testing.T) loftPayload {
	t.Helper()
	const u0, v0 = 5.0, 1.0 / 1024
	const u1, v1 = 2.5, 0.5 / 1024
	p0 := ProfileRecord{Outer: arcWedgeLoopEqualRadii(u0, v0)}
	p1 := ProfileRecord{Outer: arcWedgeLoopEqualRadii(u1, v1)}
	return loftPayloadFor(t, p0, p1, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1))
}

// TestLoftMeshProofComposition pins docs/tessellation-reach-design.md §4's
// composition table term by term, over the three builds that separate its
// legs: the pinned unplaced one where every term is zero BY VALUE, the placed
// LineSeg-only one where docs/loft-design.md §5.2's facet-departure row
// reduces matchedDelta to delta, and the chorded twisted one where the
// parameter-matched chord departure and the facet twist both charge.
func TestLoftMeshProofComposition(t *testing.T) {
	t.Parallel()
	t.Run("unplaced pinned LineSeg-only publishes zero", func(t *testing.T) {
		body := evalLoftFixture(t, boxLoftPayload(t))
		lp, proof := loftProofOf(t, body)
		require.Zero(t, lp.delta, "the fixture must actually be the pinned, unplaced case")
		require.Zero(t, lp.sectionDelta)
		require.Zero(t, proof.facetDeparture, "every held facet IS the boundary this build denotes")
		require.Zero(t, proof.areaSlack)
		require.Zero(t, proof.volSymDiff)
	})

	t.Run("placed LineSeg-only reduces the facet departure to delta", func(t *testing.T) {
		pl := boxLoftPayload(t)
		rot, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
		require.NoError(t, err)
		shift, err := r3.Translation(r3.NewVec(12, -5, 3))
		require.NoError(t, err)
		xform, err := rot.Then(shift)
		require.NoError(t, err)
		pl.xform = xform

		lp, proof := loftProofOf(t, evalLoftFixture(t, pl))
		require.Positive(t, lp.delta, "a placed build's every held vertex carries the motion's own rounding")
		require.Zero(t, lp.sectionDelta, "a LineSeg pairing chords no curve")
		// The composition is absSumUpper(chordCellDeltaUpper(0, delta), 0),
		// which is delta widened only by its own outward rounding. It must
		// never read as the ZERO the build's chorded gate is left at.
		require.Positive(t, proof.facetDeparture)
		require.GreaterOrEqual(t, proof.facetDeparture, lp.delta)
		require.InEpsilon(t, lp.delta, proof.facetDeparture, 1e-12,
			"a LineSeg-only build's facet departure is its own delta, up to the composition's outward rounding")
		require.Positive(t, proof.areaSlack, "a displaced vertex moves the area of every facet it touches")
		require.Positive(t, proof.volSymDiff, "and sweeps volume at the rate of the surface it moved")
	})

	t.Run("chorded twisted pair exceeds the sagitta", func(t *testing.T) {
		lp, proof := loftProofOf(t, evalLoftFixture(t, twistedArcWedgePayload(t)))
		require.Positive(t, lp.sectionDelta, "a circular pairing chords its own curve")
		require.Greater(t, proof.facetDeparture, lp.sectionDelta,
			"the facet departure adds the wall's own twist to the parameter-matched chord departure, "+
				"and a SET-distance sagitta can never stand in for either")
		require.Greater(t, proof.facetDeparture, lp.delta)
		require.Positive(t, proof.areaSlack)
		require.Positive(t, proof.volSymDiff)
	})

	t.Run("a chorded pair under a non-identity motion charges both halves", func(t *testing.T) {
		unplaced := twistedArcWedgePayload(t)
		_, flat := loftProofOf(t, evalLoftFixture(t, unplaced))

		placedPayload := unplaced
		motion, err := r3.Translation(r3.NewVec(1e6, 0, 0))
		require.NoError(t, err)
		placedPayload.xform = motion
		lp, proof := loftProofOf(t, evalLoftFixture(t, placedPayload))

		// The translation is large enough that its own rigidRoundAllow is far
		// above the ulp scale, so the placement leg cannot hide inside the
		// chorded one.
		require.Positive(t, lp.delta)
		require.Greater(t, proof.facetDeparture, flat.facetDeparture,
			"a placed build charges its held vertices' own displacement beside the chord departure")
		require.Greater(t, proof.volSymDiff, flat.volSymDiff,
			"and sweeps volume the unplaced build does not")
	})

	t.Run("an unplaced build holding a station §5.2 does not pin", func(t *testing.T) {
		// A TRIMMED LineSeg start is GENERATED, not pinned: walkOf places it
		// with a float lerp while the record denotes the exact rational one,
		// so the build carries a positive delta at r3.Identity() and a zero
		// sectionDelta beside it.
		p := trimmedLineTriangleProfile()
		lp, proof := loftProofOf(t, evalLoftFixture(t,
			loftPayloadFor(t, p, p, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1))))
		require.Equal(t, r3.Identity(), lp.xform, "the fixture must reach its positive delta unplaced")
		require.Positive(t, lp.delta, "a trimmed LineSeg start is a GENERATED station, never a pinned one")
		require.Zero(t, lp.sectionDelta, "a LineSeg pairing still chords no curve")
		require.GreaterOrEqual(t, proof.facetDeparture, lp.delta)
		require.InEpsilon(t, lp.delta, proof.facetDeparture, 1e-12)
	})
}

// TestTessellateLoftRestatesTheHeldTriangleSet is the restatement's own
// acceptance line (docs/tessellation-design.md §2's loftPayload row): the mesh
// carries the payload's triangles, vertices and per-face proofs unchanged, in
// fresh storage that does not alias the body's own boundary.
func TestTessellateLoftRestatesTheHeldTriangleSet(t *testing.T) {
	t.Parallel()
	body := evalLoftFixture(t, twistedArcWedgePayload(t))
	lp, proof := loftProofOf(t, body)

	mesh, err := tessellateLoft(t.Context(), body, lp)
	require.NoError(t, err)
	require.Equal(t, lp.tris, mesh.triangles, "the restatement retriangulates nothing")
	require.Equal(t, lp.verts, mesh.vertices, "and moves, rounds and welds no coordinate")
	require.NotSame(t, &lp.verts[0], &mesh.vertices[0], "the held mesh must not alias the payload's own arrays")
	require.NotSame(t, &lp.tris[0], &mesh.triangles[0])

	require.Equal(t, proof.facetDeparture, mesh.bound)
	require.Equal(t, proof.areaSlack, mesh.areaSlack)
	require.Equal(t, proof.volSymDiff, mesh.volSymDiff)
	require.True(t, mesh.symDiffOK, "the payload's occupied-volume proof has landed, so the boolean may compose it")

	// Every source face states its own bound, and each is the payload's own
	// facet departure: a face's facets ARE the payload's triangles for it, so
	// there is no tighter per-face reading to publish and no face may be
	// missing (facesOfMesh refuses a mesh that leaves one unstated).
	require.Len(t, mesh.source, len(mesh.triangles))
	seen := map[*Face]struct{}{}
	for _, f := range mesh.source {
		seen[f] = struct{}{}
		got, ok := mesh.sourceBound(f)
		require.True(t, ok)
		require.Equal(t, proof.facetDeparture, got)
	}
	require.Len(t, seen, len(body.Faces()), "every live face must own at least one facet")

	// The two caps' triangles sit after the walls, in the payload's own split
	// order, and the wall triangles carry one distinct face each — a loft
	// coalesces no wall (docs/tessellation-design.md §4).
	capStart := mesh.source[lp.walls]
	capEnd := mesh.source[len(mesh.triangles)-1]
	require.NotSame(t, capStart, capEnd)
	require.Equal(t, roleCapStart, capStart.Origins()[0].Role)
	require.Equal(t, roleCapEnd, capEnd.Origins()[0].Role)
	walls := map[*Face]struct{}{}
	for k := range lp.walls {
		walls[mesh.source[k]] = struct{}{}
	}
	require.Len(t, walls, lp.walls, "each wall triangle names its own side(i,j,k) face")
}

// TestTessellateLoftRefusesAnUnrestatablePayload covers the two refusal
// families docs/tessellation-reach-design.md §4 assigns this path. Neither is
// reachable from a build — evalLoft copies all five fields from one assembly
// and composes the proof beside them — so each is driven by handing
// tessellateLoft a payload no evaluator wrote.
func TestTessellateLoftRefusesAnUnrestatablePayload(t *testing.T) {
	t.Parallel()
	body := evalLoftFixture(t, boxLoftPayload(t))
	base, _ := loftProofOf(t, body)

	t.Run("a triangle set its own counts do not partition", func(t *testing.T) {
		cases := map[string]func(lp *loftPayload){
			"no triangles":       func(lp *loftPayload) { lp.tris = nil },
			"walls past the set": func(lp *loftPayload) { lp.walls = len(lp.tris) + 1 },
			"unnamed cells":      func(lp *loftPayload) { lp.cell = nil },
		}
		for name, mutate := range cases {
			t.Run(name, func(t *testing.T) {
				lp := base
				mutate(&lp)
				mesh, err := tessellateLoft(t.Context(), body, lp)
				require.Nil(t, mesh)
				require.ErrorIs(t, err, ErrDegenerate)
			})
		}
	})

	t.Run("a proof term the build could not state", func(t *testing.T) {
		for name, mutate := range map[string]func(p *loftMeshProof){
			"facet departure": func(p *loftMeshProof) { p.facetDeparture = math.Inf(1) },
			"area slack":      func(p *loftMeshProof) { p.areaSlack = math.Inf(1) },
			"volSymDiff":      func(p *loftMeshProof) { p.volSymDiff = math.NaN() },
		} {
			t.Run(name, func(t *testing.T) {
				lp := base
				mutate(&lp.proof)
				mesh, err := tessellateLoft(t.Context(), body, lp)
				require.Nil(t, mesh, "an absent proof must never reach a consumer as a bound")
				require.ErrorIs(t, err, ErrUnsupported)
			})
		}
	})

	t.Run("a body whose topology contradicts its payload", func(t *testing.T) {
		// The payload names side(0,j,k) roles the body no longer carries.
		stripped := &Body{doc: body.doc, origin: body.origin, solid: true}
		mesh, err := tessellateLoft(t.Context(), stripped, base)
		require.Nil(t, mesh)
		require.ErrorIs(t, err, ErrDegenerate)
	})
}
