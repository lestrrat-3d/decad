package decad

import (
	"math"
	"slices"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file tests the crossing audit's enumeration and cap proofs
// (docs/loft-design.md §6, docs/loft-gear-bounds-design.md §6):
// internal/loftmesh/loft_audit_sweep.go and internal/loftmesh/loft_cap_proof.go.
//
// FALSIFICATION LOG. Each leg was broken in the production source, the named
// test re-run, and the change reverted; every leg but the last went RED:
//
//   - sweepOrder.visit emitted every pair that overlaps on the sweep axis
//     alone, without the full three-axis box test:
//     TestLoftSweepEnumeratesEveryTouchingPair went RED on the gear tooth,
//     the chorded wedge and the untwisted box (the list no longer equals the
//     box-overlap set).
//   - sweepOrder.visit stopped its inner scan by comparing the lower bound on
//     the axis after the sweep axis: TestLoftSweepEnumeratesEveryTouchingPair
//     went RED on the gear tooth and the chorded wedge (overlapping pairs
//     missing from the list).
//   - CapFamilyProof's (d) took each loop edge's direction from the sign of
//     its net count and skipped the multiplicity and stray-edge checks:
//     TestLoftCapProofAgreesWithReference went RED on "duplicated cap
//     triangle" — the structured audit admitted what the reference refuses
//     (seen with the cap-proof count assertion disabled, which fails first).
//   - CapFamilyProof's (e) was deleted: TestLoftCapProofAgreesWithReference
//     went RED on "double-covered hole" the same way.
//   - loftCrossingAudit skipped loftAuditLookback and returned the member
//     pair's failure directly: every test stayed GREEN. No fixture here makes
//     a decided pair fail ahead of the first failing wall-wall pair. A cap
//     pair fails only when the loops are not simple; a proper crossing of
//     two loop edges forces a winding value outside {0, 1}, which (c), (d)
//     and (e) refuse, and a loop vertex touching another loop edge e fails a
//     wall-wall pair that names e's own wall, which sorts ahead of that
//     wall's cap pairs. The lookback stays as the stated guarantee that the
//     refused pair is the reference's, not as a path a fixture reaches.

// loftAuditFixture is one triangle set with the loft structure the
// structured audit reads.
type loftAuditFixture struct {
	verts     []r3.Vec
	tris      [][3]int
	structure loftmesh.LoftAuditStructure
}

func loftAuditFixtureOf(a loftmesh.Assembly) loftAuditFixture {
	return loftAuditFixture{
		verts: slices.Clone(a.Verts),
		tris:  slices.Clone(a.Tris),
		structure: loftmesh.LoftAuditStructure{
			Walls: a.Walls, CapStartCount: a.CapStartCount, Loops0: a.VIdx, Loops1: a.WIdx,
		},
	}
}

// withCap0 replaces the first cap's triangles, each wound to agree with the
// original first cap triangle's float normal.
func (f loftAuditFixture) withCap0(cap0 [][3]int) loftAuditFixture {
	s := f.structure
	ref := f.tris[s.Walls]
	refNormal := loftFloatNormal(f.verts, ref)
	tris := slices.Clone(f.tris[:s.Walls])
	for _, tri := range cap0 {
		if loftFloatNormal(f.verts, tri).Dot(refNormal) < 0 {
			tri = [3]int{tri[0], tri[2], tri[1]}
		}
		tris = append(tris, tri)
	}
	tris = append(tris, f.tris[s.Walls+s.CapStartCount:]...)
	f.tris = tris
	f.structure.CapStartCount = len(cap0)
	return f
}

func loftFloatNormal(verts []r3.Vec, tri [3]int) r3.Vec {
	a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
	return b.Sub(a).Cross(c.Sub(a))
}

func unitSquareAuditFixture(t *testing.T) loftAuditFixture {
	t.Helper()
	pl0, pl1 := planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 1))
	return loftAuditFixtureOf(assembleLoftFixture(t, loftPayload{
		profile0: unitSquareProfile(), profile1: unitSquareProfile(),
		plane0: pl0, plane1: pl1, frame0: mustFrame(t, pl0), frame1: mustFrame(t, pl1), xform: r3.Identity(),
	}))
}

func holedSquareAuditFixture(t *testing.T) loftAuditFixture {
	t.Helper()
	p := profileRecord{Outer: squareLoop(0.5, 0.5, 0.5, true), Holes: []loopRecord{squareLoop(0.5, 0.5, 0.2, false)}}
	pl0, pl1 := planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 1))
	return loftAuditFixtureOf(assembleLoftFixture(t, loftPayload{
		profile0: p, profile1: p,
		plane0: pl0, plane1: pl1, frame0: mustFrame(t, pl0), frame1: mustFrame(t, pl1), xform: r3.Identity(),
	}))
}

// steinerCap0 fans the unit square's first cap around one new vertex at
// (0.5, 0.5, z): on the cap plane at z = 0, below it otherwise.
func steinerCap0(f loftAuditFixture, z float64) loftAuditFixture {
	f.verts = append(slices.Clone(f.verts), r3.NewVec(0.5, 0.5, z))
	c := len(f.verts) - 1
	loop := f.structure.Loops0[0]
	var fan [][3]int
	for k := range loop {
		fan = append(fan, [3]int{loop[k], loop[(k+1)%len(loop)], c})
	}
	return f.withCap0(fan)
}

// loftAuditFixtures is every structured fixture the cap-proof tests run, with
// the number of caps whose proof holds on it, or -1 where the test does not
// pin it.
func loftAuditFixtures(t *testing.T) []struct {
	name      string
	fixture   loftAuditFixture
	capProofs int
} {
	t.Helper()
	square := unitSquareAuditFixture(t)
	holed := holedSquareAuditFixture(t)

	duplicated := square.withCap0(append(slices.Clone(square.tris[square.structure.Walls:square.structure.Walls+square.structure.CapStartCount]),
		square.tris[square.structure.Walls]))

	outer, hole := holed.structure.Loops0[0], holed.structure.Loops0[1]
	doubleCovered := holed.withCap0([][3]int{
		{outer[0], outer[1], outer[2]}, {outer[0], outer[2], outer[3]},
		{hole[0], hole[1], hole[2]}, {hole[0], hole[2], hole[3]},
	})

	flipped := square
	flipped.tris = slices.Clone(square.tris)
	first := flipped.tris[square.structure.Walls]
	flipped.tris[square.structure.Walls] = [3]int{first[0], first[2], first[1]}

	lifted := square
	lifted.verts = slices.Clone(square.verts)
	corner := square.structure.Loops0[0][0]
	lifted.verts[corner] = r3.NewVec(lifted.verts[corner].X, lifted.verts[corner].Y, -0.25)

	twistedPayload := func() loftPayload {
		p := unitSquareProfile()
		pl0 := planeAt(r3.NewVec(0, 0, 0))
		pl1 := planeRecord{Origin: r3.NewVec(1, 0, 1), U: r3.NewVec(-1, 0, 0), V: r3.NewVec(0, 1, 0)}
		return loftPayload{profile0: p, profile1: p, plane0: pl0, plane1: pl1,
			frame0: mustFrame(t, pl0), frame1: mustFrame(t, pl1), xform: r3.Identity()}
	}

	gon := profileRecord{Outer: manyGonLoop(0, 0, 10, 64)}
	pl0, pl1 := planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 1))
	gonFixture := loftAuditFixtureOf(assembleLoftFixture(t, loftPayload{profile0: gon, profile1: gon,
		plane0: pl0, plane1: pl1, frame0: mustFrame(t, pl0), frame1: mustFrame(t, pl1), xform: r3.Identity()}))

	return []struct {
		name      string
		fixture   loftAuditFixture
		capProofs int
	}{
		{name: "unit square", fixture: square, capProofs: 2},
		{name: "holed square", fixture: holed, capProofs: 2},
		{name: "64-gon", fixture: gonFixture, capProofs: 2},
		{name: "chorded wedge", fixture: loftAuditFixtureOf(chordedWedgeAssembly(t, wedgeSplinePoints(wedgeFitSpline(t), 24))), capProofs: 2},
		{name: "gear tooth z=8", fixture: loftAuditFixtureOf(loftGearAssembly(t, loftGearZ(8), 1)), capProofs: 2},
		{name: "on-plane fan vertex", fixture: steinerCap0(square, 0), capProofs: 2},
		{name: "off-plane fan vertex", fixture: steinerCap0(square, -0.25), capProofs: 1},
		{name: "lifted loop vertex", fixture: lifted, capProofs: 1},
		{name: "flipped cap triangle", fixture: flipped, capProofs: 1},
		{name: "duplicated cap triangle", fixture: duplicated, capProofs: 1},
		{name: "double-covered hole", fixture: doubleCovered, capProofs: 1},
		{name: "over-twisted square", fixture: loftAuditFixtureOf(assembleLoftFixture(t, twistedPayload())), capProofs: -1},
	}
}

// TestLoftSweepEnumeratesEveryTouchingPair checks sweepCandidates against
// brute force: on the gear tooth, the chorded wedge and the crossing
// fixtures, the candidate list is exactly the set of pairs whose closed boxes
// overlap, in strict lexicographic order, and it contains every pair the
// exact classification finds touching (a shared vertex index, or any
// contact). The refused pair is the reference's on every crossing fixture.
func TestLoftSweepEnumeratesEveryTouchingPair(t *testing.T) {
	t.Parallel()
	type fixture struct {
		name  string
		verts []r3.Vec
		tris  [][3]int
	}
	var fixtures []fixture
	gear := loftGearAssembly(t, loftGearZ(8), 1)
	fixtures = append(fixtures, fixture{"gear tooth z=8", gear.Verts, gear.Tris})
	wedge := chordedWedgeAssembly(t, wedgeSplinePoints(wedgeFitSpline(t), 24))
	fixtures = append(fixtures, fixture{"chorded wedge", wedge.Verts, wedge.Tris})
	for _, row := range []struct {
		name string
		make func() ([]r3.Vec, [][3]int)
	}{
		{"untwisted box", func() ([]r3.Vec, [][3]int) { return boxLoftVerts(), boxLoftTris() }},
		{"far-apart triangles", func() ([]r3.Vec, [][3]int) { return syntheticLoftTriangles(40) }},
		{"stacked triangles", func() ([]r3.Vec, [][3]int) { return stackedLoftTriangles(40, 0.5) }},
		{"zero-shared-vertex crossing", genuineCrossingFixture},
		{"touching only at a shared box boundary", boundaryTouchingFixture},
		{"one-shared-vertex pair crossing away", vertexCrossesAwayFixture},
		{"two-shared-vertex pair crossing off its edge", sameSideApexesFixture},
	} {
		verts, tris := row.make()
		fixtures = append(fixtures, fixture{row.name, verts, tris})
	}

	for _, fx := range fixtures {
		t.Run(fx.name, func(t *testing.T) {
			cands, err := loftmesh.LoftSweepCandidates(proofbound.NewWorkBudget(t.Context()), fx.verts, fx.tris)
			require.NoError(t, err)
			for k := 1; k < len(cands); k++ {
				prev, cur := cands[k-1], cands[k]
				require.True(t, prev[0] < cur[0] || (prev[0] == cur[0] && prev[1] < cur[1]),
					"candidates must run in strict lexicographic order")
			}
			listed := make(map[[2]int]struct{}, len(cands))
			for _, c := range cands {
				listed[c] = struct{}{}
			}

			data := loftmesh.NewLoftAuditData(fx.verts, fx.tris)
			overlapping, touching := 0, 0
			for i := range fx.tris {
				boxI := meshbool.TriBox(fx.verts, fx.tris[i])
				for j := i + 1; j < len(fx.tris); j++ {
					_, inList := listed[[2]int{i, j}]
					overlap := meshbool.BoxesOverlap(boxI, meshbool.TriBox(fx.verts, fx.tris[j]))
					require.Equal(t, overlap, inList, "pair (%d, %d): the list must be exactly the box-overlap set", i, j)
					if overlap {
						overlapping++
					}
					_, shared := tessellation.SharedVertexIndices(fx.tris[i], fx.tris[j])
					contact, err := meshbool.TriTriClassify(data.Corners[i], data.Corners[j], data.Xtris[i], data.Xtris[j], data.Norms[i], data.Norms[j])
					require.NoError(t, err)
					if shared == 0 && contact.Kind == meshbool.ContactNone {
						continue
					}
					touching++
					require.True(t, inList, "touching pair (%d, %d) must be a candidate", i, j)
				}
			}
			require.Equal(t, overlapping, len(cands))
			t.Logf("%s: F=%d all pairs=%d candidates=%d touching=%d", fx.name, len(fx.tris), len(fx.tris)*(len(fx.tris)-1)/2, len(cands), touching)
			requireLoftCrossingAuditVerdictsMatch(t, fx.verts, fx.tris)
		})
	}
}

// TestLoftCapProofAgreesWithReference runs every structured fixture through
// the structured audit (all four shortcuts) and through the zero-shortcut
// reference, and requires the identical verdict, refused pair included. It
// also pins how many cap proofs hold, so the agreement is not vacuous: real
// lofts decide both caps, and each mutated cap falls back to pairwise
// testing.
func TestLoftCapProofAgreesWithReference(t *testing.T) {
	t.Parallel()
	for _, row := range loftAuditFixtures(t) {
		t.Run(row.name, func(t *testing.T) {
			fx := row.fixture
			_, refErr := loftmesh.LoftCrossingAuditWork(proofbound.NewWorkBudget(t.Context()), fx.verts, fx.tris, loftAuditReference)
			for _, arm := range []struct {
				name      string
				shortcuts loftmesh.LoftAuditShortcuts
			}{
				{name: "cap proof only", shortcuts: loftmesh.LoftAuditShortcuts{CapProof: true}},
				{name: "cap proof and sweep", shortcuts: loftmesh.LoftAuditShortcuts{CapProof: true, Sweep: true}},
				{name: "structured", shortcuts: loftAuditStructured},
			} {
				work, gotErr := loftmesh.LoftCrossingAuditStructuredWork(proofbound.NewWorkBudget(t.Context()),
					fx.verts, fx.tris, fx.structure, arm.shortcuts, proofbound.MaxFacetPairTestsPerCall)
				if row.capProofs >= 0 {
					require.Equal(t, row.capProofs, work.CapProofs, "%s: cap proofs that hold", arm.name)
				}
				if refErr == nil {
					require.NoError(t, gotErr, "%s must not refuse what the reference admits", arm.name)
					continue
				}
				require.Error(t, gotErr, "%s must not admit what the reference refuses", arm.name)
				require.Equal(t, refErr.Error(), gotErr.Error(), "%s must name the reference's refused pair", arm.name)
			}
		})
	}
}

// TestLoftCapProofRefusesOverlappingTriangulation names the condition each
// mutated cap fails, and shows its pairs fall back to pairwise testing: the
// structured audit's candidate count rises above the one the unmutated cap
// leaves.
func TestLoftCapProofRefusesOverlappingTriangulation(t *testing.T) {
	t.Parallel()
	want := map[string]loftmesh.LoftCapCondition{
		"unit square":             loftmesh.LoftCapProofHolds,
		"on-plane fan vertex":     loftmesh.LoftCapProofHolds,
		"off-plane fan vertex":    loftmesh.LoftCapOffPlane,
		"lifted loop vertex":      loftmesh.LoftCapOffPlane,
		"flipped cap triangle":    loftmesh.LoftCapOrientation,
		"duplicated cap triangle": loftmesh.LoftCapBoundary,
		"double-covered hole":     loftmesh.LoftCapLoopArea,
	}
	rows := loftAuditFixtures(t)
	baseline := map[string]int{}
	for _, row := range rows {
		if row.name != "unit square" && row.name != "holed square" {
			continue
		}
		work, err := loftmesh.LoftCrossingAuditStructuredWork(proofbound.NewWorkBudget(t.Context()),
			row.fixture.verts, row.fixture.tris, row.fixture.structure, loftAuditStructured, proofbound.MaxFacetPairTestsPerCall)
		require.NoError(t, err)
		baseline[row.name] = work.Candidates
	}
	for _, row := range rows {
		cond, ok := want[row.name]
		if !ok {
			continue
		}
		t.Run(row.name, func(t *testing.T) {
			fx := row.fixture
			data := loftmesh.NewLoftAuditData(fx.verts, fx.tris)
			require.Equal(t, cond, loftmesh.CapFamilyProof(data, fx.tris, fx.structure, 0), "first cap")
			require.Equal(t, loftmesh.LoftCapProofHolds, loftmesh.CapFamilyProof(data, fx.tris, fx.structure, 1), "second cap")
			if cond == loftmesh.LoftCapProofHolds {
				return
			}
			work, _ := loftmesh.LoftCrossingAuditStructuredWork(proofbound.NewWorkBudget(t.Context()),
				fx.verts, fx.tris, fx.structure, loftAuditStructured, proofbound.MaxFacetPairTestsPerCall)
			base := baseline["unit square"]
			if row.name == "double-covered hole" {
				base = baseline["holed square"]
			}
			require.Greater(t, work.Candidates, base, "the failed cap's pairs must join the pairwise pass")
		})
	}
}

// TestLoftAuditCandidateCeilingRefuses is S8 over the structured audit's own
// work, on the gear tooth. The counting pass scans at least as many pairs as
// it finds candidates, and S8 holds that scan to the ceiling: a ceiling one
// below the scan count refuses with ErrUnsupported before any pair test, and
// a ceiling equal to it admits. Both counts are far below F*(F-1)/2, which is
// what S8 compared before the sweep.
func TestLoftAuditCandidateCeilingRefuses(t *testing.T) {
	t.Parallel()
	fx := loftAuditFixtureOf(loftGearAssembly(t, loftGearZ(8), 1))
	run := func(ceiling uint64) (loftmesh.LoftAuditWork, error) {
		return loftmesh.LoftCrossingAuditStructuredWork(proofbound.NewWorkBudget(t.Context()),
			fx.verts, fx.tris, fx.structure, loftAuditStructured, ceiling)
	}
	full, err := run(math.MaxUint64)
	require.NoError(t, err)
	require.Equal(t, 2, full.CapProofs)
	require.Positive(t, full.Candidates)
	require.LessOrEqual(t, full.Candidates, full.Scanned, "every candidate is a scanned pair")
	scanned := uint64(full.Scanned)
	allPairs := uint64(len(fx.tris) * (len(fx.tris) - 1) / 2)
	require.Less(t, scanned*2, allPairs, "the structured audit scans a small share of the pairs")
	t.Logf("gear tooth z=8: F=%d all pairs=%d scanned=%d candidates=%d", len(fx.tris), allPairs, scanned, full.Candidates)

	work, err := run(scanned - 1)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Zero(t, work.Skips+work.EdgeCerts+work.VertexCerts+work.Classifications, "S8 must refuse before any pair test")

	_, err = run(scanned)
	require.NoError(t, err)
}

// tallTubeTriangles builds an open tube of radius 1 and height 100 from n
// quads around its circumference, two triangles each. Every triangle spans
// the full height, so every pair overlaps on the sweep axis (z) while only
// neighbours overlap on all three: the sweep scans F·(F−1)/2 pairs to find
// about 3F candidates.
func tallTubeTriangles(n int) ([]r3.Vec, [][3]int) {
	verts := make([]r3.Vec, 0, 2*n)
	for k := range n {
		theta := 2 * math.Pi * float64(k) / float64(n)
		verts = append(verts, r3.NewVec(math.Cos(theta), math.Sin(theta), 0), r3.NewVec(math.Cos(theta), math.Sin(theta), 100))
	}
	tris := make([][3]int, 0, 2*n)
	for k := range n {
		b, top := 2*k, 2*k+1
		bn, tn := 2*((k+1)%n), 2*((k+1)%n)+1
		tris = append(tris, [3]int{b, bn, tn}, [3]int{b, tn, top})
	}
	return verts, tris
}

// slantedStackTriangles builds n thin triangles, each rising from z = 0 to
// z = 1 across the same unit square and shifted by 1e-4 along X from the
// last, so every two boxes overlap on all three axes.
func slantedStackTriangles(n int) ([]r3.Vec, [][3]int) {
	verts := make([]r3.Vec, 0, 3*n)
	tris := make([][3]int, 0, n)
	for k := range n {
		dx := 1e-4 * float64(k)
		verts = append(verts, r3.NewVec(dx, 0, 0), r3.NewVec(1+dx, 0, 1), r3.NewVec(dx, 1, 1))
		tris = append(tris, [3]int{3 * k, 3*k + 1, 3*k + 2})
	}
	return verts, tris
}

// TestLoftAuditEnumerationTracksCandidates is S8 over the enumeration's own
// work. A tall tube of 20000 triangles overlaps on every sweep axis, so the
// sweep would scan about 2·10^8 pairs to find about 3F candidates; the grid
// works a small multiple of the candidates instead, and the audit and the
// same shape built as a 3000-gon loft 100 tall both admit. A stack of 4100
// triangles whose boxes all overlap has more candidates than the ceiling:
// the counting pass refuses before any pair is tested, having stepped the
// budget at most F + ceiling + 1 times.
//
// Shown to fail: with newPairScan returning the sweep alone, the tube and
// the 3000-gon loft refused as S8.
func TestLoftAuditEnumerationTracksCandidates(t *testing.T) {
	t.Parallel()
	t.Run("tall tube", func(t *testing.T) {
		const n = 10000
		verts, tris := tallTubeTriangles(n)
		start := time.Now()
		work, err := loftmesh.LoftCrossingAuditWork(proofbound.NewWorkBudget(t.Context()), verts, tris, loftAuditProduction)
		t.Logf("tall tube F=%d: scanned=%d candidates=%d in %s", len(tris), work.Scanned, work.Candidates, time.Since(start))
		require.NoError(t, err)
		require.Less(t, work.Scanned, 20*work.Candidates, "the enumeration's work tracks the candidates")
	})
	t.Run("3000-gon loft 100 tall", func(t *testing.T) {
		p := profileRecord{Outer: manyGonLoop(0, 0, 1, 3000)}
		pl0, pl1 := planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 100))
		pl := loftPayload{profile0: p, profile1: p, plane0: pl0, plane1: pl1,
			frame0: mustFrame(t, pl0), frame1: mustFrame(t, pl1), xform: r3.Identity()}
		_, err := evalLoft(t.Context(), New(), producerID(0), pl, proofbound.NewWorkBudget(t.Context()),
			freeform.NewFreeformWork(), freeform.NewFreeformWork())
		require.NoError(t, err)
	})
	t.Run("overlapping stack", func(t *testing.T) {
		verts, tris := slantedStackTriangles(4100)
		calls := 0
		budget := &proofbound.WorkBudget{
			StepFn: func() error { calls++; return nil },
			ErrFn:  func() error { return nil },
		}
		work, err := loftmesh.LoftCrossingAuditWork(budget, verts, tris, loftAuditProduction)
		require.ErrorIs(t, err, ErrUnsupported)
		require.LessOrEqual(t, calls, len(tris)+proofbound.MaxFacetPairTestsPerCall+1,
			"S6 steps once per triangle and the counting pass once per unit of work, up to the ceiling")
		require.Zero(t, work.Skips+work.EdgeCerts+work.VertexCerts+work.Classifications, "no pair may be tested")
	})
}

// TestLoftAuditTriangleCeilingRefuses is S8's triangle ceiling: one triangle
// past loftmesh.MaxLoftAuditTriangles refuses before S6 steps the budget once,
// so no triangle is lifted to exact coordinates.
func TestLoftAuditTriangleCeilingRefuses(t *testing.T) {
	t.Parallel()
	verts, tris := syntheticLoftTriangles(loftmesh.MaxLoftAuditTriangles + 1)
	calls := 0
	budget := &proofbound.WorkBudget{
		StepFn: func() error { calls++; return nil },
		ErrFn:  func() error { return nil },
	}
	err := loftmesh.LoftCrossingAuditStructured(budget, verts, tris, len(tris), 0, nil, nil)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "triangles exceed the fixed ceiling")
	require.Zero(t, calls, "the triangle ceiling refuses before S6")
}
