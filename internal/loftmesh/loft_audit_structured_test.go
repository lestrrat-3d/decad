package loftmesh

import (
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// structuredAuditFixture is a triangle set with the loft structure
// LoftCrossingAuditStructured reads.
type structuredAuditFixture struct {
	verts     []r3.Vec
	tris      [][3]int
	structure LoftAuditStructure
}

func (f structuredAuditFixture) clone() structuredAuditFixture {
	f.verts = slices.Clone(f.verts)
	f.tris = slices.Clone(f.tris)
	return f
}

// withCap0 replaces the first cap's triangles.
func (f structuredAuditFixture) withCap0(cap0 [][3]int) structuredAuditFixture {
	s := f.structure
	tris := slices.Clone(f.tris[:s.Walls])
	tris = append(tris, cap0...)
	tris = append(tris, f.tris[s.Walls+s.CapStartCount:]...)
	f.tris = tris
	f.structure.CapStartCount = len(cap0)
	return f
}

// holedSquarePrism is a 4×4 square with a 2×2 square hole, extruded from
// z = 0 to z = 1, assembled as a loft assembles it: 16 wall triangles, then
// the top cap's eight annulus triangles, then the bottom cap's eight. Every
// coordinate is a small integer, so every exact sign is plain.
func holedSquarePrism() structuredAuditFixture {
	outer := [][2]float64{{0, 0}, {4, 0}, {4, 4}, {0, 4}}
	inner := [][2]float64{{1, 1}, {3, 1}, {3, 3}, {1, 3}}
	var verts []r3.Vec
	for _, z := range []float64{0, 1} {
		for _, ring := range [][][2]float64{outer, inner} {
			for _, p := range ring {
				verts = append(verts, r3.NewVec(p[0], p[1], z))
			}
		}
	}
	// Bottom outer 0-3, bottom inner 4-7, top outer 8-11, top inner 12-15.
	var tris [][3]int
	for b := range 4 {
		c := (b + 1) % 4
		tris = append(tris, [3]int{b, c, 8 + c}, [3]int{b, 8 + c, 8 + b})
		tris = append(tris, [3]int{4 + b, 12 + b, 12 + c}, [3]int{4 + b, 12 + c, 4 + c})
	}
	walls := len(tris)
	for k := range 4 {
		k1 := (k + 1) % 4
		tris = append(tris, [3]int{8 + k, 8 + k1, 12 + k1}, [3]int{8 + k, 12 + k1, 12 + k})
	}
	for k := range 4 {
		k1 := (k + 1) % 4
		tris = append(tris, [3]int{k, 4 + k1, k1}, [3]int{k, 4 + k, 4 + k1})
	}
	return structuredAuditFixture{verts: verts, tris: tris, structure: LoftAuditStructure{
		Walls: walls, CapStartCount: 8,
		Loops0: [][]int{{8, 9, 10, 11}, {12, 13, 14, 15}},
		Loops1: [][]int{{0, 1, 2, 3}, {4, 5, 6, 7}},
	}}
}

// structuredFrustum is capsTestFrustum's ring frustum in the order a loft
// assembles it: walls, then the first cap, then the second.
func structuredFrustum(t *testing.T) structuredAuditFixture {
	t.Helper()
	m := capsTestFrustum(16)
	order, ok := capsReorder(len(m.verts), m.tris, m.layout)
	require.True(t, ok)
	return structuredAuditFixture{verts: m.verts, tris: order, structure: LoftAuditStructure{
		Walls: m.layout.Walls, CapStartCount: len(m.layout.Cap0), Loops0: m.layout.Loops0, Loops1: m.layout.Loops1,
	}}
}

type structuredAuditRow struct {
	name    string
	fixture structuredAuditFixture
	// cap0 is the condition CapFamilyProof names for the first cap; the
	// second cap's proof holds on every row.
	cap0 LoftCapCondition
}

// structuredAuditRows is the holed prism and the frustum, then the holed
// prism with its first cap broken one condition at a time.
func structuredAuditRows(t *testing.T) []structuredAuditRow {
	t.Helper()
	holed := holedSquarePrism()
	s := holed.structure
	cap0 := holed.tris[s.Walls : s.Walls+s.CapStartCount]

	lifted := holed.clone()
	lifted.verts[10] = r3.NewVec(lifted.verts[10].X, lifted.verts[10].Y, 1.25)

	flipped := holed.clone()
	tri := flipped.tris[s.Walls+3]
	flipped.tris[s.Walls+3] = [3]int{tri[0], tri[2], tri[1]}

	duplicated := holed.clone().withCap0(append(slices.Clone(cap0), cap0[0]))

	outer, hole := s.Loops0[0], s.Loops0[1]
	doubleCovered := holed.clone().withCap0([][3]int{
		{outer[0], outer[1], outer[2]}, {outer[0], outer[2], outer[3]},
		{hole[0], hole[1], hole[2]}, {hole[0], hole[2], hole[3]},
	})

	return []structuredAuditRow{
		{name: "holed square prism", fixture: holed, cap0: LoftCapProofHolds},
		{name: "ring frustum", fixture: structuredFrustum(t), cap0: LoftCapProofHolds},
		{name: "lifted loop vertex", fixture: lifted, cap0: LoftCapOffPlane},
		{name: "flipped cap triangle", fixture: flipped, cap0: LoftCapOrientation},
		{name: "duplicated cap triangle", fixture: duplicated, cap0: LoftCapBoundary},
		{name: "double-covered hole", fixture: doubleCovered, cap0: LoftCapLoopArea},
	}
}

func structuredAudit(t *testing.T, f structuredAuditFixture, ceiling uint64) (LoftAuditWork, error) {
	t.Helper()
	return loftCrossingAudit(proofbound.NewWorkBudget(t.Context()), f.verts, f.tris, &f.structure,
		loftStructuredShortcuts, ceiling)
}

// TestLoftCapProofRefusesOverlappingTriangulation pins which condition of
// CapFamilyProof (agent-docs/loft-gear-bounds-design.md §6) each broken first
// cap fails: a loop vertex lifted off the plane fails (a), a flipped
// triangle (c), a second copy of one triangle (d), and a
// hole covered by its own two triangles on top of the outer square's (a
// positively oriented hole) fails (e). The second cap's proof still holds,
// and the failed cap's pairs join the pairwise pass.
func TestLoftCapProofRefusesOverlappingTriangulation(t *testing.T) {
	t.Parallel()
	intact, err := structuredAudit(t, holedSquarePrism(), proofbound.MaxFacetPairTestsPerCall)
	require.NoError(t, err)
	require.Equal(t, 2, intact.CapProofs)
	for _, row := range structuredAuditRows(t) {
		t.Run(row.name, func(t *testing.T) {
			t.Parallel()
			f := row.fixture
			data := NewLoftAuditData(f.verts, f.tris)
			require.Equal(t, row.cap0, CapFamilyProof(data, f.tris, f.structure, 0), "first cap")
			require.Equal(t, LoftCapProofHolds, CapFamilyProof(data, f.tris, f.structure, 1), "second cap")
			if row.cap0 == LoftCapProofHolds {
				return
			}
			work, _ := structuredAudit(t, f, proofbound.MaxFacetPairTestsPerCall)
			require.Equal(t, 1, work.CapProofs)
			require.Greater(t, work.Candidates, intact.Candidates, "the failed cap's pairs must join the pairwise pass")
		})
	}
}

// TestLoftCapProofAgreesWithReference runs the loft's structured audit and
// the zero-shortcut reference (every pair through the exact classification)
// on every structured fixture: both admit, or both refuse with the same
// text. The reference refuses the duplicated triangle and the double-covered
// hole.
//
// Shown to fail: deleting CapFamilyProof's (d) admits the duplicated
// triangle; deleting (e) admits the double-covered hole.
func TestLoftCapProofAgreesWithReference(t *testing.T) {
	t.Parallel()
	refused := 0
	for _, row := range structuredAuditRows(t) {
		f := row.fixture
		_, refErr := LoftCrossingAuditWork(proofbound.NewWorkBudget(t.Context()), f.verts, f.tris, LoftAuditShortcuts{})
		_, gotErr := structuredAudit(t, f, proofbound.MaxFacetPairTestsPerCall)
		if refErr == nil {
			require.NoError(t, gotErr, "%s: the structured audit must not refuse what the reference admits", row.name)
			continue
		}
		refused++
		require.Error(t, gotErr, "%s: the structured audit must not admit what the reference refuses", row.name)
		require.Equal(t, refErr.Error(), gotErr.Error(), "%s: the refusal must name the reference's pair", row.name)
	}
	require.Equal(t, 2, refused, "the duplicated triangle and the double-covered hole overlap")
}

// TestLoftAuditCandidateCeilingRefuses lowers S8's ceiling one scan unit
// below what the structured audit of the holed prism scans: the audit refuses
// with ErrLoftAuditCeiling before testing any pair, and admits at the exact
// count (agent-docs/loft-gear-bounds-design.md §6, "S8 counts candidates").
func TestLoftAuditCandidateCeilingRefuses(t *testing.T) {
	t.Parallel()
	f := holedSquarePrism()
	full, err := structuredAudit(t, f, math.MaxUint64)
	require.NoError(t, err)
	require.Equal(t, 2, full.CapProofs)
	require.Positive(t, full.Candidates)
	require.LessOrEqual(t, full.Candidates, full.Scanned, "every candidate is a scanned pair")
	allPairs := len(f.tris) * (len(f.tris) - 1) / 2
	require.Less(t, 2*full.Scanned, allPairs, "the structured audit scans a small share of the pairs")

	scanned := uint64(full.Scanned) //nolint:gosec // a pair count from a 32-triangle mesh.
	work, err := structuredAudit(t, f, scanned-1)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorIs(t, err, ErrLoftAuditCeiling)
	require.Zero(t, work.Skips+work.EdgeCerts+work.VertexCerts+work.Classifications, "S8 must refuse before any pair test")

	_, err = structuredAudit(t, f, scanned)
	require.NoError(t, err)
}

// TestLoftSweepEnumeratesEveryTouchingPair checks the audit's candidate list
// (LoftSweepCandidates) against brute force: it is exactly the set of pairs
// whose closed boxes overlap, in strict lexicographic order, so it holds
// every pair that shares a vertex or touches. The audit over it reaches the
// zero-shortcut reference's verdict, refused pair included.
//
// Shown to fail: dropping the three-axis box test from sweepOrder.visit
// lists pairs whose boxes overlap on the sweep axis alone.
func TestLoftSweepEnumeratesEveryTouchingPair(t *testing.T) {
	t.Parallel()
	type fixture struct {
		name  string
		verts []r3.Vec
		tris  [][3]int
	}
	holed := holedSquarePrism()
	frustum := structuredFrustum(t)
	fixtures := []fixture{
		{"holed square prism", holed.verts, holed.tris},
		{"ring frustum", frustum.verts, frustum.tris},
		{"crossing", []r3.Vec{
			r3.NewVec(0, 0, 0), r3.NewVec(2, 0, 0), r3.NewVec(0, 2, 0),
			r3.NewVec(0.5, 0.5, -1), r3.NewVec(0.75, 0.5, 1), r3.NewVec(0.5, 0.75, 1),
			r3.NewVec(5, 5, 5), r3.NewVec(6, 5, 5), r3.NewVec(5, 6, 5),
		}, [][3]int{{6, 7, 8}, {0, 1, 2}, {3, 4, 5}}},
		{"touching at a box corner", []r3.Vec{
			r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(1, 1, 0),
			r3.NewVec(1, 1, 0), r3.NewVec(2, 1, 0), r3.NewVec(2, 2, 0),
		}, [][3]int{{0, 1, 2}, {3, 4, 5}}},
	}
	refused := 0
	for _, fx := range fixtures {
		cands, err := LoftSweepCandidates(proofbound.NewWorkBudget(t.Context()), fx.verts, fx.tris)
		require.NoError(t, err, fx.name)
		for k := 1; k < len(cands); k++ {
			prev, cur := cands[k-1], cands[k]
			require.True(t, prev[0] < cur[0] || (prev[0] == cur[0] && prev[1] < cur[1]),
				"%s: candidates must run in strict lexicographic order", fx.name)
		}
		listed := make(map[[2]int]bool, len(cands))
		for _, c := range cands {
			listed[c] = true
		}
		overlapping := 0
		for i := range fx.tris {
			boxI := meshbool.TriBox(fx.verts, fx.tris[i])
			for j := i + 1; j < len(fx.tris); j++ {
				overlap := meshbool.BoxesOverlap(boxI, meshbool.TriBox(fx.verts, fx.tris[j]))
				require.Equal(t, overlap, listed[[2]int{i, j}], "%s pair (%d, %d): the list is the box-overlap set", fx.name, i, j)
				if overlap {
					overlapping++
				}
				if _, shared := tessellation.SharedVertexIndices(fx.tris[i], fx.tris[j]); shared > 0 {
					require.True(t, listed[[2]int{i, j}], "%s pair (%d, %d) shares a vertex", fx.name, i, j)
				}
			}
		}
		require.Len(t, cands, overlapping, fx.name)

		_, refErr := LoftCrossingAuditWork(proofbound.NewWorkBudget(t.Context()), fx.verts, fx.tris, LoftAuditShortcuts{})
		_, gotErr := LoftCrossingAuditWork(proofbound.NewWorkBudget(t.Context()), fx.verts, fx.tris, loftGenericShortcuts)
		if refErr == nil {
			require.NoError(t, gotErr, fx.name)
			continue
		}
		refused++
		require.Error(t, gotErr, fx.name)
		require.Equal(t, refErr.Error(), gotErr.Error(), "%s: the refused pair must be the reference's", fx.name)
	}
	require.Equal(t, 2, refused, "the crossing and corner-touching fixtures must be refused")
}
