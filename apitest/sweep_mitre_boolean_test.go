package apitest_test

import (
	"math"
	"math/big"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// mtiltTree is the mtilt-shaped union fixture: a tapered 16-gon trunk and
// branches tapered 1.5 → 1 mm, each built at the origin and translated so its
// first millimetre is buried below the trunk's top cap, then leaning outward.
type mtiltTree struct {
	trunkTop  float64
	ring      float64
	firstSpan float64
	factors   []float64
}

func newMtiltTree() mtiltTree {
	return mtiltTree{trunkTop: 20, ring: 7.5, firstSpan: 2, factors: []float64{1.375 / 1.5, 1.2 / 1.5, 1 / 1.5}}
}

func (m mtiltTree) branch(t *testing.T, doc *decad.Document, theta float64) *decad.Body {
	t.Helper()
	s, profile := polygonSweepSketch(t, 16, 1.5)
	u := r3.NewVec(math.Cos(theta), math.Sin(theta), 0)
	path := mustPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, m.firstSpan),
		u.Scale(1.5).Add(r3.NewVec(0, 0, 5.5)), u.Scale(4).Add(r3.NewVec(0, 0, 9)))
	body, err := doc.Sweep(t.Context(), s, profile, path,
		decad.WithMitredJoins(), decad.WithSectionScale(scalars(m.factors...)...))
	require.NoError(t, err)
	move, err := r3.Translation(u.Scale(m.ring).Add(r3.NewVec(0, 0, m.trunkTop-1)))
	require.NoError(t, err)
	placed, err := body.Placed(t.Context(), move)
	require.NoError(t, err)
	return placed
}

// buriedVolume is the exact volume of a branch's part below the trunk's top
// cap: the first span is a frustum whose section scales linearly from 1 at
// its start to factors[0] at its end, cut one millimetre up.
func (m mtiltTree) buriedVolume(area *big.Rat) *big.Rat {
	f := new(big.Rat).SetFloat64(m.factors[0])
	// s = 1 + (f − 1)·(1 / firstSpan) at the trunk's top.
	s := new(big.Rat).Sub(f, big.NewRat(1, 1))
	s.Quo(s, new(big.Rat).SetFloat64(m.firstSpan))
	s.Add(s, big.NewRat(1, 1))
	sum := new(big.Rat).Add(big.NewRat(1, 1), s)
	sum.Add(sum, new(big.Rat).Mul(s, s))
	v := new(big.Rat).Mul(area, sum)
	return v.Quo(v, big.NewRat(3, 1))
}

// polygonArea is the exact shoelace area of the unplaced branch's start cap.
func polygonArea(t *testing.T, body *decad.Body) *big.Rat {
	t.Helper()
	var startCap *decad.Face
	for _, f := range body.Faces() {
		if f.Origins()[0].Role == roleCapStart {
			startCap = f
		}
	}
	require.NotNil(t, startCap)
	var pts []r3.Vec
	for _, ce := range startCap.Loops()[0].CoEdges() {
		pts = append(pts, ce.Start().Position().Value)
	}
	sum := new(big.Rat)
	for i, p := range pts {
		q := pts[(i+1)%len(pts)]
		term := new(big.Rat).Mul(new(big.Rat).SetFloat64(p.X), new(big.Rat).SetFloat64(q.Y))
		term.Sub(term, new(big.Rat).Mul(new(big.Rat).SetFloat64(q.X), new(big.Rat).SetFloat64(p.Y)))
		sum.Add(sum, term)
	}
	sum.Abs(sum)
	return sum.Quo(sum, big.NewRat(2, 1))
}

func TestSweepMitredTreeUnionChain(t *testing.T) {
	t.Parallel()
	m := newMtiltTree()
	doc := decad.New()
	trunkSketch, trunkProfile := polygonSweepSketch(t, 16, 10)
	trunk, err := doc.Sweep(t.Context(), trunkSketch, trunkProfile,
		mustPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, m.trunkTop)),
		decad.WithSectionScale(units.Scalar(0.95)))
	require.NoError(t, err)
	trunkVol, err := trunk.Volume()
	require.NoError(t, err)

	branchSketch, branchProfile := polygonSweepSketch(t, 16, 1.5)
	unit, err := decad.New().Sweep(t.Context(), branchSketch, branchProfile, mustPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1)), decad.WithMitredJoins())
	require.NoError(t, err)
	buried := m.buriedVolume(polygonArea(t, unit))

	const branches = 15
	tree := trunk
	ref := new(big.Rat).SetFloat64(trunkVol.Value.Base())
	slack := trunkVol.Bound.Base()
	for i := range branches {
		b := m.branch(t, doc, 2*math.Pi*float64(i)/branches)
		vol, err := b.Volume()
		require.NoError(t, err)
		ref.Add(ref, new(big.Rat).SetFloat64(vol.Value.Base()))
		ref.Sub(ref, buried)
		slack += vol.Bound.Base()

		start := time.Now()
		tree, err = decad.Union(t.Context(), tree, b)
		require.NoError(t, err, `union %d`, i+1)
		t.Logf("union %2d: %v, %d faces", i+1, time.Since(start), len(tree.Faces()))
		require.Len(t, tree.Lumps(), 1, `union %d is one lump`, i+1)
	}

	got, err := tree.Volume()
	require.NoError(t, err)
	gap := new(big.Rat).Sub(new(big.Rat).SetFloat64(got.Value.Base()), ref)
	gap.Abs(gap)
	allow := new(big.Rat).SetFloat64(got.Bound.Base())
	allow.Add(allow, new(big.Rat).SetFloat64(slack))
	gapF, _ := gap.Float64()
	t.Logf("volume %v ± %v, reference gap %v, operand slack %v", got.Value.Base(), got.Bound.Base(), gapF, slack)
	require.Positive(t, got.Bound.Base())
	require.LessOrEqual(t, gap.Cmp(allow), 0, `the published volume encloses the reference`)

	mesh, err := tree.Tessellate(t.Context(), units.Millimeters(0.01), decad.WithVerification(decad.VerifyBoundary))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	requireWatertight(t, mesh)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	br, err := report.ForBody(tree)
	require.NoError(t, err)
	require.Equal(t, decad.ValidityValid, br.Validity.Outcome)
	require.Equal(t, 1, br.Topology.Lumps)
}

// TestSweepMitredCrossingUnion is docs/sweep-design.md §16.10's M2 row: two
// branches that cross union into one lump whose volume lies within its bound
// of V1 + V2 − overlap. The second branch is turned to run along x and
// shifted half a millimetre in y, so no two walls are coplanar and the
// overlap is the box 2 × 1.5 × 2.
func TestSweepMitredCrossingUnion(t *testing.T) {
	t.Parallel()
	s, profile := squareSweepSketch(t, 2)
	doc := decad.New()
	path := mustPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 10))
	a, err := doc.Sweep(t.Context(), s, profile, path, decad.WithMitredJoins())
	require.NoError(t, err)
	b, err := doc.Sweep(t.Context(), s, profile, path, decad.WithMitredJoins())
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(-5, 0.5, 5), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	turn, err := r3.FromFrame(frame)
	require.NoError(t, err)
	b, err = b.Placed(t.Context(), turn)
	require.NoError(t, err)

	// Table DM row DM4: Verify reads the same pair's overlap through the
	// mesh path the boolean admits.
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Interfering, report.Status)
	require.Len(t, report.Interferences, 1)
	overlap := report.Interferences[0].Volume
	require.InDelta(t, 2*1.5*2, overlap.Value.Base(), overlap.Bound.Base())

	u, err := decad.Union(t.Context(), a, b)
	require.NoError(t, err)
	require.Len(t, u.Lumps(), 1)
	vol, err := u.Volume()
	require.NoError(t, err)
	require.InDelta(t, 40+40-2*1.5*2, vol.Value.Base(), vol.Bound.Base())
	require.Equal(t, []*decad.Body{u}, doc.Bodies())

	mesh, err := u.Tessellate(t.Context(), units.Millimeters(0.01), decad.WithVerification(decad.VerifyBoundary))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	requireWatertight(t, mesh)
}

// TestSweepMitredTaperedBranchesCross unions two tapered 16-gon branches whose
// leaning spans cross in an X. Every vertex carries delta > 0 and the walls
// cross away from every facet corner, so the hidden-tangency gate admits the
// pair only through the segment-anchored deep witness (Table DM row DM3).
// No closed form states this overlap, so the reading is checked by inclusion
// and exclusion against the intersection and the Verify interference row.
func TestSweepMitredTaperedBranchesCross(t *testing.T) {
	t.Parallel()
	build := func(doc *decad.Document) (*decad.Body, *decad.Body) {
		s, profile := polygonSweepSketch(t, 16, 1)
		path := mustPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 2), r3.NewVec(8, 0, 10))
		a, err := doc.Sweep(t.Context(), s, profile, path, decad.WithMitredJoins(), decad.WithSectionScale(scalars(0.9, 0.6)...))
		require.NoError(t, err)
		b, err := doc.Sweep(t.Context(), s, profile, path, decad.WithMitredJoins(), decad.WithSectionScale(scalars(0.9, 0.6)...))
		require.NoError(t, err)
		frame, err := r3.NewFrame(r3.NewVec(4, -4, 0), r3.NewVec(0, 1, 0), r3.NewVec(-1, 0, 0))
		require.NoError(t, err)
		turn, err := r3.FromFrame(frame)
		require.NoError(t, err)
		b, err = b.Placed(t.Context(), turn)
		require.NoError(t, err)
		return a, b
	}

	doc := decad.New()
	a, b := build(doc)
	va, err := a.Volume()
	require.NoError(t, err)
	vb, err := b.Volume()
	require.NoError(t, err)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Len(t, report.Interferences, 1)
	overlap := report.Interferences[0].Volume

	u, err := decad.Union(t.Context(), a, b)
	require.NoError(t, err)
	require.Len(t, u.Lumps(), 1)
	vu, err := u.Volume()
	require.NoError(t, err)
	require.Positive(t, overlap.Value.Base())
	require.Less(t, vu.Value.Base(), va.Value.Base()+vb.Value.Base())

	idoc := decad.New()
	ia, ib := build(idoc)
	inter, err := decad.Intersect(t.Context(), ia, ib)
	require.NoError(t, err)
	vi, err := inter.Volume()
	require.NoError(t, err)
	slack := vu.Bound.Base() + va.Bound.Base() + vb.Bound.Base() + vi.Bound.Base()
	require.InDelta(t, va.Value.Base()+vb.Value.Base()-vi.Value.Base(), vu.Value.Base(), slack+1e-12)
	require.InDelta(t, vi.Value.Base(), overlap.Value.Base(), vi.Bound.Base()+overlap.Bound.Base()+1e-12)

	mesh, err := u.Tessellate(t.Context(), units.Millimeters(0.01), decad.WithVerification(decad.VerifyBoundary))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	requireWatertight(t, mesh)
}
