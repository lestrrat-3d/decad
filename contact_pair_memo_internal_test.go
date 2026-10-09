package decad

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestContactPairMemoMatchesUncachedProof reads random poses of an exact
// planar octagon floor, a chamfered block read through its held mesh, a
// source box and a full source cylinder through the memo twice, the second
// read served from it, and requires both reports equal to the proof run
// without the memo. Every pose is drawn twice in a row and once more after
// other poses, so hits from both distances are compared.
func TestContactPairMemoMatchesUncachedProof(t *testing.T) {
	t.Parallel()
	doc := New()
	floor := bandOctagonFloor(t, doc)
	block := bandChamferedBlock(t, doc)
	box := internalBoxBody(t, doc, -5, -5, 5, 5, 10)
	disc := internalDiscBody(t, doc, 4, 6)
	req := ContactRequest{PointResolution: units.Millimeters(1e-6), NormalResolution: units.Degrees(1),
		SupportBand: units.Millimeters(0.5), HeldChord: units.Millimeters(0.05)}
	rng := rand.New(rand.NewPCG(7, 11))
	type query struct {
		a, b         *Body
		poseA, poseB r3.Transform
	}
	var queries []query
	for range 24 {
		bodies := [][2]*Body{{floor, block}, {floor, box}, {floor, disc}, {box, block}}[rng.IntN(4)]
		axis := r3.Vec{X: rng.Float64() - 0.5, Y: rng.Float64() - 0.5, Z: rng.Float64() - 0.5}
		pose := r3.Identity()
		if rng.IntN(3) > 0 {
			turn, err := r3.Rotation(axis, units.Radians(rng.Float64()*0.3))
			require.NoError(t, err)
			pose = turn
		}
		// Heights straddle the floor's top at 8 mm: separated, inside the
		// support band, touching and shallowly overlapping.
		lift := []float64{8, 8.25, 8 - 1.0/1024, 9, 20}[rng.IntN(5)]
		pose, err := r3.FromBasis(pose.Basis(), r3.Vec{X: rng.Float64()*8 - 4, Y: rng.Float64()*8 - 4, Z: lift})
		require.NoError(t, err)
		queries = append(queries, query{a: bodies[0], b: bodies[1], poseA: r3.Identity(), poseB: pose})
	}
	relations := map[ContactRelation]int{}
	check := func(q query) {
		want, wantErr := classifyContactPair(t.Context(), q.a, q.b, q.poseA, q.poseB, req)
		got, err := doc.ContactPair(t.Context(), q.a, q.b, q.poseA, q.poseB, req)
		require.Equal(t, wantErr, err)
		require.Equal(t, want, got)
		if got != nil {
			relations[got.Relation]++
		}
	}
	for _, q := range queries {
		check(q)
		check(q)
	}
	for _, q := range queries {
		check(q)
	}
	require.Greater(t, len(relations), 2, "premise: the poses reach several relations: %v", relations)
}

// TestPairReportKeyIsBitExact requires the memo key to tell apart inputs no
// == on a float could: a −0 pose coordinate from a +0, and the zero Value
// from a zero Length.
func TestPairReportKeyIsBitExact(t *testing.T) {
	t.Parallel()
	req := ContactRequest{PointResolution: units.Millimeters(1e-6), NormalResolution: units.Degrees(1)}
	plus, err := r3.Translation(r3.Vec{X: 1})
	require.NoError(t, err)
	minus, err := r3.Translation(r3.Vec{X: 1, Y: math.Copysign(0, -1)})
	require.NoError(t, err)
	require.True(t, plus == minus, "premise: == cannot tell the poses apart")
	require.Equal(t, reportvocab.NewPairReportKey[*Body](nil, plus, plus, req),
		reportvocab.NewPairReportKey[*Body](nil, plus, plus, req))
	require.NotEqual(t, reportvocab.NewPairReportKey[*Body](nil, plus, plus, req),
		reportvocab.NewPairReportKey[*Body](nil, plus, minus, req))
	require.NotEqual(t, reportvocab.NewPairReportKey[*Body](nil, plus, plus, req),
		reportvocab.NewPairReportKey[*Body](nil, minus, plus, req))
	zeroLength := req
	zeroLength.SupportBand = units.Millimeters(0)
	require.NotEqual(t, reportvocab.NewPairReportKey[*Body](nil, plus, plus, req),
		reportvocab.NewPairReportKey[*Body](nil, plus, plus, zeroLength))
}

// TestPairReportMemoEvictsOldest fills a memo past its bound: it keeps
// exactly PairReportMemoCap reports, the oldest leaves first, and storing a
// key it holds keeps the first report.
func TestPairReportMemoEvictsOldest(t *testing.T) {
	t.Parallel()
	req := ContactRequest{PointResolution: units.Millimeters(1e-6), NormalResolution: units.Degrees(1)}
	keyAt := func(i int) reportvocab.PairReportKey[*Body] {
		pose, err := r3.Translation(r3.Vec{X: float64(i)})
		require.NoError(t, err)
		return reportvocab.NewPairReportKey[*Body](nil, r3.Identity(), pose, req)
	}
	var memo pairReportMemo
	for i := range reportvocab.PairReportMemoCap + 3 {
		memo.Store(keyAt(i), &ContactReport{Relation: ContactSeparated, Reason: ContactReason(i)})
	}
	for i := range 3 {
		_, ok := memo.Load(keyAt(i))
		require.False(t, ok, "entry %d is among the oldest", i)
	}
	for _, i := range []int{3, reportvocab.PairReportMemoCap, reportvocab.PairReportMemoCap + 2} {
		report, ok := memo.Load(keyAt(i))
		require.True(t, ok, "entry %d", i)
		require.Equal(t, ContactReason(i), report.Reason)
	}
	memo.Store(keyAt(5), &ContactReport{Reason: ContactNoGapProof})
	report, ok := memo.Load(keyAt(5))
	require.True(t, ok)
	require.Equal(t, ContactReason(5), report.Reason)
	memo.Store(keyAt(0), &ContactReport{})
	_, ok = memo.Load(keyAt(3))
	require.False(t, ok, "the next oldest leaves for the next new key")
}
