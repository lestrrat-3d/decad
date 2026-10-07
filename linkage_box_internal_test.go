package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file pins the pieces of docs/linkage-check-design.md §14 that sit
// below any public fixture's observability: a cell's τ_half, the split-axis
// choice and the shallowest-first order. Each test records the legs seen to
// fail when deleted.

// craneBoxRun is scene 6 of §14.8 read into a joint-box run: the mast
// (mover 0) turning [0°, 80°] about Z, the boom (mover 1) sliding [0, 30] mm
// along the mast's +X, and a wall at y ∈ [62, 82] (static 0).
func craneBoxRun(t *testing.T, budget int) *boxRun {
	t.Helper()
	doc := New()
	mast := internalBoxBody(t, doc, -5, -5, 5, 5, 38)
	boom := internalBoxBodyAtZ(t, doc, 10, -5, 60, 5, 40, 10)
	internalBoxBodyAtZ(t, doc, -100, 62, 150, 82, 30, 70)
	l := NewLinkage()
	turn, err := l.Ground().Revolute(r3.Vec{}, r3.NewVec(0, 0, 1), []*Body{mast})
	require.NoError(t, err)
	extend, err := turn.Prismatic(r3.NewVec(1, 0, 0), []*Body{boom})
	require.NoError(t, err)
	spec, axes, err := l.resolveBox(JointBox{
		{Link: turn, Min: units.Degrees(0), Max: units.Degrees(80)},
		{Link: extend, Min: units.Millimeters(0), Max: units.Millimeters(30)},
	})
	require.NoError(t, err)
	require.Equal(t, []int{0, 1}, axes)
	frames, ok := linkageFrames(spec)
	require.True(t, ok)
	bounds, ok := readLinkBounds(spec, frames)
	require.True(t, ok)
	cfg, budgetRead, err := resolveJointBoxOptions([]JointBoxOption{WithCellBudget(budget)})
	require.NoError(t, err)
	run := newLinkageRun(t.Context(), doc, spec, frames, bounds, cfg)
	return &boxRun{run: run, dr: run.drive.(*linkageDriver), axes: axes, budget: budgetRead}
}

func craneRoot() *boxCell {
	return &boxCell{lo: []*big.Rat{new(big.Rat), new(big.Rat)}, hi: []*big.Rat{big.NewRat(1, 1), big.NewRat(1, 1)}}
}

// TestJointBoxHalfTravel pins τ_half for scene 6's root against
// ½·(ρ·span_θ + 30), with ρ the ball reading of the boom about the mast's
// axis, 35 + √675 + 30 (§5.2), and span_θ the 80° range in radians with π at
// its upper enclosure: about ½·(91·(4π/9) + 30) ≈ 78.5 mm. A cell of a
// quarter of each range halves the root's θ term and its d term twice over.
//
// Legs seen to fail when deleted: the revolute joint's term, the prismatic
// joint's term, and the halving.
func TestJointBoxHalfTravel(t *testing.T) {
	t.Parallel()
	b := craneBoxRun(t, defaultCellBudget)
	r := b.run
	require.Len(t, r.movers, 2)
	require.Len(t, r.statics, 1)
	require.Equal(t, -1, r.pairs[1][0].other, `the boom's first pair is the wall`)
	require.False(t, r.pairs[1][0].excluded)
	require.True(t, r.pairs[0][0].excluded, `the mast's swept box clears the wall`)
	require.True(t, r.pairs[0][1].excluded, `the layer exclusion settles the mast and the boom`)

	rho := b.dr.bounds[1].rho[0]
	require.InDelta(t, 35+math.Sqrt(675)+30, linkRatFloat(t, rho), 1e-9)
	zero, _ := motionbound.ExactMotionParam(units.Degrees(0))
	top, _ := motionbound.ExactMotionParam(units.Degrees(80))
	spanTheta := zero.SpanUpper(top)

	root := craneRoot()
	want := new(big.Rat).Mul(rho, spanTheta)
	want.Add(want, big.NewRat(30, 1))
	want.Quo(want, big.NewRat(2, 1))
	got := b.halfTravel(root, 1, 0)
	require.Zero(t, want.Cmp(got), `τ_half is half the weighted span, got %s want %s`, got.FloatString(9), want.FloatString(9))
	require.InDelta(t, (91*(4*math.Pi/9)+30)/2, linkRatFloat(t, got), 0.1)

	quarter := &boxCell{lo: []*big.Rat{new(big.Rat), new(big.Rat)}, hi: []*big.Rat{big.NewRat(1, 4), big.NewRat(1, 4)}}
	terms := b.pairTerms(quarter, 1, 0)
	require.Len(t, terms, 2)
	require.Equal(t, 0, terms[0].joint)
	require.Equal(t, 1, terms[1].joint)
	twenty, _ := motionbound.ExactMotionParam(units.Degrees(20))
	require.Zero(t, new(big.Rat).Mul(rho, zero.SpanUpper(twenty)).Cmp(terms[0].value))
	require.Zero(t, big.NewRat(15, 2).Cmp(terms[1].value))
}

// TestJointBoxSplitAxis: scene 6's root is held back by (boom, wall) alone,
// whose θ term 91·(4π/9) ≈ 127 mm exceeds its d term 30 mm, so the root
// splits along θ; a cell already at the floor along θ splits along d.
//
// A cell spanning 1/512 of θ's range and all of d's holds a θ term of about
// 91·(80°/512) ≈ 0.25 mm against d's 30 mm, so it splits along d although θ
// is still wider than the floor.
//
// Legs seen to fail when deleted: weighting the axis by the held pair's term
// (the thin cell splits along θ, the earliest link); never splitting an axis
// at the floor; and keeping a cell a never-evaluated pair holds back out of
// the split.
func TestJointBoxSplitAxis(t *testing.T) {
	t.Parallel()
	b := craneBoxRun(t, defaultCellBudget)
	root := craneRoot()
	require.NoError(t, b.evaluate(root))
	require.Equal(t, CellUndecided, root.outcome)
	require.Equal(t, [][2]int{{1, 0}}, root.held)
	require.Equal(t, 0, b.splitAxis(root))

	thin := &boxCell{
		lo:      []*big.Rat{new(big.Rat), new(big.Rat)},
		hi:      []*big.Rat{big.NewRat(1, 512), big.NewRat(1, 1)},
		outcome: CellUndecided,
		held:    [][2]int{{1, 0}},
	}
	require.Equal(t, 1, b.splitAxis(thin), `the axis that lowers τ_half most`)

	floor := b.run.cfg.resolutionP.Base
	narrow := &boxCell{
		lo:      []*big.Rat{new(big.Rat), new(big.Rat)},
		hi:      []*big.Rat{new(big.Rat).Set(floor), big.NewRat(1, 1)},
		outcome: CellUndecided,
		held:    [][2]int{{1, 0}},
	}
	require.Equal(t, 1, b.splitAxis(narrow), `an axis at the floor is never split`)
	narrow.hi[1] = new(big.Rat).Set(floor)
	require.Equal(t, -1, b.splitAxis(narrow), `a cell at the floor along every axis is not splittable`)
	narrow.hi[1] = big.NewRat(1, 1)
	narrow.stuck = true
	require.Equal(t, -1, b.splitAxis(narrow), `a cell a never-evaluated pair holds back is not split`)
}

// TestJointBoxShallowestFirst: under a budget of 64 centres, scene 6's
// subdivision evaluates its centres level by level — every centre's depth is
// at least the one evaluated before it — and its leaves are in cell order,
// each one's θ range ending where the next begins or the d ranges stacking.
//
// Legs seen to fail when deleted: the level-by-level order (splitting each
// child as soon as it is evaluated puts a deeper centre before a shallower
// one).
func TestJointBoxShallowestFirst(t *testing.T) {
	t.Parallel()
	b := craneBoxRun(t, 64)
	require.NoError(t, b.subdivide())
	require.LessOrEqual(t, len(b.evaluated), 64)
	require.True(t, b.exhausted)
	for n := 1; n < len(b.evaluated); n++ {
		require.GreaterOrEqual(t, b.evaluated[n].depth, b.evaluated[n-1].depth)
	}
	area := new(big.Rat)
	for _, c := range b.leaves {
		w := new(big.Rat).Sub(c.hi[0], c.lo[0])
		area.Add(area, w.Mul(w, new(big.Rat).Sub(c.hi[1], c.lo[1])))
		require.False(t, c.split)
	}
	require.Zero(t, area.Cmp(big.NewRat(1, 1)), `the leaves tile the box exactly`)
}

// TestVerifyJointBoxKeepsTheNextProducerIdentity is the document identity
// half of the non-mutation test: the producer, level and curve counters are
// what they were before the call.
func TestVerifyJointBoxKeepsTheNextProducerIdentity(t *testing.T) {
	t.Parallel()
	doc := New()
	cube := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	internalBoxBody(t, doc, 25, 2, 35, 8, 10)
	l := NewLinkage()
	slide, err := l.Ground().Prismatic(r3.NewVec(1, 0, 0), []*Body{cube})
	require.NoError(t, err)
	before := doc.nextProducerID()
	levelBefore, curveBefore := doc.nextLevel, doc.nextCurve
	report, err := doc.VerifyJointBox(t.Context(), l, JointBox{{Link: slide, Min: units.Millimeters(0), Max: units.Millimeters(30)}},
		WithResolution(units.Scalar(1.0/4)))
	require.NoError(t, err)
	require.Greater(t, report.CellsEvaluated, 1)
	require.Equal(t, before, doc.nextProducerID())
	require.Equal(t, levelBefore, doc.nextLevel)
	require.Equal(t, curveBefore, doc.nextCurve)
}

// TestVerifyJointBoxRefusesABodyItDidNotBuild is §8's row the public API
// cannot reach: a link body this evaluator did not build is ErrUnsupported.
func TestVerifyJointBoxRefusesABodyItDidNotBuild(t *testing.T) {
	t.Parallel()
	doc := New()
	foreignBuilt := &Body{doc: doc, kind: BodySolid}
	doc.commit(foreignBuilt)
	l := NewLinkage()
	slide, err := l.Ground().Prismatic(r3.NewVec(1, 0, 0), []*Body{foreignBuilt})
	require.NoError(t, err)
	before := doc.Bodies()
	report, err := doc.VerifyJointBox(t.Context(), l, JointBox{{Link: slide, Min: units.Millimeters(0), Max: units.Millimeters(1)}})
	require.ErrorIs(t, err, ErrUnsupported)
	require.Nil(t, report)
	require.Equal(t, before, doc.Bodies())
}

// TestJointBoxBlockedAllowance pins the blocked certificate's allowance
// (docs/linkage-check-design.md §14.3): each moving body of the colliding pair
// is charged SweptVolumeAllow of its own τ_half and its proven area upper
// bound, and a static partner nothing. The boom's area is 2·(50·10 + 50·10 +
// 10·10) = 2200 mm². On a cell of a quarter of each range, the boom against
// the static wall is charged its whole path's τ_half; against the mast, which
// shares the turn as their common ancestor, only the slide's half span, 15/4
// mm. A volume whose proven lower end exceeds the allowance blocks, and one
// equal to it does not.
//
// Legs seen to fail when deleted: charging the moving body (the equal volume
// blocks); charging the static partner an area (the volume just above no
// longer blocks); charging the moving partner of a link-link pair (the equal
// volume blocks).
func TestJointBoxBlockedAllowance(t *testing.T) {
	t.Parallel()
	b := craneBoxRun(t, defaultCellBudget)
	r := b.run
	area := r.movers[1].area
	require.InDelta(t, 2200, area, 1e-6)
	require.GreaterOrEqual(t, area, 2200.0, `the area is a proven upper bound`)
	quarter := &boxCell{lo: []*big.Rat{new(big.Rat), new(big.Rat)}, hi: []*big.Rat{big.NewRat(1, 4), big.NewRat(1, 4)}}
	volume := func(v float64) Measurement {
		return Measurement{Value: units.CubicMillimeters(v), Exactness: Approximate, Bound: units.CubicMillimeters(0)}
	}
	cases := []struct {
		name   string
		mover  int
		pair   int
		travel *big.Rat
	}{
		{"against the static wall", 1, 0, b.halfTravel(quarter, 1, 0)},
		{"against the mast", 0, 1, big.NewRat(15, 4)},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			allow := proofbound.SweptVolumeAllow(proofbound.RatFloatUp(tc.travel), area)
			if tc.mover == 0 {
				// The mast's own share is 0; the two bodies' terms are summed
				// rounding up.
				allow = proofbound.AbsSumUpper(0, allow)
			}
			require.Positive(t, allow)
			require.False(t, b.blocks(quarter, tc.mover, tc.pair, volume(allow)), `an equal volume never blocks`)
			require.True(t, b.blocks(quarter, tc.mover, tc.pair, volume(math.Nextafter(allow, math.Inf(1)))))
		})
	}
}

// TestJointBoxDependentDelta pins δ_j of docs/linkage-check-design.md §16.3
// on hand-made enclosures: with the centre's enclosure M at an end of the
// hull H, as over a stretch where the dependent is monotone, δ is the hull's
// whole width; with M in its middle, half of it, plus M's own width on the
// far side.
//
// Legs seen to fail when deleted: δ as half the hull's width (the end case
// reads 1/2); δ as §15.5's two-sided travel term (the end case reads 2).
func TestJointBoxDependentDelta(t *testing.T) {
	t.Parallel()
	iv := func(lo, hi *big.Rat) proofbound.RatInterval { return proofbound.IntervalOwned(lo, hi) }
	h := iv(new(big.Rat), big.NewRat(1, 1))
	eps := big.NewRat(1, 1_000_000_000)
	cases := []struct {
		name string
		m    proofbound.RatInterval
		want *big.Rat
	}{
		{"at the lower end", iv(new(big.Rat), eps), big.NewRat(1, 1)},
		{"at the upper end", iv(new(big.Rat).Sub(big.NewRat(1, 1), eps), big.NewRat(1, 1)), big.NewRat(1, 1)},
		{"in the middle", iv(big.NewRat(1, 2), big.NewRat(1, 2)), big.NewRat(1, 2)},
		{"in the middle, wide", iv(new(big.Rat).Sub(big.NewRat(1, 2), eps), new(big.Rat).Add(big.NewRat(1, 2), eps)), new(big.Rat).Add(big.NewRat(1, 2), eps)},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			got := dependentDelta(tc.m, h)
			require.Zero(t, tc.want.Cmp(got), `δ is %s, want %s`, got.FloatString(12), tc.want.FloatString(12))
		})
	}
}

// internalRockerTheta4 is scene 7's follower angle from the ground line at
// crank angle th2, on the branch with the coupler pin above the ground line.
func internalRockerTheta4(th2 float64) float64 {
	d := math.Sqrt(100*100 + 30*30 - 2*100*30*math.Cos(th2))
	phi := math.Atan2(30*math.Sin(th2), 30*math.Cos(th2)-100)
	beta := math.Acos((70*70 + d*d - 80*80) / (2 * 70 * d))
	return math.Mod(phi-beta+2*math.Pi, 2*math.Pi)
}

// rockerGateBoxRun is internalRocker with a gate, x ∈ [−50, 150],
// y ∈ [72, 82], z ∈ [19, 29], on a prismatic joint under the ground along
// (0, −1, 0), read into a joint-box run over the crank's [0°, 90°] and the
// gate's [0, 5] mm: scene 11 of §16.8 with the follower a box and the gate's
// range halved.
func rockerGateBoxRun(t *testing.T) *boxRun {
	t.Helper()
	l, crank := internalRocker(t)
	doc := crank.bodies[0].doc
	gate, err := l.Ground().Prismatic(r3.NewVec(0, -1, 0), []*Body{internalBoxBodyAtZ(t, doc, -50, 72, 150, 82, 19, 10)})
	require.NoError(t, err)
	spec, axes, err := l.resolveBox(JointBox{
		{Link: crank, Min: units.Degrees(0), Max: units.Degrees(90)},
		{Link: gate, Min: units.Millimeters(0), Max: units.Millimeters(5)},
	})
	require.NoError(t, err)
	require.Equal(t, []int{0, 3}, axes)
	frames, ok := linkageFrames(spec)
	require.True(t, ok)
	cfg, budget, err := resolveJointBoxOptions(nil)
	require.NoError(t, err)
	require.NoError(t, spec.prepareLoops(t.Context(), cfg.resolutionP.Base))
	bounds, ok := readLinkBounds(spec, frames)
	require.True(t, ok)
	run := newLinkageRun(t.Context(), doc, spec, frames, bounds, cfg)
	return &boxRun{run: run, dr: run.drive.(*linkageDriver), axes: axes, budget: budget}
}

// TestJointBoxLoopSplitAxis pins the driver-axis attribution of
// docs/linkage-check-design.md §16.4 at the root of a gate box: the
// (follower, gate) pair's travel is the follower's dependent term, charged to
// the crank as 2·ρ·δ, and the gate's 5 mm. The follower turns over
// [−8.7632°, 3.0248°] while the crank's centre at 45° puts it near the low
// end, so δ ≈ 0.2 rad and, with the follower box's ρ = √(30² + 4²), the
// crank's share is about 12 mm: a root the pair holds back splits along the
// crank, although the crank is not on the pair's path.
//
// Legs seen to fail when deleted: charging the dependent's term to its own
// joint, which is no axis (the crank carries no share, and the root would
// split along the gate); the dependent's term itself (the crank's share is
// zero).
func TestJointBoxLoopSplitAxis(t *testing.T) {
	t.Parallel()
	b := rockerGateBoxRun(t)
	r := b.run
	require.Equal(t, 3, r.pairs[2][0].other, `the follower's first pair is the gate`)
	root := &boxCell{lo: make([]*big.Rat, 4), hi: make([]*big.Rat, 4)}
	for k := range 4 {
		root.lo[k], root.hi[k] = new(big.Rat), new(big.Rat)
	}
	root.hi[0], root.hi[3] = big.NewRat(1, 1), big.NewRat(1, 1)
	require.NoError(t, b.evaluate(root))
	require.Empty(t, root.gate)

	delta := root.delta[2]
	require.NotNil(t, delta)
	t40, t4c := internalRockerTheta4(0), internalRockerTheta4(math.Pi/4)
	low, high := internalRockerTheta4(38.5727*math.Pi/180)-t40, internalRockerTheta4(81.857366*math.Pi/180)-t40
	high = math.Max(high, internalRockerTheta4(math.Pi/2)-t40)
	high = math.Max(high, 0)
	centre := t4c - t40
	trueDelta := math.Max(high-centre, centre-low)
	require.GreaterOrEqual(t, linkRatFloat(t, delta), trueDelta-1e-9, `δ covers the follower's farthest value over the cell`)
	require.LessOrEqual(t, linkRatFloat(t, delta), trueDelta+0.05*(high-low)+1e-6, `δ is the hull's reach, not more`)

	rho := b.dr.bounds[2].rho[0]
	require.InDelta(t, math.Sqrt(30*30+4*4), linkRatFloat(t, rho), 1e-9)
	shares := b.pairShares(root, 2, 0)
	require.Contains(t, shares, 0, `the crank carries the follower's term`)
	want := new(big.Rat).Mul(delta, big.NewRat(2, 1))
	want.Mul(want, rho)
	require.Zero(t, want.Cmp(shares[0]), `the crank carries 2·ρ·δ`)
	require.Zero(t, big.NewRat(5, 1).Cmp(shares[3]), `the gate carries its span`)
	require.Len(t, shares, 2)
	require.Greater(t, linkRatFloat(t, shares[0]), 5.0)

	root.outcome, root.held = CellUndecided, [][2]int{{2, 0}}
	require.Equal(t, 0, b.splitAxis(root))
}

// TestJointBoxStuckRule pins docs/linkage-check-design.md §16.4's stuck rule
// on a hand-built decomposition that certified [0, 1/2] of the loop axis and
// refused the rest: a gated cell inside the refused stretch is left unsplit,
// one straddling it splits along the loop axis, one meeting the certified
// stretch at a point only is left unsplit, and one at the floor is not split.
//
// Legs seen to fail when deleted: the stuck rule (the cell inside the refused
// stretch splits).
func TestJointBoxStuckRule(t *testing.T) {
	t.Parallel()
	ld := &loopDrive{driver: 0, certified: [][2]*big.Rat{{new(big.Rat), big.NewRat(1, 2)}}}
	b := &boxRun{run: &motionRun{cfg: motionConfig{resolutionP: motionbound.MotionParam{Turn: new(big.Rat), Base: big.NewRat(1, 64)}}}, axes: []int{0}}
	gated := func(lo, hi *big.Rat) *boxCell {
		return &boxCell{lo: []*big.Rat{lo}, hi: []*big.Rat{hi}, outcome: CellUndecided, gate: "refused", gateLoop: ld}
	}
	require.Equal(t, -1, b.splitAxis(gated(big.NewRat(3, 4), big.NewRat(1, 1))), `a cell inside the refused stretch is stuck`)
	require.Equal(t, 0, b.splitAxis(gated(big.NewRat(1, 4), big.NewRat(3, 4))), `a cell straddling it splits along the loop axis`)
	require.Equal(t, -1, b.splitAxis(gated(big.NewRat(1, 2), big.NewRat(1, 1))), `meeting the certified stretch at a point is meeting none of it`)
	require.Equal(t, -1, b.splitAxis(gated(big.NewRat(31, 64), big.NewRat(1, 2))), `a cell at the floor is not split`)
	colliding := gated(big.NewRat(1, 4), big.NewRat(3, 4))
	colliding.outcome = CellColliding
	require.Equal(t, 0, b.splitAxis(colliding), `a gated colliding cell splits along the loop axis too`)
}
