package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file pins the pieces of docs/linkage-check-design.md §5 that sit below
// any public fixture's observability: the ball reading of ρ_{ik} down a chain,
// the telescoping travel sum, the ideal pose's composition, and a link-link
// pair's gap widened by both links' η. Each test records the legs seen to
// fail when deleted.

func internalBoxBodyAtZ(t *testing.T, doc *Document, x0, y0, x1, y1, z0, h float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z0)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(h), Dir: Along})
	require.NoError(t, err)
	return body
}

// linkageChain is a four-joint chain whose ball reading exercises every step
// of §5.2's table:
//
//   - link 1, x ∈ [0, 10], y ∈ [−5, 5], z ∈ [0, 5]: revolute about Z through
//     the origin, 0° → 90°;
//   - link 2, x ∈ [20, 40], y ∈ [−5, 5], z ∈ [0, 5]: prismatic along +X,
//     0 → 10 mm;
//   - link 3, x ∈ [60, 80], y ∈ [−5, 5], z ∈ [0, 5]: revolute about Z through
//     (60, 0, 0), 0° → 45°;
//   - link 4, x ∈ [84, 92], y ∈ [−2, 2], z ∈ [0, 4]: prismatic along +Y,
//     0 → −6 mm.
func linkageChain(t *testing.T) (*linkageSpec, []motionbound.MotionFrame, []linkBound) {
	t.Helper()
	doc := New()
	l := NewLinkage()
	l1, err := l.Ground().Revolute(r3.Vec{}, r3.NewVec(0, 0, 1), []*Body{internalBoxBody(t, doc, 0, -5, 10, 5, 5)})
	require.NoError(t, err)
	l2, err := l1.Prismatic(r3.NewVec(1, 0, 0), []*Body{internalBoxBody(t, doc, 20, -5, 40, 5, 5)})
	require.NoError(t, err)
	l3, err := l2.Revolute(r3.NewVec(60, 0, 0), r3.NewVec(0, 0, 1), []*Body{internalBoxBody(t, doc, 60, -5, 80, 5, 5)})
	require.NoError(t, err)
	l4, err := l3.Prismatic(r3.NewVec(0, 1, 0), []*Body{internalBoxBody(t, doc, 84, -2, 92, 2, 4)})
	require.NoError(t, err)
	spec, err := l.resolveDrive(Drive{
		{Link: l1, From: units.Degrees(0), To: units.Degrees(90)},
		{Link: l2, From: units.Millimeters(0), To: units.Millimeters(10)},
		{Link: l3, From: units.Degrees(0), To: units.Degrees(45)},
		{Link: l4, From: units.Millimeters(0), To: units.Millimeters(-6)},
	})
	require.NoError(t, err)
	frames, ok := linkageFrames(spec)
	require.True(t, ok)
	bounds, ok := readLinkBounds(spec, frames)
	require.True(t, ok)
	return spec, frames, bounds
}

func linkRatFloat(t *testing.T, q *big.Rat) float64 {
	t.Helper()
	require.NotNil(t, q)
	f, _ := q.Float64()
	return f
}

// TestLinkageBallReading pins ρ_{ik} on linkageChain against hand-computed
// radii.
//
// Link 4 (own prismatic): its ball is centred on its box centre (88, 0, 2)
// with radius the half-diagonal √24 plus its reach 6. ρ_34 is that centre's
// distance 28 from joint 3's axis plus the radius. Joint 3 is revolute, so
// the ball moves to its centre (60, 0, 0) with radius |(88, 0, 2) − (60, 0, 0)|
// = √788 plus the radius; joint 2 is prismatic, so the radius grows by its
// reach 10; ρ_14 is the distance 60 of (60, 0, 0) from joint 1's axis plus
// the radius. Link 3 (own revolute): ρ_33 is its far corner's distance √425
// from its own axis, its ball the far corner's distance √450 from (60, 0, 0),
// and ρ_13 is 60 plus that radius plus joint 2's reach. A prismatic joint
// carries no ρ.
//
// Legs seen to fail when deleted: the prismatic own ball's reach m_k (ρ_34
// falls by 6); the revolute step's |c − c_i| (ρ_14 falls by √788); the
// prismatic step's reach m_i (ρ_14 and ρ_13 fall by 10); dist(c, axis_i) in
// ρ_{ik} (ρ_34 falls by 28, ρ_14 by 60); the revolute own ball's corner
// radius (ρ_13 falls by √450); and the corner reading of ρ_kk (ρ_33 is lost).
func TestLinkageBallReading(t *testing.T) {
	t.Parallel()
	_, _, bounds := linkageChain(t)
	r4 := math.Sqrt(24) + 6
	r3ball := math.Sqrt(450)
	want := [][]float64{
		{math.Sqrt(125)},
		{math.Sqrt(10*10+5*5+2.5*2.5) + 10 + 30, -1},
		{60 + r3ball + 10, -1, math.Sqrt(425)},
		{60 + math.Sqrt(788) + r4 + 10, -1, 28 + r4, -1},
	}
	for k, b := range bounds {
		require.Len(t, b.path, k+1)
		for n := range b.path {
			require.Equal(t, n, b.path[n])
			if want[k][n] < 0 {
				require.Nil(t, b.rho[n], `a prismatic joint carries no ρ`)
				continue
			}
			got := linkRatFloat(t, b.rho[n])
			require.InDelta(t, want[k][n], got, 1e-9, "ρ_%d%d", n+1, k+1)
		}
	}
	// Link 2's ρ_12: its own prismatic ball centred on (30, 0, 2.5), radius
	// the half-diagonal √131.25 plus 10, at distance 30 from joint 1's axis.
	require.InDelta(t, 30+math.Sqrt(131.25)+10, linkRatFloat(t, bounds[1].rho[0]), 1e-9)
	reach := linkRatFloat(t, bounds[3].reach)
	require.InDelta(t, (60+math.Sqrt(788)+r4+10)*math.Pi/2+10+(28+r4)*math.Pi/4+6, reach, 1e-9)
}

// TestLinkageChainTravel pins τ^(L)_k's telescoping sum on linkageChain over
// s ∈ [1/4, 1/2]: against a static body every joint on link 4's path enters,
// ρ_14·(π/2)/4 + 10/4 + ρ_34·(π/4)/4 + 6/4; against link 3, whose path is
// link 4's up to joint 3, joint 4 alone enters, 6/4 exactly; against link 2,
// joints 3 and 4 enter on link 4's side and none on link 2's.
//
// Legs seen to fail when deleted: each joint's term of the sum.
func TestLinkageChainTravel(t *testing.T) {
	t.Parallel()
	spec, _, bounds := linkageChain(t)
	a, b := big.NewRat(1, 4), big.NewRat(1, 2)
	rho14, rho34 := linkRatFloat(t, bounds[3].rho[0]), linkRatFloat(t, bounds[3].rho[2])
	static := linkRatFloat(t, chainTravel(spec, bounds[3], 0, a, b))
	require.InDelta(t, rho14*(math.Pi/2)/4+10.0/4+rho34*(math.Pi/4)/4+6.0/4, static, 1e-9)

	below := commonDepth(bounds[3].path, bounds[2].path)
	require.Equal(t, 3, below)
	require.Zero(t, chainTravel(spec, bounds[3], below, a, b).Cmp(big.NewRat(3, 2)))
	require.Zero(t, chainTravel(spec, bounds[2], below, a, b).Sign())

	below = commonDepth(bounds[3].path, bounds[1].path)
	require.Equal(t, 2, below)
	require.InDelta(t, rho34*(math.Pi/4)/4+6.0/4, linkRatFloat(t, chainTravel(spec, bounds[3], below, a, b)), 1e-9)
	require.Zero(t, chainTravel(spec, bounds[1], below, a, b).Sign())
}

// TestLinkageChainTravelAcrossWaypoints pins |Δq_i| of
// docs/linkage-check-design.md §5.2 on a three-segment drive: a blade
// x ∈ [0, 50], y ∈ [−0.5, 0.5], z ∈ [0, 10] turning about Z through
// 0° → 60° → 10° → 20°, and a block on it sliding along +Z through
// 0 → 20 → 20 → 0 mm. The waypoints sit at s = 1/3 and 2/3.
//
//   - [0, 1/4] lies inside the first segment: the blade turns 45°.
//   - [1/4, 3/8] holds the 60° corner: 45° → 60° → 53.75°, 21.25° of travel
//     for ends 8.75° apart.
//   - [1/4, 3/4] holds both waypoints: 45° → 60° → 10° → 12.5°, 67.5°; the
//     block runs 15 → 20 → 20 → 15 mm, exactly 10 mm for ends that agree.
//
// At s = 1/3 the exact joint value is the waypoint itself, and the reach m
// is the farthest waypoint, 60°, not either end.
//
// Legs seen to fail when deleted: the waypoints inside an interval (the
// travel is the ends' difference, 8.75° and 32.5°, and 0 mm for the block),
// and the Via values in the reach (it reads 20°).
func TestLinkageChainTravelAcrossWaypoints(t *testing.T) {
	t.Parallel()
	doc := New()
	l := NewLinkage()
	turn, err := l.Ground().Revolute(r3.Vec{}, r3.NewVec(0, 0, 1), []*Body{internalBoxBody(t, doc, 0, -0.5, 50, 0.5, 10)})
	require.NoError(t, err)
	lift, err := turn.Prismatic(r3.NewVec(0, 0, 1), []*Body{internalBoxBodyAtZ(t, doc, 40, -2, 44, 2, 10, 4)})
	require.NoError(t, err)
	spec, err := l.resolveDrive(Drive{
		{Link: turn, From: units.Degrees(0), Via: []units.Value{units.Degrees(60), units.Degrees(10)}, To: units.Degrees(20)},
		{Link: lift, From: units.Millimeters(0), Via: []units.Value{units.Millimeters(20), units.Millimeters(20)}, To: units.Millimeters(0)},
	})
	require.NoError(t, err)
	frames, ok := linkageFrames(spec)
	require.True(t, ok)
	bounds, ok := readLinkBounds(spec, frames)
	require.True(t, ok)

	rho := linkRatFloat(t, bounds[0].rho[0])
	require.InDelta(t, math.Sqrt(50*50+0.5*0.5), rho, 1e-9)
	deg := math.Pi / 180
	for _, tc := range []struct {
		a, b    *big.Rat
		degrees float64
	}{
		{new(big.Rat), big.NewRat(1, 4), 45},
		{big.NewRat(1, 4), big.NewRat(3, 8), 21.25},
		{big.NewRat(1, 4), big.NewRat(3, 4), 67.5},
	} {
		got := linkRatFloat(t, chainTravel(spec, bounds[0], 0, tc.a, tc.b))
		require.InDelta(t, rho*tc.degrees*deg, got, 1e-9, "[%s, %s]", tc.a, tc.b)
	}
	below := commonDepth(bounds[1].path, bounds[0].path)
	require.Equal(t, 1, below)
	require.Zero(t, chainTravel(spec, bounds[1], below, big.NewRat(1, 4), big.NewRat(3, 4)).Cmp(big.NewRat(10, 1)))

	corner := jointParam(spec.joints[0], big.NewRat(1, 3))
	require.Zero(t, corner.Turn.Cmp(big.NewRat(1, 6)), `60° is the exact turn 1/6`)
	require.Zero(t, corner.Base.Sign())
	require.InDelta(t, 60*deg, linkRatFloat(t, jointReach(spec.joints[0])), 1e-12)
	require.Zero(t, jointReach(spec.joints[1]).Cmp(big.NewRat(20, 1)))
}

// foldingArmRun is scene 1 of docs/linkage-check-design.md §11 as
// VerifyLinkage's own run state, with the arms and the wall.
func foldingArmRun(t *testing.T) (*motionRun, *Body, *Body) {
	t.Helper()
	doc := New()
	upper := internalBoxBody(t, doc, 0, -14, 48, 14, 10)
	fore := internalBoxBodyAtZ(t, doc, 48, -14, 96, 14, 12, 10)
	internalBoxBodyAtZ(t, doc, -100, 38, 150, 58, -10, 50)
	l := NewLinkage()
	shoulder, err := l.Ground().Revolute(r3.Vec{}, r3.NewVec(0, 0, 1), []*Body{upper})
	require.NoError(t, err)
	elbow, err := shoulder.Revolute(r3.NewVec(48, 0, 0), r3.NewVec(0, 0, 1), []*Body{fore})
	require.NoError(t, err)
	spec, err := l.resolveDrive(Drive{
		{Link: shoulder, From: units.Degrees(0), To: units.Degrees(90)},
		{Link: elbow, From: units.Degrees(0), To: units.Degrees(-90)},
	})
	require.NoError(t, err)
	frames, ok := linkageFrames(spec)
	require.True(t, ok)
	bounds, ok := readLinkBounds(spec, frames)
	require.True(t, ok)
	cfg, err := resolveMotionOptions(nil, motionSpec{motionDomain: fractionDomain()})
	require.NoError(t, err)
	run := newLinkageRun(t.Context(), doc, spec, frames, bounds, cfg)
	return run, upper, fore
}

// TestLinkageIdealPoseComposesTheChain: at s = 1/3 the forearm's ideal pose,
// its elbow's exact −30° then the shoulder's exact 30°, carries its far end
// (96, 0, 0) to (48·cos 30° + 48, 48·sin 30°, 0) and the elbow centre to
// (48·cos 30°, 48·sin 30°, 0), each inside an enclosure of width below 1e-12.
//
// Legs seen to fail when deleted: the parent's ideal pose in the composition
// (the far end lands at (48·cos 30° + 48 − …) about the elbow alone), and the
// composition order (the elbow turns about its zero-pose centre after the
// shoulder has moved it).
func TestLinkageIdealPoseComposesTheChain(t *testing.T) {
	t.Parallel()
	run, _, _ := foldingArmRun(t)
	dr, ok := run.drive.(*linkageDriver)
	require.True(t, ok)
	ideals := idealPosesAt(dr.spec, dr.frames, big.NewRat(1, 3))
	c, s := math.Cos(math.Pi/6), math.Sin(math.Pi/6)
	for _, tc := range []struct {
		link  int
		point r3.Vec
		want  r3.Vec
	}{
		{0, r3.NewVec(48, 0, 0), r3.NewVec(48*c, 48*s, 0)},
		{1, r3.NewVec(48, 0, 0), r3.NewVec(48*c, 48*s, 0)},
		{1, r3.NewVec(96, 0, 0), r3.NewVec(48*c+48, 48*s, 0)},
	} {
		p, ok := motionbound.RatVecOf(tc.point)
		require.True(t, ok)
		ideal := ideals[tc.link]
		image := motionbound.IvVecAdd(motionbound.IvVecAdd(ideal.Rot.Apply(motionbound.IvVecSub(motionbound.PointVec(p), ideal.Pivot)), ideal.Pivot), ideal.Shift)
		for i, want := range []float64{tc.want.X, tc.want.Y, tc.want.Z} {
			lo, hi := linkRatFloat(t, image[i].Lo), linkRatFloat(t, image[i].Hi)
			require.LessOrEqual(t, lo, want+1e-12)
			require.GreaterOrEqual(t, hi, want-1e-12)
			require.Less(t, hi-lo, 1e-12)
		}
	}
}

// TestLinkagePairGapCarriesBothDeviations: at s = 1/3 both arms of scene 1
// move, so the arms' published gap interval is the kernel's own interval for
// the two transient placements widened by η_A + η_B, not by either alone.
//
// Leg seen to fail when deleted: the partner's η in the widening (the lower
// end then sits only η_A below the kernel's).
func TestLinkagePairGapCarriesBothDeviations(t *testing.T) {
	t.Parallel()
	run, upper, fore := foldingArmRun(t)
	// The layer exclusion settles the arms (§5.7); reopen the pair so the
	// pose evaluates it, which is the widening this test pins.
	require.True(t, run.pairs[0][1].excluded)
	run.pairs[0][1] = motionPair{other: 1}
	f := big.NewRat(1, 3)
	pose, err := run.evaluatePose(f, run.dom.label(f))
	require.NoError(t, err)
	require.Len(t, run.pairs[0], 2, `the upper arm's row: the wall, then the forearm`)
	require.Equal(t, 1, run.pairs[0][1].other)
	got := pose.pairs[0][1]
	require.True(t, got.hasGap)

	placed := func(b *Body, group int) (*Body, float64) {
		placement := b.payload.transform()
		composed, err := placement.Then(pose.groups[group].pose)
		require.NoError(t, err)
		transient, err := b.payload.placed(t.Context(), run.transient, transientProducer, composed)
		require.NoError(t, err)
		eta, _ := motionbound.PoseDeviation(composed, placement, pose.groups[group].ideals[0], moverRecordRadius(t.Context(), b))
		return transient, eta
	}
	ta, etaA := placed(upper, 0)
	tb, etaB := placed(fore, 1)
	require.Greater(t, etaA, 0.0)
	require.Greater(t, etaB, 0.0)
	res, err := clearancePair(t.Context(), ta, tb, boxesDisjoint(ta.bounds, tb.bounds))
	require.NoError(t, err)
	require.Equal(t, pairDisjoint, res.verdict)
	require.LessOrEqual(t, got.lo, res.lo-etaA-etaB)
	require.Less(t, got.lo, res.lo-etaA, `the forearm's own deviation is charged too`)
	require.GreaterOrEqual(t, got.hi, res.hi+etaA+etaB)
	require.LessOrEqual(t, got.lo, 2.0)
	require.GreaterOrEqual(t, got.hi, 2.0)
}

// TestVerifyLinkageKeepsTheNextProducerIdentity is the document identity half
// of §11's non-mutation test: transient link placements mint no live
// producer, level or curve identity, and a Duplicate after the call receives
// the next producer.
func TestVerifyLinkageKeepsTheNextProducerIdentity(t *testing.T) {
	t.Parallel()
	doc := New()
	cube := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	internalBoxBody(t, doc, 25, 2, 35, 8, 10)
	l := NewLinkage()
	slide, err := l.Ground().Prismatic(r3.NewVec(1, 0, 0), []*Body{cube})
	require.NoError(t, err)
	before := doc.nextProducerID()
	levelBefore, curveBefore := doc.nextLevel, doc.nextCurve
	report, err := doc.VerifyLinkage(t.Context(), l, Drive{{Link: slide, From: units.Millimeters(0), To: units.Millimeters(30)}},
		WithResolution(units.Scalar(1)))
	require.NoError(t, err)
	require.Len(t, report.Poses, 2)
	require.Equal(t, before, doc.nextProducerID())
	require.Equal(t, levelBefore, doc.nextLevel)
	require.Equal(t, curveBefore, doc.nextCurve)
	dup, err := cube.Duplicate(t.Context())
	require.NoError(t, err)
	require.Equal(t, before, dup.originProducer())
}

// TestVerifyLinkageRefusesABodyItDidNotBuild is §8's row the public API
// cannot reach: a link body with no payload is ErrUnsupported, as for Placed.
func TestVerifyLinkageRefusesABodyItDidNotBuild(t *testing.T) {
	t.Parallel()
	doc := New()
	foreignBuilt := &Body{doc: doc, kind: BodySolid}
	doc.commit(foreignBuilt)
	l := NewLinkage()
	slide, err := l.Ground().Prismatic(r3.NewVec(1, 0, 0), []*Body{foreignBuilt})
	require.NoError(t, err)
	before := doc.Bodies()
	report, err := doc.VerifyLinkage(t.Context(), l, Drive{{Link: slide, From: units.Millimeters(0), To: units.Millimeters(1)}})
	require.ErrorIs(t, err, ErrUnsupported)
	require.Nil(t, report)
	require.Equal(t, before, doc.Bodies())
}

// TestLinkageConstantPlacementIsReused pins §6 step 2's constant placement: a
// link held at 30° has the same pose at every s, so its body's transient
// placement is built at the first pose and the same one serves every later
// pose, kept in the carrier cache between them; a moving link's is built
// afresh and dropped.
//
// Leg seen to fail when deleted: the reuse (each pose builds its own
// placement and no constant placement is kept).
func TestLinkageConstantPlacementIsReused(t *testing.T) {
	t.Parallel()
	doc := New()
	base := internalBoxBody(t, doc, -10, -10, 10, 10, 10)
	arm := internalBoxBodyAtZ(t, doc, 0, -14, 48, 14, 12, 10)
	internalBoxBody(t, doc, 15, -30, 25, 30, 10)
	l := NewLinkage()
	pedestal, err := l.Ground().Revolute(r3.Vec{}, r3.NewVec(0, 0, 1), []*Body{base})
	require.NoError(t, err)
	swing, err := pedestal.Revolute(r3.Vec{}, r3.NewVec(0, 0, 1), []*Body{arm})
	require.NoError(t, err)
	spec, err := l.resolveDrive(Drive{
		{Link: pedestal, From: units.Degrees(30), To: units.Degrees(30)},
		{Link: swing, From: units.Degrees(0), To: units.Degrees(90)},
	})
	require.NoError(t, err)
	frames, ok := linkageFrames(spec)
	require.True(t, ok)
	bounds, ok := readLinkBounds(spec, frames)
	require.True(t, ok)
	cfg, err := resolveMotionOptions(nil, motionSpec{motionDomain: fractionDomain()})
	require.NoError(t, err)
	run := newLinkageRun(t.Context(), doc, spec, frames, bounds, cfg)
	require.Equal(t, []linkStanding{linkConstant, linkMoving}, run.drive.(*linkageDriver).standing)

	_, err = run.evaluatePose(new(big.Rat), run.dom.label(new(big.Rat)))
	require.NoError(t, err)
	require.NotNil(t, run.constPlaced)
	held := run.constPlaced[0]
	require.NotNil(t, held)
	require.NotSame(t, base, held.body, `a constant placement is a transient body`)
	require.Nil(t, run.constPlaced[1], `a moving link's placement is never kept`)
	half := big.NewRat(1, 2)
	_, err = run.evaluatePose(half, run.dom.label(half))
	require.NoError(t, err)
	require.Same(t, held, run.constPlaced[0])
	_, cached := run.cache.entries[held.body]
	require.True(t, cached, `the kept placement's carriers stay cached between poses`)
}

// linkageRunOf is VerifyLinkage's own run state for a tree linkage under a
// drive, before any pose.
func linkageRunOf(t *testing.T, doc *Document, l *Linkage, drive Drive, opts ...MotionOption) *motionRun {
	t.Helper()
	spec, err := l.resolveDrive(drive)
	require.NoError(t, err)
	frames, ok := linkageFrames(spec)
	require.True(t, ok)
	bounds, ok := readLinkBounds(spec, frames)
	require.True(t, ok)
	cfg, err := resolveMotionOptions(opts, motionSpec{motionDomain: fractionDomain()})
	require.NoError(t, err)
	return newLinkageRun(t.Context(), doc, spec, frames, bounds, cfg)
}

// TestLinkageSecondDerivativeBound pins B_ij of docs/linkage-check-design.md
// §5.8 on linkageChain — revolute, prismatic, revolute, prismatic — against
// central second differences of every rest-box corner's world position, read
// off Linkage.Configuration at a grid of configurations: every difference
// quotient of every coordinate, for every link and every pair of joints on
// its path, sits at or below its B_ij. A revolute ancestor turns the deeper
// joint's velocity, so the mixed derivatives under joints 1 and 3 are
// nonzero; a prismatic ancestor turns nothing.
//
// Leg seen to fail when deleted: the rule's two halves exchanged — zero under
// a revolute ancestor, w_j under a prismatic one (the difference quotients
// under joint 1 exceed a zero bound).
func TestLinkageSecondDerivativeBound(t *testing.T) {
	t.Parallel()
	spec, _, bounds := linkageChain(t)
	l := spec.linkage
	const step = 1e-3
	value := func(k int, q float64) units.Value {
		if spec.joints[k].revolute {
			return units.Radians(q)
		}
		return units.Millimeters(q)
	}
	corner := func(q [4]float64, k int, c r3.Vec) r3.Vec {
		values := make([]units.Value, 4)
		for n := range values {
			values[n] = value(n, q[n])
		}
		conf, err := l.Configuration(values)
		require.NoError(t, err)
		return conf.Poses[k].Apply(c)
	}
	checked := 0
	for _, q1 := range []float64{0.1, 0.7, 1.4} {
		for _, q2 := range []float64{1, 9} {
			for _, q3 := range []float64{0.1, 0.7} {
				q := [4]float64{q1, q2, q3, -3}
				for k := range spec.joints {
					box := spec.joints[k].link.bodies[0].bounds
					for _, c := range []r3.Vec{box.Max, box.Min, r3.NewVec(box.Max.X, box.Min.Y, box.Max.Z)} {
						for m := 0; m <= k; m++ {
							for n := m; n <= k; n++ {
								at := func(dm, dn float64) r3.Vec {
									p := q
									p[m] += dm
									p[n] += dn
									return corner(p, k, c)
								}
								var fd r3.Vec
								if m == n {
									fd = at(step, 0).Add(at(-step, 0)).Sub(corner(q, k, c).Scale(2)).Scale(1 / (step * step))
								} else {
									fd = at(step, step).Sub(at(step, -step)).Sub(at(-step, step)).Add(at(-step, -step)).Scale(1 / (4 * step * step))
								}
								bound := 0.0
								if w := secondDerivativeBound(bounds[k], m, n); w != nil {
									bound = linkRatFloat(t, w)
								}
								for _, d := range []float64{fd.X, fd.Y, fd.Z} {
									require.LessOrEqual(t, math.Abs(d), bound+1e-4, "link %d, joints %d and %d at %v", k+1, m+1, n+1, q)
								}
								checked++
							}
						}
					}
				}
			}
		}
	}
	require.NotZero(t, checked)
	require.Nil(t, secondDerivativeBound(bounds[3], 1, 2), `a prismatic ancestor turns nothing`)
	require.Zero(t, secondDerivativeBound(bounds[3], 0, 1).Cmp(big.NewRat(1, 1)), `a revolute ancestor turns a slide's unit velocity`)
	require.Zero(t, secondDerivativeBound(bounds[3], 0, 2).Cmp(bounds[3].rho[2]), `a revolute ancestor turns a revolute's velocity, at most ρ_jk long`)
}

// TestLinkageCornerVelocity pins v_{i,c} of docs/linkage-check-design.md
// §5.8 on scene 1's forearm at s = 1/3, the shoulder at 30° and the elbow at
// −30°: the forearm keeps its zero-pose orientation, so its corner
// (96, 14, 22) sits at x = (48·cos 30° + 48, 48·sin 30° + 14, 22). Under the
// shoulder, about Z through the origin, it moves at Z × x; under the elbow,
// whose axis the shoulder has carried to (48·cos 30°, 48·sin 30°, 0), at
// Z × (48, 14, 22) = (−14, 48, 0). Each enclosure is narrower than 1e-12.
//
// Leg seen to fail when deleted: the parent's pose in the elbow's axis (the
// pivot stays at (48, 0, 0) and the velocity reads Z × (x − (48, 0, 0))).
func TestLinkageCornerVelocity(t *testing.T) {
	t.Parallel()
	run, _, fore := foldingArmRun(t)
	dr := run.drive.(*linkageDriver)
	s := big.NewRat(1, 3)
	params := []motionbound.MotionParam{jointParam(dr.spec.joints[0], s), jointParam(dr.spec.joints[1], s)}
	lo, hi, ok := boxCornersExact(fore.bounds, new(big.Rat))
	require.True(t, ok)
	reading := readCorners(dr.spec, dr.frames, params, dr.bounds[1], 0, lo, hi)
	c30, s30 := math.Cos(math.Pi/6), math.Sin(math.Pi/6)
	x := r3.NewVec(48*c30+48, 48*s30+14, 22)
	requireEncloses := func(t *testing.T, want r3.Vec, got motionbound.IvVec) {
		t.Helper()
		for i, w := range []float64{want.X, want.Y, want.Z} {
			lo, hi := linkRatFloat(t, got[i].Lo), linkRatFloat(t, got[i].Hi)
			require.LessOrEqual(t, lo, w+1e-12)
			require.GreaterOrEqual(t, hi, w-1e-12)
			require.Less(t, hi-lo, 1e-12)
		}
	}
	found := false
	for c, pos := range reading.pos {
		if linkRatFloat(t, pos[0].Lo) < 85 || linkRatFloat(t, pos[1].Lo) < 35 || linkRatFloat(t, pos[2].Lo) < 21 {
			continue
		}
		found = true
		requireEncloses(t, x, pos)
		require.Len(t, reading.vel[c], 2)
		requireEncloses(t, r3.NewVec(-x.Y, x.X, 0), reading.vel[c][0])
		requireEncloses(t, r3.NewVec(-14, 48, 0), reading.vel[c][1])
	}
	require.True(t, found, `the corner (96, 14, 22) is read`)
}

// TestLinkageProjectionBoundHandSum pins L_n of docs/linkage-check-design.md
// §5.8 on scene 13's pendulum over its first interval at
// WithResolution(1/16), θ ∈ [0, π/32] (π at its upper enclosure in the
// span). The wall's face y = 20 is β along +Y. From θ = 0 the block's
// highest corners sit at y = −40 and move at |v_y| = |x| = 5 per radian, so
// L_{+Y} = 20 − (−40 + 5·h) − ½·ρ_11·h² with ρ_11 = √(50² + 5²); from θ = h
// each corner (x, y) sits at x·sin h + y·cos h and moves at
// |x·cos h − y·sin h|. The bound is the larger of the two ends' readings;
// every other direction separates nothing, and here θ = 0's reading is the
// larger.
//
// Legs seen to fail when deleted: the remainder, and the first-order term.
func TestLinkageProjectionBoundHandSum(t *testing.T) {
	t.Parallel()
	doc := New()
	block := internalBoxBody(t, doc, -5, -50, 5, -40, 10)
	internalBoxBodyAtZ(t, doc, -100, 20, 100, 40, -10, 30)
	l := NewLinkage()
	swing, err := l.Ground().Revolute(r3.Vec{}, r3.NewVec(0, 0, 1), []*Body{block})
	require.NoError(t, err)
	run := linkageRunOf(t, doc, l, Drive{{Link: swing, From: units.Degrees(0), To: units.Degrees(90)}}, WithResolution(units.Scalar(1.0/16)))
	dr := run.drive.(*linkageDriver)
	fa, fb := new(big.Rat), big.NewRat(1, 16)
	a, err := run.evaluatePose(fa, run.dom.label(fa))
	require.NoError(t, err)
	b, err := run.evaluatePose(fb, run.dom.label(fb))
	require.NoError(t, err)
	got := dr.projection(0, 0, a, b)
	require.NotNil(t, got)

	h := linkRatFloat(t, jointSpan(dr.spec.joints[0], fa, fb))
	require.InDelta(t, math.Pi/32, h, 1e-15)
	rho := math.Sqrt(50*50 + 5*5)
	require.InDelta(t, rho, linkRatFloat(t, dr.bounds[0].rho[0]), 1e-12)
	rem := rho * h * h / 2
	fromA := 20 - (-40 + 5*h) - rem
	reach := math.Inf(-1)
	for _, c := range [][2]float64{{-5, -50}, {5, -50}, {-5, -40}, {5, -40}} {
		y := c[0]*math.Sin(h) + c[1]*math.Cos(h)
		v := math.Abs(c[0]*math.Cos(h) - c[1]*math.Sin(h))
		reach = math.Max(reach, y+v*h)
	}
	fromB := 20 - reach - rem
	require.InDelta(t, math.Max(fromA, fromB), linkRatFloat(t, got), 1e-9)
	require.Greater(t, fromA, fromB, `the near end's reading is the larger here`)
}

// TestLinkageProjectionLeavesTheDiscToTheTravelBound: a disc of radius 5
// spinning a quarter turn about its own axis beside a wall 7 mm away keeps
// its gap, but its box corner sweeps a circle of radius 5·√2, so the
// projection bound sits up to (√2 − 1)·5 below the gap and the travel bound
// is the larger on every interval. Every interval's published bound is the
// travel bound, bit for bit, and the run evaluates the 513 poses the travel
// bound alone evaluates.
//
// Leg seen to fail when deleted: the larger of the two bounds (taking the
// projection bound alone, every interval reads it).
func TestLinkageProjectionLeavesTheDiscToTheTravelBound(t *testing.T) {
	t.Parallel()
	doc := New()
	disc := internalDiscBody(t, doc, 5, 10)
	internalBoxBodyAtZ(t, doc, 12, -20, 22, 20, -5, 20)
	l := NewLinkage()
	spin, err := l.Ground().Revolute(r3.Vec{}, r3.NewVec(0, 0, 1), []*Body{disc})
	require.NoError(t, err)
	run := linkageRunOf(t, doc, l, Drive{{Link: spin, From: units.Degrees(0), To: units.Degrees(90)}})
	run.cfg.readingP = &motionbound.MotionParam{Turn: new(big.Rat), Base: big.NewRat(1, linkageReadingFloor)}
	poses, spans, err := run.refine()
	require.NoError(t, err)
	require.Len(t, poses, 513)
	dr := run.drive.(*linkageDriver)
	for k, span := range spans {
		a, b := poses[k], poses[k+1]
		require.Equal(t, IntervalClear, span.outcome)
		pa, pb := a.pairs[0][0], b.pairs[0][0]
		travel := new(big.Rat).Add(proofarith.FloatRat(pa.lo), proofarith.FloatRat(pb.lo))
		travel.Sub(travel, dr.travel(0, 0, a.param, b.param))
		travel.Quo(travel, big.NewRat(2, 1))
		require.Equal(t, proofbound.RatFloatDown(travel), span.clearance.Value.Base(), "interval %d", k)
		require.Negative(t, dr.projection(0, 0, a, b).Cmp(travel), "interval %d", k)
	}
}
