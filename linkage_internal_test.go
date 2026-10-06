package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/motionbound"

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
