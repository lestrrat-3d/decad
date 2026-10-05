package decad

import (
	"math"
	"math/big"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The sweep's §10.4 transfer legs (docs/multibody-dynamics-design.md), shown
// on the transfer itself. Each was deleted in turn and its test went red:
//   - the pose deviation on a band: TestPlanarBandTransferChargesDeviation
//     publishes the rounded pose's width unchanged;
//   - δ on the overlap transfer's margin: TestPlanarOverlapTransferChargesDisplacement
//     carries a held overlap shallower than δ to the ideal path;
//   - δ on the vertex-hull gap: TestPlanarHullGapChargesDisplacement reads the
//     held hull gap whole, and clears a span whose held gap lies inside δ.
//
// A sweep's own ContactPair report already charges δ, so a public fixture
// reaches the overlap transfer only with a held depth between δ and δ plus a
// pose rounding of about 1e-10 mm, and a hull gap inside δ only between two
// samples the endpoints already separate; these tests hand the transfer
// those cases directly.

const bandTestOffset = 1 << 20

func bandOctagonFloor(t *testing.T, doc *Document) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	corners := [][2]float64{{-64, -32}, {-32, -64}, {32, -64}, {64, -32}, {64, 32}, {32, 64}, {-32, 64}, {-64, 32}}
	points := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		points[i] = s.CreatePoint(c[0], c[1])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	s.Fix(points[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(8), Dir: Along})
	require.NoError(t, err)
	return body
}

func bandChamferedBlock(t *testing.T, doc *Document) *Body {
	t.Helper()
	box := internalBoxBody(t, doc, -6, -6, 6, 6, 12)
	block, err := box.Chamfer(t.Context(), Edges(CreatedBy(CapEnd(box))), units.Millimeters(.1))
	require.NoError(t, err)
	return block
}

func bandKnobBody(t *testing.T, doc *Document) *Body {
	t.Helper()
	base := internalBoxBody(t, doc, -5, -5, 5, 5, 10)
	disc := internalDiscBody(t, doc, 2, 4)
	lift, err := r3.Translation(r3.Vec{Z: 8})
	require.NoError(t, err)
	disc, err = disc.Placed(t.Context(), lift)
	require.NoError(t, err)
	union, err := Union(t.Context(), base, disc)
	require.NoError(t, err)
	return union
}

// bandSpin turns about Z at 1 rad/s through (bandTestOffset, 0, 0) for a
// second, starting from a translation.
func bandSpin(t *testing.T, at r3.Vec) RigidDriftSegment {
	t.Helper()
	from, err := r3.Translation(at)
	require.NoError(t, err)
	zero := units.MillimetersPerSecond(0)
	return RigidDriftSegment{From: from, Center: r3.Vec{X: bandTestOffset},
		LinearVelocity:  QuantityVec{X: zero, Y: zero, Z: zero},
		AngularVelocity: QuantityVec{X: units.RadiansPerSecond(0), Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(1)},
		Duration:        units.Seconds(1)}
}

// bandRun prepares the general planar sweep of a pair as SweepPair does.
func bandRun(t *testing.T, doc *Document, a, b *Body, pathA, pathB PairPath) *rotationalPairSweep {
	t.Helper()
	budget := newWorkBudget(t.Context())
	var paths [2]rotationalSweepPath
	for i, side := range []struct {
		body *Body
		path PairPath
	}{{a, pathA}, {b, pathB}} {
		path, err := validatePairPath(side.path)
		require.NoError(t, err)
		solid, delta, ok, err := planarSolidAtPose(t.Context(), budget, side.body, r3.Identity())
		require.NoError(t, err)
		require.True(t, ok)
		paths[i], ok = preparePlanarSweepPath(side.body, path, &solid, delta)
		require.True(t, ok)
	}
	req := SweepRequest{ContactRequest: ContactRequest{PointResolution: units.Millimeters(1e-3),
		NormalResolution: units.Degrees(1)}}
	return &rotationalPairSweep{doc: doc, a: paths[0], b: paths[1], req: req,
		resolution: big.NewRat(1, 1<<20), report: &SweepReport{}, planar: true}
}

func TestPlanarBandTransferChargesDeviation(t *testing.T) {
	// A floor and a chamfered block resting on it spin together far from the
	// origin: held, they touch at every rounded pose, and the rounded poses
	// deviate from the ideal path.
	doc := New()
	floor := internalOffsetBox(t, doc, -100, -100, 100, 100, -10, Distance{D: units.Millimeters(10), Dir: Along})
	block := bandChamferedBlock(t, doc)
	run := bandRun(t, doc, floor, block, bandSpin(t, r3.Vec{X: bandTestOffset}), bandSpin(t, r3.Vec{X: bandTestOffset}))
	half := big.NewRat(1, 2)
	poseA, err := run.a.poseAt(half)
	require.NoError(t, err)
	poseB, err := run.b.poseAt(half)
	require.NoError(t, err)
	contact, err := doc.ContactPair(t.Context(), floor, block, poseA, poseB, run.req.ContactRequest)
	require.NoError(t, err)
	require.Equal(t, ContactBand, contact.Relation)
	_, etaA, okA, err := run.a.pointDeviation(poseA, half, noSweepPoll)
	require.NoError(t, err)
	_, etaB, okB, err := run.b.pointDeviation(poseB, half, noSweepPoll)
	require.NoError(t, err)
	require.True(t, okA && okB)
	require.Positive(t, etaA+etaB, "the far spin rounds its poses")

	event, err := run.planarIdealEvent(t.Context(), half, sweepInstant(half, big.NewRat(1, 1)), poseA, poseB, contact)
	require.NoError(t, err)
	require.Equal(t, ContactBand, event.Relation)
	require.Nil(t, event.Manifold)
	want := new(big.Rat).Add(proofarith.FloatRat(contact.Gap.Bound.Base()),
		new(big.Rat).Add(proofarith.FloatRat(etaA), proofarith.FloatRat(etaB)))
	require.GreaterOrEqual(t, proofarith.FloatRat(event.Gap.Bound.Base()).Cmp(want), 0)
}

func TestPlanarOverlapTransferChargesDisplacement(t *testing.T) {
	doc := New()
	floor := bandOctagonFloor(t, doc)
	knob := bandKnobBody(t, doc)
	delta := knob.payload.(facetedPayload).meshBound
	require.Greater(t, delta, math.Ldexp(1, -14))
	half := big.NewRat(1, 2)
	transfer := func(depth float64) ContactRelation {
		t.Helper()
		run := bandRun(t, doc, floor, knob, bandSpin(t, r3.Vec{X: bandTestOffset}),
			bandSpin(t, r3.Vec{X: bandTestOffset, Z: 8 - depth}))
		poseA, err := run.a.poseAt(half)
		require.NoError(t, err)
		poseB, err := run.b.poseAt(half)
		require.NoError(t, err)
		_, eta, ok, err := run.b.pointDeviation(poseB, half, noSweepPoll)
		require.NoError(t, err)
		require.True(t, ok)
		require.Positive(t, eta)
		require.Less(t, eta, math.Ldexp(1, -20), "the pose rounding alone is far under the depth")
		// The transfer's own check decides; the report only names the case.
		event, err := run.planarIdealEvent(t.Context(), half, sweepInstant(half, big.NewRat(1, 1)), poseA, poseB,
			&ContactReport{Relation: ContactOverlapping})
		require.NoError(t, err)
		return event.Relation
	}
	// Held 2^-14 mm deep, well past the pose rounding but inside δ: the true
	// bodies may not overlap at all.
	require.Equal(t, ContactUndecided, transfer(math.Ldexp(1, -14)))
	require.Equal(t, ContactOverlapping, transfer(1))
}

func TestPlanarHullGapChargesDisplacement(t *testing.T) {
	doc := New()
	floor := bandOctagonFloor(t, doc)
	knob := bandKnobBody(t, doc)
	delta := proofarith.FloatRat(knob.payload.(facetedPayload).meshBound)
	gap := func(z float64) *big.Rat {
		t.Helper()
		still := PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: units.Seconds(1)}
		at, err := r3.Translation(r3.Vec{Z: 8 + z})
		require.NoError(t, err)
		run := bandRun(t, doc, floor, knob, still, PoseSegment{From: at, To: at, Duration: units.Seconds(1)})
		return run.intervalAxisGap(new(big.Rat), big.NewRat(1, 1))
	}
	wide := math.Ldexp(1, -10)
	require.Zero(t, new(big.Rat).Sub(proofarith.FloatRat(wide), delta).Cmp(gap(wide)))
	require.Nil(t, gap(math.Ldexp(1, -14)))
}
