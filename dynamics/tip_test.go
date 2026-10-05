package dynamics_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The dynamics fixture of docs/multibody-dynamics-design.md §13 PR 13: an
// 8 mm source cube stands on one edge on a fixed floor, turned 30° about Y,
// with its center of mass beyond the edge, so gravity tips it over. Every
// step kicks it, solves the edge's impact, carries the touch on a §10.3 band
// track until the band reaches PenetrationResidual, and lets the edge lift
// clear; the next step's kick brings it down again as a rotating impact,
// which the step advances through at the bracket's right end. The cube lands
// on its far edge. With restitution zero it then rocks between its two
// bottom edges with a shrinking tilt, until the far edge closes within one
// grid step of a touch on the near one, which no certificate covers: the
// step stops there, Undecided.
//
// Legs shown to fail (each deleted in turn, fixture red, then restored):
//   - the band cut (bandEnd's search for the last grid fraction within the
//     residual): cutting at the track's end publishes band slices whose
//     BandAt exceeds PenetrationResidual;
//   - the impact manifold read at the rounded event poses: the first rotating
//     impact stops the step with StepManifoldMissing;
//   - the planar replay inside a rotating impact bracket
//     (contact_sweep_replay.go's bracketDepthWithin): the first rotating
//     impact stops the step with StepPairUndecided;
//   - the oriented-box planar manifold (contact_pair.go's
//     classifyPlanarManifold): the initial edge contact stops the first step
//     with StepManifoldMissing; dropping its face-pair exclusion publishes
//     the degenerate face-pair manifolds the box patches withhold in
//     TestClippedRotatedFaceRefusesUnprovedContacts and
//     TestObliqueBoxGeometryIntegration;
//   - the shallow edge penetration of §9.3 (internal/pair's shallowSupport):
//     the first rotating impact's rounded poses overlap without a manifold,
//     StepManifoldMissing.
//
// The band's own depth legs are shown in contact_sweep_band_test.go.

// tipScene is the floor, the cube and two far fixed boxes.
type tipScene struct {
	doc        *decad.Document
	floor, box *decad.Body
	world      *dynamics.World
	state      dynamics.State
	config     dynamics.StepConfig
}

// tipMass is the cube's mass at density 0.001 kg/mm³: 0.512 kg. A cube's
// central inertia is isotropic, m·(8² + 8²)/12, in every orientation.
const (
	tipMass    = .512
	tipInertia = tipMass * 128 / 12
)

func newTipScene(t *testing.T) tipScene {
	t.Helper()
	scene := tipScene{doc: decad.New()}
	scene.floor = makeBox(t, scene.doc, -100, -100, 100, 100, -10, 10)
	scene.box = makeBox(t, scene.doc, 0, -4, 8, 4, 0, 8)
	far, farther := farBox(t, scene.doc, 1000), farBox(t, scene.doc, 2000)
	density := units.KilogramsPerCubicMillimeter(.001)
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	scene.config = pairMaterialStepConfig()
	scene.config.ImpactSpeed = units.MillimetersPerSecond(64)
	scene.config.MaxIterations = 256
	scene.config.MaxEvents = 64
	scene.config.MaxPoseEvaluations = 512
	var err error
	scene.world, err = dynamics.NewWorld(t.Context(), scene.doc, dynamics.WorldConfig{Bodies: []dynamics.RigidBody{
		{Body: scene.floor, Role: dynamics.Fixed, Material: material},
		{Body: scene.box, Role: dynamics.Dynamic, Density: &density, Material: material},
		{Body: far, Role: dynamics.Fixed, Material: material},
		{Body: farther, Role: dynamics.Fixed, Material: material},
	}, Step: scene.config})
	require.NoError(t, err)
	// Turned −30° about Y: the edge x = z = 0 stays on the floor at the
	// origin and the cube's x = 8 edge stands 4 mm up.
	c := math.Sqrt(3) / 2
	pose, err := r3.FromBasis(r3.Basis{EX: r3.Vec{X: c, Z: .5}, EY: r3.Vec{Y: 1}, EZ: r3.Vec{X: -.5, Z: c}}, r3.Vec{})
	require.NoError(t, err)
	scene.state, err = scene.world.NewState([]dynamics.BodyState{
		{Body: scene.floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: scene.box, Pose: pose, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: far, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: farther, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	return scene
}

// localPoint maps a world point into the cube's frame at pose.
func localPoint(t *testing.T, pose r3.Transform, p r3.Vec) r3.Vec {
	t.Helper()
	inverse, err := pose.Inverse()
	require.NoError(t, err)
	return inverse.Apply(p)
}

// requireTipEventLaws checks each island event of the frictionless cube
// against the discrete laws: the floor's impulse J along +Z changes the
// cube's momentum by J, and its moment about the center of mass changes the
// isotropic angular momentum by Σ r×J.
func requireTipEventLaws(t *testing.T, event dynamics.ContactEvent) {
	t.Helper()
	require.Equal(t, dynamics.ContactImpact, event.Kind)
	j := event.NormalImpulse.Base()
	require.GreaterOrEqual(t, j, 0.0)
	// The certificate's linear gate is ImpulseResidual + m·VelocityResidual.
	require.InDelta(t, j, tipMass*(event.PostVelocityB.Z.Base()-event.PreVelocityB.Z.Base()), 2e-6)
	require.Equal(t, event.PreVelocityB.X, event.PostVelocityB.X)
	center := event.PoseB.Apply(r3.Vec{X: 4, Z: 4})
	moment := 0.0
	for i, point := range event.Manifold.Points {
		require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
		lever := point.OnB.Value.Sub(center)
		// (r × J·ẑ)_y = −r_x·J.
		moment -= lever.X * event.PointImpulses[i].Normal.Base()
	}
	change := tipInertia * (event.PostAngularVelocityB.Y.Base() - event.PreAngularVelocityB.Y.Base())
	require.InDelta(t, moment, change, 1e-5)
}

// requireBandCuts checks every band slice of a trace: a slice that ends
// before its band track's end ends at the last fraction of the sweep's grid
// whose band lies within PenetrationResidual, so the next grid fraction's
// band exceeds it. It returns the number of band slices.
func requireBandCuts(t *testing.T, trace dynamics.Trace, residual float64) int {
	t.Helper()
	times := dynamics.TraceSliceTimes(trace)
	count := 0
	for i, slice := range dynamics.TraceSliceProofs(trace) {
		for _, proof := range slice {
			if proof.Sweep == nil || proof.Sweep.Outcome != decad.SweepPersistentBand {
				continue
			}
			count++
			track := proof.Sweep.ContactTrack
			start, end, span := times[i][0].Base(), times[i][1].Base(), times[i][2].Base()
			f := new(big.Rat).Quo(new(big.Rat).Sub(rat(end), rat(start)), new(big.Rat).Sub(rat(span), rat(start)))
			fraction, _ := f.Float64()
			band, err := track.BandAt(units.Scalar(fraction))
			require.NoError(t, err)
			require.NotNil(t, band)
			require.LessOrEqual(t, band.Value.Base()+band.Bound.Base(), residual, "slice %d fraction %v", i, fraction)
			if fraction >= track.End().Fraction.Base() {
				continue
			}
			duration := proof.Sweep.PathA.(decad.PoseSegment).Duration.Base()
			grid := proof.Sweep.Request.TimeResolution.Base()
			step := 1.0
			for step*duration > grid && step > math.Ldexp(1, -52) {
				step /= 2
			}
			next := fraction + step
			if next > track.End().Fraction.Base() {
				continue
			}
			beyond, err := track.BandAt(units.Scalar(next))
			require.NoError(t, err)
			require.Greater(t, beyond.Value.Base()+beyond.Bound.Base(), residual, "slice %d fraction %v", i, fraction)
		}
	}
	return count
}

func rat(v float64) *big.Rat { return new(big.Rat).SetFloat64(v) }

func TestBoxTipsOverOnBandTracks(t *testing.T) {
	scene := newTipScene(t)
	timeline, err := dynamics.NewTimeline(scene.world, scene.state)
	require.NoError(t, err)
	dt := units.Seconds(1.0 / 256)
	residual := scene.config.PenetrationResidual.Base()
	var stopped *dynamics.StepReport
	bands, impacts := 0, 0
	for range 16 {
		report, err := timeline.Advance(t.Context(), dynamics.StepInput{Gravity: gravityZ(-9810)}, dt)
		require.NoError(t, err)
		bands += requireBandCuts(t, report.Trace, residual)
		for _, event := range report.Events {
			requireTipEventLaws(t, event)
			// The trace replays a rotating impact's bracket: its rounded
			// poses lie within PointResolution of contact.
			from, to := event.Bracket.From.Elapsed.Value.Base(), event.Bracket.To.Elapsed.Value.Base()
			if from == to {
				continue
			}
			impacts++
			sample, err := report.Trace.Sample(units.Seconds(event.SliceStart.Base() + (from+to)/2))
			require.NoError(t, err)
			boxAt, ok := sample.Body(scene.box)
			require.True(t, ok)
			contact, err := scene.doc.ContactPair(t.Context(), scene.floor, scene.box, r3.Identity(), boxAt.Pose,
				scene.config.Contact)
			require.NoError(t, err)
			if contact.Relation == decad.ContactOverlapping {
				require.NotNil(t, contact.Manifold)
				require.GreaterOrEqual(t, contact.Manifold.Points[0].Separation.Value.Base(),
					-scene.config.Contact.PointResolution.Base())
			}
		}
		if report.Status != dynamics.Advanced {
			stopped = report
			break
		}
		require.NotEmpty(t, report.Events, "every step's kick brings the edge down")
	}
	require.NotNil(t, stopped, "the cube lands within 16 steps")
	require.GreaterOrEqual(t, len(timeline.Steps()), 4)
	require.GreaterOrEqual(t, impacts, 4, "each step after the first lands its edge as a rotating impact")
	require.GreaterOrEqual(t, bands, len(timeline.Steps()), "each solved edge rides a band track")

	// The first step's initial contact is the resting edge x = z = 0.
	first := timeline.Steps()[0].Events[0]
	require.Zero(t, first.Time.Base())
	require.Len(t, first.Manifold.Points, 2)
	for _, point := range first.Manifold.Points {
		require.InDelta(t, 0, localPoint(t, first.PoseB, point.OnB.Value).X, 1e-9)
		require.InDelta(t, 0, point.OnB.Value.Z, point.OnB.Bound.Base())
	}

	// The cube lands on its far edge, x = 8, nearly flat, while tipping.
	var landing *dynamics.ContactEvent
	for i := range stopped.Events {
		event := &stopped.Events[i]
		if math.Abs(localPoint(t, event.PoseB, event.Manifold.Points[0].OnB.Value).X-8) < 1e-6 {
			landing = event
			break
		}
	}
	require.NotNil(t, landing, "the far edge lands")
	require.Len(t, landing.Manifold.Points, 2)
	require.Positive(t, landing.PreAngularVelocityB.Y.Base())
	require.Less(t, math.Abs(landing.PoseB.ApplyDir(r3.Vec{X: 1}).Z), 1e-2)
	for _, point := range landing.Manifold.Points {
		local := localPoint(t, landing.PoseB, point.OnB.Value)
		require.InDelta(t, 8, local.X, 1e-6)
		require.InDelta(t, 4, math.Abs(local.Y), 1e-6)
		require.InDelta(t, 0, local.Z, 1e-6)
	}

	// The rocking stops the step where a far edge closes within a grid step
	// of the near edge's touch; the certified prefix covers the landing.
	require.Equal(t, dynamics.Undecided, stopped.Status)
	require.NotEmpty(t, stopped.Diagnostics)
	require.Equal(t, dynamics.StepPairUndecided, stopped.Diagnostics[0].Code, "%+v", stopped.Diagnostics)
	_, err = stopped.Trace.Sample(landing.Time)
	require.NoError(t, err)
	_, err = timeline.Advance(t.Context(), dynamics.StepInput{Gravity: gravityZ(-9810)}, dt)
	require.ErrorIs(t, err, dynamics.ErrTimelineStopped)
}
