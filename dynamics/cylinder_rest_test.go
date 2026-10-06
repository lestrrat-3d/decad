package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestScheduledStepCylinderLandsAndRests drops a source cylinder on its end
// disk onto a fixed floor in a four-body world, through SweepPair's disk
// impact bracket and its disk-on-face persistent track
// (docs/contact-sweep-design.md §4.6). Every step kicks the cylinder by
// g·dt = −9810/256 = −38.3203125 mm/s, below ImpactSpeed 64 mm/s, so each
// impact targets zero: the first lands the cylinder from 1/16 mm at
// (1/16)/38.3203125 s, and every later step starts in touch, stops the kick
// at time zero and rests on the track for the rest of the step. Without the
// track the first step stops after the landing with StepPairUndecided.
func TestScheduledStepCylinderLandsAndRests(t *testing.T) {
	t.Parallel()
	const kick = 9810.0 / 256
	for _, cylinderFirst := range []bool{false, true} {
		t.Run(map[bool]string{false: "floor then cylinder", true: "cylinder then floor"}[cylinderFirst], func(t *testing.T) {
			doc := decad.New()
			floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
			cylinder := makeCylinder(t, doc)
			material := dynamics.Material{Restitution: units.Scalar(0.3), Friction: units.Scalar(0.4)}
			mass := cylinderMass()
			bodies := []dynamics.RigidBody{{Body: floor, Role: dynamics.Fixed, Material: material},
				{Body: makeBox(t, doc, 500, 0, 510, 10, 0, 10), Role: dynamics.Fixed, Material: material},
				{Body: makeBox(t, doc, 600, 0, 610, 10, 0, 10), Role: dynamics.Fixed, Material: material}}
			dropped := dynamics.RigidBody{Body: cylinder, Role: dynamics.Dynamic, Supplied: &mass, Material: material}
			if cylinderFirst {
				bodies = append([]dynamics.RigidBody{dropped}, bodies...)
			} else {
				bodies = append(bodies, dropped)
			}
			config := pairMaterialStepConfig()
			config.ImpactSpeed = units.MillimetersPerSecond(64)
			config.MaxEvents = 16
			world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Bodies: bodies, Step: config})
			require.NoError(t, err)
			start, err := r3.Translation(r3.Vec{Z: 1.0 / 16})
			require.NoError(t, err)
			entries := make([]dynamics.BodyState, 0, len(bodies))
			for _, body := range bodies {
				pose := r3.Identity()
				if body.Body == cylinder {
					pose = start
				}
				entries = append(entries, dynamics.BodyState{Body: body.Body, Pose: pose,
					LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)})
			}
			state, err := world.NewState(entries)
			require.NoError(t, err)
			timeline, err := dynamics.NewTimeline(world, state)
			require.NoError(t, err)
			pair := dynamics.BodyPair{A: floor, B: cylinder}
			if cylinderFirst {
				pair = dynamics.BodyPair{A: cylinder, B: floor}
			}
			cylinderZ := func(state dynamics.State) (float64, float64) {
				entry, ok := state.Body(cylinder)
				require.True(t, ok)
				return entry.Pose.Translation().Z, entry.LinearVelocity.Z.Base()
			}
			dt := pyramidDt()
			resolution := config.TimeResolution.Base()
			for step := range 8 {
				report, err := timeline.Advance(t.Context(), dynamics.StepInput{Gravity: gravityZ(-9810)}, dt)
				require.NoError(t, err)
				require.Equal(t, dynamics.Advanced, report.Status, "step %d: %+v", step, report.Diagnostics)
				require.Len(t, report.Events, 1, "step %d", step)
				event := report.Events[0]
				require.Equal(t, dynamics.ContactImpact, event.Kind, "step %d", step)
				require.Equal(t, pair, event.Pair, "step %d", step)
				pre, post := event.PreVelocityB.Z.Base(), event.PostVelocityB.Z.Base()
				if cylinderFirst {
					pre, post = event.PreVelocityA.Z.Base(), event.PostVelocityA.Z.Base()
				}
				require.Equal(t, -kick, pre, "step %d", step)
				require.InDelta(t, 0, post, 1e-9, "step %d", step)
				// One kilogram stops from 38.3203125 mm/s.
				require.InDelta(t, kick, event.NormalImpulse.Base(), 1e-6, "step %d", step)
				if step == 0 {
					landing := (1.0 / 16) / kick
					require.GreaterOrEqual(t, event.Time.Base(), landing)
					require.InDelta(t, landing, event.Time.Base(), resolution)
				} else {
					require.Zero(t, event.Time.Base(), "step %d", step)
				}
				// The last slice rests the disk on the floor through a
				// persistent track whose point is the disk center.
				proofs := dynamics.TraceSliceProofs(report.Trace)
				var track *decad.SweepContactTrack
				for _, proof := range proofs[len(proofs)-1] {
					if proof.Pair == pair {
						require.NotNil(t, proof.Sweep, "step %d", step)
						require.Equal(t, decad.SweepPersistentTouch, proof.Sweep.Outcome, "step %d", step)
						track = proof.Sweep.ContactTrack
					}
				}
				require.NotNil(t, track, "step %d", step)
				manifold, err := track.ManifoldAt(units.Scalar(0.5))
				require.NoError(t, err, "step %d", step)
				require.Len(t, manifold.Points, 1, "step %d", step)
				normal := r3.Vec{Z: 1}
				if cylinderFirst {
					normal.Z = -1
				}
				require.Equal(t, normal, manifold.Points[0].Normal.Value, "step %d", step)
				require.Equal(t, r3.Vec{}, manifold.Points[0].OnA.Value, "step %d", step)
				require.Zero(t, manifold.Points[0].Separation.Value.Base(), "step %d", step)
				sample, err := report.Trace.Sample(units.Seconds(0.75 / 256))
				require.NoError(t, err, "step %d", step)
				z, v := cylinderZ(sample)
				require.Zero(t, z, "step %d", step)
				require.InDelta(t, 0, v, 1e-9, "step %d", step)
				global, err := timeline.Sample(units.Seconds((float64(step) + 0.75) / 256))
				require.NoError(t, err, "step %d", step)
				globalZ, _ := cylinderZ(global)
				require.Zero(t, globalZ, "step %d", step)
			}
			require.Equal(t, 8.0/256, timeline.End().Base())
			final, err := timeline.Sample(timeline.End())
			require.NoError(t, err)
			z, v := cylinderZ(final)
			require.Zero(t, z)
			require.InDelta(t, 0, v, 1e-9)
		})
	}
}
