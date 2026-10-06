package dynamics_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// sideCylinder is a Ø20 source cylinder about the world X axis over
// x ∈ [−15, 15]: a full revolve of the rectangle [−15, 15] × [0, 10].
func sideCylinder(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-15, 0, 15, 10)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Revolve(s, s.Profiles()[0], decad.SketchLine{
		Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

// TestScheduledStepCylinderRolls rolls a Ø20 cylinder on its side across a
// fixed floor for one turn at ω = 2π rad/s in a four-body world, through the
// scheduled step and the rolling track of docs/multibody-dynamics-design.md
// §10.4. Sixteen steps of 1/16 s move it π·20 mm along −y with its contact
// point at rest within VelocityResidual.
//
// The first step starts at a signed-axis pose, so the rolling track is an
// exact touch. Every later step starts at a turned pose whose float basis is
// orthonormal only to rounding: ContactPair proves a band of about 1e-15 mm
// there, and the track carries it. Under gravity each kick takes the pair
// out of the contact set, so every step opens with the initial-contact solve
// on the ruling's two ends, whose normal impulse stops the kick's g·dt and
// whose friction rows keep the contact point at rest. Without gravity the
// contact set carries the pair from step to step with no event.
//
// On §2's tray the floor is a face-local plane (docs/multibody-dynamics-design.md
// §10.6): the walls and rim stand in front of it, so every placed ruling and
// rolling track also runs the column test against them. The cylinder starts
// at y = 30 so its whole turn keeps clear of the walls, and it rolls under
// gravity as on the plain floor.
func TestScheduledStepCylinderRolls(t *testing.T) {
	t.Parallel()
	const (
		omega = 2 * math.Pi
		steps = 16
		dt    = 1.0 / steps
		g     = 9810.0
		// The closed forms below run in float64 over values below 100 mm;
		// every pose is a float composition of sixteen turns.
		slack = 1e-9
	)
	scenes := []struct {
		name    string
		tray    bool
		gravity float64
	}{{name: "gravity", gravity: -g}, {name: "no gravity"}, {name: "tray, gravity", tray: true, gravity: -g}}
	for _, cylinderFirst := range []bool{false, true} {
		for _, scene := range scenes {
			gravity := scene.gravity
			name := map[bool]string{false: "floor then cylinder", true: "cylinder then floor"}[cylinderFirst] +
				", " + scene.name
			t.Run(name, func(t *testing.T) {
				doc := decad.New()
				floor := makeBox(t, doc, -40, -100, 40, 40, -10, 10)
				startY := 0.0
				if scene.tray {
					floor, startY = tumbleTray(t, doc), 30
				}
				cylinder := sideCylinder(t, doc)
				material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(.5)}
				density := units.KilogramsPerCubicMillimeter(.001)
				mass, err := cylinder.MassProperties(t.Context(), density)
				require.NoError(t, err)
				bodies := []dynamics.RigidBody{{Body: floor, Role: dynamics.Fixed, Material: material},
					{Body: makeBox(t, doc, 500, 0, 510, 10, 0, 10), Role: dynamics.Fixed, Material: material},
					{Body: makeBox(t, doc, 600, 0, 610, 10, 0, 10), Role: dynamics.Fixed, Material: material}}
				rolling := dynamics.RigidBody{Body: cylinder, Role: dynamics.Dynamic, Density: &density, Material: material}
				pair := dynamics.BodyPair{A: floor, B: cylinder}
				if cylinderFirst {
					bodies = append([]dynamics.RigidBody{rolling}, bodies...)
					pair = dynamics.BodyPair{A: cylinder, B: floor}
				} else {
					bodies = append(bodies, rolling)
				}
				config := pairMaterialStepConfig()
				config.MaxEvents = 16
				world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Bodies: bodies, Step: config})
				require.NoError(t, err)
				start, err := r3.Translation(r3.Vec{Y: startY, Z: 10})
				require.NoError(t, err)
				entries := make([]dynamics.BodyState, 0, len(bodies))
				for _, body := range bodies {
					entry := dynamics.BodyState{Body: body.Body, Pose: r3.Identity(),
						LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)}
					if body.Body == cylinder {
						// Rolling without slip: v = −ω × (−r·ẑ) = (0, −ω·r, 0).
						entry.Pose = start
						entry.LinearVelocity.Y = units.MillimetersPerSecond(-omega * 10)
						entry.AngularVelocity.X = units.RadiansPerSecond(omega)
					}
					entries = append(entries, entry)
				}
				state, err := world.NewState(entries)
				require.NoError(t, err)
				timeline, err := dynamics.NewTimeline(world, state)
				require.NoError(t, err)
				velocityResidual := config.VelocityResidual.Base()

				// contactSpeed is the cylinder's material speed at a point, with
				// its world mass center at center.
				contactSpeed := func(entry dynamics.BodyState, center, point r3.Vec) float64 {
					v := r3.Vec{X: entry.LinearVelocity.X.Base(), Y: entry.LinearVelocity.Y.Base(),
						Z: entry.LinearVelocity.Z.Base()}
					w := r3.Vec{X: entry.AngularVelocity.X.Base(), Y: entry.AngularVelocity.Y.Base(),
						Z: entry.AngularVelocity.Z.Base()}
					return v.Add(w.Cross(point.Sub(center))).Len()
				}
				for step := range steps {
					report, err := timeline.Advance(t.Context(), dynamics.StepInput{Gravity: gravityZ(gravity)},
						units.Seconds(dt))
					require.NoError(t, err)
					require.Equal(t, dynamics.Advanced, report.Status, "step %d: %+v", step, report.Diagnostics)
					entry, ok := report.Next.Body(cylinder)
					require.True(t, ok)
					// The axis stays on z = 10 and travels ω·r per second.
					at := entry.Pose.Translation()
					require.Zero(t, at.X, "step %d", step)
					require.InDelta(t, 10, at.Z, slack, "step %d", step)
					require.InDelta(t, startY-omega*10*dt*float64(step+1), at.Y, slack, "step %d", step)
					require.InDelta(t, 0, entry.Pose.Basis().EX.Sub(r3.Vec{X: 1}).Len(), slack, "step %d", step)
					require.InDelta(t, -omega*10, entry.LinearVelocity.Y.Base(), velocityResidual, "step %d", step)
					require.InDelta(t, 0, entry.LinearVelocity.Z.Base(), velocityResidual, "step %d", step)
					require.InDelta(t, omega, entry.AngularVelocity.X.Base(), config.AngularVelocityResidual.Base(),
						"step %d", step)

					if gravity == 0 {
						require.Empty(t, report.Events, "step %d", step)
					} else {
						// The kick's g·dt is stopped at the step start on the
						// ruling's two ends, with no tangential impulse: the
						// contact point is already at rest.
						require.Len(t, report.Events, 1, "step %d", step)
						event := report.Events[0]
						require.Equal(t, dynamics.ContactImpact, event.Kind, "step %d", step)
						require.Equal(t, pair, event.Pair, "step %d", step)
						require.Zero(t, event.Time.Base(), "step %d", step)
						require.Len(t, event.Manifold.Points, 2, "step %d", step)
						kick := g * dt * mass.Mass.Value.Base()
						impulseSlack := g*dt*mass.Mass.Bound.Base() + config.ImpulseResidual.Base() + slack
						require.InDelta(t, kick, event.NormalImpulse.Base(), impulseSlack, "step %d", step)
						tangent := r3.Vec{X: event.TangentImpulse.X.Base(), Y: event.TangentImpulse.Y.Base(),
							Z: event.TangentImpulse.Z.Base()}
						require.LessOrEqual(t, tangent.Len(), config.ImpulseResidual.Base(), "step %d", step)
						pre, post := event.PreVelocityB, event.PostVelocityB
						if cylinderFirst {
							pre, post = event.PreVelocityA, event.PostVelocityA
						}
						require.Equal(t, -g*dt, pre.Z.Base(), "step %d", step)
						require.InDelta(t, 0, post.Z.Base(), velocityResidual, "step %d", step)
					}

					// The step's last slice rolls on the track: an exact touch
					// from the first step's signed-axis pose, a band of the
					// turned pose's rounding after it.
					proofs := dynamics.TraceSliceProofs(report.Trace)
					var sweep *decad.SweepReport
					for _, proof := range proofs[len(proofs)-1] {
						if proof.Pair == pair {
							sweep = proof.Sweep
						}
					}
					require.NotNil(t, sweep, "step %d", step)
					track := sweep.ContactTrack
					require.NotNil(t, track, "step %d", step)
					require.Equal(t, 1.0, track.End().Fraction.Base(), "step %d", step)
					if step == 0 {
						require.Equal(t, decad.SweepPersistentTouch, sweep.Outcome)
						require.Nil(t, track.Band())
					} else {
						require.Equal(t, decad.SweepPersistentBand, sweep.Outcome, "step %d", step)
						band := track.Band()
						require.NotNil(t, band, "step %d", step)
						require.LessOrEqual(t, band.Value.Base()+band.Bound.Base(), config.PenetrationResidual.Base(),
							"step %d", step)
					}
					// Mid-step, the published contact points are at rest on the
					// cylinder: v + ω × (p − c) within VelocityResidual, beyond
					// |ω| times each point's ball.
					middle := (float64(step) + .5) * dt
					sample, err := timeline.Sample(units.Seconds(middle))
					require.NoError(t, err)
					mid, ok := sample.Body(cylinder)
					require.True(t, ok)
					require.InDelta(t, startY-omega*10*middle, mid.Pose.Translation().Y, slack, "step %d", step)
					manifold, err := track.ManifoldAt(units.Scalar(.5))
					require.NoError(t, err, "step %d", step)
					require.Len(t, manifold.Points, 2, "step %d", step)
					for i, point := range manifold.Points {
						onCylinder := point.OnB
						if cylinderFirst {
							onCylinder = point.OnA
						}
						require.InDelta(t, float64(2*i-1)*15, onCylinder.Value.X, onCylinder.Bound.Base(), "step %d", step)
						require.InDelta(t, 0, onCylinder.Value.Z, onCylinder.Bound.Base(), "step %d", step)
						speed := contactSpeed(mid, mid.Pose.Translation(), onCylinder.Value)
						require.LessOrEqual(t, speed, velocityResidual+omega*onCylinder.Bound.Base(),
							"step %d end %d", step, i)
					}
				}
				// One turn: π·20 mm along −y, and the basis back at the identity.
				require.Equal(t, 1.0, timeline.End().Base())
				final, err := timeline.Sample(timeline.End())
				require.NoError(t, err)
				entry, ok := final.Body(cylinder)
				require.True(t, ok)
				require.InDelta(t, startY-math.Pi*20, entry.Pose.Translation().Y, slack)
				basis := entry.Pose.Basis()
				require.InDelta(t, 1, basis.EY.Y, slack)
				require.InDelta(t, 0, basis.EY.Z, slack)
				require.InDelta(t, 1, basis.EZ.Z, slack)
				require.InDelta(t, 0, basis.EZ.Y, slack)
			})
		}
	}
}
