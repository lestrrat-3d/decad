package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// Legs of the correction of a resting pair and of the anchored push
// (docs/multibody-dynamics-design.md §6.6), each shown to fail by deleting
// it in island.go, watching the named tests go red, and restoring it:
//   - the push moves only a body with a correction allowance:
//     TestIslandSphereLandsOnRestingSphere (step 41) and
//     TestIslandSphereRestsOnSphere (step 36) push the lower sphere, which
//     rests on the floor and has none, and refuse with "push of body 1
//     exceeds its correction allowance";
//   - the search that places a resting pair back in touch:
//     TestSettleRestingDiskIntoTouch pushes the disk apart instead, past its
//     allowance;
//   - a resting pair off every persistent track is pushed apart:
//     TestIslandSphereRestsOnSphere (step 36), TestSettleRestingTiltedFace,
//     TestSettleRestingDiskIntoTouch's small allowance and
//     TestIslandPushRestsACurvedPairApart (island_push_test.go) refuse with
//     "corrected pair relation is 3, not touching";
//   - a resting pair on a persistent track is never pushed apart:
//     TestSettleRestingTiltedFace's tracked half ends apart;
//   - the resting margin: TestIslandSphereRestsOnSphere refuses at step 40
//     with "correction sweep returned 8", and TestSettleRestingTiltedFace
//     and TestIslandPushRestsACurvedPairApart end an ulp apart;
//   - the push of a resting pair the correction leaves apart by less than
//     its margin: TestIslandSphereRestsOnSphere refuses at step 40 the same
//     way. (The two push fixtures end a hair short of the margin without
//     it, inside the quarter-margin slack their assertion keeps.)
//
// restSearchLimit and the untouched set bound the search and are not gates:
// a search that stops early only falls back to the push apart, whose
// allowance check still applies.

// TestSettleRestingDiskIntoTouch places the end disk of a source cylinder
// 2^-44 mm into a fixed floor, as a correction that rounds one way on one
// architecture leaves it, and settles the pair as one the solve leaves
// resting. The disk lies on the floor's top face at the exact pose z = 0,
// which the search finds: the cylinder moves up by exactly 2^-44 mm within a
// 1e-6 mm allowance, ContactPair proves the pair touching, and the
// pair stays in the contact set. Both body orders.
func TestSettleRestingDiskIntoTouch(t *testing.T) {
	t.Parallel()
	for _, cylinderFirst := range []bool{false, true} {
		t.Run(map[bool]string{false: "floor then cylinder", true: "cylinder then floor"}[cylinderFirst], func(t *testing.T) {
			doc := decad.New()
			floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
			cylinder := makeCylinder(t, doc)
			material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0.4)}
			mass := cylinderMass()
			bodies := []dynamics.RigidBody{{Body: floor, Role: dynamics.Fixed, Material: material},
				{Body: farBox(t, doc, 1000), Role: dynamics.Fixed, Material: material},
				{Body: farBox(t, doc, 2000), Role: dynamics.Fixed, Material: material}}
			dropped := dynamics.RigidBody{Body: cylinder, Role: dynamics.Dynamic, Supplied: &mass, Material: material}
			if cylinderFirst {
				bodies = append([]dynamics.RigidBody{dropped}, bodies...)
			} else {
				bodies = append(bodies, dropped)
			}
			config := pairMaterialStepConfig()
			world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Bodies: bodies, Step: config})
			require.NoError(t, err)
			state := func(x, z float64) dynamics.State {
				pose, err := r3.Translation(r3.Vec{X: x, Y: -2, Z: z})
				require.NoError(t, err)
				entries := make([]dynamics.BodyState, 0, len(bodies))
				for _, body := range bodies {
					entry := dynamics.BodyState{Body: body.Body, Pose: r3.Identity(), LinearVelocity: zeroVelocity(),
						AngularVelocity: zeroAngular(t)}
					if body.Body == cylinder {
						entry.Pose = pose
					}
					entries = append(entries, entry)
				}
				s, err := world.NewState(entries)
				require.NoError(t, err)
				return s
			}
			// The pre-event pose sits 2^-20 mm aside along X and 2^-30 mm up,
			// so every move counts that offset against the allowance.
			pre, sunk := state(3+0x1p-20, 0x1p-30), state(3, -0x1p-44)
			a, b, normal := floor, cylinder, r3.Vec{Z: 1}
			if cylinderFirst {
				a, b, normal = cylinder, floor, r3.Vec{Z: -1}
			}
			contact, err := doc.ContactPair(t.Context(), a, b, poseOf(t, sunk, a), poseOf(t, sunk, b), config.Contact)
			require.NoError(t, err)
			require.Equal(t, decad.ContactOverlapping, contact.Relation)
			reason, settled, kept, err := dynamics.SettleResting(t.Context(), world, pre, sunk, a, b,
				map[*decad.Body]units.Value{cylinder: units.Millimeters(1e-6)}, normal, false)
			require.NoError(t, err)
			require.Empty(t, reason)
			require.True(t, kept)
			require.Equal(t, r3.Vec{X: 3, Y: -2}, poseOf(t, settled, cylinder).Translation())
			contact, err = doc.ContactPair(t.Context(), a, b, poseOf(t, settled, a), poseOf(t, settled, b), config.Contact)
			require.NoError(t, err)
			require.Equal(t, decad.ContactTouching, contact.Relation)
			// The touching pose lies 2^-20 + 2^-30 mm from the pre-event pose,
			// and any pose apart lies further: an allowance of 2^-21 mm
			// refuses both.
			reason, _, _, err = dynamics.SettleResting(t.Context(), world, pre, sunk, a, b,
				map[*decad.Body]units.Value{cylinder: units.Millimeters(0x1p-21)}, normal, false)
			require.NoError(t, err)
			require.Contains(t, reason, "exceeds its correction allowance")
		})
	}
}

// TestSettleRestingTiltedFace sinks the bounce scene's sphere 2^-40 mm into
// the face tilted 45° about Y and settles the pair as one the solve leaves
// resting. No touching pose is found along the tilted normal, so a pair off
// every persistent track is pushed apart by at least half of ContactSlop
// and leaves the contact set, while a pair on a persistent track, whose
// track needs exact touch, is refused.
func TestSettleRestingTiltedFace(t *testing.T) {
	t.Parallel()
	scene := newPushScene(t, 0, pairMaterialStepConfig())
	sunk, err := r3.Translation(scene.normal.Scale(15 - 0x1p-40))
	require.NoError(t, err)
	var entries []dynamics.BodyState
	for _, entry := range scene.state.Entries() {
		if entry.Body == scene.ball {
			entry.Pose = sunk
		}
		entries = append(entries, entry)
	}
	state, err := scene.world.NewState(entries)
	require.NoError(t, err)
	allowance := map[*decad.Body]units.Value{scene.ball: units.Millimeters(1e-6)}
	reason, settled, kept, err := dynamics.SettleResting(t.Context(), scene.world, state, state, scene.face,
		scene.ball, allowance, scene.normal, false)
	require.NoError(t, err)
	require.Empty(t, reason)
	require.False(t, kept)
	face := poseOf(t, settled, scene.face)
	contact, err := scene.doc.ContactPair(t.Context(), scene.face, scene.ball, face, poseOf(t, settled, scene.ball),
		scene.config.Contact)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, contact.Relation)
	slop := scene.config.ContactSlop.Base()
	// The margin is half of ContactSlop; its last push may round a hair
	// short of it, so the proved gap is held to a quarter.
	require.GreaterOrEqual(t, contact.Gap.Value.Base()-contact.Gap.Bound.Base(), slop/4)
	require.Less(t, contact.Gap.Value.Base(), slop)
	reason, _, _, err = dynamics.SettleResting(t.Context(), scene.world, state, state, scene.face, scene.ball,
		allowance, scene.normal, true)
	require.NoError(t, err)
	require.Contains(t, reason, "not touching")
}

// TestIslandSphereLandsOnRestingSphere offsets the stack-and-drop sphere
// column along Y (0, 2 and −2 mm): the upper sphere glances off the lower
// one and the top sphere lands on the lower one, which rests on the floor
// through a persistent track. That landing's correction moves only the top
// sphere and leaves the pair an ulp into each other; the push that follows
// moves the top sphere alone too, since the lower sphere has no correction
// allowance. The first 64 steps advance, no two spheres ever overlap by
// more than PenetrationResidual at a step end, and the lower sphere never
// sinks into the floor.
func TestIslandSphereLandsOnRestingSphere(t *testing.T) {
	t.Parallel()
	scene := newSphereColumn(t, [3]float64{0, 2, -2})
	advanceColumn(t, scene, 64)
}

// TestIslandSphereRestsOnSphere offsets the upper sphere of the column by
// 0.5 mm along Y: it lands on the lower sphere below ImpactSpeed and rests
// on it off center, and the top sphere comes to rest on it. Two spheres
// whose center line is off every axis have no touching pose at float
// centers, so each resting pair is pushed apart by at least half of
// ContactSlop and leaves the contact set; the margin keeps a later
// correction in the same step, which moves the middle sphere toward its
// other neighbor, from reaching that pair. The first 64 steps advance with
// the same overlap and floor checks.
func TestIslandSphereRestsOnSphere(t *testing.T) {
	t.Parallel()
	scene := newSphereColumn(t, [3]float64{0, 0.5, 0})
	advanceColumn(t, scene, 64)
}

// advanceColumn advances a sphere column, requiring every step to advance
// with no two spheres overlapping by more than PenetrationResidual and no
// sphere center below its radius at the step end.
func advanceColumn(t *testing.T, scene *sphereColumn, steps int) {
	t.Helper()
	spheres := []*decad.Body{scene.lower, scene.upper, scene.top}
	residual := scene.config.PenetrationResidual.Base()
	for step := range steps {
		report, err := scene.timeline.Advance(t.Context(), dynamics.StepInput{Gravity: gravityZ(-9810)},
			units.Seconds(1.0/256))
		require.NoError(t, err)
		require.Equal(t, dynamics.Advanced, report.Status, "step %d: %+v", step, report.Diagnostics)
		for i, first := range spheres {
			a := poseOf(t, *report.Next, first).Translation()
			require.GreaterOrEqual(t, a.Z, 8-residual, "step %d", step)
			for _, second := range spheres[i+1:] {
				b := poseOf(t, *report.Next, second).Translation()
				require.GreaterOrEqual(t, b.Sub(a).Len()-16, -residual, "step %d", step)
			}
		}
	}
}

func poseOf(t *testing.T, state dynamics.State, body *decad.Body) r3.Transform {
	t.Helper()
	entry, ok := state.Body(body)
	require.True(t, ok)
	return entry.Pose
}
