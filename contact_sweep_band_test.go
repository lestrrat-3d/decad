package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The fixtures of docs/multibody-dynamics-design.md §13 PR 13: §10.2's
// departure from touch under rotation, §10.3's band track, and the replay
// proof of the general planar sweep. Every input is dyadic; the edge-box scene
// sits tumbleOffset from the origin, so its rounded poses deviate from the
// ideal path by about an ULP of 2^20 mm.
//
// Each expected horizon below is recomputed here in float64 from the closed
// forms of §10.2 and compared within two grid steps; each leg changes it by
// far more than that.
//
// Legs shown to fail (each deleted or zeroed in turn, fixture red, then
// restored):
//   - the contact rate c of §10.2: TestSweepPairPlanarDepartureFromEdge loses
//     its departure;
//   - |ω_M|²·ρ_M: TestSweepPairPlanarDepartureFromEdge departs over the whole
//     second, and TestSweepPairPlanarBandTrack's spinning box reads as an
//     exact touch;
//   - |ω_S|²·ρ_S, 2·|ω_S|·(|Δv| + |ω_M|·ρ_M + |ω_S|·ρ_S) term by term, and
//     |ω_S|²·D with each of D's terms: TestSweepPairPlanarDepartureRotatingSupport
//     moves its horizon;
//   - the band's contact-rate term: TestSweepPairPlanarBandTrack's closing
//     case publishes K·h² alone, in Band() and in BandAt();
//   - BandAt's curvature term: TestSweepPairPlanarBandTrack's prefix band
//     reads |c|·u alone;
//   - the band's clearance of the other vertices: TestSweepPairPlanarBandTrack's
//     one-second track reaches the duration;
//   - the band's face containment: TestSweepPairPlanarBandLeavesFace reaches
//     the duration with its edge past the floor's rim;
//   - the replay's pose deviation: TestSweepPairPlanarDepartureFromEdge
//     replays a pose whose gap lies below its own rounding.
//
// The band replay's height check only refuses; on a sound certificate it
// cannot fire, so no fixture reaches it. The replay's travel charge inside a
// certified clear span is §4.3's own inequality; a sound sweep leaves no
// span whose margin falls below a pose rounding, so
// contact_sweep_band_internal_test.go shows that leg on the bound itself.

// edgeBoxScene is a source box 8 mm square in x and z and 8 mm long in y,
// turned 45° about Y so its edge x = z = 0 rests on a wide floor whose top
// face is z = 0, both moved tumbleOffset along X. The box drifts with v and
// spins at 1 rad/s about Y through that edge's midpoint, so the edge lies on
// the rotation axis.
func edgeBoxScene(t *testing.T, doc *decad.Document, v r3.Vec, seconds float64) (*decad.Body,
	*decad.Body, decad.RigidDriftSegment, decad.RigidDriftSegment) {
	t.Helper()
	floor := boxBodyAtZ(t, doc, -100, -100, 100, 100, -10, 10)
	box := boxBody(t, doc, 0, -4, 8, 4, 8)
	floorPath := sweepDrift(r3.Vec{}, seconds)
	floorPath.From = contactPose(t, r3.Vec{X: tumbleOffset})
	boxPath := sweepDrift(v, seconds)
	boxPath.From = edgeBoxPose(t, r3.Vec{X: tumbleOffset})
	boxPath.Center = r3.Vec{X: tumbleOffset}
	boxPath.AngularVelocity.Y = units.RadiansPerSecond(1)
	return floor, box, floorPath, boxPath
}

// edgeBoxPose turns the box onto its edge and moves that edge's midpoint to at.
func edgeBoxPose(t *testing.T, at r3.Vec) r3.Transform {
	t.Helper()
	c := math.Sqrt2 / 2
	return basisPose(t, r3.Vec{X: c, Z: c}, r3.Vec{Y: 1}, r3.Vec{X: -c, Z: c}, at)
}

// edgeBoxVertices are the box's vertices relative to the pivot.
func edgeBoxVertices() []r3.Vec {
	c := math.Sqrt2 / 2
	var out []r3.Vec
	for _, x := range []float64{0, 8} {
		for _, y := range []float64{-4, 4} {
			for _, z := range []float64{0, 8} {
				out = append(out, r3.Vec{X: c*x - c*z, Y: y, Z: c*x + c*z})
			}
		}
	}
	return out
}

// edgeBoxCurvature is §10.2's K for a stationary support: |ω|²·ρ/2 with ρ the
// largest vertex distance from the pivot, 12 mm here.
func edgeBoxCurvature() float64 {
	rho := 0.0
	for _, v := range edgeBoxVertices() {
		rho = math.Max(rho, v.Len())
	}
	return rho / 2
}

// edgeBoxClearRoot is the earliest time a vertex off the edge can reach the
// floor under the §10.2 bound h0 + (vz − x)·u − K·u², the spin about +Y
// lowering each vertex at x mm/s.
func edgeBoxClearRoot(vz float64) float64 {
	k := edgeBoxCurvature()
	root := math.Inf(1)
	for _, v := range edgeBoxVertices() {
		if v.Z == 0 {
			continue
		}
		d := vz - v.X
		root = math.Min(root, (d+math.Sqrt(d*d+4*k*v.Z))/(2*k))
	}
	return root
}

// gridFloor is the largest multiple of step at or below x.
func gridFloor(x, step float64) float64 { return math.Floor(x/step) * step }

func bandRequest() decad.SweepRequest {
	req := tumbleRequest()
	req.StartPolicy = decad.ContinueCertifiedTouch
	return req
}

func TestSweepPairPlanarBandTrack(t *testing.T) {
	// The grid step of a 1 s sweep at the 2^-20 s resolution.
	const step = 1.0 / (1 << 20)
	// The closed forms run in float64 over values below 20.
	const slack = 1e-12
	k := edgeBoxCurvature()
	for _, tc := range []struct {
		name    string
		vz      float64
		seconds float64
		end     float64 // track end, as a fraction
	}{
		// A quarter second keeps every other vertex clear: the track covers
		// the whole sweep and its depth is K·h².
		{name: "quarter", seconds: .25, end: 1},
		// Over a second the bound lets the lowering vertex reach the floor at
		// 0.608 s, so the track ends on the grid below that time.
		{name: "second", seconds: 1, end: gridFloor(edgeBoxClearRoot(0), step)},
		// A slow closing speed adds its rate to the depth: |c|·h + K·h².
		{name: "closing", vz: -.25, seconds: .25, end: 1},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New()
			floor, box, floorPath, boxPath := edgeBoxScene(t, doc, r3.Vec{Z: tc.vz}, tc.seconds)
			before := doc.Bodies()
			for order := range 2 {
				a, b := floor, box
				pathA, pathB := decad.PairPath(floorPath), decad.PairPath(boxPath)
				if order == 1 {
					a, b, pathA, pathB = box, floor, pathB, pathA
				}
				report, err := doc.SweepPair(t.Context(), a, b, pathA, pathB, bandRequest())
				require.NoError(t, err)
				require.Equal(t, decad.SweepPersistentBand, report.Outcome, "order %d cause=%v", order, report.Cause)
				track := report.ContactTrack
				require.NotNil(t, track)
				require.Zero(t, track.Start().Fraction.Base())
				require.InDelta(t, tc.end, track.End().Fraction.Base(), 2*step)
				h := track.End().Elapsed.Value.Base()
				band := track.Band()
				require.NotNil(t, band)
				require.Equal(t, units.Length, band.Value.Kind())
				require.InDelta(t, math.Abs(tc.vz)*h+k*h*h, band.Value.Base(), band.Bound.Base()+slack)
				// BandAt bounds the band over a prefix of the track: the same
				// closed form at half its length, and Band() at its end.
				half := track.End().Fraction.Base() / 2
				prefix, err := track.BandAt(units.Scalar(half))
				require.NoError(t, err)
				u := half * tc.seconds
				require.InDelta(t, math.Abs(tc.vz)*u+k*u*u, prefix.Value.Base(), prefix.Bound.Base()+slack)
				whole, err := track.BandAt(track.End().Fraction)
				require.NoError(t, err)
				require.Equal(t, band.Value, whole.Value)
				if tc.end < 1 {
					_, err = track.BandAt(units.Scalar(1))
					require.ErrorIs(t, err, decad.ErrDegenerate)
				}

				normal := 1.0
				if order == 1 {
					normal = -1
				}
				require.Equal(t, r3.Vec{Z: normal}, track.Normal().Value)
				for _, fraction := range []float64{0, track.End().Fraction.Base() / 2, track.End().Fraction.Base()} {
					manifold, err := track.ManifoldAt(units.Scalar(fraction))
					require.NoError(t, err, "fraction %v", fraction)
					require.Len(t, manifold.Points, 2)
					// The edge rides the rotation axis, which moves with the box.
					elapsed := fraction * tc.seconds
					sides := 0.0
					for _, point := range manifold.Points {
						onBox, onFloor := point.OnB, point.OnA
						if order == 1 {
							onBox, onFloor = point.OnA, point.OnB
						}
						want := r3.Vec{X: tumbleOffset, Y: math.Copysign(4, onBox.Value.Y), Z: tc.vz * elapsed}
						sides += want.Y
						require.InDelta(t, 0, onBox.Value.Sub(want).Len(), onBox.Bound.Base()+slack)
						require.InDelta(t, want.X, onFloor.Value.X, onFloor.Bound.Base()+slack)
						require.InDelta(t, want.Y, onFloor.Value.Y, onFloor.Bound.Base()+slack)
						require.InDelta(t, 0, onFloor.Value.Z, onFloor.Bound.Base()+slack)
						require.Equal(t, r3.Vec{Z: normal}, point.Normal.Value)
						require.Zero(t, point.Separation.Value.Base())
						require.GreaterOrEqual(t, point.Separation.Bound.Base(), band.Value.Base())
					}
					require.Zero(t, sides, "both ends of the edge")
				}
				replayA, replayB, err := report.CertifiedPosesAt(units.Seconds(h / 2))
				require.NoError(t, err)
				contact, err := doc.ContactPair(t.Context(), a, b, replayA, replayB, contactRequest())
				require.NoError(t, err)
				require.NotEqual(t, decad.ContactSeparated, contact.Relation)
				if tc.end < 1 {
					_, _, err = report.CertifiedPosesAt(units.Seconds(tc.seconds))
					require.ErrorIs(t, err, decad.ErrUnsupported)
				}
			}
			require.Equal(t, before, doc.Bodies())

			// A departure policy never publishes a band.
			req := bandRequest()
			req.StartPolicy = decad.ContinueSeparatingTouch
			separating, err := doc.SweepPair(t.Context(), floor, box, floorPath, boxPath, req)
			require.NoError(t, err)
			require.Equal(t, decad.SweepUndecided, separating.Outcome)
			require.Equal(t, decad.SweepDepartureUnproved, separating.Cause)
		})
	}
}

func TestSweepPairPlanarBandLeavesFace(t *testing.T) {
	// The grid step of a quarter-second sweep at the 2^-20 s resolution.
	const step = 1.0 / (1 << 18)
	const slack = 1e-12
	// The box starts with its edge 10 mm inside the floor's x = 100 rim and
	// slides toward it at 64 mm/s. The edge's foot, grown by the band depth
	// K·u², must stay strictly inside the face: 64·u + K·u² < 10.
	doc := decad.New()
	floor, box, floorPath, boxPath := edgeBoxScene(t, doc, r3.Vec{X: 64}, .25)
	boxPath.From = edgeBoxPose(t, r3.Vec{X: tumbleOffset + 90})
	boxPath.Center.X += 90
	k := edgeBoxCurvature()
	root := (-64 + math.Sqrt(64*64+40*k)) / (2 * k)
	report, err := doc.SweepPair(t.Context(), floor, box, floorPath, boxPath, bandRequest())
	require.NoError(t, err)
	require.Equal(t, decad.SweepPersistentBand, report.Outcome, "cause=%v", report.Cause)
	end := report.ContactTrack.End()
	require.InDelta(t, gridFloor(root/.25, step), end.Fraction.Base(), 2*step)
	h := end.Elapsed.Value.Base()
	require.InDelta(t, k*h*h, report.ContactTrack.Band().Value.Base(), report.ContactTrack.Band().Bound.Base()+slack)
	require.Less(t, 90+64*h+k*h*h, 100.0)
}

func TestSweepPairPlanarExactTouchTrack(t *testing.T) {
	// A wedge resting on its lower face on a floor, both sliding along X at
	// 16 mm/s: every contact rate is zero and nothing rotates, so the track
	// is an exact touch with no band.
	doc := decad.New()
	floor := boxBodyAtZ(t, doc, -100, -100, 100, 100, -10, 10)
	wedge := tumblerBody(t, doc)
	floorPath := sweepDrift(r3.Vec{X: 16}, 1)
	floorPath.From = contactPose(t, r3.Vec{X: tumbleOffset})
	wedgePath := sweepDrift(r3.Vec{X: 16}, 1)
	wedgePath.From = contactPose(t, r3.Vec{X: tumbleOffset})
	report, err := doc.SweepPair(t.Context(), floor, wedge, floorPath, wedgePath, bandRequest())
	require.NoError(t, err)
	require.Equal(t, decad.SweepPersistentTouch, report.Outcome, "cause=%v", report.Cause)
	track := report.ContactTrack
	require.Nil(t, track.Band())
	require.Equal(t, 1.0, track.End().Fraction.Base())
	manifold, err := track.ManifoldAt(units.Scalar(.5))
	require.NoError(t, err)
	require.Len(t, manifold.Points, 3)
	want := []r3.Vec{{X: 0}, {X: 8}, {X: 2, Y: 6}}
	for _, point := range manifold.Points {
		require.Zero(t, point.Separation.Bound.Base())
		require.InDelta(t, 0, point.OnB.Value.Z, point.OnB.Bound.Base())
		found := false
		for _, w := range want {
			at := r3.Vec{X: tumbleOffset + 8 + w.X, Y: w.Y}
			if point.OnB.Value.Sub(at).Len() <= point.OnB.Bound.Base() {
				found = true
			}
		}
		require.True(t, found, "point %v", point.OnB.Value)
	}
	poseA, poseB, err := report.CertifiedPosesAt(units.Seconds(.75))
	require.NoError(t, err)
	contact, err := doc.ContactPair(t.Context(), floor, wedge, poseA, poseB, contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
}

func TestSweepPairPlanarDepartureFromEdge(t *testing.T) {
	const step = 1.0 / (1 << 20)
	// The box rises at 2.5 mm/s while spinning about its resting edge. The
	// edge's rate is 2.5 mm/s, so §10.2 departs while 2.5 − K·u stays
	// positive, before 2.5/K s; the other vertices allow longer.
	doc := decad.New()
	floor, box, floorPath, boxPath := edgeBoxScene(t, doc, r3.Vec{Z: 2.5}, 1)
	// The floor slides along its own face, which leaves every height alone
	// but rounds its far placement: at 2^-36 s it has moved 2^-34 mm, a
	// quarter ULP of 2^20, so its rounded pose stays put.
	floorPath.LinearVelocity.X = units.MillimetersPerSecond(4)
	k := edgeBoxCurvature()
	horizon := gridFloor(math.Min(2.5/k, edgeBoxClearRoot(2.5)), step)
	req := tumbleRequest()
	req.StartPolicy = decad.ContinueSeparatingTouch
	for order := range 2 {
		a, b := floor, box
		pathA, pathB := decad.PairPath(floorPath), decad.PairPath(boxPath)
		if order == 1 {
			a, b, pathA, pathB = box, floor, pathB, pathA
		}
		report, err := doc.SweepPair(t.Context(), a, b, pathA, pathB, req)
		require.NoError(t, err)
		require.Equal(t, decad.SweepDepartedClear, report.Outcome, "order %d cause=%v", order, report.Cause)
		require.NotNil(t, report.Departure)
		require.InDelta(t, horizon, report.Departure.Until.Fraction.Base(), 2*step)
		gap := report.Departure.GapAtUntil
		require.Greater(t, gap.Value.Base()-gap.Bound.Base(), 0.0)

		// Replay: the start is the exact touch; inside the departure and the
		// later clear search the rounded pair is separated.
		for _, elapsed := range []float64{0, horizon / 2, horizon, .75, 1} {
			poseA, poseB, err := report.CertifiedPosesAt(units.Seconds(elapsed))
			require.NoError(t, err, "order %d time %v", order, elapsed)
			contact, err := doc.ContactPair(t.Context(), a, b, poseA, poseB, contactRequest())
			require.NoError(t, err)
			want := decad.ContactSeparated
			if elapsed == 0 {
				want = decad.ContactTouching
			}
			require.Equal(t, want, contact.Relation, "order %d time %v", order, elapsed)
		}
		// At 2^-36 s the proven gap, about 2.5·2^-36 mm, lies below the
		// floor's 2^-34 mm pose rounding, so replay refuses.
		_, _, err = report.CertifiedPosesAt(units.Seconds(math.Ldexp(1, -36)))
		require.ErrorIs(t, err, decad.ErrUnsupported)
	}
}

func TestSweepPairPlanarDepartureRotatingSupport(t *testing.T) {
	// The grid step of an eighth-second sweep at the 2^-20 s resolution.
	const step = 1.0 / (1 << 17)
	// A 4 mm cube rests on a 16×16×4 mm slab. The slab spins at 1 rad/s about
	// Y through its center; the cube spins at 1/2 rad/s about Y through its
	// own center and rises at 3 mm/s. Against the cube's lower face the slab's
	// corners have rate 3 ± 8·(1 − 1/2), one of them negative; against the
	// slab's upper face the cube's corners have rate 3 + x/2 ≥ 2, so that is
	// the support plane, and every term of §10.2's curvature is nonzero.
	doc := decad.New()
	slab := boxBodyAtZ(t, doc, -8, -8, 8, 8, -4, 4)
	cube := boxBody(t, doc, -2, -2, 2, 2, 4)
	const seconds = .125
	slabPath := sweepDrift(r3.Vec{}, seconds)
	slabPath.Center = r3.Vec{Z: -2}
	slabPath.AngularVelocity.Y = units.RadiansPerSecond(1)
	cubePath := sweepDrift(r3.Vec{Z: 3}, seconds)
	cubePath.Center = r3.Vec{Z: 2}
	cubePath.AngularVelocity.Y = units.RadiansPerSecond(.5)

	rho := func(center r3.Vec, x, y, z0, z1 float64) float64 {
		best := 0.0
		for _, sx := range []float64{-x, x} {
			for _, sy := range []float64{-y, y} {
				for _, z := range []float64{z0, z1} {
					best = math.Max(best, r3.Vec{X: sx, Y: sy, Z: z}.Sub(center).Len())
				}
			}
		}
		return best
	}
	rhoM, rhoS := rho(cubePath.Center, 2, 2, 0, 4), rho(slabPath.Center, 8, 8, -4, 0)
	const omegaM, omegaS, speed, centers = .5, 1.0, 3.0, 4.0
	curvature := func(u float64) float64 {
		reach := rhoM + rhoS + centers + speed*u
		bound := omegaM*omegaM*rhoM + omegaS*omegaS*rhoS +
			2*omegaS*(speed+omegaM*rhoM+omegaS*rhoS) + omegaS*omegaS*reach
		return bound / 2
	}
	// The least contact rate is 3 − 2/2 = 2 mm/s, at the cube's x = −2
	// corners; the cube's upper corners start 4 mm up and only rise.
	lo, hi := 0.0, seconds
	for range 200 {
		mid := (lo + hi) / 2
		if 2-curvature(mid)*mid > 0 {
			lo = mid
		} else {
			hi = mid
		}
	}
	horizon := gridFloor(lo/seconds, step)

	req := tumbleRequest()
	req.StartPolicy = decad.ContinueSeparatingTouch
	report, err := doc.SweepPair(t.Context(), slab, cube, slabPath, cubePath, req)
	require.NoError(t, err)
	require.NotNil(t, report.Departure, "outcome=%v cause=%v", report.Outcome, report.Cause)
	require.Contains(t, []decad.SweepOutcome{decad.SweepDepartedClear, decad.SweepImpactBracket}, report.Outcome)
	require.InDelta(t, horizon, report.Departure.Until.Fraction.Base(), 2*step)
	gap := report.Departure.GapAtUntil
	require.Greater(t, gap.Value.Base()-gap.Bound.Base(), 0.0)
}

func TestSweepPairPlanarClearReplay(t *testing.T) {
	// The tumbling wedge's impact report replays its certified clear prefix,
	// where the rounded pair stays separated, then its bracket through the
	// right edge, which the sweep narrowed until it replays, and refuses past
	// that edge.
	doc := decad.New()
	floor, wedge, floorPath, wedgePath := tumbleScene(t, doc)
	report, err := doc.SweepPair(t.Context(), floor, wedge, floorPath, wedgePath, tumbleRequest())
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, report.Outcome)
	left := report.Bracket.From.Elapsed.Value.Base()
	for _, elapsed := range []float64{0, left / 3, left / 2, left} {
		poseA, poseB, err := report.CertifiedPosesAt(units.Seconds(elapsed))
		require.NoError(t, err, "time %v", elapsed)
		contact, err := doc.ContactPair(t.Context(), floor, wedge, poseA, poseB, contactRequest())
		require.NoError(t, err)
		require.Equal(t, decad.ContactSeparated, contact.Relation, "time %v", elapsed)
		require.InDelta(t, tumbleGap(elapsed), contact.Gap.Value.Base(), contact.Gap.Bound.Base()+1e-9)
	}
	right := report.Bracket.To.Elapsed.Value.Base()
	_, _, err = report.CertifiedPosesAt(units.Seconds(right))
	require.NoError(t, err)
	_, _, err = report.CertifiedPosesAt(units.Seconds(right + tumbleRequest().TimeResolution.Base()))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

// The sweep fixtures of docs/multibody-dynamics-design.md §13 PR 14a: §10.5's
// band track over the support set. The tilted cube of
// contact_support_band_test.go rests on its near edge, which lies on the Y
// axis, and turns about that axis at 1 rad/s, lowering its far edge from
// 2⁻²¹ mm at 8·cos θ mm/s. The sweep lasts 2⁻²² s at a 2⁻³² s resolution, so
// its grid step is 2⁻¹⁰ of the duration, and the far edge would reach the
// floor near a quarter of it. The floor stays at the identity, so the rounded
// poses read the ideal ones up to the cube's own rotation rounding.
//
// Legs shown to fail (each deleted in turn, fixture red, then restored):
//   - the lifted term of the band's depth (planarSupport.depthAt):
//     TestSweepPairSupportSetBand's BandAt no longer encloses the far edge's
//     height;
//   - the lifted vertices' lower height bound (clearAt over the lifted set):
//     TestSweepPairSupportSetArrivalEndsTrack's track runs to the duration,
//     past the far edge's arrival.

const (
	supportSweepSeconds = 1.0 / (1 << 22)
	supportSweepStep    = 1.0 / (1 << 10)
	// supportSweepCurvature is §10.2's K for the cube turning about its near
	// edge at 1 rad/s: |ω|²·ρ/2 with ρ = 12 mm, its farthest vertex.
	supportSweepCurvature = 6.0
)

// supportSweepScene is the floor and the tilted cube, the cube turning at
// spin rad/s about +Y through its near edge and moving at v.
func supportSweepScene(t *testing.T, doc *decad.Document, lift, spin float64, v r3.Vec) (*decad.Body, *decad.Body,
	decad.RigidDriftSegment, decad.RigidDriftSegment) {
	t.Helper()
	floor, cube := supportCubeScene(t, doc, 100)
	floorPath := sweepDrift(r3.Vec{}, supportSweepSeconds)
	cubePath := sweepDrift(v, supportSweepSeconds)
	cubePath.From = tiltedCubePose(t, r3.Vec{Z: lift})
	cubePath.Center = r3.Vec{Z: lift}
	cubePath.AngularVelocity.Y = units.RadiansPerSecond(spin)
	return floor, cube, floorPath, cubePath
}

func supportSweepRequest(policy decad.SweepStartPolicy) decad.SweepRequest {
	req := sweepRequest()
	req.SupportBand = units.Millimeters(2 * supportLift)
	req.TimeResolution = units.Seconds(1.0 / (1 << 32))
	req.MaxPoseEvaluations = 512
	req.StartPolicy = policy
	return req
}

// supportSweepArrival is the time at which the far edge's §10.2 lower bound
// 2⁻²¹ − 8·cos θ·u − K·u² reaches zero.
func supportSweepArrival() float64 {
	rate := 8 * supportCosine
	k := supportSweepCurvature
	return (-rate + math.Sqrt(rate*rate+4*k*supportLift)) / (2 * k)
}

func TestSweepPairSupportSetBand(t *testing.T) {
	doc := decad.New()
	floor, cube, floorPath, cubePath := supportSweepScene(t, doc, 0, 1, r3.Vec{})
	theta := math.Asin(supportSine)
	for order := range 2 {
		a, b := floor, cube
		pathA, pathB := decad.PairPath(floorPath), decad.PairPath(cubePath)
		if order == 1 {
			a, b, pathA, pathB = cube, floor, pathB, pathA
		}
		report, err := doc.SweepPair(t.Context(), a, b, pathA, pathB, supportSweepRequest(decad.ContinueCertifiedTouch))
		require.NoError(t, err)
		require.Equal(t, decad.SweepPersistentBand, report.Outcome, "order %d cause=%v", order, report.Cause)
		require.Equal(t, decad.ContactTouching, report.InitialEvent.Relation)
		require.Len(t, report.InitialEvent.Manifold.Points, 4)
		track := report.ContactTrack
		end := track.End().Fraction.Base()
		// The track ends on the last grid fraction before the far edge's
		// lower bound reaches zero.
		require.InDelta(t, gridFloor(supportSweepArrival()/supportSweepSeconds, supportSweepStep), end, supportSweepStep)
		require.Less(t, end*supportSweepSeconds, supportSweepArrival())
		// The band encloses every support-set vertex's height through each
		// sampled fraction: the near edge on the axis stays at zero, and the
		// far edge's exact height, 8·sin(θ − u), is largest at the start.
		for _, f := range []float64{end / 4, end / 2, end} {
			band, err := track.BandAt(units.Scalar(f))
			require.NoError(t, err)
			upper := band.Value.Base() + band.Bound.Base()
			for _, u := range []float64{0, f * supportSweepSeconds / 2, f * supportSweepSeconds} {
				height := 8 * math.Sin(theta-u)
				// The heights are evaluated in float64; 1e-15 mm covers that
				// rounding and nothing the depth's terms decide.
				require.LessOrEqual(t, height, upper+1e-15, "fraction %v time %v", f, u)
				require.GreaterOrEqual(t, height, -upper)
			}
			manifold, err := track.ManifoldAt(units.Scalar(f))
			require.NoError(t, err)
			require.Len(t, manifold.Points, 4)
			for _, point := range manifold.Points {
				require.Zero(t, point.Separation.Value.Base())
				require.GreaterOrEqual(t, point.Separation.Bound.Base(), band.Value.Base())
			}
		}
		band := track.Band()
		require.NotNil(t, band)
		require.InDelta(t, supportLift, band.Value.Base(), supportLift/1e6)
	}
}

func TestSweepPairSupportSetDeparts(t *testing.T) {
	// The cube 2⁻³⁰ mm above the floor rises at 1 mm/s: its start is a
	// ContactBand, and every vertex of its lifted set rises, so the exact
	// pair departs over the whole sweep.
	const lift = 1.0 / (1 << 30)
	doc := decad.New()
	floor, cube, floorPath, cubePath := supportSweepScene(t, doc, lift, 0, r3.Vec{Z: 1})
	report, err := doc.SweepPair(t.Context(), floor, cube, floorPath, cubePath,
		supportSweepRequest(decad.ContinueSeparatingTouch))
	require.NoError(t, err)
	require.Equal(t, decad.SweepDepartedClear, report.Outcome, "cause=%v", report.Cause)
	require.Equal(t, decad.ContactBand, report.InitialEvent.Relation)
	require.NotNil(t, report.Departure)
	gap := report.Departure.GapAtUntil
	require.Greater(t, gap.Value.Base()-gap.Bound.Base(), lift)

	// A stop policy reports the band start as an initial contact.
	stopped, err := doc.SweepPair(t.Context(), floor, cube, floorPath, cubePath,
		supportSweepRequest(decad.StopAtInitialContact))
	require.NoError(t, err)
	require.Equal(t, decad.SweepInitiallyTouching, stopped.Outcome)
	require.Equal(t, decad.ContactBand, stopped.InitialEvent.Relation)
	require.Len(t, stopped.InitialEvent.Manifold.Points, 4)
}

func TestSweepPairSupportSetArrivalEndsTrack(t *testing.T) {
	// The far edge would reach the floor near a quarter of the sweep. The
	// track over the support set ends before it does, the rounded pair at the
	// track's end still rests on its near edge with the far edge above the
	// floor, and the sweep's replay refuses every instant past the end.
	doc := decad.New()
	floor, cube, floorPath, cubePath := supportSweepScene(t, doc, 0, 1, r3.Vec{})
	report, err := doc.SweepPair(t.Context(), floor, cube, floorPath, cubePath,
		supportSweepRequest(decad.ContinueCertifiedTouch))
	require.NoError(t, err)
	require.Equal(t, decad.SweepPersistentBand, report.Outcome, "cause=%v", report.Cause)
	end := report.ContactTrack.End()
	require.Less(t, end.Fraction.Base(), .5)
	require.Less(t, end.Elapsed.Value.Base(), supportSweepArrival())
	poseA, poseB, err := report.CertifiedPosesAt(end.Elapsed.Value)
	require.NoError(t, err)
	contact, err := doc.ContactPair(t.Context(), floor, cube, poseA, poseB, supportBandRequest(2*supportLift))
	require.NoError(t, err)
	require.Contains(t, []decad.ContactRelation{decad.ContactTouching, decad.ContactBand}, contact.Relation)
	require.NotNil(t, contact.Manifold)
	require.Len(t, contact.Manifold.Points, 4)
	for _, point := range contact.Manifold.Points {
		require.GreaterOrEqual(t, point.Separation.Value.Base()+point.Separation.Bound.Base(), 0.0)
	}
	_, _, err = report.CertifiedPosesAt(units.Seconds(supportSweepSeconds / 2))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

// The sweep fixtures of docs/multibody-dynamics-design.md §13 PR 14c: §10.6's
// face-local support plane. The §2 tray's walls rise above its floor, so no
// face of the tray holds the whole tray behind it; its floor carries the
// departure and the band track through the column test, which keeps every
// tray triangle in front of the floor's plane laterally clear of the 8 mm
// cube's ideal path box.
//
// Legs shown to fail (each deleted in turn, fixture red, then restored):
//   - the column test in the departure's grid search:
//     TestSweepPairColumnEndsDepartureAtWall departs over the whole sweep,
//     past the wall, and its departure's end sample overlaps the wall, so the
//     sweep is SweepUndecided with SweepDepartureUnproved;
//   - the lateral clearance in the departure's lower gap:
//     contact_sweep_band_internal_test.go's
//     TestPlanarDepartureLowerGapIsBoundedByLateralClearance publishes the
//     rising cube's height, far above its 2⁻¹⁰ mm clearance from the wall;
//   - the column test in the band track's grid search:
//     TestSweepPairColumnRefusesTrackOverRim publishes a quarter-second band
//     track through which the box sinks into a low wall's rim.

// sceneTrayBody is the §2 tray: a floor and four walls around the inside
// [-80, 80]², the floor's top face at z = 0 and the walls rising to z = 40.
// The inner box's side diagonals cross the rim's plane at dyadic points, so
// the Boolean is exact (zero-bound), which §9 requires.
func sceneTrayBody(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	return cutOf(t, doc, [6]float64{-90, -90, 90, 90, -10, 50}, [6]float64{-80, -80, 80, 80, 0, 50})
}

// lowestCornerHeight stages the 8 mm cube's corners through pose in float64
// and returns the least z.
func lowestCornerHeight(pose r3.Transform) float64 {
	lowest := math.Inf(1)
	for _, x := range []float64{0, 8} {
		for _, y := range []float64{-4, 4} {
			for _, z := range []float64{0, 8} {
				lowest = math.Min(lowest, pose.Apply(r3.Vec{X: x, Y: y, Z: z}).Z)
			}
		}
	}
	return lowest
}

func TestSweepPairDepartsFromTrayFloor(t *testing.T) {
	// The cube rests flat on the tray's floor around the origin, rises at
	// 100 mm/s and turns at 1 rad/s about Y through its center. Its bottom
	// corners rise at 100 ∓ 4 mm/s, so §10.2 departs over the whole
	// sixteenth of a second, and its path stays far from every wall.
	const seconds = 1.0 / 16
	doc := decad.New()
	tray := sceneTrayBody(t, doc)
	cube := boxBody(t, doc, 0, -4, 8, 4, 8)
	trayPath := sweepDrift(r3.Vec{}, seconds)
	cubePath := sweepDrift(r3.Vec{Z: 100}, seconds)
	cubePath.From = contactPose(t, r3.Vec{X: -4})
	cubePath.Center = r3.Vec{Z: 4}
	cubePath.AngularVelocity.Y = units.RadiansPerSecond(1)
	req := tumbleRequest()
	req.StartPolicy = decad.ContinueSeparatingTouch
	for order := range 2 {
		a, b := tray, cube
		pathA, pathB := decad.PairPath(trayPath), decad.PairPath(cubePath)
		if order == 1 {
			a, b, pathA, pathB = cube, tray, pathB, pathA
		}
		report, err := doc.SweepPair(t.Context(), a, b, pathA, pathB, req)
		require.NoError(t, err)
		require.Equal(t, decad.SweepDepartedClear, report.Outcome, "order %d cause=%v", order, report.Cause)
		require.Equal(t, decad.ContactTouching, report.InitialEvent.Relation)
		require.NotNil(t, report.Departure)
		until := report.Departure.Until
		require.Equal(t, 1.0, until.Fraction.Base())
		poseA, poseB, err := report.CertifiedPosesAt(until.Elapsed.Value)
		require.NoError(t, err)
		cubePose := poseB
		if order == 1 {
			cubePose = poseA
		}
		// The published gap is the exact relation's at the rounded poses: the
		// lowest corner's height. Staging the corners in float64 rounds each
		// coordinate by about an ULP of 10 mm, which 1e-12 mm covers.
		gap := report.Departure.GapAtUntil
		require.Greater(t, gap.Value.Base()-gap.Bound.Base(), 0.0)
		require.InDelta(t, lowestCornerHeight(cubePose), gap.Value.Base(), gap.Bound.Base()+1e-12)

		// Inside the departure the rounded pair replays separated.
		poseA, poseB, err = report.CertifiedPosesAt(units.Seconds(seconds / 2))
		require.NoError(t, err)
		contact, err := doc.ContactPair(t.Context(), a, b, poseA, poseB, contactRequest())
		require.NoError(t, err)
		require.Equal(t, decad.ContactSeparated, contact.Relation, "order %d", order)
	}
}

func TestSweepPairBandTrackOnTrayFloor(t *testing.T) {
	// The edge box of TestSweepPairPlanarBandTrack, turning about its resting
	// edge on the tray's floor instead of a plain floor, both moved
	// tumbleOffset along X: the quarter-second track covers the sweep with
	// the same K·h² depth.
	const seconds = .25
	// The closed form runs in float64 over values below 20.
	const slack = 1e-12
	doc := decad.New()
	_, box, _, boxPath := edgeBoxScene(t, doc, r3.Vec{}, seconds)
	tray := sceneTrayBody(t, doc)
	trayPath := sweepDrift(r3.Vec{}, seconds)
	trayPath.From = contactPose(t, r3.Vec{X: tumbleOffset})
	k := edgeBoxCurvature()
	for order := range 2 {
		a, b := tray, box
		pathA, pathB := decad.PairPath(trayPath), decad.PairPath(boxPath)
		if order == 1 {
			a, b, pathA, pathB = box, tray, pathB, pathA
		}
		report, err := doc.SweepPair(t.Context(), a, b, pathA, pathB, bandRequest())
		require.NoError(t, err)
		require.Equal(t, decad.SweepPersistentBand, report.Outcome, "order %d cause=%v", order, report.Cause)
		track := report.ContactTrack
		require.Equal(t, 1.0, track.End().Fraction.Base())
		h := track.End().Elapsed.Value.Base()
		band := track.Band()
		require.NotNil(t, band)
		require.InDelta(t, k*h*h, band.Value.Base(), band.Bound.Base()+slack)
		prefix, err := track.BandAt(units.Scalar(.5))
		require.NoError(t, err)
		u := h / 2
		require.InDelta(t, k*u*u, prefix.Value.Base(), prefix.Bound.Base()+slack)
		normal := 1.0
		if order == 1 {
			normal = -1
		}
		require.Equal(t, r3.Vec{Z: normal}, track.Normal().Value)
		manifold, err := track.ManifoldAt(units.Scalar(.5))
		require.NoError(t, err)
		require.Len(t, manifold.Points, 2)
		replayA, replayB, err := report.CertifiedPosesAt(units.Seconds(h / 2))
		require.NoError(t, err)
		contact, err := doc.ContactPair(t.Context(), a, b, replayA, replayB, contactRequest())
		require.NoError(t, err)
		require.NotEqual(t, decad.ContactSeparated, contact.Relation)
	}
}

func TestSweepPairColumnEndsDepartureAtWall(t *testing.T) {
	// The cube rests flat on the tray's floor with its +X face 1 mm from the
	// wall at x = 80, rises at 16 mm/s and slides toward the wall at
	// 200 mm/s. A slow spin about Z, 2⁻¹⁶ rad/s through the cube's center,
	// leaves every corner height alone and routes the pair to the general
	// planar sweep: an axis-aligned source box that only translates against
	// a faceted body takes the faceted-floor sweep, which does not read a
	// tray. Every contact rate stays at 16 mm/s, so only the column test ends
	// the departure: at the last grid fraction before the cube's path box,
	// whose +X side the spin moves by under 1e-6 mm, reaches the wall's
	// projection at 79 + 200·t = 80.
	const seconds = 1.0 / 64
	// The grid step of a 2⁻⁶ s sweep at the 2⁻²⁰ s resolution.
	const step = 1.0 / (1 << 14)
	const spin = 1.0 / (1 << 16)
	doc := decad.New()
	tray := sceneTrayBody(t, doc)
	cube := boxBody(t, doc, 0, -4, 8, 4, 8)
	trayPath := sweepDrift(r3.Vec{}, seconds)
	cubePath := sweepDrift(r3.Vec{X: 200, Z: 16}, seconds)
	cubePath.From = contactPose(t, r3.Vec{X: 71})
	cubePath.Center = r3.Vec{X: 75, Z: 4}
	cubePath.AngularVelocity.Z = units.RadiansPerSecond(spin)
	// The exact drift's impact: the leading corner reaches x = 80 when
	// 79 + 200·t + 4·(cos ωt − 1) + 4·sin ωt = 80, increasing in t.
	lo, hi := 0.0, seconds
	for range 200 {
		mid := (lo + hi) / 2
		if 79+200*mid+4*(math.Cos(spin*mid)-1)+4*math.Sin(spin*mid) < 80 {
			lo = mid
		} else {
			hi = mid
		}
	}
	impact := lo
	req := tumbleRequest()
	req.StartPolicy = decad.ContinueSeparatingTouch
	resolution := req.TimeResolution.Base()
	for order := range 2 {
		a, b := tray, cube
		pathA, pathB := decad.PairPath(trayPath), decad.PairPath(cubePath)
		if order == 1 {
			a, b, pathA, pathB = cube, tray, pathB, pathA
		}
		report, err := doc.SweepPair(t.Context(), a, b, pathA, pathB, req)
		require.NoError(t, err)
		require.Equal(t, decad.SweepImpactBracket, report.Outcome, "order %d cause=%v", order, report.Cause)
		require.NotNil(t, report.Departure)
		until := report.Departure.Until.Fraction.Base()
		// The spin's enclosure moves the box's +X side by far less than the
		// 0.88 grid step, about 1.7e-4 mm, that separates the last fraction
		// before 79 + 200·t = 80 from the root.
		require.Equal(t, gridFloor(1.0/200/seconds, step), until)
		require.Less(t, 79+200*until*seconds, 80.0)
		gap := report.Departure.GapAtUntil
		require.Greater(t, gap.Value.Base()-gap.Bound.Base(), 0.0)
		from := report.Bracket.From.Elapsed.Value.Base()
		to := report.Bracket.To.Elapsed.Value.Base()
		require.LessOrEqual(t, to-from, resolution)
		require.InDelta(t, impact, from, resolution)
		require.InDelta(t, impact, to, resolution)
		// The departure replays separated, its end included.
		poseA, poseB, err := report.CertifiedPosesAt(report.Departure.Until.Elapsed.Value)
		require.NoError(t, err)
		contact, err := doc.ContactPair(t.Context(), a, b, poseA, poseB, contactRequest())
		require.NoError(t, err)
		require.Equal(t, decad.ContactSeparated, contact.Relation, "order %d", order)
	}
}

func TestSweepPairColumnRefusesTrackOverRim(t *testing.T) {
	// A low tray, its walls 2 mm high, holds the edge box of
	// TestSweepPairPlanarBandTrack with its resting edge 3 mm inside the wall
	// at x = 80. The box's lower +X face leans over the wall's top, 1 mm above
	// it where it crosses x = 80, and the spin about its edge lowers that
	// face onto the rim within the quarter second. The contact edge's feet
	// stay well inside the floor's face, so only the column test refuses the
	// track: the box's path box lies over the rim's projection from the
	// start. Deleting it publishes a quarter-second band track through which
	// the box sinks into the rim.
	const seconds = .25
	doc := decad.New()
	_, box, _, boxPath := edgeBoxScene(t, doc, r3.Vec{}, seconds)
	boxPath.From = edgeBoxPose(t, r3.Vec{X: tumbleOffset + 77})
	boxPath.Center.X += 77
	tray := cutOf(t, doc, [6]float64{-90, -90, 90, 90, -10, 12}, [6]float64{-80, -80, 80, 80, 0, 4})
	trayPath := sweepDrift(r3.Vec{}, seconds)
	trayPath.From = contactPose(t, r3.Vec{X: tumbleOffset})
	start, err := doc.ContactPair(t.Context(), tray, box, trayPath.From, boxPath.From, contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, start.Relation)
	for order := range 2 {
		a, b := tray, box
		pathA, pathB := decad.PairPath(trayPath), decad.PairPath(boxPath)
		if order == 1 {
			a, b, pathA, pathB = box, tray, pathB, pathA
		}
		report, err := doc.SweepPair(t.Context(), a, b, pathA, pathB, bandRequest())
		require.NoError(t, err)
		require.Equal(t, decad.SweepUndecided, report.Outcome, "order %d", order)
		require.Equal(t, decad.SweepContactTrackUnproved, report.Cause, "order %d", order)
	}
	// Turned by the quarter radian, the box's lower +X face crosses x = 80
	// about 1.78 mm up, inside the 2 mm wall: a track over the whole sweep
	// would hide that overlap.
	c, cos, sin := math.Sqrt2/2, math.Cos(seconds), math.Sin(seconds)
	turned := basisPose(t, r3.Vec{X: c * (cos + sin), Z: c * (cos - sin)}, r3.Vec{Y: 1},
		r3.Vec{X: c * (sin - cos), Z: c * (sin + cos)}, r3.Vec{X: tumbleOffset + 77})
	end, err := doc.ContactPair(t.Context(), tray, box, trayPath.From, turned, contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactOverlapping, end.Relation)
}
