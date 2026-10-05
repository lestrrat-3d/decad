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
	// where the rounded pair stays separated, and refuses past the bracket's
	// left edge.
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
	_, _, err = report.CertifiedPosesAt(report.Bracket.To.Elapsed.Value)
	require.ErrorIs(t, err, decad.ErrUnsupported)
}
