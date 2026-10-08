package decad

import (
	"math/big"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/pair/planar"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sweepmemo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestPlanarReplayLowerGapChargesTravel reads the replay's lower gap inside a
// certified clear span directly. A sound sweep leaves no span whose margin
// falls below a pose rounding, so no public fixture can show the travel charge
// failing; this one does: deleting the charge reads the span's end gaps
// unreduced, 3 mm instead of 2 mm.
func TestPlanarReplayLowerGapChargesTravel(t *testing.T) {
	replay := planarReplay{travel: big.NewRat(4, 1), spans: []planarClearSpan{
		{from: new(big.Rat), to: big.NewRat(1, 2), left: big.NewRat(3, 1), right: big.NewRat(1, 1)},
		{from: big.NewRat(1, 2), to: big.NewRat(1, 1), axis: big.NewRat(1, 8)},
	}}
	// At 1/4: the left end's 3 mm less 4·(1/4), against the right end's
	// 1 mm less 4·(1/4).
	require.Zero(t, big.NewRat(2, 1).Cmp(replay.lowerGap(big.NewRat(1, 4))))
	// A hull-separated span reads its hull gap anywhere inside it.
	require.Zero(t, big.NewRat(1, 8).Cmp(replay.lowerGap(big.NewRat(3, 4))))
	require.Nil(t, (&planarReplay{}).lowerGap(big.NewRat(1, 2)))
}

// TestRotatingBracketDepthChargesTravelAndDeviation reads the replay inside a
// rotating impact bracket directly (bracketDepthWithin): a fraction f past the
// left edge lo replays when (f − lo)·T − g + deviation, the farthest the
// rounded pair can lie inside a separated one, fits PointResolution. A public
// fixture's bracket travel and pose rounding sit far below any useful
// resolution, so this one shows both charges failing: deleting the travel
// charge accepts f = 3/4 at resolution 3/4, and deleting the deviation
// accepts f = 1/2 at resolution −1/4. The left gap g is a credit, not a
// charge: deleting it only refuses more.
func TestRotatingBracketDepthChargesTravelAndDeviation(t *testing.T) {
	lo, hi, gap, travel := big.NewRat(1, 2), big.NewRat(1, 1), big.NewRat(1, 4), big.NewRat(4, 1)
	deviation := big.NewRat(1, 8)
	// (3/4 − 1/2)·4 − 1/4 + 1/8 = 7/8.
	require.True(t, sweepmemo.BracketDepthWithin(big.NewRat(3, 4), deviation, big.NewRat(7, 8), lo, hi, gap, travel))
	require.False(t, sweepmemo.BracketDepthWithin(big.NewRat(3, 4), deviation, big.NewRat(3, 4), lo, hi, gap, travel))
	// At the left edge the gap alone remains: −1/4 + 1/8 = −1/8.
	require.True(t, sweepmemo.BracketDepthWithin(big.NewRat(1, 2), deviation, new(big.Rat), lo, hi, gap, travel))
	require.False(t, sweepmemo.BracketDepthWithin(big.NewRat(1, 2), deviation, big.NewRat(-1, 4), lo, hi, gap, travel))
	// Past the right edge nothing replays.
	require.False(t, sweepmemo.BracketDepthWithin(big.NewRat(9, 8), deviation, big.NewRat(100, 1), lo, hi, gap, travel))
	require.False(t, sweepmemo.BracketDepthWithin(big.NewRat(3, 4), deviation, big.NewRat(100, 1), lo, hi, nil, nil))
}

// TestPlanarDepartureLowerGapIsBoundedByLateralClearance reads §10.6's lower
// gap off a proven departure: the 8 mm cube rests on a tray's floor 2⁻¹⁰ mm
// from its wall at x = 80 and rises at 100 mm/s. Its corners soon stand far
// above the floor, but the wall stays 2⁻¹⁰ mm away, so the departure's lower
// gap is the lateral clearance, exactly 2⁻¹⁰ mm: the wall's projection is a
// segment on x = 80 and the cube's path box ends at x = 80 − 2⁻¹⁰. Deleting
// the clearance publishes the height bound, about 6 mm.
func TestPlanarDepartureLowerGapIsBoundedByLateralClearance(t *testing.T) {
	doc := New()
	outer := internalBoxBody(t, doc, -90, -90, 90, 90, 50)
	lower, err := r3.Translation(r3.Vec{Z: -10})
	require.NoError(t, err)
	outer, err = outer.Placed(t.Context(), lower)
	require.NoError(t, err)
	inner := internalBoxBody(t, doc, -80, -80, 80, 80, 50)
	tray, err := Cut(t.Context(), outer, inner)
	require.NoError(t, err)
	cube := internalBoxBody(t, doc, 0, -4, 8, 4, 8)

	clearance := big.NewRat(1, 1<<10)
	at, err := r3.Translation(r3.Vec{X: 72 - 1.0/(1<<10)})
	require.NoError(t, err)
	zero := units.MillimetersPerSecond(0)
	still := units.RadiansPerSecond(0)
	cubePath := RigidDriftSegment{From: at,
		LinearVelocity:  QuantityVec{X: zero, Y: zero, Z: units.MillimetersPerSecond(100)},
		AngularVelocity: QuantityVec{X: still, Y: still, Z: still}, Duration: units.Seconds(1.0 / 16)}
	trayPath := RigidDriftSegment{From: r3.Identity(), LinearVelocity: QuantityVec{X: zero, Y: zero, Z: zero},
		AngularVelocity: QuantityVec{X: still, Y: still, Z: still}, Duration: units.Seconds(1.0 / 16)}
	for _, run := range []*rotationalPairSweep{
		bandRun(t, doc, tray, cube, trayPath, cubePath),
		bandRun(t, doc, cube, tray, cubePath, trayPath),
	} {
		until, ok, err := run.planarDepartureFraction(t.Context())
		require.NoError(t, err)
		require.True(t, ok)
		require.Zero(t, until.Cmp(big.NewRat(1, 1)))
		require.True(t, run.departure.support.local)
		// At the start of the sweep the height bound is the smaller term; at
		// its end the clearance is.
		early := big.NewRat(1, 1<<20)
		require.Positive(t, run.departure.lowerGap(early).Cmp(new(big.Rat)))
		require.Negative(t, run.departure.lowerGap(early).Cmp(clearance))
		require.Zero(t, run.departure.lowerGap(until).Cmp(clearance))
	}
}

// planarSupportsScan is planarSupports as it read before the plane key and the
// float pre-test: each plane compared against every plane tried so far, and
// every new plane checked exactly. TestPlanarSupportsMatchScan holds the
// shipped form to it.
func (r *rotationalPairSweep) planarSupportsScan(poll func() error) ([]planarSupport, error) {
	paths := [2]*rotationalSweepPath{&r.a, &r.b}
	var motions [2]planarMotion
	var spins [2][]*big.Rat
	for i, path := range paths {
		motion, ok := planarMotionOf(path)
		if !ok {
			return nil, nil
		}
		spin, ok := vertexSpins(path, motion)
		if !ok {
			return nil, nil
		}
		motions[i], spins[i] = motion, spin
	}
	rest := new(big.Rat)
	if r.req.RestSpeed != (units.Value{}) {
		speed, ok := exactBaseValue(r.req.RestSpeed)
		if !ok {
			return nil, nil
		}
		rest = speed
	}
	var out []planarSupport
	for _, s := range []int{1, 0} {
		m := 1 - s
		S, M := paths[s], paths[m]
		spinM := spins[m]
		var tried []planarSupport
		for t, tri := range S.solid.Tris {
			if err := poll(); err != nil {
				return nil, err
			}
			a := S.startPoints[tri[0]]
			n := proofarith.DvCross(proofarith.DvSub(S.startPoints[tri[1]], a),
				proofarith.DvSub(S.startPoints[tri[2]], a))
			if proofarith.DvIsZero(n) || planarPlaneTried(tried, n, a) {
				continue
			}
			tried = append(tried, planarSupport{normal: n, origin: a})
			support, ok, err := planarSupportOf(S, M, n, a, r.req.ContactRequest, poll)
			if err != nil {
				return nil, err
			}
			if !ok {
				continue
			}
			support.m, support.s, support.tri = m, s, t
			support.motionM, support.motionS = motions[m], motions[s]
			support.pathM, support.pathS = M, S
			support.duration = r.a.path.duration
			support.rates = make([]*big.Rat, len(M.startPoints))
			relative := ratSub3(support.motionM.Velocity, support.motionS.Velocity)
			normal := ratOfDyV3(n)
			for i, v := range M.startPoints {
				p := ratOfDyV3(v)
				rate := ratAdd3(relative, ratCross3(support.motionM.Omega, ratSub3(p, support.motionM.Center)))
				rate = ratSub3(rate, ratCross3(support.motionS.Omega, ratSub3(p, support.motionS.Center)))
				support.rates[i] = ratDot3(normal, rate)
			}
			support.spin = spinM
			support.rested = restedVertices(&support, rest)
			out = append(out, support)
		}
	}
	return out, nil
}

// supportFixture draws one body for TestPlanarSupportsMatchScan: lattice
// points with z from zLow to zLow + 2 lattice steps, so many triangles share a plane, in both windings, with normals of
// different lengths, and on parallel planes at other offsets. A random scale
// and offset, applied exactly, give the coordinates long mantissas, so their
// float bounds round, without moving any point off its plane. The shared
// points put vertices of the other body exactly on these planes.
func supportFixture(rng *rand.Rand, shared []proofarith.DyV3, scale proofarith.Dyadic,
	offset proofarith.DyV3, zLow int) rotationalSweepPath {
	lattice := func() proofarith.DyV3 {
		var v proofarith.DyV3
		for k := range 3 {
			c := rng.IntN(5) - 2
			if k == 2 {
				c = zLow + rng.IntN(3)
			}
			v[k] = proofarith.DyAdd(proofarith.DyMul(proofarith.DyInt(int64(c)), scale), offset[k])
		}
		return v
	}
	var points []proofarith.DyV3
	for range 4 + rng.IntN(8) {
		switch rng.IntN(4) {
		case 0:
			if len(shared) > 0 {
				points = append(points, shared[rng.IntN(len(shared))])
				continue
			}
			points = append(points, lattice())
		case 1:
			// A point near the lattice, so float heights straddle zero.
			v := lattice()
			v[rng.IntN(3)] = proofarith.DyAdd(v[rng.IntN(3)], proofarith.DyShift(proofarith.DyInt(1), -40-rng.IntN(20)))
			points = append(points, v)
		default:
			points = append(points, lattice())
		}
	}
	// Half the triangles lie in one z level, so parallel planes at other
	// offsets come before or after a support plane.
	level := func(i int) []int {
		var out []int
		for j, v := range points {
			if proofarith.DyCmp(v[2], points[i][2]) == 0 {
				out = append(out, j)
			}
		}
		return out
	}
	solid := &planar.PlanarSolid{Verts: points}
	for range 6 + rng.IntN(30) {
		i := rng.IntN(len(points))
		pool := level(i)
		if rng.IntN(2) == 0 {
			pool = make([]int, len(points))
			for j := range pool {
				pool[j] = j
			}
		}
		solid.Tris = append(solid.Tris, [3]int{i, pool[rng.IntN(len(pool))], pool[rng.IntN(len(pool))]})
	}
	return rotationalSweepPath{startPoints: points, solid: solid,
		path: affinePairPath{duration: big.NewRat(1, 1)}}
}

// TestPlanarSupportsMatchScan holds planarSupports to planarSupportsScan, the
// form it shortens, on random lattice bodies under no band, a narrow band and
// a wide one. It also holds planarSupportRuledOut to its one-way claim, that
// a ruled-out plane is one planarSupportOf rejects, and checks that the
// pre-test rules out planes by both of its cases and that some planes reach
// the exact test and pass.
//
// Legs shown to fail: a plane key without the offset, without the direction's
// sign, or with the raw normal in place of its primitive direction each
// changes the supports listed; a pre-test that rules out on a lower bound
// below zero, or on every lower bound above zero under a positive band,
// rejects planes the exact test accepts.
func TestPlanarSupportsMatchScan(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(71, 73))
	poll := func() error { return nil }
	var behind, above, accepted, supports int
	for range 1000 {
		scale := proofarith.MustDyOf(1 + rng.Float64())
		if rng.IntN(3) == 0 {
			scale = proofarith.DyInt(1)
		}
		var offset proofarith.DyV3
		for k := range 3 {
			offset[k] = proofarith.MustDyOf(rng.Float64()*8 - 4)
		}
		// Stacked bodies, a below z = 0 and b above it, share support planes;
		// overlapping ones mostly do not.
		zA, zB := -2, -2
		if rng.IntN(2) == 0 {
			zB = 0
		}
		a := supportFixture(rng, nil, scale, offset, zA)
		b := supportFixture(rng, a.startPoints, scale, offset, zB)
		band := []units.Value{{}, units.Millimeters(1e-12), units.Millimeters(0.3), units.Millimeters(2.5)}[rng.IntN(4)]
		run := &rotationalPairSweep{a: a, b: b}
		run.req.SupportBand = band
		got, err := run.planarSupports(poll)
		require.NoError(t, err)
		want, err := run.planarSupportsScan(poll)
		require.NoError(t, err)
		require.Equal(t, want, got)
		supports += len(got)
		for _, sides := range [][2]*rotationalSweepPath{{&run.b, &run.a}, {&run.a, &run.b}} {
			S, M := sides[0], sides[1]
			boxes := make([]proofarith.FloatBox3, len(M.startPoints))
			for i, v := range M.startPoints {
				boxes[i] = proofarith.DvFloatBox(v)
			}
			for _, tri := range S.solid.Tris {
				origin := S.startPoints[tri[0]]
				n := proofarith.DvCross(proofarith.DvSub(S.startPoints[tri[1]], origin),
					proofarith.DvSub(S.startPoints[tri[2]], origin))
				if proofarith.DvIsZero(n) {
					continue
				}
				_, ok, err := planarSupportOf(S, M, n, origin, run.req.ContactRequest, poll)
				require.NoError(t, err)
				if !planarSupportRuledOut(boxes, n, origin, run.req.ContactRequest) {
					if ok {
						accepted++
					}
					continue
				}
				require.False(t, ok, "a ruled-out plane passes the exact test")
				cLo, cHi := proofarith.FloatBounds(proofarith.DvDot(n, origin))
				if slices.ContainsFunc(boxes, func(box proofarith.FloatBox3) bool {
					_, hi := proofarith.DotSubEnclosure(proofarith.DvFloatBox(n), box, cLo, cHi)
					return hi < 0
				}) {
					behind++
					continue
				}
				above++
			}
		}
	}
	require.Positive(t, supports, "premise: some planes are supports")
	require.Positive(t, accepted, "premise: some planes pass the exact test")
	require.Positive(t, behind, "premise: the pre-test rules out a vertex behind a plane")
	require.Positive(t, above, "premise: the pre-test rules out a plane every vertex stands clear of")
}
