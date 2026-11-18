package filletband_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

// stepDelta is the stepped loop's radial step, 2⁻³⁰ mm: large enough for
// float64 sampling to see, small enough for the restated corners to stay
// within the near-tangent class.
var stepDelta = math.Ldexp(1, -30)

// steppedLoop is a CCW loop whose quarter arc about the origin, radius 5,
// runs from (−5, 0) to (0, −5 − δ): its End sits |δ| off the circle its Start
// states, outside it for δ > 0 and inside it, a step into the material, for
// δ < 0, so the record holds a radial step at the corner where the bottom line
// leaves (agent-docs/modify-design.md Part L §5.6–§5.7). Both of the arc's
// joins are exactly tangent about the recorded centre. The rest of the loop
// closes with three right-angle miters.
func steppedLoop(delta float64) sectionrecord.LoopRecord {
	y := -5 - delta
	return loopOf(
		line(-5, 5, -5, 0),
		ccwArc(0, 0, -5, 0, 0, y),
		line(0, y, 10, y),
		line(10, y, 10, 5),
		line(10, 5, -5, 5),
	)
}

// steppedDistance is the distance from p to the stepped loop's boundary
// near its stepped corner: the quarter arc, the radial step from (0, −5) to
// (0, −5 − δ), and the bottom line.
func steppedDistance(px, py, delta float64) float64 {
	var arc float64
	if px <= 0 && py <= 0 {
		arc = math.Abs(5 - math.Hypot(px, py))
	} else {
		arc = math.Min(math.Hypot(px+5, py), math.Hypot(px, py+5))
	}
	stepY := math.Max(math.Min(-5, -5-delta), math.Min(math.Max(-5, -5-delta), py))
	step := math.Hypot(px, py-stepY)
	bottom := math.Abs(py - (-5 - delta))
	if px < 0 {
		bottom = math.Hypot(px, py-(-5-delta))
	}
	return math.Min(arc, math.Min(step, bottom))
}

// steppedLevelY is the height at which the vertical line x = px reaches the
// level d = t above the bottom, by bisection (the distance rises with height
// there).
func steppedLevelY(px, t, delta float64) float64 {
	lo, hi := -5-delta, -5+2*t
	for range 200 {
		mid := (lo + hi) / 2
		if steppedDistance(px, mid, delta) < t {
			lo = mid
		} else {
			hi = mid
		}
	}
	return (lo + hi) / 2
}

// steppedStatedY is the stated patches' level curve at offset t on the
// vertical line x = px: the moved arc (radius 5 − t) left of the corner and
// the moved bottom line right of it.
func steppedStatedY(px, t, delta float64) float64 {
	if px <= 0 {
		return -math.Sqrt((5-t)*(5-t) - px*px)
	}
	return -5 - delta + t
}

// steppedCorner reads the stepped loop of step delta at radius 1 and returns
// its terms and the index of the stepped corner.
func steppedCorner(t *testing.T, delta float64) (filletband.Loop, int) {
	t.Helper()
	read, walks, bad, why := readLoop(t, steppedLoop(delta))
	require.Equal(t, -1, bad, why)
	for k, w := range walks {
		if w.StartU == 0 && w.StartV == -5-delta {
			return read, k
		}
	}
	t.Fatal("no stepped corner")
	return filletband.Loop{}, -1
}

// TestJunctionGapAndRestatement reads the stepped loop: the stepped corner's
// gap encloses the 256-bit step ||v| − 5| and stays within a part in 10¹² of
// it, the arc's other corner has none, the loop restates its one arc with a
// positive shift, and every corner is classed about the restated centre (the
// stepped corner is LF5 or LF5′ there, never refused).
func TestJunctionGapAndRestatement(t *testing.T) {
	t.Parallel()
	for _, delta := range []float64{stepDelta, -stepDelta} {
		read, k := steppedCorner(t, delta)
		ref := new(big.Float).SetPrec(256).SetFloat64(5 + delta)
		ref.Sub(ref, big.NewFloat(5))
		ref.Abs(ref)
		want, _ := ref.Float64()
		require.GreaterOrEqual(t, read.Gap[k], want)
		require.InEpsilon(t, want, read.Gap[k], 1e-12)
		require.Zero(t, read.Gap[(k+len(read.Gap)-1)%len(read.Gap)])
		require.Equal(t, 1, read.Restate.Arcs)
		require.Positive(t, read.Restate.Shift)
		require.Contains(t, []filletband.Corner{filletband.Tangent, filletband.NearTangent}, read.Corners[k])
	}
}

// TestLocalReachEnclosesSteppedLevels samples the true level curve near the
// stepped corner, {d = t} for the record's arc, step and line, at three
// offsets and both step senses, against the stated patches' level curve (the
// moved arc and moved line). Every true sample lies within the corner's local
// reach σ of the stated curve and every stated sample within σ of the true
// one (each measured along its vertical line, an upper bound on the
// distance), and the two curves' lengths over the sampled window differ by at
// most the corner's level-length bound. The samples sit near δ, the bounds
// near 10⁻⁴: stepRoot covers the lateral width of the notch a step into the
// material cuts, which a vertical reading does not see.
func TestLocalReachEnclosesSteppedLevels(t *testing.T) {
	t.Parallel()
	for _, delta := range []float64{stepDelta, -stepDelta} {
		steppedLevels(t, delta)
	}
}

func steppedLevels(t *testing.T, stepDelta float64) {
	t.Helper()
	read, k := steppedCorner(t, stepDelta)
	r1, err := filletband.RadiusInterval(1, 0)
	require.NoError(t, err)
	sigma := read.LocalReach(r1)[k]
	length := read.LevelLength(r1)[k]
	require.Positive(t, sigma)
	for _, level := range []float64{0.25, 0.5, 1} {
		w := 8 * math.Sqrt(level*math.Abs(stepDelta))
		const n = 4000
		worst := 0.0
		trueLen, statedLen := 0.0, 0.0
		var prevTX, prevTY, prevSX, prevSY float64
		for i := 0; i <= n; i++ {
			x := -w + 2*w*float64(i)/n
			ty := steppedLevelY(x, level, stepDelta)
			sy := steppedStatedY(x, level, stepDelta)
			worst = math.Max(worst, math.Abs(ty-sy))
			if i > 0 {
				trueLen += math.Hypot(x-prevTX, ty-prevTY)
				if (prevSX <= 0) == (x <= 0) {
					statedLen += math.Hypot(x-prevSX, sy-prevSY)
				} else {
					statedLen += math.Abs(x - prevSX)
				}
			}
			prevTX, prevTY, prevSX, prevSY = x, ty, x, sy
		}
		require.Positive(t, worst, "the step shows in the level %g", level)
		require.LessOrEqual(t, worst, sigma, "level %g", level)
		require.LessOrEqual(t, math.Abs(trueLen-statedLen), length, "level %g", level)
	}
}

// TestRestatedPieceWithinTube compares Table CF's annular piece of the
// stepped loop's arc read about its recorded centre (radius 5 to End's angle)
// with the piece read about its restated centre C′ (both ends on one circle),
// at the fillet's full offset: their areas differ by at most §5.6's tube
// 2Δ·(4π(R + T) + 2T) + πΔ², with Δ capband.OffRadiusShift's bound.
func TestRestatedPieceWithinTube(t *testing.T) {
	t.Parallel()
	const T = 1.0
	arc := sectionrecord.ArcSeg{Center: sectionrecord.Point2{}, Start: sectionrecord.Point2{U: -5}, End: sectionrecord.Point2{V: -5 - stepDelta}, TStart: 0, TEnd: 1}
	shift, ok := capband.OffRadiusShift(arc, T)
	require.True(t, ok)
	centre, ok := capband.RestatedCentre(arc)
	require.True(t, ok)
	cu, _ := centre[0].Float64()
	cv, _ := centre[1].Float64()
	piece := func(cx, cy float64) float64 {
		r := math.Hypot(-5-cx, -cy)
		a0 := math.Atan2(-cy, -5-cx)
		a1 := math.Atan2(-5-stepDelta-cy, -cx)
		beta := math.Mod(a1-a0+4*math.Pi, 2*math.Pi)
		return beta * (r*T - T*T/2)
	}
	diff := math.Abs(piece(0, 0) - piece(cu, cv))
	tube := 2*shift*(4*math.Pi*(5+stepDelta+T)+2*T) + math.Pi*shift*shift
	require.Positive(t, diff)
	require.LessOrEqual(t, diff, tube)
}

// TestRestateMassAllowance pins §5.6's closed form on hand terms and on the
// stepped loop: MassOf widens three times the volume by three times
// RestateMassAllowance's volume, on top of the near-tangent terms.
func TestRestateMassAllowance(t *testing.T) {
	t.Parallel()
	r1, err := filletband.RadiusInterval(1, 0)
	require.NoError(t, err)
	terms := filletband.RestateTerms{ReflexBeside: 1}
	terms.Shift, terms.Perimeter, terms.RadiusHi, terms.Arcs = 0.5, 10, 2, 1
	vol, moment, ok := filletband.RestateMassAllowance(terms, 3, r1)
	require.True(t, ok)
	// 4·0.5·10 + (2·0.5·(4π·3 + 2) + π/4) + 1·0.5 at π's rational upper ends.
	want := 20 + (1*(12.5664*3+2) + 3.1416*0.25) + 0.5
	got, _ := vol.Float64()
	require.InEpsilon(t, want, got, 1e-12)
	gotM, _ := moment.Float64()
	require.InEpsilon(t, want*4.5, gotM, 1e-12)
	zero, _, ok := filletband.RestateMassAllowance(filletband.RestateTerms{}, 3, r1)
	require.True(t, ok)
	require.Zero(t, zero.Sign())

	read, walks, bad, why := readLoop(t, steppedLoop(stepDelta))
	require.Equal(t, -1, bad, why)
	pieces, err := read.Pieces()
	require.NoError(t, err)
	in := filletband.BandMassInput{Pieces: pieces, Walks: walks, Corners: read.Corners, Rho: read.Rho, Restate: read.Restate,
		Radius: r1, MaterialSign: 1, Sigma: -1, Embed: brepgeom.Embed{Axis: [3]int{0, 1, 2}, Sign: [3]float64{1, 1, 1}}}
	charged, err := filletband.MassOf(in)
	require.NoError(t, err)
	plain := in
	plain.Restate = filletband.RestateTerms{}
	without, err := filletband.MassOf(plain)
	require.NoError(t, err)
	v, _, ok := filletband.RestateMassAllowance(read.Restate, filletband.WalkCoordUpper(walks, 0), r1)
	require.True(t, ok)
	grow := new(big.Rat).Sub(without.Volume3.Lo, charged.Volume3.Lo)
	require.Zero(t, grow.Cmp(new(big.Rat).Mul(big.NewRat(3, 1), v)))
}

// TestNormalTurnAndMeridianExcess reads the turn §5.7 charges: 3/2 beside the
// stepped corner, NearTangentNormalTurn at the slot's LF5′ corners (no step),
// and zero at an exact corner; and the seam excess, which beside the step
// holds twice the local reach and at the slot's LF5′ corners encloses the
// seam's own excess, integrated from Lemma 1's corner locus in 256-bit
// arithmetic.
func TestNormalTurnAndMeridianExcess(t *testing.T) {
	t.Parallel()
	r1, err := filletband.RadiusInterval(1, 0)
	require.NoError(t, err)
	read, k := steppedCorner(t, stepDelta)
	_, walks, bad, why := readLoop(t, steppedLoop(stepDelta))
	require.Equal(t, -1, bad, why)
	turn, ok := read.NormalTurn(walks, k, r1)
	require.True(t, ok)
	require.Zero(t, turn.Cmp(big.NewRat(3, 2)))
	excess, ok := read.MeridianExcess(walks, r1)
	require.True(t, ok)
	require.GreaterOrEqual(t, excess[k], 2*read.LocalReach(r1)[k])

	shift := 5 * math.Ldexp(1, -34)
	slot, slotWalks, slotBad, slotWhy := readLoop(t, slotLoop(shift))
	require.Equal(t, -1, slotBad, slotWhy)
	turn, ok = slot.NormalTurn(slotWalks, 1, r1)
	require.True(t, ok)
	near, ok := filletband.NearTangentNormalTurn(slotWalks, 1, slot.Rho[1], r1)
	require.True(t, ok)
	require.Zero(t, turn.Cmp(near))
	exact, ok := slot.NormalTurn(slotWalks, 0, r1)
	require.True(t, ok)
	require.Zero(t, exact.Sign())

	slotExcess, ok := slot.MeridianExcess(slotWalks, r1)
	require.True(t, ok)
	seam := lemmaOneSeamExcess(shift, 1)
	require.Positive(t, seam)
	require.LessOrEqual(t, seam, slotExcess[1])
}

// lemmaOneSeamExcess integrates, in 256-bit arithmetic, how much longer the
// convex seam of slotLoop's bottom line and right arc is than the quarter
// circle (π/2)·r: Lemma 1 of agent-docs/modify-design.md Part D §9.1.1 puts the
// corner at x(t) along the moved line with |x′| = R(1 − cos θ)/√Δ(t), so the
// seam's speed is r·√(1 + x′²·sin²φ) against the meridian's r.
func lemmaOneSeamExcess(shift, r float64) float64 {
	const prec = 256
	f := func(x float64) *big.Float { return new(big.Float).SetPrec(prec).SetFloat64(x) }
	sq := func(x *big.Float) *big.Float { return new(big.Float).SetPrec(prec).Sqrt(x) }
	mul := func(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(prec).Mul(a, b) }
	sub := func(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(prec).Sub(a, b) }
	add := func(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(prec).Add(a, b) }
	quo := func(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(prec).Quo(a, b) }
	s := f(shift)
	big5 := f(5)
	radius := sq(add(mul(big5, big5), mul(s, s)))
	cos := quo(big5, radius)
	oneMinus := sub(f(1), cos)
	total := f(0)
	const n = 2000
	for i := range n {
		phi := (float64(i) + 0.5) * math.Pi / 2 / n
		tt := r * (1 - math.Cos(phi))
		moved := sub(radius, f(tt))
		disc := mul(mul(radius, oneMinus), sub(mul(f(2), moved), mul(radius, oneMinus)))
		xp := quo(mul(radius, oneMinus), sq(disc))
		sin := f(math.Sin(phi))
		a := mul(mul(xp, xp), mul(sin, sin))
		// r·(√(1 + a) − 1) = r·a/(√(1 + a) + 1), summed by the midpoint rule.
		term := quo(mul(f(r), a), add(sq(add(f(1), a)), f(1)))
		total = add(total, mul(term, f(math.Pi/2/n)))
	}
	out, _ := total.Float64()
	return out
}

// TestReadLoopOffRadiusRefusals pins SF1's restatement refusals: a sharp
// convex corner beside an off-radius arc (a D outline whose arc ends 2⁻⁴⁰ mm
// off its circle), a corner exactly tangent about the recorded centre whose
// restated centre turns it past the near-tangent class (an R5 corner whose
// End sits 2⁻²⁶ mm off), and an LF5′ wall whose other end is a reflex corner.
// Each row refuses by its own rule's reason.
func TestReadLoopOffRadiusRefusals(t *testing.T) {
	t.Parallel()
	e40 := math.Ldexp(1, -40)
	right := 20 + math.Ldexp(1, -26)
	shift := 5 * math.Ldexp(1, -34)
	for _, tc := range []struct {
		name string
		loop sectionrecord.LoopRecord
		want string
	}{
		{"curved miter beside an off-radius arc", loopOf(
			line(-5, e40, 5, 0),
			ccwArc(0, 0, 5, 0, -5, e40),
		), "a curved miter beside an arc whose recorded end is off the circle"},
		{"restated turn past the class", loopOf(
			line(-15, -5, 15, -5),
			ccwArc(15, 0, 15, -5, right, 0),
			line(right, 0, right, 10),
			line(right, 10, -15, 10),
			line(-15, 10, -15, -5),
		), "exceeds the admitted sin²θ ≤ 2⁻⁶⁴"},
		{"near-tangent wall beside a reflex corner", loopOf(
			line(-15, -5, 15, -5),
			ccwArc(15+shift, 0, 15, -5, 15, 5),
			line(15, 5, -20, 5),
			line(-20, 5, -20, -10),
			line(-20, -10, -15, -10),
			line(-15, -10, -15, -5),
		), "whose other end is a curved miter or a reflex corner"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, _, bad, why := readLoop(t, tc.loop)
			require.GreaterOrEqual(t, bad, 0)
			require.Contains(t, why, tc.want)
		})
	}
	// The restated turn row is exact about the recorded centre: the class
	// that refuses it reads the restated one.
	require.True(t, capband.JoinIsG1(line(-15, -5, 15, -5), ccwArc(15, 0, 15, -5, right, 0)))
}

// stepNotchWidth samples, at offset t, how far either side of the stepped
// corner the true level curve departs from the stated one by more than a
// thousandth of the step, for a quarter arc of radius R about the origin
// ending at (0, −R) and a line leaving (0, −R − δ) to the right. outside puts
// the material outside the arc's circle and below the line, as at a hole
// rim's arc, so the stated arc is the moved circle of radius R + t; otherwise
// the material sits inside and above, at radius R − t.
func stepNotchWidth(R, delta, t float64, outside bool) float64 {
	dist := func(px, py float64) float64 {
		var arc float64
		if px <= 0 && py <= 0 {
			arc = math.Abs(R - math.Hypot(px, py))
		} else {
			arc = math.Min(math.Hypot(px+R, py), math.Hypot(px, py+R))
		}
		stepY := math.Max(math.Min(-R, -R-delta), math.Min(math.Max(-R, -R-delta), py))
		step := math.Hypot(px, py-stepY)
		bottom := math.Abs(py - (-R - delta))
		if px < 0 {
			bottom = math.Hypot(px, py-(-R-delta))
		}
		return math.Min(arc, math.Min(step, bottom))
	}
	sign := 1.0
	if outside {
		sign = -1
	}
	level := func(px float64) float64 {
		base := math.Min(-R, -R-delta)
		if !outside {
			base = math.Max(math.Min(-R, -R-delta), -R-delta)
		}
		lo, hi := base, base+sign*2*t
		for range 200 {
			mid := (lo + hi) / 2
			if dist(px, mid) < t {
				lo = mid
			} else {
				hi = mid
			}
		}
		return (lo + hi) / 2
	}
	stated := func(px float64) float64 {
		if px <= 0 {
			return -math.Sqrt((R-sign*t)*(R-sign*t) - px*px)
		}
		return -R - delta + sign*t
	}
	w := 0.0
	span := 64 * math.Sqrt(t*math.Abs(delta)*(R+t)/R)
	const n = 20000
	for i := 0; i <= n; i++ {
		x := -span + 2*span*float64(i)/n
		if math.Abs(level(x)-stated(x)) > math.Abs(delta)/1000 && math.Abs(x) > w {
			w = math.Abs(x)
		}
	}
	return w
}

// TestStepRootCoversHoleArcNotch builds the step's corner as a two-walk loop
// by hand and checks that the lateral width of the notch the true level curve
// cuts, sampled at the full offset 1, stays within LocalReach less its 3δ
// term (the stepRoot share, which bounds the differing stated piece). The
// rows cover a radius-5 arc with its material inside and a radius-0.03 arc
// with its material outside, a hole rim's corner round, whose moved circle of
// radius R + t curves round the corner. Without stepCurvature's √((R + T)/R)
// the second row's bound falls below its sampled notch.
func TestStepRootCoversHoleArcNotch(t *testing.T) {
	t.Parallel()
	r1, err := filletband.RadiusInterval(1, 0)
	require.NoError(t, err)
	for _, tc := range []struct {
		name    string
		R       float64
		outside bool
	}{{"material inside", 5, false}, {"hole rim", 0.03, true}} {
		for _, delta := range []float64{stepDelta, -stepDelta} {
			R := tc.R
			loop := filletband.Loop{
				Walks: []filletband.Walk{
					{Circular: true, CCW: !tc.outside, Start: filletband.Point{U: -R}, End: filletband.Point{V: -R - delta}, Radius: R},
					{Start: filletband.Point{V: -R - delta}, End: filletband.Point{U: 10, V: -R - delta}},
				},
				Corners: []filletband.Corner{filletband.Tangent, filletband.Tangent},
				Rho:     []float64{0, 0}, RhoRec: []float64{0, 0}, Gap: []float64{0, math.Abs(delta)},
			}
			bound := loop.LocalReach(r1)[1] - 3*math.Abs(delta)
			width := stepNotchWidth(R, delta, 1, tc.outside)
			require.LessOrEqual(t, width, bound, "%s, step %g", tc.name, delta)
		}
	}
}

// TestChargedCorners pins filletband.Loop.Charged, which the partial-fillet
// route refuses on: on the stepped loop both corners of the restated arc are
// charged, as is any corner with a near-tangent reach or a step, and the
// three right-angle miters away from the arc are not.
func TestChargedCorners(t *testing.T) {
	t.Parallel()
	read, walks, bad, why := readLoop(t, steppedLoop(stepDelta))
	require.Equal(t, -1, bad, why)
	for k := range walks {
		arcAt := walks[k].IsCircular() || walks[(k+len(walks)-1)%len(walks)].IsCircular()
		require.Equal(t, arcAt, read.Charged(k), "corner %d", k)
	}
	exact, _, bad, why := readLoop(t, slotLoop(0))
	require.Equal(t, -1, bad, why)
	for k := range exact.Corners {
		require.False(t, exact.Charged(k), "corner %d", k)
	}
}

// TestNearTangentNormalTurnCoversTorusTilt samples the tilt §5.7 charges at
// the slot's convex near-tangent corner, where the bottom line meets the
// right arc: the stated cylinder face runs on to its own foot f_ℓ = (15, −5 + t)
// at the full offset t = 1, past the true corner, and there the true surface
// is the torus about the arc's centre C. The unit-vector distance between the
// cylinder's normal (0, 1) and the torus's (C − f_ℓ)/|C − f_ℓ|, read in 256-bit
// arithmetic, is about θ·R/(R − t). It stays within NearTangentNormalTurn,
// whose wall turn alone would miss it.
func TestNearTangentNormalTurnCoversTorusTilt(t *testing.T) {
	t.Parallel()
	shift := 5 * math.Ldexp(1, -34)
	read, walks, bad, why := readLoop(t, slotLoop(shift))
	require.Equal(t, -1, bad, why)
	r1, err := filletband.RadiusInterval(1, 0)
	require.NoError(t, err)
	turn, ok := filletband.NearTangentNormalTurn(walks, 1, read.Rho[1], r1)
	require.True(t, ok)
	const prec = 256
	f := func(x float64) *big.Float { return new(big.Float).SetPrec(prec).SetFloat64(x) }
	du := f(shift) // C − f_ℓ along u: (15 + shift) − 15
	dv := f(5 - 1) // C − f_ℓ along v: 0 − (−5 + 1)
	norm := new(big.Float).SetPrec(prec).Sqrt(new(big.Float).SetPrec(prec).Add(new(big.Float).SetPrec(prec).Mul(du, du), new(big.Float).SetPrec(prec).Mul(dv, dv)))
	ux := new(big.Float).SetPrec(prec).Quo(du, norm)
	uy := new(big.Float).SetPrec(prec).Sub(new(big.Float).SetPrec(prec).Quo(dv, norm), f(1))
	tilt, _ := new(big.Float).SetPrec(prec).Sqrt(new(big.Float).SetPrec(prec).Add(new(big.Float).SetPrec(prec).Mul(ux, ux), new(big.Float).SetPrec(prec).Mul(uy, uy))).Float64()
	got, _ := turn.Float64()
	require.Positive(t, tilt)
	require.LessOrEqual(t, tilt, got)
	wall, _ := new(big.Rat).Quo(new(big.Rat).SetFloat64(read.Rho[1]), r1.Lo).Float64()
	require.Greater(t, tilt, wall, "the wall turn alone misses the torus's tilt")
}
