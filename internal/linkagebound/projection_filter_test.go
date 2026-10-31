package linkagebound

import (
	"math"
	"math/big"
	mrand "math/rand"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

// hullCandidates are the two candidates of one direction n, in the exact code's
// form. ok is false when the direction cannot be
// normed. It is the body of the unfiltered hull bound's direction loop for a
// face normal.
func hullCandidates(a, b Side, n motionbound.RatVec, pads *big.Rat) ([2]*big.Rat, bool) {
	var cands [2]*big.Rat
	sq := axisSq(n)
	normUp := sqrtUpRat(sq)
	normDown := proofarith.FloatRat(proofbound.RatSqrtDown(sq))
	if normUp == nil || normDown == nil || normDown.Sign() <= 0 {
		return cands, false
	}
	aUp, aDown := a.ExtentsAlong(n, normUp)
	bUp, bDown := b.ExtentsAlong(n, normUp)
	for i, num := range []*big.Rat{
		new(big.Rat).Neg(proofarith.AddRat(new(big.Rat), bDown, aUp)),
		new(big.Rat).Neg(proofarith.AddRat(new(big.Rat), bUp, aDown)),
	} {
		norm := normDown
		if num.Sign() > 0 {
			norm = normUp
		}
		l := num.Quo(num, norm)
		cands[i] = l.Sub(l, pads)
	}
	return cands, true
}

// lowerHullReference is LowerHullWithWitness as it was before the direction
// filter: every face normal runs the exact path, and the exact axial test
// skips only the directions whose rounded norms equal their component.
func lowerHullReference(a, b Side) HullWitness {
	one, zero := big.NewRat(1, 1), new(big.Rat)
	dirs := []motionbound.RatVec{{one, zero, zero}, {zero, one, zero}, {zero, zero, one}}
	dirs = append(dirs, FaceNormals(a.Corners)...)
	dirs = append(dirs, FaceNormals(b.Corners)...)
	pads := new(big.Rat)
	for _, p := range []*big.Rat{a.Corners.Pad, b.Corners.Pad} {
		if p != nil {
			pads.Add(pads, p)
		}
	}
	var best *big.Rat
	var winner motionbound.RatVec
	var winnerNorm *big.Rat
	winnerSense := 0
	var aAxisUp, aAxisDown, bAxisUp, bAxisDown [3]*big.Rat
	for direction, n := range dirs {
		sq := axisSq(n)
		normUp := sqrtUpRat(sq)
		normDown := proofarith.FloatRat(proofbound.RatSqrtDown(sq))
		if normUp == nil || normDown == nil || normDown.Sign() <= 0 {
			continue
		}
		if direction >= 3 {
			var component *big.Rat
			axial := true
			for _, v := range n {
				if v.Sign() == 0 {
					continue
				}
				if component != nil {
					axial = false
					break
				}
				component = v
			}
			if axial && component != nil {
				magnitude := new(big.Rat).Abs(component)
				if normUp.Cmp(magnitude) == 0 && normDown.Cmp(magnitude) == 0 {
					continue
				}
			}
		}
		var aUp, aDown, bUp, bDown *big.Rat
		if direction < 3 {
			if direction == 0 {
				aAxisUp, aAxisDown = a.Extents()
				bAxisUp, bAxisDown = b.Extents()
			}
			aUp, aDown = aAxisUp[direction], aAxisDown[direction]
			bUp, bDown = bAxisUp[direction], bAxisDown[direction]
		} else {
			aUp, aDown = a.ExtentsAlong(n, normUp)
			bUp, bDown = b.ExtentsAlong(n, normUp)
		}
		for _, candidate := range []struct {
			num   *big.Rat
			sense int
		}{
			{new(big.Rat).Neg(proofarith.AddRat(new(big.Rat), bDown, aUp)), 1},
			{new(big.Rat).Neg(proofarith.AddRat(new(big.Rat), bUp, aDown)), -1},
		} {
			num := candidate.num
			norm := normDown
			if num.Sign() > 0 {
				norm = normUp
			}
			l := num.Quo(num, norm)
			l.Sub(l, pads)
			if best == nil || l.Cmp(best) > 0 {
				best = l
				winner, winnerNorm, winnerSense = n, normUp, candidate.sense
			}
		}
	}
	return HullWitness{Bound: best, Axis: winner, Norm: winnerNorm, Sense: winnerSense}
}

type fixtureGen struct {
	rng *rand.Rand
}

// scale draws a float of either sign at a magnitude 10^[-3, 3].
func (g *fixtureGen) scale() float64 {
	v := math.Pow(10, g.rng.Float64()*6-3) * (0.5 + g.rng.Float64())
	if g.rng.IntN(2) == 0 {
		v = -v
	}
	return v
}

func (g *fixtureGen) rotation() [3][3]float64 {
	var m [3][3]float64
	m[0][0], m[1][1], m[2][2] = 1, 1, 1
	for axis := range 3 {
		t := g.rng.Float64() * 2 * math.Pi
		c, s := math.Cos(t), math.Sin(t)
		i, j := (axis+1)%3, (axis+2)%3
		var r [3][3]float64
		r[axis][axis], r[i][i], r[i][j], r[j][i], r[j][j] = 1, c, -s, s, c
		var out [3][3]float64
		for x := range 3 {
			for y := range 3 {
				for z := range 3 {
					out[x][y] += r[x][z] * m[z][y]
				}
			}
		}
		m = out
	}
	return m
}

func rotate(m [3][3]float64, p [3]float64) [3]float64 {
	var out [3]float64
	for i := range 3 {
		out[i] = m[i][0]*p[0] + m[i][1]*p[1] + m[i][2]*p[2]
	}
	return out
}

// points draws a body's corner points: a prism of K sides (2K corners) or a
// box (8 corners), axis-aligned or rotated.
func (g *fixtureGen) points(k int, aligned bool) (pts [][3]float64, prismK int) {
	origin := [3]float64{g.scale(), g.scale(), g.scale()}
	if k == 0 {
		d := [3]float64{g.scale(), g.scale(), g.scale()}
		for i := range 8 {
			var p [3]float64
			for axis := range 3 {
				p[axis] = origin[axis]
				if i&(1<<axis) != 0 {
					p[axis] += d[axis]
				}
			}
			pts = append(pts, p)
		}
	} else {
		h := g.scale()
		base := make([][3]float64, k)
		if k == 4 && aligned {
			dx, dy := g.scale(), g.scale()
			base = [][3]float64{{0, 0, 0}, {dx, 0, 0}, {dx, dy, 0}, {0, dy, 0}}
		} else {
			r := math.Abs(g.scale())
			for i := range k {
				t := 2 * math.Pi * (float64(i) + 0.8*g.rng.Float64()) / float64(k)
				base[i] = [3]float64{r * math.Cos(t), r * math.Sin(t), 0}
			}
		}
		for _, p := range base {
			pts = append(pts, [3]float64{origin[0] + p[0], origin[1] + p[1], origin[2]})
		}
		for _, p := range base {
			pts = append(pts, [3]float64{origin[0] + p[0], origin[1] + p[1], origin[2] + h})
		}
		prismK = k
	}
	if !aligned {
		m := g.rotation()
		for i, p := range pts {
			rel := rotate(m, [3]float64{p[0] - origin[0], p[1] - origin[1], p[2] - origin[2]})
			pts[i] = [3]float64{origin[0] + rel[0], origin[1] + rel[1], origin[2] + rel[2]}
		}
	}
	return pts, prismK
}

// pythagorean is a box turned about z by a Pythagorean angle with dyadic
// coordinates: its face normals have an exactly representable length, so the
// exact bound along them carries no norm rounding.
func (g *fixtureGen) pythagorean() [][3]float64 {
	triple := [][2]float64{{3, 4}, {5, 12}, {8, 15}}[g.rng.IntN(3)]
	t := math.Ldexp(1, g.rng.IntN(7)-3)
	h := math.Ldexp(float64(1+g.rng.IntN(9)), g.rng.IntN(5)-2)
	origin := [3]float64{math.Ldexp(float64(g.rng.IntN(64)-32), -2), math.Ldexp(float64(g.rng.IntN(64)-32), -2), 0}
	e1 := [3]float64{triple[0] * t, triple[1] * t, 0}
	e2 := [3]float64{-triple[1] * t, triple[0] * t, 0}
	var pts [][3]float64
	for i := range 8 {
		p := origin
		for axis, e := range [][3]float64{e1, e2, {0, 0, h}} {
			if i&(1<<axis) != 0 {
				for d := range 3 {
					p[d] += e[d]
				}
			}
		}
		pts = append(pts, p)
	}
	return pts
}

func floatInterval(lo, hi float64) proofbound.RatInterval {
	return proofbound.IntervalOwned(proofarith.FloatRat(lo), proofarith.FloatRat(hi))
}

type sideMode struct {
	rangeCorners bool // build through RoundCorners (LoF, HiF set)
	negativeH    bool // a negative span on the box-form path
	negativeRem  bool
	segment      bool
	static       bool // no motion, no pad: the float bound is tight
	small        bool // one joint, small bodies: cheap premise families
}

func (g *fixtureGen) side(shift [3]float64, mode sideMode) Side {
	k := 0
	if g.rng.IntN(3) > 0 {
		k = 3 + g.rng.IntN(6)
		if mode.small {
			k = 3 + g.rng.IntN(2)
		}
	}
	aligned := g.rng.IntN(2) == 0
	pts, prismK := g.points(k, aligned)
	if mode.static && g.rng.IntN(2) == 0 {
		pts, prismK = g.pythagorean(), 0
	}
	joints := 1 + g.rng.IntN(2)
	if mode.small || mode.static {
		joints = 1
	}
	reading := Reading{
		Pos: make([]motionbound.IvVec, len(pts)), Vel: make([][]motionbound.IvVec, len(pts)),
		PrismK: prismK,
	}
	if g.rng.IntN(2) == 0 {
		reading.Pad = proofarith.FloatRat(math.Abs(g.scale()))
		if mode.static {
			reading.Pad = proofarith.FloatRat(math.Ldexp(float64(g.rng.IntN(64)), -3))
		}
	}
	for c, p := range pts {
		for d := range 3 {
			x := p[d] + shift[d]
			w := 0.0
			if g.rng.IntN(3) == 0 {
				w = math.Abs(x) * 1e-9 * g.rng.Float64()
			}
			reading.Pos[c][d] = floatInterval(x-w, x+w)
		}
		reading.Vel[c] = make([]motionbound.IvVec, joints)
		for j := range joints {
			for d := range 3 {
				v := g.scale()
				w := math.Abs(v) * 1e-6 * g.rng.Float64()
				reading.Vel[c][j][d] = floatInterval(v-w, v+w)
			}
		}
	}
	corners, ok := RoundCorners(reading)
	if !ok {
		panic("fixture overflows a float")
	}
	if !mode.rangeCorners {
		// A literal Bounds: no float copies, and some ends not floats.
		corners.LoF, corners.HiF = nil, nil
		if g.rng.IntN(2) == 0 {
			for c := range corners.Lo {
				for d := range 3 {
					third := new(big.Rat).SetFrac64(int64(g.rng.IntN(7)), 3)
					third.Mul(third, big.NewRat(1, 1000))
					corners.Lo[c][d] = new(big.Rat).Sub(corners.Lo[c][d], third)
					corners.Hi[c][d] = new(big.Rat).Add(corners.Hi[c][d], third)
				}
			}
		}
	}
	s := Side{Corners: corners}
	span := func() *big.Rat {
		switch g.rng.IntN(3) {
		case 0:
			return new(big.Rat)
		case 1:
			return proofarith.FloatRat(math.Abs(g.scale()) * 1e-3)
		}
		return proofarith.FloatRat(math.Abs(g.scale()))
	}
	for range joints {
		s.H = append(s.H, span())
	}
	if mode.static {
		s.H[0] = new(big.Rat)
	}
	if mode.segment {
		for range joints {
			lo, hi := g.scale()*1e-2, g.scale()*1e-2
			if lo > hi {
				lo, hi = hi, lo
			}
			s.Seg = append(s.Seg, floatInterval(lo, hi))
		}
		s.H = nil
		for range joints {
			s.H = append(s.H, span())
		}
	}
	if mode.negativeH {
		s.H[g.rng.IntN(joints)] = proofarith.FloatRat(-math.Abs(g.scale()) * 1e3)
		s.Seg = nil
	}
	switch {
	case mode.negativeRem:
		s.Rem = big.NewRat(-int64(1+g.rng.IntN(5)), 3)
	case g.rng.IntN(2) == 0 && !mode.static:
		s.Rem = new(big.Rat).SetFrac64(int64(g.rng.IntN(9)), 3)
		s.Rem.Mul(s.Rem, big.NewRat(1, 1000))
	}
	return s
}

func (g *fixtureGen) pair() (Side, Side) {
	random := func() sideMode {
		return sideMode{rangeCorners: g.rng.IntN(2) == 0, segment: g.rng.IntN(2) == 0}
	}
	shift := [3]float64{g.scale(), g.scale(), g.scale()}
	if g.rng.IntN(3) == 0 {
		shift = [3]float64{g.scale() * 10, g.scale() * 10, g.scale() * 10}
	}
	ma, mb := random(), random()
	switch g.rng.IntN(6) {
	case 0, 1:
		// Static bodies with nothing but rounding between the float bound
		// and the exact one.
		shift = [3]float64{math.Ldexp(float64(g.rng.IntN(4096)-2048), -1), math.Ldexp(float64(g.rng.IntN(4096)-2048), -1), 0}
		ma = sideMode{rangeCorners: ma.rangeCorners, static: true}
		mb = sideMode{rangeCorners: mb.rangeCorners, static: true}
	case 2:
		// A negative span makes a motion term negative.
		ma = sideMode{rangeCorners: ma.rangeCorners, small: true, negativeH: true}
		mb.small = true
	case 3:
		// A negative remainder shrinks the remainder charge.
		ma = sideMode{rangeCorners: ma.rangeCorners, small: true, negativeRem: true}
		mb.small = true
	}
	return g.side([3]float64{}, ma), g.side(shift, mb)
}

func floatOf(q *big.Rat) float64 {
	f, _ := q.Float64()
	return f
}

// floors are the floors one reference bound is checked against.
func floors(ref *big.Rat) []*big.Rat {
	out := []*big.Rat{nil, big.NewRat(-1_000_000_000, 1), big.NewRat(1_000_000_000, 1)}
	if ref == nil {
		return out
	}
	thousandth := big.NewRat(1, 1000)
	tiny := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 80))
	relUp := new(big.Rat).Mul(ref, new(big.Rat).SetFrac(big.NewInt(1<<53+1), big.NewInt(1<<53)))
	relDown := new(big.Rat).Mul(ref, new(big.Rat).SetFrac(big.NewInt(1<<53-1), big.NewInt(1<<53)))
	return append(out,
		new(big.Rat).Sub(ref, thousandth), new(big.Rat).Set(ref), new(big.Rat).Add(ref, thousandth),
		new(big.Rat).Sub(ref, tiny), new(big.Rat).Add(ref, tiny), relUp, relDown,
	)
}

func requireSameHull(t *testing.T, want, got HullWitness, floor *big.Rat) {
	t.Helper()
	if want.Bound != nil && (floor == nil || want.Bound.Cmp(floor) > 0) {
		require.NotNil(t, got.Bound)
		require.Zero(t, want.Bound.Cmp(got.Bound))
		for d := range 3 {
			require.Zero(t, want.Axis[d].Cmp(got.Axis[d]))
		}
		require.Zero(t, want.Norm.Cmp(got.Norm))
		require.Equal(t, want.Sense, got.Sense)
		return
	}
	if got.Bound != nil {
		require.NotNil(t, floor)
		require.LessOrEqual(t, got.Bound.Cmp(floor), 0)
	}
}

func TestLowerHullAboveMatchesReference(t *testing.T) {
	g := &fixtureGen{rng: rand.New(rand.NewPCG(103, 107))}
	var axialSkips, floatSkips, refused int
	for range 250 {
		a, b := g.pair()
		want := lowerHullReference(a, b)
		fa, fb := floatSide(a), floatSide(b)
		if !fa.ok || !fb.ok {
			refused++
		} else if want.Bound != nil {
			limit, _ := ratEnclose(want.Bound)
			pads := new(big.Rat)
			for _, p := range []*big.Rat{a.Corners.Pad, b.Corners.Pad} {
				if p != nil {
					pads.Add(pads, p)
				}
			}
			padsLo, _ := ratEnclose(pads)
			for _, v := range append(faceNormalVecs(a.Corners), faceNormalVecs(b.Corners)...) {
				switch {
				case nonzeroComponents(v) == 1:
					axialSkips++
				case hullDirectionBelow(fa, fb, v, padsLo, limit):
					floatSkips++
				}
			}
		}
		// nil and the exact bound always; one other floor per trial, so that
		// every kind is drawn across the trials at a third of the cost.
		all := floors(want.Bound)
		for _, floor := range []*big.Rat{all[0], all[g.rng.IntN(len(all))], want.Bound} {
			requireSameHull(t, want, LowerHullWithWitnessAbove(a, b, floor), floor)
		}
		if want.Bound != nil && want.Bound.Cmp(big.NewRat(0, 1)) >= 0 {
			require.Zero(t, want.Bound.Cmp(LowerHullAbove(a, b, big.NewRat(-1, 1))))
		}
	}
	require.Positive(t, axialSkips)
	require.Positive(t, floatSkips)
	require.Positive(t, refused)
}

func TestLowerWithDirectionMatchesExistingCalculations(t *testing.T) {
	g := &fixtureGen{rng: rand.New(rand.NewPCG(307, 311))}
	for range 250 {
		a, b := g.pair()
		want := Lower(a, b)
		axis, sense := AttainedDirection(a, b, want)
		got, gotAxis, gotSense := LowerWithDirection(a, b)
		require.Zero(t, want.Cmp(got))
		require.Equal(t, axis, gotAxis)
		require.Equal(t, sense, gotSense)
	}

	zero := new(big.Rat)
	point := Side{Corners: Bounds{
		Lo: [][3]*big.Rat{{zero, zero, zero}}, Hi: [][3]*big.Rat{{zero, zero, zero}},
		Vel: [][][3]proofbound.RatInterval{{}},
	}}
	want := Lower(point, point)
	axis, sense := AttainedDirection(point, point, want)
	got, gotAxis, gotSense := LowerWithDirection(point, point)
	require.Zero(t, want.Cmp(got))
	require.Equal(t, 0, axis)
	require.Equal(t, 1, sense)
	require.Equal(t, axis, gotAxis)
	require.Equal(t, sense, gotSense)
}

// TestHullDirectionBelowIsSound checks the float test alone: whenever it
// reports a direction below a limit, both exact candidates of the direction are
// below the limit. The limits straddle the exact larger candidate to within a
// few floats, so a bound that is not rounded outward fails.
func TestHullDirectionBelowIsSound(t *testing.T) {
	g := &fixtureGen{rng: rand.New(rand.NewPCG(211, 223))}
	var proven, checked int
	minGap := math.Inf(1)
	for range 300 {
		a, b := g.pair()
		fa, fb := floatSide(a), floatSide(b)
		if !fa.ok || !fb.ok {
			continue
		}
		pads := new(big.Rat)
		for _, p := range []*big.Rat{a.Corners.Pad, b.Corners.Pad} {
			if p != nil {
				pads.Add(pads, p)
			}
		}
		padsLo, _ := ratEnclose(pads)
		for _, v := range append(faceNormalVecs(a.Corners), faceNormalVecs(b.Corners)...) {
			n, _ := motionbound.RatVecOf(v)
			cands, ok := hullCandidates(a, b, n, pads)
			if !ok {
				continue
			}
			larger := cands[0]
			if cands[1].Cmp(larger) > 0 {
				larger = cands[1]
			}
			at := floatOf(larger)
			upper := hullDirectionUpper(fa, fb, v, padsLo)
			if !math.IsInf(upper, 0) && !math.IsNaN(upper) {
				// The proven upper bound is at least both exact candidates,
				// and equal to the limit it does not prove anything.
				require.LessOrEqual(t, larger.Cmp(proofarith.FloatRat(upper)), 0)
				require.False(t, hullDirectionBelow(fa, fb, v, padsLo, upper))
				if gap := (upper - at) / math.Abs(at); gap < minGap {
					minGap = gap
				}
			}
			for k := -2; k <= 12; k++ {
				limit := at
				for range max(k, -k) {
					if k > 0 {
						limit = math.Nextafter(limit, math.Inf(1))
					} else {
						limit = math.Nextafter(limit, math.Inf(-1))
					}
				}
				checked++
				if !hullDirectionBelow(fa, fb, v, padsLo, limit) {
					continue
				}
				proven++
				require.Negative(t, larger.Cmp(proofarith.FloatRat(limit)), "k=%d", k)
			}
			// Far above the candidates the test must prove (not vacuous).
			far := at + math.Abs(at) + 1
			if hullDirectionBelow(fa, fb, v, padsLo, far) {
				proven++
			}
		}
	}
	_ = minGap
	require.Positive(t, checked)
	require.Positive(t, proven)
}

func TestUnitAxisNorms(t *testing.T) {
	one := big.NewRat(1, 1)
	require.Equal(t, 1.0, proofbound.RatSqrtUp(one))
	require.Equal(t, 1.0, proofbound.RatSqrtDown(one))
}

func TestFaceNormalsFloatHalf(t *testing.T) {
	g := &fixtureGen{rng: rand.New(rand.NewPCG(5, 9))}
	check := func(c Bounds) {
		t.Helper()
		want := FaceNormals(c)
		var viaVecs []motionbound.RatVec
		for _, v := range faceNormalVecs(c) {
			r, ok := motionbound.RatVecOf(v)
			require.True(t, ok)
			require.False(t, ZeroVec(r))
			viaVecs = append(viaVecs, r)
		}
		require.Len(t, viaVecs, len(want))
		for i := range want {
			for d := range 3 {
				require.Zero(t, want[i][d].Cmp(viaVecs[i][d]))
			}
		}
	}
	for range 200 {
		a, _ := g.pair()
		check(a.Corners)
		stripped := a.Corners
		stripped.LoF, stripped.HiF = nil, nil
		check(stripped)
		// The float copies equal the exact ends.
		for c := range a.Corners.Lo {
			for d := range 3 {
				if len(a.Corners.LoF) == 0 {
					continue
				}
				require.Zero(t, a.Corners.Lo[c][d].Cmp(proofarith.FloatRat(a.Corners.LoF[c][d])))
				require.Zero(t, a.Corners.Hi[c][d].Cmp(proofarith.FloatRat(a.Corners.HiF[c][d])))
			}
		}
	}
	// A prism with a repeated corner (a zero edge) and negative zeros.
	negZero := math.Copysign(0, -1)
	pts := [][3]float64{
		{0, 0, 0}, {0, 0, 0}, {1, 1, 0}, {negZero, negZero, 1}, {negZero, negZero, 1}, {1, 1, 1},
	}
	reading := Reading{Pos: make([]motionbound.IvVec, len(pts)), Vel: make([][]motionbound.IvVec, len(pts)), PrismK: 3}
	for c, p := range pts {
		for d := range 3 {
			reading.Pos[c][d] = floatInterval(p[d], p[d])
		}
	}
	c, ok := RoundCorners(reading)
	require.True(t, ok)
	check(c)
	require.Len(t, faceNormalVecs(c), 3)
	// A non-finite direction is left out.
	huge := c
	huge.LoF = append([][3]float64(nil), c.LoF...)
	huge.LoF[2] = [3]float64{math.MaxFloat64, 0, 0}
	huge.LoF[0] = [3]float64{-math.MaxFloat64, 0, 0}
	for _, v := range faceNormalVecs(huge) {
		require.True(t, finiteVec(v))
	}
}

// TestFloatPrimitivesEnclose checks each outward primitive of the float test
// against exact rational arithmetic, on operands whose results need rounding
// and on subnormal, huge and overflowing ones.
func TestFloatPrimitivesEnclose(t *testing.T) {
	rng := rand.New(rand.NewPCG(31, 37))
	draw := func() float64 {
		switch rng.IntN(8) {
		case 0:
			return math.Ldexp(rng.Float64()+0.5, -1060+rng.IntN(60)) // subnormal range
		case 1:
			return math.Ldexp(rng.Float64()+0.5, 990+rng.IntN(34)) // near overflow
		case 2:
			return float64(rng.IntN(1000)-500) / 8
		}
		v := math.Ldexp(rng.Float64()+0.5, rng.IntN(80)-40)
		if rng.IntN(2) == 0 {
			v = -v
		}
		return v
	}
	rat := proofarith.FloatRat
	var below, above int
	for range 20000 {
		x, y := draw(), draw()
		rx, ry := rat(x), rat(y)
		if sum := new(big.Rat).Add(rx, ry); true {
			if d := sumDown(x, y); !math.IsInf(d, -1) {
				require.LessOrEqual(t, rat(d).Cmp(sum), 0, "sumDown %g %g", x, y)
			}
			if u := sumUp(x, y); !math.IsInf(u, 1) {
				require.GreaterOrEqual(t, rat(u).Cmp(sum), 0, "sumUp %g %g", x, y)
			}
			if nearest := rat(float64(x + y)); nearest == nil {
				continue
			} else if nearest.Cmp(sum) < 0 {
				below++
			} else {
				above++
			}
		}
		if diff := new(big.Rat).Sub(rx, ry); true {
			if u := differenceUp(x, y); !math.IsInf(u, 1) {
				require.GreaterOrEqual(t, rat(u).Cmp(diff), 0, "differenceUp %g %g", x, y)
			}
		}
		prod := new(big.Rat).Mul(rx, ry)
		if d := productDown(x, y); !math.IsInf(d, -1) {
			require.LessOrEqual(t, rat(d).Cmp(prod), 0, "productDown %g %g", x, y)
		}
		if u := productUp(x, y); !math.IsInf(u, 1) {
			require.GreaterOrEqual(t, rat(u).Cmp(prod), 0, "productUp %g %g", x, y)
		}
		if y != 0 {
			quo := new(big.Rat).Quo(rx, ry)
			if u := quotientUp(x, y); !math.IsInf(u, 1) {
				require.GreaterOrEqual(t, rat(u).Cmp(quo), 0, "quotientUp %g %g", x, y)
			}
		}
		if x > 0 {
			if d := sqrtDown(x); d > 0 {
				require.LessOrEqual(t, new(big.Rat).Mul(rat(d), rat(d)).Cmp(rx), 0, "sqrtDown %g", x)
			}
			if u := sqrtUp(x); !math.IsInf(u, 1) {
				require.GreaterOrEqual(t, new(big.Rat).Mul(rat(u), rat(u)).Cmp(rx), 0, "sqrtUp %g", x)
			}
		}
	}
	// Both directions of the nearest rounding occur, so a primitive that
	// skips its outward step is caught.
	require.Positive(t, below)
	require.Positive(t, above)
	// Overflow never yields a finite bound on the wrong side.
	require.True(t, math.IsInf(productUp(math.MaxFloat64, 2), 1))
	require.Equal(t, math.MaxFloat64, productDown(math.MaxFloat64, 2))
	require.True(t, math.IsNaN(sumUp(math.Inf(1), math.Inf(-1))))
}

func TestRatEncloseBrackets(t *testing.T) {
	rng := rand.New(rand.NewPCG(41, 43))
	bigRand := mrand.New(mrand.NewSource(47))
	check := func(q *big.Rat) {
		lo, hi := ratEnclose(q)
		if !math.IsInf(lo, 0) {
			require.LessOrEqual(t, proofarith.FloatRat(lo).Cmp(q), 0, "lo of %v", q)
		}
		if !math.IsInf(hi, 0) {
			require.GreaterOrEqual(t, proofarith.FloatRat(hi).Cmp(q), 0, "hi of %v", q)
		}
		require.LessOrEqual(t, lo, hi)
	}
	for range 5000 {
		num := new(big.Int).Rand(bigRand, new(big.Int).Lsh(big.NewInt(1), uint(1+rng.IntN(120))))
		den := new(big.Int).Add(big.NewInt(1), new(big.Int).Rand(bigRand, new(big.Int).Lsh(big.NewInt(1), uint(1+rng.IntN(120)))))
		if rng.IntN(2) == 0 {
			num.Neg(num)
		}
		check(new(big.Rat).SetFrac(num, den))
	}
	for _, f := range []float64{0, 1, -1, 0.1, math.MaxFloat64, -math.MaxFloat64, math.SmallestNonzeroFloat64} {
		check(proofarith.FloatRat(f))
	}
	// Past the largest float the ends do not claim a finite side they cannot.
	huge := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 1100))
	check(huge)
	check(new(big.Rat).Neg(huge))
}

// TestAxialCandidatesNeverBeatCoordinateDirections checks the proof the
// axial skip rests on: for a side pair with nonnegative spans and remainders,
// a direction with one nonzero component c has candidates at most the
// coordinate candidates of its axis, whatever c's sign and magnitude.
func TestAxialCandidatesNeverBeatCoordinateDirections(t *testing.T) {
	g := &fixtureGen{rng: rand.New(rand.NewPCG(71, 73))}
	zero := new(big.Rat)
	var checked int
	for range 150 {
		a, b := g.pair()
		if !floatSide(a).ok || !floatSide(b).ok {
			continue
		}
		pads := new(big.Rat)
		for _, p := range []*big.Rat{a.Corners.Pad, b.Corners.Pad} {
			if p != nil {
				pads.Add(pads, p)
			}
		}
		aUp, aDown := a.Extents()
		bUp, bDown := b.Extents()
		for axis := range 3 {
			coord := [2]*big.Rat{
				new(big.Rat).Sub(new(big.Rat).Neg(new(big.Rat).Add(bDown[axis], aUp[axis])), pads),
				new(big.Rat).Sub(new(big.Rat).Neg(new(big.Rat).Add(bUp[axis], aDown[axis])), pads),
			}
			for _, c := range []float64{1, -1, 3, -0.1, 1e-150, -1e-150, 1e150, -1e150, g.scale(), g.scale(), 5e-324, -math.MaxFloat64} {
				n := motionbound.RatVec{zero, zero, zero}
				n[axis] = proofarith.FloatRat(c)
				cands, ok := hullCandidates(a, b, n, pads)
				if !ok {
					continue
				}
				checked++
				// A negative c turns the direction around: its senses swap.
				first, second := coord[0], coord[1]
				if c < 0 {
					first, second = second, first
				}
				require.LessOrEqual(t, cands[0].Cmp(first), 0, "c=%g axis=%d", c, axis)
				require.LessOrEqual(t, cands[1].Cmp(second), 0, "c=%g axis=%d", c, axis)
			}
		}
	}
	require.Positive(t, checked)
}

// TestHullWitnessOfTurnedBoxes pins a witness by value, so a change in the
// face normals' signs or order cannot hide behind a reference that shares
// the normal generator. Two stationary boxes with edges (3,4,0), (-4,3,0)
// and (0,0,1), at the origin and at (6,8,0), are separated by 5 along the
// first edge of the first box; the coordinate directions only reach 2 and 1.
func TestHullWitnessOfTurnedBoxes(t *testing.T) {
	box := func(shift [3]float64) Side {
		edges := [3][3]float64{{3, 4, 0}, {-4, 3, 0}, {0, 0, 1}}
		reading := Reading{Pos: make([]motionbound.IvVec, 8), Vel: make([][]motionbound.IvVec, 8)}
		for i := range 8 {
			p := shift
			for axis, e := range edges {
				if i&(1<<axis) != 0 {
					for d := range 3 {
						p[d] += e[d]
					}
				}
			}
			for d := range 3 {
				reading.Pos[i][d] = floatInterval(p[d], p[d])
			}
		}
		corners, ok := RoundCorners(reading)
		require.True(t, ok)
		return Side{Corners: corners}
	}
	a, b := box([3]float64{}), box([3]float64{6, 8, 0})
	want := func(w HullWitness) {
		t.Helper()
		require.Zero(t, w.Bound.Cmp(big.NewRat(5, 1)))
		for d, c := range []int64{3, 4, 0} {
			require.Zero(t, w.Axis[d].Cmp(big.NewRat(c, 1)), "axis %d", d)
		}
		require.Zero(t, w.Norm.Cmp(big.NewRat(5, 1)))
		require.Equal(t, 1, w.Sense)
	}
	want(LowerHullWithWitness(a, b))
	want(LowerHullWithWitnessAbove(a, b, big.NewRat(4, 1)))
	// Literal bounds without float copies give the same witness.
	a.Corners.LoF, a.Corners.HiF, b.Corners.LoF, b.Corners.HiF = nil, nil, nil, nil
	want(LowerHullWithWitness(a, b))
}
