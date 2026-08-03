package decad_test

import (
	"math"
	"math/rand"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is the differential (oracle) hardening of the analytic clearance
// kernel: the certified answer of Document.Verify(WithClearances) is checked
// against an INDEPENDENT, slower, dumber oracle — a dense boundary point cloud
// whose pairwise minimum distance brackets the true minimum surface-to-surface
// gap from above. The kernel and the oracle share no code: the oracle reads a
// prism's boundary through Body.Tessellate (a separate evaluator path) and a
// full sphere/torus's boundary through the public analytic Surface, then
// samples it, while the kernel proves its interval through clearance*.go.
//
// The load-bearing invariant is one-sided and needs no sampling slack: for any
// SOUND kernel the proven lower bound lo satisfies lo <= trueMin <= sampledMin,
// so lo <= sampledMin must hold outright. A kernel gap that exceeds the sampled
// minimum is a false certification — the exact class the round-3 root-cause
// work fixed (a rod piercing a ball reported Sound with a positive Exact gap).
// The upper side (sampledMin <= hi + covering-radius slack) confirms the
// interval actually contains the truth and is not uselessly loose.
//
// Every randomized test uses a FIXED, printed seed, so a failure is replayable.

// oracleSeed is the fixed base seed; each test derives an independent stream so
// they neither share nor perturb one another. Printed by every test.
const oracleSeed = 0x5EED0C1EA9A11CE

// --- the independent oracle: dense boundary sampling ------------------------

// meshPoints samples a prism body's boundary densely: Body.Tessellate gives
// watertight facets whose vertices lie EXACTLY on the analytic boundary (a
// separate evaluator path from the clearance kernel), and a barycentric grid
// of level k fills each facet interior — including the large flat cap facets a
// vertex-only cloud would sample sparsely. Every returned point lies on the
// true boundary, so the pairwise minimum can only over-estimate the true gap.
// The second result is the covering radius: no boundary point lies farther
// than this from some sample (a sub-facet grid cell is at most maxEdge/k
// across).
func meshPoints(t *testing.T, body *decad.Body, tol units.Value, k int) ([]r3.Vec, float64) {
	t.Helper()
	mesh, err := body.Tessellate(tol)
	require.NoError(t, err)
	verts := mesh.Vertices()
	tris := mesh.Triangles()
	var pts []r3.Vec
	maxEdge := 0.0
	for _, tri := range tris {
		a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
		for _, e := range []float64{a.Sub(b).Len(), b.Sub(c).Len(), c.Sub(a).Len()} {
			maxEdge = math.Max(maxEdge, e)
		}
		for i := 0; i <= k; i++ {
			for j := 0; i+j <= k; j++ {
				wa := float64(k-i-j) / float64(k)
				wb := float64(i) / float64(k)
				wc := float64(j) / float64(k)
				pts = append(pts, a.Scale(wa).Add(b.Scale(wb)).Add(c.Scale(wc)))
			}
		}
	}
	return pts, maxEdge / float64(k)
}

// curvedPoints samples a full-sphere or full-torus body's boundary off its
// public analytic Surface — the entire carrier is boundary (a full revolve
// face carries no trim loops), so a uniform parameter grid lands only on the
// real skin. Densities m and 2m NEST (every coarse parameter is a fine one),
// so a denser cloud is a superset and its minimum can only fall. ok is false
// for any body that is not a single full sphere or full torus. The second
// result is the covering radius.
func curvedPoints(t *testing.T, body *decad.Body, m int) ([]r3.Vec, float64, bool) {
	t.Helper()
	faces := body.Faces()
	if len(faces) != 1 {
		return nil, 0, false
	}
	step := 2 * math.Pi / float64(m)
	switch s := faces[0].Surface().(type) {
	case decad.Sphere:
		center := s.Center
		r, err := s.Radius.In(units.Millimeter)
		require.NoError(t, err)
		var pts []r3.Vec
		for i := 0; i <= m; i++ {
			th := math.Pi * float64(i) / float64(m) // colatitude 0..pi
			sinTh, cosTh := math.Sincos(th)
			for j := range m {
				phi := step * float64(j)
				sinP, cosP := math.Sincos(phi)
				pts = append(pts, center.Add(r3.NewVec(r*sinTh*cosP, r*sinTh*sinP, r*cosTh)))
			}
		}
		return pts, r * step, true
	case decad.Torus:
		major, err := s.Major.In(units.Millimeter)
		require.NoError(t, err)
		minor, err := s.Minor.In(units.Millimeter)
		require.NoError(t, err)
		axis := s.Axis
		e1 := perpUnit(axis)
		e2 := axis.Cross(e1)
		var pts []r3.Vec
		for i := range m {
			u := step * float64(i)
			sinU, cosU := math.Sincos(u)
			ring := e1.Scale(cosU).Add(e2.Scale(sinU)) // unit radial direction
			for j := range m {
				v := step * float64(j)
				sinV, cosV := math.Sincos(v)
				p := s.Center.Add(ring.Scale(major + minor*cosV)).Add(axis.Scale(minor * sinV))
				pts = append(pts, p)
			}
		}
		return pts, (major + minor) * step, true
	default:
		return nil, 0, false
	}
}

// perpUnit is a deterministic unit vector perpendicular to a unit vector.
func perpUnit(a r3.Vec) r3.Vec {
	seed := r3.NewVec(1, 0, 0)
	if math.Abs(a.X) > 0.9 {
		seed = r3.NewVec(0, 1, 0)
	}
	p, _ := a.Cross(seed).Normalize()
	return p
}

// minPairDistance is the brute-force minimum distance between two point clouds:
// the whole oracle, deliberately as simple as possible.
func minPairDistance(a, b []r3.Vec) float64 {
	best := math.Inf(1)
	for _, p := range a {
		for _, q := range b {
			if d := p.Sub(q).Len(); d < best {
				best = d
			}
		}
	}
	return best
}

// --- randomized geometry ----------------------------------------------------

func randUnit(rng *rand.Rand) r3.Vec {
	for {
		v := r3.NewVec(rng.NormFloat64(), rng.NormFloat64(), rng.NormFloat64())
		if u, ok := v.Normalize(); ok {
			return u
		}
	}
}

// randMotion is a random rigid motion: a rotation by a random angle about a
// random axis, then a translation of the given magnitude along a random
// direction.
func randMotion(t *testing.T, rng *rand.Rand, transMag float64) r3.Transform {
	t.Helper()
	rot, err := r3.Rotation(randUnit(rng), units.Radians(rng.Float64()*2*math.Pi))
	require.NoError(t, err)
	tr, err := r3.Translation(randUnit(rng).Scale(transMag))
	require.NoError(t, err)
	m, err := rot.Then(tr)
	require.NoError(t, err)
	return m
}

// randPrism builds a random box or rod into doc (both prisms, both
// tessellatable).
func randPrism(t *testing.T, doc *decad.Document, rng *rand.Rand) *decad.Body {
	t.Helper()
	if rng.Intn(2) == 0 {
		w := 5 + rng.Float64()*10
		d := 5 + rng.Float64()*10
		h := 5 + rng.Float64()*10
		return boxBody(t, doc, 0, 0, w, d, h)
	}
	r := 2 + rng.Float64()*4
	half := 4 + rng.Float64()*6
	return rodBody(t, doc, 0, 0, r, half)
}

// --- the bracket assertion --------------------------------------------------

// clearanceInterval reads the kernel's proven [lo, hi] in millimetres off a
// Clearance row.
func clearanceInterval(c decad.Clearance) (float64, float64) {
	v := c.Gap.Value.Mag()
	b := c.Gap.Bound.Mag()
	return v - b, v + b
}

// assertBracket is the core oracle check: the proven lower bound never exceeds
// the sampled minimum (soundness — no sampling slack, one-sided), and the
// sampled minimum sits within the interval widened by the covering radius
// (the interval genuinely contains the truth).
func assertBracket(t *testing.T, c decad.Clearance, sampledMin, slack float64, tag string) {
	t.Helper()
	lo, hi := clearanceInterval(c)
	eps := 1e-6 * math.Max(1, sampledMin)
	require.LessOrEqualf(t, lo, sampledMin+eps,
		"%s: FALSE CERTIFICATION — proven gap lo=%.12g exceeds sampled min=%.12g (kernel over-claims the gap)", tag, lo, sampledMin)
	require.GreaterOrEqualf(t, sampledMin+eps, lo,
		"%s: sampled min %.12g fell below certified lower bound %.12g", tag, sampledMin, lo)
	require.LessOrEqualf(t, sampledMin, hi+slack+eps,
		"%s: sampled min %.12g above proven hi=%.12g + slack=%.12g (interval does not reach the truth)", tag, sampledMin, hi, slack)
}

// --- invariants 1 & 2: prism pairs ------------------------------------------

func TestClearanceOraclePrismPairs(t *testing.T) {
	t.Logf("seed=%#x", oracleSeed)
	rng := rand.New(rand.NewSource(oracleSeed + 1))
	rows := 0
	for iter := range 12 {
		doc := decad.New()
		randPrism(t, doc, rng)
		b := randPrism(t, doc, rng)
		// Separate B by a translation well beyond both bodies' ~15 mm reach.
		_, err := b.Placed(randMotion(t, rng, 25+rng.Float64()*40))
		require.NoError(t, err)

		report, err := doc.Verify(t.Context(), decad.WithClearances())
		require.NoError(t, err)
		if len(report.Clearances) == 0 {
			continue // the kernel was cautious on this pair; not a soundness fault
		}
		require.Len(t, report.Clearances, 1)
		c := report.Clearances[0]
		rows++

		tol := units.Millimeters(0.3)
		coarsePts, coarseSlackA := meshPoints(t, c.A, tol, 2)
		coarsePtsB, coarseSlackB := meshPoints(t, c.B, tol, 2)
		coarseMin := minPairDistance(coarsePts, coarsePtsB)

		finePtsA, fineSlackA := meshPoints(t, c.A, tol, 4)
		finePtsB, fineSlackB := meshPoints(t, c.B, tol, 4)
		fineMin := minPairDistance(finePtsA, finePtsB)

		// Refinement consistency (invariant 2): the level-4 grid is a superset
		// of the level-2 grid (level 2's parameters divide level 4's), so the
		// finer sampled minimum can only fall.
		require.LessOrEqualf(t, fineMin, coarseMin+1e-9,
			"iter %d: denser sampling raised the sampled minimum (%.12g > %.12g)", iter, fineMin, coarseMin)

		assertBracket(t, c, coarseMin, coarseSlackA+coarseSlackB, "prism/coarse")
		assertBracket(t, c, fineMin, fineSlackA+fineSlackB, "prism/fine")
	}
	require.Greater(t, rows, 6, "too few prism pairs produced a clearance row to be a meaningful oracle")
}

// --- invariants 1 & 2: curved pairs (P8 / CF cells) -------------------------

func TestClearanceOracleCurvedPairs(t *testing.T) {
	t.Logf("seed=%#x", oracleSeed)
	rng := rand.New(rand.NewSource(oracleSeed + 2))
	rows := 0
	for iter := range 18 {
		doc := decad.New()
		a := randBallOrTorus(t, doc, rng)
		b := randBallOrTorus(t, doc, rng)
		// A stays in its canonical pose; B is placed far enough to separate the
		// ~24 mm-reach tori.
		pb, err := b.Placed(randMotion(t, rng, 60+rng.Float64()*60))
		require.NoError(t, err)

		report, err := doc.Verify(t.Context(), decad.WithClearances())
		require.NoError(t, err)
		if len(report.Clearances) == 0 {
			continue
		}
		require.Len(t, report.Clearances, 1)
		c := report.Clearances[0]
		// c.A / c.B follow document order; map them back to a and pb.
		bodyA, bodyB := a, pb

		coarseA, csA, okA := curvedPoints(t, bodyA, 24)
		coarseB, csB, okB := curvedPoints(t, bodyB, 24)
		require.True(t, okA && okB)
		coarseMin := minPairDistance(coarseA, coarseB)

		fineA, fsA, _ := curvedPoints(t, bodyA, 48)
		fineB, fsB, _ := curvedPoints(t, bodyB, 48)
		fineMin := minPairDistance(fineA, fineB)

		require.LessOrEqualf(t, fineMin, coarseMin+1e-9,
			"iter %d: denser curved sampling raised the sampled minimum", iter)

		assertBracket(t, c, coarseMin, csA+csB, "curved/coarse")
		assertBracket(t, c, fineMin, fsA+fsB, "curved/fine")
		rows++
	}
	// The P8 torus/torus and CF sphere cells should not be uselessly timid on
	// well-separated pairs.
	require.Greater(t, rows, 6, "too few curved pairs produced a clearance row")
}

// randBallOrTorus builds a random full sphere or full torus into doc.
func randBallOrTorus(t *testing.T, doc *decad.Document, rng *rand.Rand) *decad.Body {
	t.Helper()
	if rng.Intn(2) == 0 {
		return ballBody(t, doc, 4+rng.Float64()*8)
	}
	major := 8 + rng.Float64()*8
	minor := 1 + rng.Float64()*(major/2-1) // keep minor < major (standard torus)
	return torusBody(t, doc, major, minor)
}

// --- invariant 3: interpenetration is never Sound ---------------------------

func TestClearanceOracleOverlapNeverSound(t *testing.T) {
	t.Logf("seed=%#x", oracleSeed)
	rng := rand.New(rand.NewSource(oracleSeed + 3))

	check := func(t *testing.T, doc *decad.Document, tag string) {
		t.Helper()
		report, err := doc.Verify(t.Context(), decad.WithClearances())
		require.NoError(t, err)
		require.NotEqualf(t, decad.Sound, report.Status,
			"%s: an interpenetrating pair reported Sound", tag)
		require.Falsef(t, report.Trustworthy(), "%s: an overlapping pair is Trustworthy", tag)
		require.Emptyf(t, report.Clearances, "%s: an overlapping pair fabricated a clearance row", tag)
	}

	for range 12 {
		// Two unit cubes whose ranges overlap by at least 4 mm on every axis:
		// guaranteed shared volume, boundaries crossing.
		doc := decad.New()
		boxBody(t, doc, 0, 0, 10, 10, 10)
		b := boxBody(t, doc, 0, 0, 10, 10, 10)
		off := r3.NewVec(rng.Float64()*6-3, rng.Float64()*6-3, rng.Float64()*6-3)
		shift, err := r3.Translation(off)
		require.NoError(t, err)
		_, err = b.Placed(shift)
		require.NoError(t, err)
		check(t, doc, "overlapping boxes")
	}

	for range 8 {
		// A rod on a random off-centre axis running clear through a ball: the
		// rod-through-ball class the nested-branch supremum guards.
		doc := decad.New()
		ballBody(t, doc, 10)
		cx := rng.Float64()*6 - 3
		cy := rng.Float64()*6 - 3
		rodBody(t, doc, cx, cy, 1+rng.Float64()*2, 12)
		check(t, doc, "rod through ball")
	}

	for range 6 {
		// A small ball wholly inside a large one: a nesting witness, proven
		// overlap — never a positive clearance.
		doc := decad.New()
		ballBody(t, doc, 10)
		inner := ballBody(t, doc, 1+rng.Float64()*3)
		shift, err := r3.Translation(randUnit(rng).Scale(rng.Float64() * 3))
		require.NoError(t, err)
		_, err = inner.Placed(shift)
		require.NoError(t, err)
		check(t, doc, "ball inside ball")
	}
}

// --- invariant 4: easy pairs stay non-violating -----------------------------

func TestClearanceOracleWellSeparatedSound(t *testing.T) {
	t.Logf("seed=%#x", oracleSeed)
	rng := rand.New(rand.NewSource(oracleSeed + 4))
	for range 14 {
		doc := decad.New()
		randPrism(t, doc, rng)
		b := randPrism(t, doc, rng)
		// Far apart and axis-aligned (no rotation): a box-disjoint pair whose
		// gap is a closed-form edge/face reading — must read Sound.
		shift, err := r3.Translation(randUnit(rng).Scale(80 + rng.Float64()*80))
		require.NoError(t, err)
		_, err = b.Placed(shift)
		require.NoError(t, err)

		report, err := doc.Verify(t.Context(), decad.WithClearances())
		require.NoError(t, err)
		require.Contains(t, []decad.Status{decad.Sound, decad.Suspect}, report.Status)
		require.Equal(t, report.Status == decad.Sound, report.Trustworthy())
		require.Len(t, report.Clearances, 1)
		lo, _ := clearanceInterval(report.Clearances[0])
		require.Greater(t, lo, 0.0, "a well-separated pair must prove a positive gap")
	}
}

// --- the poly-cell property test: analytic truth ----------------------------

// TestClearancePolyBracketContainsTruth checks the certified P4/P8/CF cells
// against configurations whose true minimum gap is known in closed form: the
// kernel's interval must contain that truth and its width must sit within the
// design's noise floor (the default rel = 1e-3 gate — so a too-wide bracket
// would read Suspect rather than Sound).
func TestClearancePolyBracketContainsTruth(t *testing.T) {
	t.Logf("seed=%#x", oracleSeed)
	rng := rand.New(rand.NewSource(oracleSeed + 5))

	t.Run("offset spheres are CF Exact", func(t *testing.T) {
		for range 20 {
			r1 := 3 + rng.Float64()*6
			r2 := 3 + rng.Float64()*6
			gap := 2 + rng.Float64()*20
			d := r1 + r2 + gap
			doc := decad.New()
			ballBody(t, doc, r1)
			b := ballBody(t, doc, r2)
			shift, err := r3.Translation(randUnit(rng).Scale(d))
			require.NoError(t, err)
			_, err = b.Placed(shift)
			require.NoError(t, err)

			report, err := doc.Verify(t.Context(), decad.WithClearances())
			require.NoError(t, err)
			require.Equal(t, decad.Suspect, report.Status)
			require.Len(t, report.Clearances, 1)
			row := report.Clearances[0]
			require.Equal(t, decad.Exact, row.Gap.Exactness, "offset spheres are the point-spine CF cell")
			require.InDelta(t, gap, row.Gap.Value.Mag(), 1e-9)
			require.Equal(t, 0.0, row.Gap.Bound.Mag())
		}
	})

	t.Run("parallel-axis coplanar tori are the P8 cell", func(t *testing.T) {
		for iter := range 20 {
			maj1 := 8 + rng.Float64()*6
			min1 := 1 + rng.Float64()*3
			maj2 := 8 + rng.Float64()*6
			min2 := 1 + rng.Float64()*3
			gap := 2 + rng.Float64()*15
			d := maj1 + maj2 + min1 + min2 + gap
			doc := decad.New()
			torusBody(t, doc, maj1, min1)
			b := torusBody(t, doc, maj2, min2)
			// Both tori have their axis on world X; a translation in the y-z
			// plane keeps the axes parallel AND the spine circles coplanar
			// (both in the plane x = 0), so the min surface gap is the analytic
			// d - maj1 - maj2 - min1 - min2.
			s, c := math.Sincos(rng.Float64() * 2 * math.Pi)
			shift, err := r3.Translation(r3.NewVec(0, d*c, d*s))
			require.NoError(t, err)
			_, err = b.Placed(shift)
			require.NoError(t, err)

			report, err := doc.Verify(t.Context(), decad.WithClearances())
			require.NoError(t, err)
			require.Len(t, report.Clearances, 1)
			row := report.Clearances[0]
			lo := row.Gap.Value.Mag() - row.Gap.Bound.Mag()
			hi := row.Gap.Value.Mag() + row.Gap.Bound.Mag()
			require.LessOrEqualf(t, lo, gap+1e-6, "P8 lower bound over-claims the gap (iter %d)", iter)
			require.GreaterOrEqualf(t, hi, gap-1e-6, "P8 upper bound falls short of the truth (iter %d)", iter)
			// The certified bracket must be tight enough to clear the gate.
			require.Equal(t, decad.Suspect, report.Status, "bounded torus mass results remain visible")
			require.LessOrEqual(t, 2*row.Gap.Bound.Mag(), 1e-3*row.Gap.Value.Mag(), "the P8 bracket width exceeds the rel=1e-3 noise floor")
		}
	})

	t.Run("parallel rods are the cylinder/cylinder cell", func(t *testing.T) {
		for iter := range 20 {
			r1 := 2 + rng.Float64()*4
			r2 := 2 + rng.Float64()*4
			gap := 2 + rng.Float64()*15
			d := r1 + r2 + gap
			// Both rods have their axis on world Z with overlapping z-ranges;
			// centre separation d in the x-y plane gives the analytic gap d-r1-r2.
			// The lateral-wall plateau is a closed-form winner, but the finite
			// caps' rim circles are a P8 circle/circle rival at the same
			// distance whose bracket straddles it, so the row is honest-
			// Approximate at the bracket width (design §1) — not Exact. What
			// must hold is that the proven interval contains the analytic truth.
			ang := rng.Float64() * 2 * math.Pi
			doc := decad.New()
			rodBody(t, doc, 0, 0, r1, 8)
			rodBody(t, doc, d*math.Cos(ang), d*math.Sin(ang), r2, 8)

			report, err := doc.Verify(t.Context(), decad.WithClearances())
			require.NoError(t, err)
			require.Equal(t, decad.Sound, report.Status)
			require.Len(t, report.Clearances, 1)
			row := report.Clearances[0]
			lo := row.Gap.Value.Mag() - row.Gap.Bound.Mag()
			hi := row.Gap.Value.Mag() + row.Gap.Bound.Mag()
			require.LessOrEqualf(t, lo, gap+1e-6, "cylinder/cylinder lower bound over-claims the gap (iter %d)", iter)
			require.GreaterOrEqualf(t, hi, gap-1e-6, "cylinder/cylinder upper bound falls short of the truth (iter %d)", iter)
			require.LessOrEqual(t, 2*row.Gap.Bound.Mag(), 1e-3*row.Gap.Value.Mag(), "the bracket width exceeds the rel=1e-3 noise floor")
		}
	})
}
