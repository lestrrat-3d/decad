package proof_test

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// requireSameOrientation checks OrientSignExpansion against the previous
// grow-expansion implementation, and a certain answer against the integer
// determinant.
func requireSameOrientation(t *testing.T, p [4]r3.Vec, label string) {
	t.Helper()
	got, certain := proof.OrientSignExpansion(p[0], p[1], p[2], p[3])
	legacy, legacyCertain := proof.LegacyOrientSignExpansion(p[0], p[1], p[2], p[3])
	require.Equal(t, legacyCertain, certain, "%s: certainty differs from the previous implementation", label)
	require.Equal(t, legacy, got, "%s: sign differs from the previous implementation", label)
	if !certain {
		return
	}
	want := proof.OrientSignExact(proof.XptOf(p[0]), proof.XptOf(p[1]), proof.XptOf(p[2]), proof.XptOf(p[3]))
	require.Equal(t, want, got, "%s: sign differs from the exact determinant", label)
}

// nearCoplanarQuads returns point quadruples of the kinds the loft crossing
// audit hands the expansion: four nearby points of a faceted cylinder or
// cone, whose determinant is tiny against its permanent; exactly coplanar
// points on a dyadic grid, and the same points with one coordinate moved by
// one ulp; repeated points; and quadruples with wide exponents.
func nearCoplanarQuads(n int) [][4]r3.Vec {
	rng := rand.New(rand.NewPCG(0x51a2, 0x96e3))
	quads := make([][4]r3.Vec, 0, n)
	for len(quads) < n {
		switch kind := len(quads) % 10; {
		case kind < 6:
			// Points of a cylinder or cone of radius about 10 with a small
			// angular step, as a fine faceting produces.
			cx, cy := 20*rng.Float64()-10, 20*rng.Float64()-10
			r, slope := 1+19*rng.Float64(), rng.Float64()-0.5
			theta, step := 2*math.Pi*rng.Float64(), math.Ldexp(1+rng.Float64(), -4-rng.IntN(12))
			var q [4]r3.Vec
			for k := range q {
				z := 3*rng.Float64() - 1.5
				rk := r + slope*z
				angle := theta + step*float64(k%2) + step*0.5*float64(k/2)
				q[k] = r3.Vec{X: cx + rk*math.Cos(angle), Y: cy + rk*math.Sin(angle), Z: z}
			}
			quads = append(quads, q)
		case kind < 8:
			// Exactly coplanar dyadic points on z = x + 2y; the second copy
			// moves the last z by one ulp.
			var q [4]r3.Vec
			for k := range q {
				x := float64(rng.IntN(1<<16)-1<<15) * 0x1p-10
				y := float64(rng.IntN(1<<16)-1<<15) * 0x1p-10
				q[k] = r3.Vec{X: x, Y: y, Z: x + 2*y}
			}
			quads = append(quads, q)
			q[3].Z = math.Nextafter(q[3].Z, math.Inf(1-2*rng.IntN(2)))
			quads = append(quads, q)
		case kind < 9:
			p := r3.Vec{X: rng.Float64(), Y: rng.Float64(), Z: rng.Float64()}
			o := r3.Vec{X: rng.Float64(), Y: rng.Float64(), Z: rng.Float64()}
			quads = append(quads, [4]r3.Vec{p, o, p, {X: rng.Float64(), Y: rng.Float64(), Z: rng.Float64()}})
		default:
			c := func() float64 { return math.Ldexp(rng.Float64()-0.5, rng.IntN(400)-200) }
			var q [4]r3.Vec
			for k := range q {
				q[k] = r3.Vec{X: c(), Y: c(), Z: c()}
			}
			quads = append(quads, q)
		}
	}
	return quads[:n]
}

func TestOrientSignExpansionMatchesPreviousImplementation(t *testing.T) {
	t.Parallel()
	decided := 0
	for _, q := range nearCoplanarQuads(40000) {
		requireSameOrientation(t, q, "generated")
		if _, ok := proof.OrientSignSum2(q[0], q[1], q[2], q[3]); ok {
			decided++
		}
	}
	// The compensated sum must decide most of them, or the comparison above
	// only exercises the fallback.
	require.Greater(t, decided, 20000)
	edge := []float64{0, math.Copysign(0, -1), 1, -1, 0.5, 3, -7.25, 0x1p-200, -0x1p-200,
		0x1p200, math.Nextafter(0x1p-200, 0), math.Nextafter(0x1p200, math.Inf(1)),
		1 + 0x1p-52, 1 - 0x1p-53, 31.7, -12.3, math.SmallestNonzeroFloat64, math.MaxFloat64}
	rng := rand.New(rand.NewPCG(0x2d, 0x4f))
	pick := func() float64 { return edge[rng.IntN(len(edge))] }
	for range 20000 {
		var q [4]r3.Vec
		for k := range q {
			q[k] = r3.Vec{X: pick(), Y: pick(), Z: pick()}
		}
		requireSameOrientation(t, q, "edge")
	}
}

// TestOrientSignSum2LeavesUnprovenSignsToTheExpansion holds quadruples from
// the sector-union loft audit whose compensated sum is wrong or too small to
// trust. Each fixture breaks one leg: a bound test reduced to a bare sign
// test, or a missing fallback, returns the wrong sign for it.
func TestOrientSignSum2LeavesUnprovenSignsToTheExpansion(t *testing.T) {
	t.Parallel()
	apex := r3.Vec{X: 4.499999999999999, Y: 8.999999999999998}
	cases := []struct {
		name    string
		points  [4]r3.Vec
		want    int
		decided bool
	}{
		{"positive determinant, negative sum", [4]r3.Vec{apex,
			{X: 6.166710699058806, Y: 8.166644650470595, Z: -2.7888338615008292},
			{X: 5.914190210477992, Y: 8.292904894761001, Z: -2.9580538468282236},
			{X: 2.8332893009411926, Y: 9.833355349529402, Z: 2.788833861500829}}, 1, false},
		{"negative determinant, positive sum", [4]r3.Vec{apex,
			{X: 7.324632195549061, Y: 7.587683902225468, Z: -1.129962919672413},
			{X: 7.271638597533859, Y: 7.614180701233068, Z: -1.283559252946925},
			{X: 7.211967879370329, Y: 7.644016060314833, Z: -1.4340633795545752}}, -1, false},
		{"zero determinant, negative sum", [4]r3.Vec{apex,
			{X: 4.5, Y: 8.999999999999998, Z: -3.3541019662496834},
			{X: 4.205948579011319, Y: 9.147025710494338, Z: -3.3379510485128696},
			{X: 4.794051420988679, Y: 8.852974289505658, Z: 3.3379510485128696}}, 0, false},
		{"zero determinant, positive sum", [4]r3.Vec{apex,
			{X: 4.205948579011319, Y: 9.147025710494338, Z: -3.3379510485128696},
			{X: 3.9147290339516143, Y: 9.29263548302419, Z: -3.2896538374692224},
			{X: 4.794051420988679, Y: 8.852974289505658, Z: 3.3379510485128696}}, 0, false},
		{"positive determinant within the bound", [4]r3.Vec{apex,
			{X: 7.4423558412096895, Y: 7.5288220793951535, Z: -0.65435283267058},
			{X: 7.370821007196625, Y: 7.564589496401685, Z: -0.9736444067513462},
			{X: 1.5576441587903087, Y: 10.471177920604843, Z: 0.6543528326705795}}, 1, false},
		{"negative determinant within the bound", [4]r3.Vec{
			{X: 7.809612084391139, Y: 6.571144806551388, Z: 0.008279451495772382},
			{X: 4.881005155959784, Y: 4.106963487397883, Z: 0.006899539525171074},
			{X: 4.881007552744461, Y: 4.106965504094617, Z: 0.005174657184857738},
			{X: 7.809608249535655, Y: 6.571141579836613, Z: 0.011039263240273718}}, -1, false},
		{"near-coplanar determinant the sum decides", [4]r3.Vec{{},
			{X: 0.6260990336999415, Y: 10.867290370648979, Z: 0.4},
			{X: -0.44721359549995743, Y: 11.403946685248927, Z: 0.4},
			{X: -0.8944271909999157, Y: 11.627553482998907, Z: 0.4}}, -1, true},
	}
	for _, tc := range cases {
		p := tc.points
		exact := proof.OrientSignExact(proof.XptOf(p[0]), proof.XptOf(p[1]), proof.XptOf(p[2]), proof.XptOf(p[3]))
		require.Equal(t, tc.want, exact, tc.name)
		_, decided := proof.OrientSignSum2(p[0], p[1], p[2], p[3])
		require.Equal(t, tc.decided, decided, tc.name)
		got, certain := proof.OrientSignExpansion(p[0], p[1], p[2], p[3])
		require.True(t, certain, tc.name)
		require.Equal(t, tc.want, got, tc.name)
		requireSameOrientation(t, p, tc.name)
	}
}
