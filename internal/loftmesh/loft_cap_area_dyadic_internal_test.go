package loftmesh

import (
	"math"
	"math/big"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

func TestCapLoopAreaSignMatchesRational(t *testing.T) {
	makeData := func(points ...r3.Vec) *LoftAuditData {
		data := &LoftAuditData{Xverts: make([]proof.Xpt, len(points))}
		for i, p := range points {
			data.Xverts[i] = proof.XptOf(p)
		}
		return data
	}
	check := func(name string, data *LoftAuditData, loop []int) {
		t.Helper()
		before := make([][4]string, len(data.Xverts))
		for i, p := range data.Xverts {
			before[i] = [4]string{p.X.String(), p.Y.String(), p.Z.String(), p.W.String()}
		}
		reversed := slices.Clone(loop)
		slices.Reverse(reversed)
		for _, axes := range [][2]int{{0, 1}, {0, 2}, {1, 2}} {
			for _, winding := range [][]int{loop, reversed} {
				got := capLoopAreaSign(data, winding, axes[0], axes[1])
				want := capLoopAreaSignRat(data, winding, axes[0], axes[1])
				if got != want {
					t.Fatalf("%s axes=%v loop=%v: sign=%d, rational=%d", name, axes, winding, got, want)
				}
			}
		}
		for i, p := range data.Xverts {
			if after := [4]string{p.X.String(), p.Y.String(), p.Z.String(), p.W.String()}; after != before[i] {
				t.Fatalf("%s vertex %d changed: %v to %v", name, i, before[i], after)
			}
		}
	}

	check("empty", &LoftAuditData{}, nil)
	check("bowtie", makeData(
		r3.NewVec(0, 0, 7), r3.NewVec(1, 1, 7), r3.NewVec(0, 1, 7), r3.NewVec(1, 0, 7),
	), []int{0, 1, 2, 3})
	check("subnormal", makeData(
		r3.NewVec(math.SmallestNonzeroFloat64, 0, 1),
		r3.NewVec(0, math.SmallestNonzeroFloat64, 1),
		r3.NewVec(-math.SmallestNonzeroFloat64, 0, 1),
	), []int{0, 1, 2})
	check("large", makeData(
		r3.NewVec(math.MaxFloat64, 0, -2), r3.NewVec(0, math.MaxFloat64, -1),
		r3.NewVec(-math.MaxFloat64, 0, 0),
	), []int{0, 1, 2})

	den := new(big.Int).Lsh(big.NewInt(3), 300)
	nondyadic := &LoftAuditData{Xverts: []proof.Xpt{
		{X: big.NewInt(1), Y: big.NewInt(0), Z: big.NewInt(1), W: new(big.Int).Set(den)},
		{X: big.NewInt(0), Y: big.NewInt(2), Z: big.NewInt(1), W: new(big.Int).Set(den)},
		{X: big.NewInt(-3), Y: big.NewInt(0), Z: big.NewInt(1), W: new(big.Int).Set(den)},
	}}
	check("odd-denominator", nondyadic, []int{0, 1, 2})

	rng := rand.New(rand.NewPCG(747, 991))
	for range 200 {
		points := make([]r3.Vec, 3+rng.IntN(12))
		loop := make([]int, len(points))
		for i := range points {
			coord := func() float64 { return math.Ldexp(float64(rng.IntN(17)-8), -rng.IntN(250)) }
			points[i] = r3.NewVec(coord(), coord(), coord())
			loop[i] = i
		}
		check("random", makeData(points...), loop)
	}
}

func BenchmarkCapLoopAreaSign(b *testing.B) {
	const vertices = 281
	data := &LoftAuditData{Xverts: make([]proof.Xpt, vertices)}
	loop := make([]int, vertices)
	for i := range vertices {
		angle := 2 * math.Pi * float64(i) / vertices
		data.Xverts[i] = proof.XptOf(r3.NewVec(10*math.Cos(angle), 10*math.Sin(angle), 7))
		loop[i] = i
	}
	for _, variant := range []struct {
		name string
		sign func(*LoftAuditData, []int, int, int) int
	}{{"dyadic", capLoopAreaSign}, {"rational", capLoopAreaSignRat}} {
		b.Run(variant.name, func(b *testing.B) {
			for b.Loop() {
				if variant.sign(data, loop, 0, 1) == 0 {
					b.Fatal("zero area")
				}
			}
		})
	}
}
