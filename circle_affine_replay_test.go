package decad

import (
	"context"
	"errors"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

func TestCircleExactAffineAllowance(t *testing.T) {
	for _, mode := range []string{"rotation", "inverse", "translation"} {
		t.Run(mode, func(t *testing.T) {
			translationOnly := mode == "translation"
			w := sketch.NewWorld()
			f, err := r3.NewFrame(r3.Vec{X: 7.1, Y: -3.4, Z: 2.3}, r3.Vec{X: .6, Y: .8}, r3.Vec{X: -.8, Y: .6})
			if err != nil {
				t.Fatal(err)
			}
			plane, err := w.CreatePlaneFromFrame(f)
			if err != nil {
				t.Fatal(err)
			}
			sk, err := w.CreateSketch(plane)
			if err != nil {
				t.Fatal(err)
			}
			c := sk.CreatePoint(1, -2)
			sk.Fix(c)
			sk.CreateCircle(c, 5)
			if _, err = sk.Solve(t.Context()); err != nil {
				t.Fatal(err)
			}
			doc := New()
			base, err := Extrude(t.Context(), doc, sk, sk.Profiles()[0], Distance{D: units.Millimeters(4), Dir: Along})
			if err != nil {
				t.Fatal(err)
			}
			first, err := r3.Translation(r3.Vec{X: .1, Y: -.2, Z: .3})
			if err != nil {
				t.Fatal(err)
			}
			second, err := r3.Translation(r3.Vec{X: .2, Y: .3, Z: -.1})
			if err != nil {
				t.Fatal(err)
			}
			if !translationOnly {
				angle := .37
				if mode == "inverse" {
					angle = .7
				}
				first, err = r3.Rotation(r3.Vec{X: 1, Y: 2, Z: 3}, units.Radians(angle))
				if err != nil {
					t.Fatal(err)
				}
				secondAngle := -.23
				if mode == "inverse" {
					secondAngle = -.9
				}
				second, err = r3.Rotation(r3.Vec{X: -2, Y: 3, Z: 1}, units.Radians(secondAngle))
				if err != nil {
					t.Fatal(err)
				}
			}
			once, err := Transform(t.Context(), base, first)
			if err != nil {
				t.Fatal(err)
			}
			placed, err := Transform(t.Context(), once, second)
			if err != nil {
				t.Fatal(err)
			}
			b := proofbound.NewCircleReplayBudget()
			s, err := captureCircleSource(t.Context(), doc, placed, b)
			if err != nil {
				t.Fatal(err)
			}
			d, linear, err := s.exactAffineAllowance(t.Context(), b)
			if err != nil {
				t.Fatal(err)
			}
			if translationOnly && (linear != 0 || !(d > 0)) {
				t.Fatalf("translation d=%g linear=%g", d, linear)
			}
			p := placed.n.Payload.(prismrecord.Payload)
			for _, point := range []r3.Vec{{}, {X: 6, Y: -2, Z: 4}, {X: -4, Y: -2}, {X: 1, Y: 3, Z: 4}} {
				if !affineOracleWithin(p.Frame, []r3.Transform{first, second}, p.Xform, point, d, false) {
					t.Fatalf("point outside displacement bound: %v", point)
				}
			}
			for _, axis := range []r3.Vec{{X: 1}, {Y: 1}, {Z: 1}} {
				if !affineOracleWithin(p.Frame, []r3.Transform{first, second}, p.Xform, axis, linear, true) {
					t.Fatal("linear map outside bound")
				}
			}
			_, lift, _, err := massmoment.ReplayPrismChord(t.Context(), p.Frame, p.Xform, .2, b)
			if err != nil {
				t.Fatal(err)
			}
			actual := affineOracleColumns(p.Frame, []r3.Transform{first, second})
			held := affineOracleColumns(p.Frame, []r3.Transform{p.Xform})
			areaFactor := proofbound.ProductUpper(linear, proofbound.AbsSumUpper(proofbound.ProductUpper(2, lift), linear))
			areaLimit := new(big.Rat).SetFloat64(areaFactor)
			areaLimit.Mul(areaLimit, areaLimit)
			for _, pair := range [][2]int{{0, 1}, {1, 2}, {2, 0}} {
				actualCross := affineOracleCross(actual, pair[0], pair[1])
				heldCross := affineOracleCross(held, pair[0], pair[1])
				difference := new(big.Rat)
				for i := range 3 {
					d := new(big.Rat).Sub(actualCross[i], heldCross[i])
					difference.Add(difference, new(big.Rat).Mul(d, d))
				}
				if difference.Cmp(areaLimit) > 0 {
					t.Fatal("area allowance does not contain cross-product difference")
				}
			}
			detDifference := new(big.Rat).Sub(affineOracleDet(actual), affineOracleDet(held))
			t.Logf("determinant difference sign=%d", detDifference.Sign())
			if (mode == "rotation" && detDifference.Sign() != 1) || (mode == "inverse" && detDifference.Sign() != -1) || (mode == "translation" && detDifference.Sign() != 0) {
				t.Fatal("fixture did not exercise the expected determinant sign")
			}
			detDifference.Abs(detDifference)
			// Divide the cylinder volume inequality by πr: r*h*|Δdet| ≤ D*(L+δ)²*2(r+h).
			left := new(big.Rat).Mul(detDifference, big.NewRat(20, 1))
			upper := proofbound.ProductUpper(d, proofbound.ProductUpper(proofbound.ProductUpper(proofbound.AbsSumUpper(lift, linear), proofbound.AbsSumUpper(lift, linear)), 18))
			if left.Cmp(new(big.Rat).SetFloat64(upper)) > 0 {
				t.Fatal("volume allowance does not contain determinant difference")
			}
			view, err := s.circleReplayView(t.Context(), b)
			if err != nil {
				t.Fatal(err)
			}
			mesh, err := publicCircleCandidate(t.Context(), placed, b)
			if err != nil {
				t.Fatal(err)
			}
			beforeArea, beforeVolume := mesh.AreaSlack, mesh.VolSymDiff
			if err := widenCirclePrimitive(t.Context(), view, mesh, lift, d, linear, b); err != nil {
				t.Fatal(err)
			}
			rat := func(v float64) *big.Rat { return new(big.Rat).SetFloat64(v) }
			multiply := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }
			localAreaUpper := multiply(big.NewRat(90, 1), proofbound.PiUpper)
			areaFactorExact := multiply(rat(linear), new(big.Rat).Add(multiply(big.NewRat(2, 1), rat(lift)), rat(linear)))
			pathLift := new(big.Rat).Add(rat(lift), rat(linear))
			volumeFactorExact := multiply(rat(d), multiply(pathLift, pathLift))
			areaIncrease := new(big.Rat).Sub(rat(mesh.AreaSlack), rat(beforeArea))
			volumeIncrease := new(big.Rat).Sub(rat(mesh.VolSymDiff), rat(beforeVolume))
			if areaIncrease.Cmp(multiply(areaFactorExact, localAreaUpper)) < 0 || volumeIncrease.Cmp(multiply(volumeFactorExact, localAreaUpper)) < 0 {
				t.Fatal("candidate widening did not contain exact area or volume allowance")
			}
			if mode == "translation" && (d <= 0 || linear != 0 || volumeIncrease.Sign() <= 0) {
				t.Fatal("translation swept volume was not charged")
			}
			ctx, cancel := context.WithCancel(t.Context())
			cancel()
			if _, _, err := s.exactAffineAllowance(ctx, b); !errors.Is(err, context.Canceled) {
				t.Fatalf("cancel: %v", err)
			}
		})
	}
}

func affineOracleWithin(frame r3.Frame, steps []r3.Transform, held r3.Transform, point r3.Vec, bound float64, linear bool) bool {
	rat := func(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }
	vals := func(v r3.Vec) [3]float64 { return [3]float64{v.X, v.Y, v.Z} }
	apply := func(cols [3]r3.Vec, origin r3.Vec, p [3]*big.Rat, translate bool) [3]*big.Rat {
		var out [3]*big.Rat
		o := vals(origin)
		for i := range 3 {
			out[i] = rat(0)
			if translate {
				out[i].Set(rat(o[i]))
			}
			for j, col := range cols {
				v := vals(col)
				out[i].Add(out[i], new(big.Rat).Mul(rat(v[i]), p[j]))
			}
		}
		return out
	}
	v := vals(point)
	input := [3]*big.Rat{rat(v[0]), rat(v[1]), rat(v[2])}
	local := apply([3]r3.Vec{frame.U(), frame.V(), frame.N()}, frame.Origin(), input, !linear)
	exact := local
	for _, step := range steps {
		b := step.Basis()
		exact = apply([3]r3.Vec{b.EX, b.EY, b.EZ}, step.Translation(), exact, !linear)
	}
	b := held.Basis()
	got := apply([3]r3.Vec{b.EX, b.EY, b.EZ}, held.Translation(), local, !linear)
	sum := rat(0)
	for i := range 3 {
		q := new(big.Rat).Sub(exact[i], got[i])
		sum.Add(sum, new(big.Rat).Mul(q, q))
	}
	r := rat(bound)
	return sum.Cmp(r.Mul(r, r)) <= 0
}

func TestCircleAffineLargeRadius(t *testing.T) {
	var displacement [2]float64
	var linear [2]float64
	for i, radius := range []float64{5, 1e6} {
		w := sketch.NewWorld()
		frame, err := r3.NewFrame(r3.Vec{X: 7.1, Y: -3.4, Z: 2.3}, r3.Vec{X: .6, Y: .8}, r3.Vec{X: -.8, Y: .6})
		if err != nil {
			t.Fatal(err)
		}
		plane, err := w.CreatePlaneFromFrame(frame)
		if err != nil {
			t.Fatal(err)
		}
		sk, err := w.CreateSketch(plane)
		if err != nil {
			t.Fatal(err)
		}
		center := sk.CreatePoint(1, -2)
		sk.Fix(center)
		sk.CreateCircle(center, radius)
		if _, err = sk.Solve(t.Context()); err != nil {
			t.Fatal(err)
		}
		doc := New()
		body, err := Extrude(t.Context(), doc, sk, sk.Profiles()[0], Distance{D: units.Millimeters(4), Dir: Along})
		if err != nil {
			t.Fatal(err)
		}
		a, err := r3.Rotation(r3.Vec{X: 1, Y: 2, Z: 3}, units.Radians(.37))
		if err != nil {
			t.Fatal(err)
		}
		b, err := r3.Rotation(r3.Vec{X: -2, Y: 3, Z: 1}, units.Radians(-.23))
		if err != nil {
			t.Fatal(err)
		}
		body, err = Transform(t.Context(), body, a)
		if err != nil {
			t.Fatal(err)
		}
		body, err = Transform(t.Context(), body, b)
		if err != nil {
			t.Fatal(err)
		}
		budget := proofbound.NewCircleReplayBudget()
		source, err := captureCircleSource(t.Context(), doc, body, budget)
		if err != nil {
			t.Fatal(err)
		}
		displacement[i], linear[i], err = source.exactAffineAllowance(t.Context(), budget)
		if err != nil {
			t.Fatal(err)
		}
		p := body.n.Payload.(prismrecord.Payload)
		if !affineOracleWithin(p.Frame, []r3.Transform{a, b}, p.Xform, r3.Vec{X: 1 + radius, Y: -2, Z: 4}, displacement[i], false) {
			t.Fatal("large-radius endpoint escaped displacement")
		}
	}
	if !(linear[0] > 0) || linear[0] != linear[1] || !(displacement[1] > displacement[0]) {
		t.Fatalf("linear=%v displacement=%v", linear, displacement)
	}
}

// affineOracleColumns computes complete linear maps using independent rational operations.
func affineOracleColumns(frame r3.Frame, steps []r3.Transform) [3][3]*big.Rat {
	rat := func(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }
	vectors := [3]r3.Vec{frame.U(), frame.V(), frame.N()}
	var matrix [3][3]*big.Rat
	for j, v := range vectors {
		xyz := [3]float64{v.X, v.Y, v.Z}
		for i := range 3 {
			matrix[i][j] = rat(xyz[i])
		}
	}
	for _, step := range steps {
		basis := step.Basis()
		columns := [3]r3.Vec{basis.EX, basis.EY, basis.EZ}
		var next [3][3]*big.Rat
		for i := range 3 {
			for j := range 3 {
				next[i][j] = rat(0)
				for k, v := range columns {
					xyz := [3]float64{v.X, v.Y, v.Z}
					next[i][j].Add(next[i][j], new(big.Rat).Mul(rat(xyz[i]), matrix[k][j]))
				}
			}
		}
		matrix = next
	}
	return matrix
}

func affineOracleDet(m [3][3]*big.Rat) *big.Rat {
	prod := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }
	a := new(big.Rat).Sub(prod(m[1][1], m[2][2]), prod(m[1][2], m[2][1]))
	b := new(big.Rat).Sub(prod(m[1][0], m[2][2]), prod(m[1][2], m[2][0]))
	c := new(big.Rat).Sub(prod(m[1][0], m[2][1]), prod(m[1][1], m[2][0]))
	return new(big.Rat).Add(new(big.Rat).Sub(prod(m[0][0], a), prod(m[0][1], b)), prod(m[0][2], c))
}

func affineOracleCross(m [3][3]*big.Rat, a, b int) [3]*big.Rat {
	var out [3]*big.Rat
	for i := range 3 {
		j, k := (i+1)%3, (i+2)%3
		out[i] = new(big.Rat).Sub(new(big.Rat).Mul(m[j][a], m[k][b]), new(big.Rat).Mul(m[k][a], m[j][b]))
	}
	return out
}
