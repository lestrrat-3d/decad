package retainedblend

import (
	"errors"
	"fmt"
	"math"
	"math/rand/v2"
	"reflect"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// earClipSorted is the ear selection earClip used before it kept its ears in a heap: it rebuilds the ear list on every
// iteration and sorts it whenever the best ear is blocked. It is the oracle for the differential test.
func earClipSorted(ps []pt2, poly []int) ([][3]int, error) {
	idx := append([]int(nil), poly...)
	tris := make([][3]int, 0, len(idx))
	var ears []planarEar
	turns := make([]int, len(idx))
	qualities := make([]float64, len(idx))
	update := func(at int) {
		n := len(idx)
		a, b, c := ps[idx[(at+n-1)%n]], ps[idx[at]], ps[idx[(at+1)%n]]
		turns[at] = orient(a, b, c)
		if turns[at] > 0 {
			qualities[at] = earQuality(a, b, c)
		}
	}
	for i := range idx {
		update(i)
	}
	for len(idx) > 3 {
		ears = ears[:0]
		n := len(idx)
		for i := range n {
			if turns[i] <= 0 {
				continue
			}
			ears = append(ears, planarEar{i, qualities[i]})
		}
		best, ok := bestPlanarEar(ears)
		if !ok {
			return nil, fmt.Errorf("%w: the face's boundary has no ear to clip", decaderr.ErrUnsupported)
		}
		ia, ib, ic := idx[(best.at+n-1)%n], idx[best.at], idx[(best.at+1)%n]
		if earBlocked(ps, idx, ia, ib, ic) {
			sortPlanarEars(ears)
			found := false
			for _, e := range ears[1:] {
				ia, ib, ic = idx[(e.at+n-1)%n], idx[e.at], idx[(e.at+1)%n]
				if earBlocked(ps, idx, ia, ib, ic) {
					continue
				}
				best = e
				found = true
				break
			}
			if !found {
				return nil, fmt.Errorf("%w: the face's boundary has no ear to clip", decaderr.ErrUnsupported)
			}
		}
		tris = append(tris, [3]int{ia, ib, ic})
		idx = append(idx[:best.at], idx[best.at+1:]...)
		turns = append(turns[:best.at], turns[best.at+1:]...)
		qualities = append(qualities[:best.at], qualities[best.at+1:]...)
		if len(idx) > 3 {
			update((best.at + len(idx) - 1) % len(idx))
			update(best.at % len(idx))
		}
	}
	if orient(ps[idx[0]], ps[idx[1]], ps[idx[2]]) <= 0 {
		return nil, fmt.Errorf("%w: the face's boundary closes on a degenerate triangle", decaderr.ErrUnsupported)
	}
	return append(tris, [3]int{idx[0], idx[1], idx[2]}), nil
}

// earClipOutcome is everything a call to an ear clipper can show its caller.
type earClipOutcome struct {
	tris     [][3]int
	err      string
	errKind  bool
	panicked bool
	panicVal string
}

func runEarClip(f func([]pt2, []int) ([][3]int, error), ps []pt2, poly []int) (out earClipOutcome) {
	defer func() {
		if r := recover(); r != nil {
			out = earClipOutcome{panicked: true, panicVal: fmt.Sprint(r)}
		}
	}()
	tris, err := f(ps, append([]int(nil), poly...))
	out.tris = tris
	if err != nil {
		out.err = err.Error()
		out.errKind = isUnsupported(err)
	}
	return out
}

func isUnsupported(err error) bool { return errors.Is(err, decaderr.ErrUnsupported) }

func sameEarClipOutcome(a, b earClipOutcome) bool { return reflect.DeepEqual(a, b) }

// earClipPolygon is a random input for the differential test.
type earClipPolygon struct {
	name string
	ps   []pt2
	poly []int
}

func identityLoop(n int) []int {
	poly := make([]int, n)
	for i := range poly {
		poly[i] = i
	}
	return poly
}

func scalePoints(ps []pt2, k float64) {
	for i := range ps {
		ps[i].u *= k
		ps[i].w *= k
	}
}

// earClipPolygons returns counter-clockwise and arbitrary polygons that stress the ear order: ties in quality,
// collinear runs, repeated vertices, extreme scales and many blocked ears.
func earClipPolygons(rng *rand.Rand, count, maxN int) []earClipPolygon {
	var out []earClipPolygon
	for len(out) < count {
		n := 3 + rng.IntN(maxN-2)
		var p earClipPolygon
		switch kind := rng.IntN(9); kind {
		case 0: // convex, equal radii
			p.name = "regular"
			for i := range n {
				a := 2 * math.Pi * float64(i) / float64(n)
				p.ps = append(p.ps, pt2{math.Cos(a), math.Sin(a)})
			}
		case 1: // star: radius alternates, so many ears are reflex or blocked
			p.name = "star"
			for i := range n {
				a := 2 * math.Pi * float64(i) / float64(n)
				r := 1.0
				if i%2 == 1 {
					r = 0.2 + 0.7*rng.Float64()
				}
				p.ps = append(p.ps, pt2{r * math.Cos(a), r * math.Sin(a)})
			}
		case 2: // star-shaped with random radii
			p.name = "random star"
			for i := range n {
				a := 2 * math.Pi * float64(i) / float64(n)
				r := 0.1 + rng.Float64()
				p.ps = append(p.ps, pt2{r * math.Cos(a), r * math.Sin(a)})
			}
		case 3: // grid points on a square boundary: collinear runs and equal qualities
			p.name = "square runs"
			side := max(n/4, 1)
			for i := range side {
				p.ps = append(p.ps, pt2{float64(i), 0})
			}
			for i := range side {
				p.ps = append(p.ps, pt2{float64(side), float64(i)})
			}
			for i := range side {
				p.ps = append(p.ps, pt2{float64(side - i), float64(side)})
			}
			for i := range side {
				p.ps = append(p.ps, pt2{0, float64(side - i)})
			}
		case 4: // comb: deep teeth that block the ears across the gap
			p.name = "comb"
			teeth := max(n/4, 2)
			for i := range teeth {
				x := float64(2 * i)
				p.ps = append(p.ps, pt2{x, 0})
			}
			p.ps = append(p.ps, pt2{float64(2 * teeth), 0})
			for i := teeth - 1; i >= 0; i-- {
				x := float64(2*i) + 1
				p.ps = append(p.ps, pt2{x + 1, 5}, pt2{x, 5}, pt2{x, 1 + float64(i%3)*0.5})
			}
		case 5: // random integer points: usually not simple, which every refusal path must treat alike
			p.name = "random grid"
			for range n {
				p.ps = append(p.ps, pt2{float64(rng.IntN(6)), float64(rng.IntN(6))})
			}
		case 6: // nearly collinear boundary run
			p.name = "near collinear"
			for i := range n - 1 {
				p.ps = append(p.ps, pt2{float64(i), float64(rng.IntN(3)-1) * 3e-15})
			}
			p.ps = append(p.ps, pt2{float64(n) / 2, 8})
		case 7: // random floats
			p.name = "random float"
			for range n {
				p.ps = append(p.ps, pt2{rng.NormFloat64(), rng.NormFloat64()})
			}
		default: // spiral: a long chain of ears blocked by the next turn
			p.name = "spiral"
			for i := range n {
				a := 0.5 * float64(i)
				r := 1 + 0.15*float64(i)
				p.ps = append(p.ps, pt2{r * math.Cos(a), r * math.Sin(a)})
			}
		}
		n = len(p.ps)
		p.poly = identityLoop(n)
		switch rng.IntN(6) {
		case 0:
			scalePoints(p.ps, 1e-150)
		case 1:
			scalePoints(p.ps, 1e150)
		case 2: // repeated vertices: some slots reuse another slot's vertex id, as the second copy of a bridge does
			for range 1 + rng.IntN(3) {
				p.poly[rng.IntN(n)] = rng.IntN(n)
			}
		case 3: // clockwise and rotated
			for i, j := 0, n-1; i < j; i, j = i+1, j-1 {
				p.poly[i], p.poly[j] = p.poly[j], p.poly[i]
			}
		case 4: // rotated start
			k := rng.IntN(n)
			p.poly = append(p.poly[k:], p.poly[:k]...)
		}
		out = append(out, p)
	}
	return out
}

// gearLoop is a gear-like polygon of n points: a ring of flat teeth with collinear runs, as a cutter's cap has.
func gearLoop(n int) []pt2 {
	ps := make([]pt2, n)
	for i := range ps {
		a := 2 * math.Pi * float64(i) / float64(n)
		r := 10.0
		if (i/12)%2 == 1 {
			r = 9.2
		}
		ps[i] = pt2{r * math.Cos(a), r * math.Sin(a)}
	}
	return ps
}

func BenchmarkEarClip(b *testing.B) {
	shapes := map[string]func(int) []pt2{"gear": gearLoop, "comb": combLoop}
	for _, c := range []struct {
		shape string
		n     int
	}{{"gear", 300}, {"gear", 1100}, {"comb", 300}, {"comb", 1100}} {
		ps := shapes[c.shape](c.n)
		n := len(ps)
		poly := identityLoop(n)
		b.Run(fmt.Sprintf("%s/heap/%d", c.shape, n), func(b *testing.B) {
			for range b.N {
				if _, err := earClip(ps, poly); err != nil {
					b.Fatal(err)
				}
			}
		})
		b.Run(fmt.Sprintf("%s/sorted/%d", c.shape, n), func(b *testing.B) {
			for range b.N {
				if _, err := earClipSorted(ps, poly); err != nil {
					b.Fatal(err)
				}
			}
		})
	}
}

// combLoop is a comb of about n points whose teeth rise from one base line, so ears between teeth stay blocked.
func combLoop(n int) []pt2 {
	teeth := max(n/4, 2)
	var ps []pt2
	for i := range teeth {
		ps = append(ps, pt2{float64(2 * i), 0})
	}
	ps = append(ps, pt2{float64(2 * teeth), 0})
	for i := teeth - 1; i >= 0; i-- {
		x := float64(2*i) + 1
		ps = append(ps, pt2{x + 1, 5}, pt2{x, 5}, pt2{x, 1 + float64(i%3)*0.5})
	}
	return ps
}
