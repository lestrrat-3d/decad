package occupancy

import (
	"cmp"
	"context"
	"fmt"
	"math"
	"math/big"
	"runtime"
	"slices"
	"sync"
	"sync/atomic"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
)

// Budget counts exact work: one unit per exact projection of one vertex on
// one axis. A certification that would exceed Limit refuses with
// ErrUnsupported instead of running on.
// Charges are atomic, so one Budget may serve concurrent facet searches.
type Budget struct {
	Limit int64
	used  atomic.Int64
}

// Used reports the work charged so far.
func (b *Budget) Used() int64 { return b.used.Load() }

func (b *Budget) charge(n int64) error {
	if b.used.Add(n) > b.Limit {
		return fmt.Errorf("%w: occupancy certificate exceeds its fixed work ceiling of %d", decaderr.ErrUnsupported, b.Limit)
	}
	return nil
}

// maxExactTries is how many float-ranked axes one Misses call verifies
// exactly before it answers false.
const maxExactTries = 3

// Misses reports whether the triangle tri, moved by shift, is proven disjoint
// from p. Float arithmetic ranks candidate separating axes (the triangle's
// normal, the piece's face normals, and every cross product of a triangle
// edge with a piece edge); a candidate is accepted only when the exact
// projections of the moved triangle's rational vertices and the piece's
// rational vertices onto it do not overlap. False means "not proven", never
// "intersects".
func Misses(tri [3]r3.Vec, shift r3.Vec, p *Piece, budget *Budget) (bool, error) {
	var ft [3]r3.Vec
	for i, v := range tri {
		ft[i] = v.Add(shift)
	}
	ok, err := missesBySide(tri, shift, ft, p, budget)
	if ok || err != nil {
		return ok, err
	}
	type cand struct {
		axis r3.Vec
		gap  float64
	}
	var cands []cand
	try := func(a r3.Vec) {
		l := a.Len()
		if !(l > 0) || math.IsInf(l, 0) {
			return
		}
		a = a.Scale(1 / l)
		tlo, thi := math.Inf(1), math.Inf(-1)
		for _, v := range ft {
			d := a.Dot(v)
			tlo, thi = math.Min(tlo, d), math.Max(thi, d)
		}
		plo, phi := math.Inf(1), math.Inf(-1)
		for _, v := range p.fverts {
			d := a.Dot(v)
			plo, phi = math.Min(plo, d), math.Max(phi, d)
		}
		if g := math.Max(plo-thi, tlo-phi); g > 0 {
			cands = append(cands, cand{a, g})
		}
	}
	try(ft[1].Sub(ft[0]).Cross(ft[2].Sub(ft[0])))
	for _, f := range p.faces {
		try(f.n)
	}
	if len(cands) == 0 {
		// Only an edge-edge axis can separate what no face normal does.
		for i := range 3 {
			te := ft[(i+1)%3].Sub(ft[i])
			for _, e := range p.edges {
				try(te.Cross(p.fverts[e[1]].Sub(p.fverts[e[0]])))
			}
		}
	}
	if len(cands) == 0 {
		return false, nil
	}
	slices.SortFunc(cands, func(a, b cand) int { return cmp.Compare(b.gap, a.gap) })
	sx := xvecOf(shift)
	var xt [3]xvec
	for i, v := range tri {
		x := xvecOf(v)
		for k := range 3 {
			x[k].Add(x[k], sx[k])
		}
		xt[i] = x
	}
	for _, c := range cands[:min(len(cands), maxExactTries)] {
		if err := budget.charge(int64(3 + len(p.verts))); err != nil {
			return false, err
		}
		if separated(c.axis, xt, p.verts) {
			return true, nil
		}
	}
	return false, nil
}

// missesBySide tries the cheap certificate first: one of the piece's own
// half-spaces (a side or a level, in local coordinates) that all three moved
// corners lie strictly outside. Floats rank the half-spaces; the exact local
// coordinates of the moved corners decide.
func missesBySide(tri [3]r3.Vec, shift r3.Vec, ft [3]r3.Vec, p *Piece, budget *Budget) (bool, error) {
	var lf [3]r3.Vec
	for i, v := range ft {
		for _, a := range p.finv {
			v = a.applyFloat(v)
		}
		lf[i] = v
	}
	best, bestGap := -1, 0.0
	gapOf := func(f func(r3.Vec) float64) float64 {
		g := math.Inf(1)
		for _, v := range lf {
			g = math.Min(g, f(v))
		}
		return g
	}
	for k, sd := range p.sides {
		if g := gapOf(func(v r3.Vec) float64 { return sd.fux*v.X + sd.fuy*v.Y - sd.fc }); g > bestGap {
			best, bestGap = k, g
		}
	}
	if g := gapOf(func(v r3.Vec) float64 { return p.fz0 - v.Z }); g > bestGap {
		best, bestGap = len(p.sides), g
	}
	if g := gapOf(func(v r3.Vec) float64 { return v.Z - p.fz1 }); g > bestGap {
		best = len(p.sides) + 1
	}
	if best < 0 {
		return false, nil
	}
	if err := budget.charge(3); err != nil {
		return false, err
	}
	sx := xvecOf(shift)
	for _, v := range tri {
		x := xvecOf(v)
		for k := range 3 {
			x[k].Add(x[k], sx[k])
		}
		for _, e := range p.inv {
			x = e.apply(x)
		}
		var beyond bool
		switch {
		case best < len(p.sides):
			sd := p.sides[best]
			d := new(big.Rat).Mul(sd.ux, x[0])
			d.Add(d, new(big.Rat).Mul(sd.uy, x[1]))
			beyond = d.Cmp(sd.c) > 0
		case best == len(p.sides):
			beyond = x[2].Cmp(p.z0) < 0
		default:
			beyond = x[2].Cmp(p.z1) > 0
		}
		if !beyond {
			return false, nil
		}
	}
	return true, nil
}

func separated(axis r3.Vec, tri [3]xvec, verts []xvec) bool {
	var tlo, thi, plo, phi *big.Rat
	for _, v := range tri {
		d := dotExact(axis, v)
		if tlo == nil || d.Cmp(tlo) < 0 {
			tlo = d
		}
		if thi == nil || d.Cmp(thi) > 0 {
			thi = d
		}
	}
	for _, v := range verts {
		d := dotExact(axis, v)
		if plo == nil || d.Cmp(plo) < 0 {
			plo = d
		}
		if phi == nil || d.Cmp(phi) > 0 {
			phi = d
		}
	}
	return thi.Cmp(plo) < 0 || phi.Cmp(tlo) < 0
}

// Search fixes the facet search of agent-docs/body-refine-design.md §5.
type Search struct {
	// H0 and HMax bound the translation ladder H0·Step^j. A Step at or
	// below 1 reads as 1.25.
	H0, HMax, Step float64
	// Inner are pieces inside the true holes a Cut removed: a moved facet
	// piece wholly inside one of them is outside the true solid.
	Inner []*Piece
	// MaxDepth is how many times a facet piece may be bisected across its
	// longest edge before the search refuses.
	MaxDepth int
}

// facesPerPiece is how many of a near piece's faces propose escape
// directions for one facet.
const facesPerPiece = 3

// shrink keeps a float-scaled unit direction's length at most h even after
// the rounding of its normalization and scaling.
const shrink = 1 - 1e-12

// FacetBound returns a length h such that every piece of tri (bisected at
// most s.MaxDepth times) has a translation of length at most h that misses every
// outer piece: the smallest ladder rung that certifies each piece, plus the
// split slack of every split. Translations are drawn from the facet's outward
// normal, the faces of each near piece the facet is least deep behind, and
// their normalized sums. The caller's tri must be a held facet with
// outward normal outward, or a zero vector when it has none in floats.
// Pieces whose widened float box stays clear of the moved facet piece's
// widened box are missed without exact work.
func FacetBound(tri [3]r3.Vec, outward r3.Vec, pieces []*Piece, s Search, budget *Budget) (float64, error) {
	// A sliver whose float cross product vanishes has no float outward
	// direction; it then draws its translations from the near pieces alone.
	// The direction only proposes: Misses decides every translation exactly.
	n, hasNormal := outward.Normalize()
	lo, hi := triBox(tri)
	var near []*Piece
	for _, p := range pieces {
		if boxesNear(lo, hi, p.box, s.HMax) {
			near = append(near, p)
		}
	}
	g := tri[0].Add(tri[1]).Add(tri[2]).Scale(1.0 / 3)
	// Per near piece, the faces the facet's centroid is least deep behind
	// are the ones a short translation can cross.
	var perPiece [][]r3.Vec
	for _, p := range near {
		ranked := append([]face(nil), p.faces...)
		slices.SortFunc(ranked, func(a, b face) int {
			return cmp.Compare(a.off-a.n.Dot(g), b.off-b.n.Dot(g))
		})
		var ds []r3.Vec
		for _, f := range ranked[:min(len(ranked), facesPerPiece)] {
			ds = append(ds, f.n)
		}
		perPiece = append(perPiece, ds)
	}
	var dirs []r3.Vec
	if hasNormal {
		dirs = append(dirs, n)
	}
	add := func(v r3.Vec) {
		if d, ok := v.Normalize(); ok {
			dirs = append(dirs, d)
		}
	}
	for i, ds := range perPiece {
		for _, d := range ds {
			add(d)
			add(d.Add(n))
			for _, other := range perPiece[i+1:] {
				for _, e := range other {
					add(d.Add(e))
				}
			}
		}
	}
	var inner []*Piece
	for _, p := range s.Inner {
		if boxesNear(lo, hi, p.box, s.HMax) {
			inner = append(inner, p)
		}
	}
	s.Inner = inner
	return pieceBound(tri, dirs, near, s, 0, budget)
}

func pieceBound(tri [3]r3.Vec, dirs []r3.Vec, near []*Piece, s Search, depth int, budget *Budget) (float64, error) {
	step := s.Step
	if !(step > 1) {
		step = 1.25
	}
	for h := s.H0; h <= s.HMax; h *= step {
		for _, d := range dirs {
			shift := d.Scale(h * shrink)
			ok, err := insideAny(tri, shift, s.Inner, budget)
			if err == nil && !ok {
				ok, err = missesAll(tri, shift, near, budget)
			}
			if err != nil {
				return 0, err
			}
			if ok {
				return h, nil
			}
		}
	}
	if depth >= s.MaxDepth {
		return 0, fmt.Errorf("%w: no translation within %v mm certifies a held facet piece", decaderr.ErrUnsupported, s.HMax)
	}
	// Bisect the longest edge: a held wall facet is a long sliver, and only
	// its length needs splitting.
	k := 0
	for i := 1; i < 3; i++ {
		if tri[(i+1)%3].Sub(tri[i]).Len() > tri[(k+1)%3].Sub(tri[k]).Len() {
			k = i
		}
	}
	a, b, c := tri[k], tri[(k+1)%3], tri[(k+2)%3]
	m := a.Add(b).Scale(0.5)
	// The float midpoint may sit off its edge by rounding, so the two halves
	// can miss a sliver of tri that thin. Every point of such a sliver lies
	// within splitSlack of a covered point, which the bound adds.
	worst := 0.0
	for _, sub := range [2][3]r3.Vec{{a, m, c}, {m, b, c}} {
		h, err := pieceBound(sub, dirs, near, s, depth+1, budget)
		if err != nil {
			return 0, err
		}
		worst = math.Max(worst, h)
	}
	return worst + splitSlack(tri), nil
}

// insideAny reports whether tri moved by shift lies wholly inside one of
// inner: its three corners inside a convex piece, decided exactly.
func insideAny(tri [3]r3.Vec, shift r3.Vec, inner []*Piece, budget *Budget) (bool, error) {
	for _, p := range inner {
		if err := budget.charge(int64(3 * (len(p.sides) + 2))); err != nil {
			return false, err
		}
		if p.containsMoved(tri, shift) {
			return true, nil
		}
	}
	return false, nil
}

func missesAll(tri [3]r3.Vec, shift r3.Vec, near []*Piece, budget *Budget) (bool, error) {
	var moved [3]r3.Vec
	for i, v := range tri {
		moved[i] = v.Add(shift)
	}
	lo, hi := triBox(moved)
	for _, p := range near {
		if !boxesNear(lo, hi, p.box, 0) {
			continue
		}
		ok, err := Misses(tri, shift, p, budget)
		if err != nil || !ok {
			return false, err
		}
	}
	return true, nil
}

// splitSlack bounds the distance from a point of tri to the two float halves
// its longest-edge bisection covers: the midpoint's coordinates are each
// within half an ulp of the exact midpoint, so eight ulps of the largest
// coordinate magnitude cover the uncovered sliver with room to spare.
func splitSlack(tri [3]r3.Vec) float64 {
	m := 0.0
	for _, v := range tri {
		m = math.Max(m, math.Max(math.Abs(v.X), math.Max(math.Abs(v.Y), math.Abs(v.Z))))
	}
	return 8 * (math.Nextafter(m, math.Inf(1)) - m)
}

// triBox is a float box that contains the exact triangle: each corner's float
// sum is within half an ulp of its exact value, so one ulp outward covers it.
func triBox(t [3]r3.Vec) (r3.Vec, r3.Vec) {
	lo, hi := t[0], t[0]
	for _, v := range t[1:] {
		lo = r3.Vec{X: math.Min(lo.X, v.X), Y: math.Min(lo.Y, v.Y), Z: math.Min(lo.Z, v.Z)}
		hi = r3.Vec{X: math.Max(hi.X, v.X), Y: math.Max(hi.Y, v.Y), Z: math.Max(hi.Z, v.Z)}
	}
	inf := math.Inf(1)
	lo = r3.Vec{X: math.Nextafter(lo.X, -inf), Y: math.Nextafter(lo.Y, -inf), Z: math.Nextafter(lo.Z, -inf)}
	hi = r3.Vec{X: math.Nextafter(hi.X, inf), Y: math.Nextafter(hi.Y, inf), Z: math.Nextafter(hi.Z, inf)}
	return lo, hi
}

func boxesNear(lo, hi r3.Vec, b [2]r3.Vec, pad float64) bool {
	return lo.X-pad <= b[1].X && b[0].X <= hi.X+pad &&
		lo.Y-pad <= b[1].Y && b[0].Y <= hi.Y+pad &&
		lo.Z-pad <= b[1].Z && b[0].Z <= hi.Z+pad
}

// Certify runs FacetBound over the facets of the held mesh (verts, tris) that
// want lists, against the record's pieces, on up to GOMAXPROCS workers, and
// returns each listed facet's h in the order of want. The first refusal or
// cancellation stops every worker and is returned.
func Certify(ctx context.Context, verts []r3.Vec, tris [][3]int, want []int, pieces []*Piece, s Search, budget *Budget) ([]float64, error) {
	out := make([]float64, len(want))
	var next atomic.Int64
	var once sync.Once
	var first error
	stop := make(chan struct{})
	fail := func(err error) {
		once.Do(func() {
			first = err
			close(stop)
		})
	}
	var wg sync.WaitGroup
	for range max(1, min(runtime.GOMAXPROCS(0), len(want))) {
		wg.Go(func() {
			for {
				i := int(next.Add(1) - 1)
				if i >= len(want) {
					return
				}
				select {
				case <-stop:
					return
				default:
				}
				if err := ctx.Err(); err != nil {
					fail(err)
					return
				}
				t := tris[want[i]]
				tri := [3]r3.Vec{verts[t[0]], verts[t[1]], verts[t[2]]}
				h, err := FacetBound(tri, tri[1].Sub(tri[0]).Cross(tri[2].Sub(tri[0])), pieces, s, budget)
				if err != nil {
					fail(err)
					return
				}
				out[i] = h
			}
		})
	}
	wg.Wait()
	if first != nil {
		return nil, first
	}
	return out, nil
}
