package decad

import (
	"math"

	"github.com/lestrrat-3d/r3"
)

// This file is the pair kernel of docs/clearance-design.md §1/§2/§5/§6/§7:
// one non-mutating pass over a pair of proven solids that proves the
// partition (disjoint interiors, a certified touching contact, or undecided
// — never a fabricated verdict) and, when asked, measures the gap as a
// proven interval [lo, hi] whose exactness is the interval's, not the
// winning candidate's: Exact exactly when the interval is a point, which
// requires a closed-form winner AND every bracketed rival proven to sit at
// or above it. PR 1 of the §8 increment plan: the tier enumeration with
// exact admission, every CF cell, the P4/P8 certified brackets, the nesting
// exclusion, the coplanar Plane × Plane contact certificate, and the report
// wiring; cone-involved pairs read a coarse enclosure interval, and every
// non-coplanar contact type stays undecided.

// pairVerdict is the kernel's partition answer for one pair.
type pairVerdict int

const (
	// pairUndecided: the kernel can prove the pair neither overlapping nor
	// disjoint — it joins neither report list and reads Suspect
	// (verification §6). Proven-not-disjoint findings (a nesting witness, an
	// admitted crossing) deliberately land here too: an Interference row
	// carries an overlap volume this evaluator cannot yet bound
	// (evaluator §10).
	pairUndecided pairVerdict = iota
	// pairDisjoint: boundary clearance proven positive and nesting excluded.
	pairDisjoint
	// pairTouching: every zero-distance contact certified (the coplanar
	// plane-pair certificate) — the gap is a measured Exact zero (§6).
	pairTouching
)

// pairResult is one pair's kernel answer: the verdict, the proven gap
// interval (when disjoint or touching), and the pair-diameter reading the
// §7 gate needs.
type pairResult struct {
	verdict pairVerdict
	lo, hi  float64
	exact   bool
	diam    float64
}

// pairKernel is one pair's working state.
type pairKernel struct {
	a, b  *bodyGeom
	scale float64
	tol   float64
	slack float64
}

// clearancePair runs the kernel over one pair of proven solids.
// nestingExcluded is true when box separation has already excluded nesting
// (a box-proven pair needs the kernel only for its gap — §7).
func clearancePair(a, b *Body, nestingExcluded bool) pairResult {
	ga, oka := newBodyGeom(a)
	gb, okb := newBodyGeom(b)
	if !oka || !okb {
		return pairResult{}
	}
	scale := 1.0
	for _, bb := range [][2]r3.Vec{{a.bounds.Min, a.bounds.Max}, {b.bounds.Min, b.bounds.Max}} {
		for _, p := range bb {
			for _, c := range []float64{p.X, p.Y, p.Z} {
				if v := math.Abs(c); v > scale {
					scale = v
				}
			}
		}
	}
	k := &pairKernel{a: ga, b: gb, scale: scale, tol: 1e-9 * scale, slack: 1e-9 * scale}
	diam := k.pairDiameter()

	// The coplanar Plane × Plane contact certificate (§6): coplanar caps
	// with strictly opposing outward normals and a positive-area trim
	// overlap, with each body's material wholly on its own side of the
	// shared plane — the separating plane clears the interiors globally and
	// certifies the whole contact set, so the gap is a measured Exact zero.
	if k.coplanarContactCertified() {
		return pairResult{verdict: pairTouching, exact: true, diam: diam}
	}

	sink := k.enumerate()
	if sink.unsure {
		return pairResult{diam: diam}
	}
	hi := math.Inf(1)
	for _, c := range sink.contribs {
		if c.hi < hi {
			hi = c.hi
		}
	}
	if math.IsInf(hi, 1) {
		return pairResult{diam: diam}
	}
	lo := hi
	exact := false
	for _, c := range sink.contribs {
		if c.lo >= hi {
			continue // §5 pruning: a bound at or above the best upper bound cannot hold the minimum
		}
		if c.lo < lo {
			lo = c.lo
		}
	}
	for _, c := range sink.contribs {
		if c.exact && c.lo == hi && c.hi == hi {
			exact = true
		}
	}
	exact = exact && lo == hi
	if lo <= k.tol {
		return pairResult{diam: diam}
	}
	if !nestingExcluded && !k.nestingExcluded() {
		return pairResult{diam: diam}
	}
	return pairResult{verdict: pairDisjoint, lo: lo, hi: hi, exact: exact, diam: diam}
}

// coplanarContactCertified scans the plane-face pairs for the §6 coplanar
// certificate.
func (k *pairKernel) coplanarContactCertified() bool {
	for _, fa := range k.a.faces {
		if fa.kind != ckPlane {
			continue
		}
		for _, fb := range k.b.faces {
			if fb.kind != ckPlane {
				continue
			}
			// The certificate is EXACT and reject-only: a zero row may
			// only come from a proven contact, so any nonzero plane
			// separation, tilt, or side penetration leaves the pair
			// undecided — a tolerance here would bless a real sub-tol
			// overlap as a Sound Exact-zero clearance.
			if fa.n.Dot(fb.n) != -1 {
				continue // not exactly opposing
			}
			if fb.o.Sub(fa.o).Dot(fa.n) != 0 {
				continue // not exactly coplanar
			}
			if rel, _ := k.coplanarRelation(fa, fb); rel != 1 {
				continue // no proven positive-area overlap
			}
			c := fa.planeOffset()
			aLo, aHi, okA := payloadExtent(k.a.body, fa.n)
			bLo, bHi, okB := payloadExtent(k.b.body, fa.n)
			if !okA || !okB {
				continue
			}
			_ = aLo
			_ = bHi
			if aHi <= c && bLo >= c {
				return true
			}
			if bHi <= c && aLo >= c {
				return true
			}
		}
	}
	return false
}

// payloadExtent is the body's exact extent interval along a direction, read
// off its own payload.
func payloadExtent(b *Body, g r3.Vec) (float64, float64, bool) {
	switch pl := b.payload.(type) {
	case prismPayload:
		lo, hi, err := pl.extentAlong(g)
		if err != nil {
			return 0, 0, false
		}
		return lo, hi, true
	case revolvePayload:
		lo, hi, err := pl.extentAlong(g)
		if err != nil {
			return 0, 0, false
		}
		return lo, hi, true
	default:
		return 0, 0, false
	}
}

// nestingExcluded runs the §2 membership casts: with the boundaries provenly
// apart, every shell of each body must lie wholly outside the other. A
// witness strictly inside is a proven nesting (proven overlap — not
// disjoint), and a cast the ladder cannot certify leaves the pair undecided.
func (k *pairKernel) nestingExcluded() bool {
	for _, w := range k.a.shellWit {
		inside, ok := k.b.pointInBody(w, k.tol)
		if !ok || inside {
			return false
		}
	}
	for _, w := range k.b.shellWit {
		inside, ok := k.a.pointInBody(w, k.tol)
		if !ok || inside {
			return false
		}
	}
	return true
}

// pairDiameter reads the pair's diameter D — the greatest distance between
// two points drawn from either body — from exact vertex positions and
// per-face analytic support points (§7). The reading may understate the true
// diameter, which only lowers the noise floor: the safe direction.
func (k *pairKernel) pairDiameter() float64 {
	pts := append(append([]r3.Vec{}, k.a.supports...), k.b.supports...)
	best := 0.0
	for i := range pts {
		for j := i + 1; j < len(pts); j++ {
			if d := pts[i].Sub(pts[j]).Len(); d > best {
				best = d
			}
		}
	}
	return best
}
