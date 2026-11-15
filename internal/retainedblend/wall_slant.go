package retainedblend

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// wallSlant bounds how far along a slanted wall the band's boundary on an end
// plane lies from a point that is close to both the wall and the plane
// (agent-docs/retained-edge-blend-design.md §4.3, the crossing's slant). The
// distance from a point to the plane alone is no bound: where the true wall meets
// the plane at sine s, the true boundary lies about 1/s farther along the wall.
type wallSlant interface {
	// reach is a bound on the distance from a point, within beta of the true wall
	// and within e of the plane, to the line where the true wall crosses the plane
	// on the point's side. That line lies in the closure of the kept wall. It is
	// false when no bound follows.
	reach(beta, e float64) (float64, bool)
	// tilt is the sine of the angle between the unit normal n of another plane and
	// the direction the line of reach runs in, and false when the tags allow no bound
	// on it: how far the wall's crossing line leaves n's plane per unit of offset.
	tilt(n r3.Vec) (float64, bool)
	// sameSide reports whether the true points of two held points, within their
	// bounds ba and bc of them, which lie on the plane by construction, lie on one
	// line of crossing, so that the straight segment between them lies on it.
	sameSide(a, c r3.Vec, ba, bc float64) bool
}

// cylSlant is the wallSlant of a cylinder wall whose axis runs parallel to the end
// plane g: the crossing lines are generators.
type cylSlant struct {
	radius float64
	// hc is the axis's distance from g, and r the rounding and axial charge it
	// carries.
	hc, r float64
	axis  r3.Vec
	// o is a point of the axis, u the direction in g normal to it, and far the
	// charge for the rounding of an offset read from o.
	o, u r3.Vec
	far  float64
}

// wallSlant is the slant bound of the true wall face fi at end plane g, or nil when
// none follows from its tag. The wall must be an exact cylinder whose axis runs
// parallel to g (|n·a| ≤ parallelTol, the arc reader's gate, arc.go), and its tag
// must state no positional departure and no axial delta: Retention Claim R then
// puts every true point of the wall on the tag's cylinder.
func (w *work) wallSlant(fi int, g plane) wallSlant {
	f := w.faceAt(fi)
	if f == nil || !f.RetainedTag || f.TagDeparture != 0 || (f.HasAxialDelta && f.AxialDelta != 0) {
		return nil
	}
	sf, ok := f.Surface.(surfacegeom.Cylinder)
	if !ok {
		return nil
	}
	radius, err := sf.Radius.In(units.Millimeter)
	axis, ok := sf.Axis.Normalize()
	if err != nil || !(radius > 0) || !ok {
		return nil
	}
	along := math.Abs(g.n.Dot(axis))
	if along > parallelTol {
		return nil
	}
	u, ok := g.n.Cross(axis).Normalize()
	if !ok {
		return nil
	}
	rounding := 64 * ulp
	far := proofbound.AbsSumUpper(w.r.scale, sf.Origin.Len())
	hc := math.Abs(g.n.Dot(sf.Origin.Sub(g.o)))
	r := proofbound.AbsSumUpper(proofbound.ProductUpper(rounding, proofbound.AbsSumUpper(far, g.o.Len())),
		proofbound.ProductUpper(along, far))
	return &cylSlant{radius: radius, hc: hc, r: r, axis: axis, o: sf.Origin, u: u, far: far}
}

// reach is the cross-section bound. Work in the section normal to the axis, where
// g is a line at distance h from the axis, h ∈ [h⁻, h⁺] (hc ± r). A point x within b
// of the true wall, which is the tag's cylinder, and within e of g, lies at distance
// ρ ∈ [R − b, R + b] from the axis and at |Y − h| ≤ e from it measured along g's
// normal, so |Y| ∈ [y⁻, y⁺] with y⁺ = h⁺ + e and y⁻ = max(0, h⁻ − e), and its
// coordinate X along g satisfies X² = ρ² − Y² ∈ [X⁻², X⁺²] with
// X⁻ = √((R − b)² − y⁺²) and X⁺ = √((R + b)² − y⁻²). X⁻ > 0 puts x on one side of
// the axis's foot, so the generator it is near is the one at Z = √(R² − h²) on that
// side, Z ∈ [Z⁻, Z⁺] with Z⁻ = √(R² − h⁺²) and Z⁺ = √(R² − h⁻²). The distance is
// then at most √((|X| − Z)² + e²) ≤ hypot(max(X⁺ − Z⁻, Z⁺ − X⁻), e). Every
// operation rounds outward.
func (c *cylSlant) reach(beta, e float64) (float64, bool) {
	up := func(x float64) float64 { return x * (1 + 8*ulp) }
	dn := func(x float64) float64 { return x * (1 - 8*ulp) }
	sqUp := func(x float64) float64 { return up(math.Sqrt(up(x))) }
	sqDn := func(x float64) float64 {
		if !(x > 0) {
			return 0
		}
		return dn(math.Sqrt(dn(x)))
	}
	hHi := proofbound.AbsSumUpper(c.hc, c.r)
	hLo := math.Max(0, dn(c.hc-c.r))
	yHi := proofbound.AbsSumUpper(hHi, e)
	yLo := math.Max(0, dn(hLo-e))
	rIn := dn(c.radius - beta)
	rOut := proofbound.AbsSumUpper(c.radius, beta)
	if !(rIn > yHi) || !(hHi < c.radius) {
		return 0, false
	}
	xLo := sqDn(dn(rIn*rIn) - up(yHi*yHi))
	xHi := sqUp(up(rOut*rOut) - dn(yLo*yLo))
	zLo := sqDn(dn(c.radius*c.radius) - up(hHi*hHi))
	zHi := sqUp(up(c.radius*c.radius) - dn(hLo*hLo))
	if !(xLo > 0) {
		return 0, false
	}
	dx := math.Max(up(xHi-zLo), up(zHi-xLo))
	return up(math.Hypot(dx, e)), true
}

// tilt is |n × axis|, the sine of the angle between n and the generators'
// direction. It is false above parallelTol.
func (c *cylSlant) tilt(n r3.Vec) (float64, bool) {
	t := n.Cross(c.axis).Len()
	return t, t <= parallelTol
}

// sameSide reads the side of the axis's foot each point lies on along g, from the
// coordinate u·(p − o): the true point of a point on g lies on g ∩ wall, at
// coordinate ±Z, and has the sign of the held point's coordinate when that
// coordinate exceeds the point's bound plus the rounding of the reading.
func (c *cylSlant) sameSide(a, p r3.Vec, ba, bp float64) bool {
	rounding := proofbound.ProductUpper(64*ulp, c.far)
	ta, tp := c.u.Dot(a.Sub(c.o)), c.u.Dot(p.Sub(c.o))
	ma := proofbound.AbsSumUpper(ba, rounding, c.r)
	mp := proofbound.AbsSumUpper(bp, rounding, c.r)
	return (ta > ma && tp > mp) || (ta < -ma && tp < -mp)
}

// errSlant is the refusal of a band point that lies on a slanted end plane whose
// slant the tags cannot bound.
func errSlant() error {
	return fmt.Errorf("%w: the wall meets an end face's plane at a slant the tags cannot bound", decaderr.ErrUnsupported)
}

// slantedEnd is the end plane q of the band when its wall meets it at a slant.
func (b *wallBand) slantedEnd(q int) *bandEnd {
	if q < 0 || !isEnd(q) {
		return nil
	}
	if e := b.exact(q); e != nil && e.slanted {
		return e
	}
	return nil
}

// slantBound is the bound a cut point p of the band publishes where it lies on plane
// q, and on plane other when other is not -1, and the planes' slant is not square:
// the larger of old, which a square plane publishes, and the distance to the true
// boundary of the kept wall. beta is the distance the held claims put p from the true
// wall, before the interpolation's rounding. It also returns pos, the bound on the
// distance from p to its true place that the strip's band charge must cover (the
// cutDelta of a corner), or zero when the point's slant adds nothing to old.
//
//   - A point on a slanted end plane alone is within reach(b, e) of the crossing
//     line, b = beta + slack and e = |offset from the plane| + slack.
//   - A corner of that plane and a plane whose normal runs along the crossing
//     line's direction, within tilt: the true corner is where the line meets that
//     plane, so the corner lies within reach + (|v| + tilt·reach)/√(1 − tilt²) of it,
//     v the point's offset from the other plane, plus the rounding.
//   - A corner of the plane and a plane that does not, or of two end planes, has no
//     bound the tags give: it refuses.
func (b *wallBand) slantBound(p r3.Vec, q, other int, beta, old float64) (float64, float64, error) {
	g, o := q, other
	if b.slantedEnd(g) == nil {
		g, o = other, q
	}
	e := b.slantedEnd(g)
	if e == nil {
		return old, 0, nil
	}
	if e.slant == nil {
		return 0, 0, errSlant()
	}
	bb := proofbound.AbsSumUpper(beta, b.slack)
	ee := proofbound.AbsSumUpper(math.Abs(b.value(g, p)), b.slack)
	d, ok := e.slant.reach(bb, ee)
	if !ok {
		return 0, 0, errSlant()
	}
	if o < 0 {
		return max(old, d), 0, nil
	}
	if b.slantedEnd(o) != nil {
		return 0, 0, errSlant()
	}
	n := b.planeNormal(o)
	t, ok := e.slant.tilt(n)
	if !ok {
		return 0, 0, errSlant()
	}
	axial := proofbound.AbsSumUpper(math.Abs(b.value(o, p)), b.slack, proofbound.ProductUpper(t, d))
	axial = proofbound.DivUpper(axial, math.Sqrt(math.Max(0, 1-t*t))*(1-4*ulp))
	pos := proofbound.AbsSumUpper(d, axial)
	return max(old, pos), pos, nil
}

// planeNormal is the unit normal of band plane q.
func (b *wallBand) planeNormal(q int) r3.Vec {
	if e := b.exact(q); e != nil {
		return e.g.n
	}
	return b.nu
}

// cutDelta is the facet bound noteCut records for the corner pt that the strip's band
// charge (chargeBand) reads: the facet's bound, or, for a corner on a slanted end
// plane, the part of its distance to its true place that cutOffset's 2·slack does not
// add, so that the strip's triangles at the corner charge that distance and no less.
// The strip's slack is the band's own.
func (b *wallBand) cutDelta(pt *bandPt, facetBound float64) float64 {
	return max(facetBound, pt.slantPos-2*b.slack)
}
