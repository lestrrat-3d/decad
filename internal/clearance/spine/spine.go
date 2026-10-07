package spine

import (
	"context"
	"errors"
	"math"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Engine computes stationary spine pairs for one clearance operation.
// Each caller creates an Engine from its pair's context, tolerance, and slack.
type Engine struct {
	Context   context.Context //nolint:containedctx // An Engine is per-operation state.
	Tolerance float64
	Slack     float64
	Err       error
	Refused   bool
}

func (e *Engine) oracle() clearance.Oracle {
	return clearance.Oracle{Tol: e.Tolerance}
}

// SpineCriticals encloses every critical of the spine-pair distance; ok is
// false when the configuration is off the shipped path (handled coarse).
func (e *Engine) SpineCriticals(f, g *clearance.CFace) ([]clearance.SpineCrit, bool) {
	sf, sg := clearance.SpineOf(f), clearance.SpineOf(g)
	if sg < sf {
		out, ok := e.SpineCriticals(g, f)
		for i := range out {
			out[i].Fa, out[i].Fb = out[i].Fb, out[i].Fa
		}
		return out, ok
	}
	switch {
	case sf == 0 && sg == 0:
		return []clearance.SpineCrit{clearance.ExactCrit(f.Anchor, g.Anchor)}, true
	case sf == 0 && sg == 1:
		foot := clearance.LinePoint(g.Anchor, g.Axis, f.Anchor)
		return []clearance.SpineCrit{clearance.ExactCrit(f.Anchor, foot)}, true
	case sf == 0 && sg == 2:
		return e.PointCircleCrits(f.Anchor, g.Anchor, g.Axis, g.RefU, g.RefV, g.Major, g.Sweep)
	case sf == 1 && sg == 1:
		return e.LineLineCrits(f, g)
	case sf == 1 && sg == 2:
		cp := polynomial.CircleParam{
			C: [3]float64{g.Anchor.X, g.Anchor.Y, g.Anchor.Z},
			U: [3]float64{g.RefU.X, g.RefU.Y, g.RefU.Z},
			V: [3]float64{g.RefV.X, g.RefV.Y, g.RefV.Z},
			R: g.Major,
		}
		return e.LineCircleBracketCrits(cp, g.Anchor, g.RefU, g.RefV, f.Anchor, f.Axis)
	default:
		return e.CircleCircleCrits(f, g)
	}
}

// PointCircleCrits are the near and far criticals of a point against a
// circle — closed form. A point PROVENLY on the circle's axis has the same
// distance at every azimuth, so one representative inside the trim's own
// window carries the whole family; a point provenly off the axis has the two
// criticals along its radial direction. An offset the oracle cannot decide
// leaves the radial direction unresolvable — and the azimuth it would produce
// decides a trim admission — so the cell reports no criticals rather than
// guess (ok=false).
func (e *Engine) PointCircleCrits(p, c, axis, refU, refV r3.Vec, rad float64, win clearance.AngWindow) ([]clearance.SpineCrit, bool) {
	switch e.oracle().OnAxis(p, c, axis) {
	case clearance.DegYes:
		th := 0.0
		if !win.Full {
			th = (win.Lo + win.Hi) / 2
		}
		s, cs := math.Sincos(th)
		q := c.Add(refU.Scale(rad * cs)).Add(refV.Scale(rad * s))
		return []clearance.SpineCrit{clearance.ExactCrit(p, q)}, true
	case clearance.DegNo:
		rel := p.Sub(c)
		perp := rel.Sub(axis.Scale(rel.Dot(axis)))
		dir, ok := perp.Normalize()
		if !ok {
			return nil, false
		}
		near := c.Add(dir.Scale(rad))
		far := c.Sub(dir.Scale(rad))
		return []clearance.SpineCrit{clearance.ExactCrit(p, near), clearance.ExactCrit(p, far)}, true
	default:
		return nil, false
	}
}

// LineLineCrits: EXACTLY parallel axes carry one constant-distance family
// (represented at the axial-overlap midpoint — the family really is constant,
// so the representative is a proof); provenly non-parallel axes carry the
// single common-perpendicular critical. An axis pair in the oracle's undecided
// band has neither — a plateau there would be an Exact reading the true
// minimum undercuts, and the common perpendicular is not resolvable — so the
// cell falls to the coarse enclosure (ok=false).
func (e *Engine) LineLineCrits(f, g *clearance.CFace) ([]clearance.SpineCrit, bool) {
	switch e.oracle().Parallel(f.Axis, g.Axis) {
	case clearance.DegYes:
		// The representative sits at the overlap midpoint of the two axial
		// windows, projected onto each axis.
		rel := g.Anchor.Sub(f.Anchor)
		sign := 1.0
		if g.Axis.Dot(f.Axis) < 0 {
			sign = -1
		}
		gLoOnF := rel.Dot(f.Axis) + sign*g.ZWin.Lo
		gHiOnF := rel.Dot(f.Axis) + sign*g.ZWin.Hi
		gw := clearance.NewLinWindow(gLoOnF, gHiOnF)
		lo := math.Max(f.ZWin.Lo, gw.Lo)
		hi := math.Min(f.ZWin.Hi, gw.Hi)
		z := (lo + hi) / 2
		if lo > hi {
			z = math.Max(f.ZWin.Lo, math.Min(f.ZWin.Hi, (gw.Lo+gw.Hi)/2))
		}
		fa := f.Anchor.Add(f.Axis.Scale(z))
		return []clearance.SpineCrit{clearance.ExactCrit(fa, clearance.LinePoint(g.Anchor, g.Axis, fa))}, true
	case clearance.DegNo:
		c, ok := e.LineLinePerp(f.Anchor, f.Axis, g.Anchor, g.Axis)
		if !ok {
			return nil, false
		}
		return []clearance.SpineCrit{c}, true
	default:
		return nil, false
	}
}

// LineCircleBracketCrits runs the P4 machinery for an explicit circle.
func (e *Engine) LineCircleBracketCrits(cp polynomial.CircleParam, center, refU, refV, la, ld r3.Vec) ([]clearance.SpineCrit, bool) {
	brs, ok, err := polynomial.LineCircleBracketsContext(e.Context, cp, [3]float64{la.X, la.Y, la.Z}, [3]float64{ld.X, ld.Y, ld.Z}, e.Slack)
	if err != nil {
		if errors.Is(err, polynomial.ErrNonFiniteClearancePolynomial) {
			e.Refused = true
			return nil, false
		}
		e.Err = err
		return nil, false
	}
	if !ok {
		// Constant distance over the circle (a coaxial configuration):
		// closed form at a deterministic azimuth.
		q := center.Add(refU.Scale(cp.R))
		d := q.Sub(clearance.LinePoint(la, ld, q)).Len()
		return []clearance.SpineCrit{{Lo: d, Hi: d, Exact: true, Fa: clearance.LinePoint(la, ld, q), Fb: q}}, true
	}
	var out []clearance.SpineCrit
	for _, br := range brs {
		s, c := math.Sincos(br.Mid())
		q := center.Add(refU.Scale(cp.R * c)).Add(refV.Scale(cp.R * s))
		out = append(out, clearance.SpineCrit{Lo: br.Lo, Hi: br.Hi, Fa: clearance.LinePoint(la, ld, q), Fb: q})
	}
	return out, true
}

// CircleCircleCrits is the P8 spine cell: EXACTLY coaxial spines are the
// certified constant-distance closed form; provenly non-coaxial spines take
// the Sturm brackets, guarded against the ρ = 0 kink (a spine meeting the
// other's axis, where the distance is not differentiable — off the shipped
// path, handled coarse). A pair the oracle cannot decide coaxial gets neither:
// the constant closed form would be an Exact reading a tilt undercuts, and the
// brackets' own foot map is not resolvable there.
func (e *Engine) CircleCircleCrits(f, g *clearance.CFace) ([]clearance.SpineCrit, bool) {
	rel := f.Anchor.Sub(g.Anchor)
	switch e.oracle().Coaxial(g, f) {
	case clearance.DegYes:
		// Coaxial: constant distance hypot(dz, ΔR) at matched azimuths.
		dz := rel.Dot(g.Axis)
		d := math.Hypot(dz, f.Major-g.Major)
		pf := f.Anchor.Add(f.RefU.Scale(f.Major))
		perp := pf.Sub(g.Anchor).Sub(g.Axis.Scale(pf.Sub(g.Anchor).Dot(g.Axis)))
		dir, ok := perp.Normalize()
		if !ok {
			return nil, false
		}
		pg := g.Anchor.Add(dir.Scale(g.Major))
		return []clearance.SpineCrit{{Lo: d, Hi: d, Exact: true, Fa: pf, Fb: pg}}, true
	case clearance.DegUnknown:
		return nil, false
	}
	// Kink guard: the P8 stationarity is smooth only while f's spine stays
	// clear of g's axis (§4's foot-map caveat, at the spine level); a spine
	// that can reach the axis is off the shipped path.
	axisDist := rel.Sub(g.Axis.Scale(rel.Dot(g.Axis))).Len()
	if axisDist-f.Major <= e.Tolerance {
		return nil, false
	}
	c1 := polynomial.CircleParam{
		C: [3]float64{f.Anchor.X, f.Anchor.Y, f.Anchor.Z},
		U: [3]float64{f.RefU.X, f.RefU.Y, f.RefU.Z},
		V: [3]float64{f.RefV.X, f.RefV.Y, f.RefV.Z},
		R: f.Major,
	}
	c2 := polynomial.CircleParam{
		C: [3]float64{g.Anchor.X, g.Anchor.Y, g.Anchor.Z},
		U: [3]float64{g.RefU.X, g.RefU.Y, g.RefU.Z},
		V: [3]float64{g.RefV.X, g.RefV.Y, g.RefV.Z},
		R: g.Major,
	}
	brs, ok, err := polynomial.CircleCircleBracketsContext(e.Context, c1, c2, [3]float64{g.Axis.X, g.Axis.Y, g.Axis.Z}, e.Slack)
	if err != nil {
		if errors.Is(err, polynomial.ErrNonFiniteClearancePolynomial) {
			e.Refused = true
			return nil, false
		}
		e.Err = err
		return nil, false
	}
	if !ok {
		return nil, false
	}
	var out []clearance.SpineCrit
	for _, br := range brs {
		s, c := math.Sincos(br.Mid())
		pf := f.Anchor.Add(f.RefU.Scale(f.Major * c)).Add(f.RefV.Scale(f.Major * s))
		// The matching foot on g's spine: the nearest spine point.
		relP := pf.Sub(g.Anchor)
		perp := relP.Sub(g.Axis.Scale(relP.Dot(g.Axis)))
		dir, dok := perp.Normalize()
		if !dok {
			return nil, false
		}
		pg := g.Anchor.Add(dir.Scale(g.Major))
		out = append(out, clearance.SpineCrit{Lo: br.Lo, Hi: br.Hi, Fa: pf, Fb: pg})
	}
	return out, true
}

// LineLinePerp is the common perpendicular critical between a segment carrier
// and a spine line — for lines the oracle has already proven NOT parallel. The
// solve still owes a guard: cancellation can drive the denominator to zero (or
// the feet to infinity) on a pair the oracle only just separated, and a
// non-finite foot is no critical at all. ok is false there — the caller owes
// an enclosure, never a fabricated critical.
func (e *Engine) LineLinePerp(a, u, b, v r3.Vec) (clearance.SpineCrit, bool) {
	rel := b.Sub(a)
	uv := u.Dot(v)
	den := 1 - uv*uv
	if !(den > 0) {
		return clearance.SpineCrit{}, false
	}
	ru := rel.Dot(u)
	rv := rel.Dot(v)
	s := (ru - uv*rv) / den
	t := (uv*ru - rv) / den
	fa := a.Add(u.Scale(s))
	fb := b.Add(v.Scale(t))
	if !proofbound.FiniteVec(fa) || !proofbound.FiniteVec(fb) {
		return clearance.SpineCrit{}, false
	}
	return clearance.ExactCrit(fa, fb), true
}
