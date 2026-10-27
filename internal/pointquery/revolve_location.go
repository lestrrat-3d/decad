package pointquery

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// LocateRevolve classifies the world point p against the solid a full
// Revolve's record denotes, in exact rational arithmetic over its recorded
// meridian, resolved axis, frame and placement
// (agent-docs/point-containment-design.md §10). decided is false for a
// record the route does not read exactly or a point it leaves open. p must
// be finite.
func LocateRevolve(ctx context.Context, p r3.Vec, rp revolveplan.Payload) (Location, bool, error) {
	if err := ctx.Err(); err != nil {
		return Undecided, false, err
	}
	axis, ok := exactRevolveAxis(rp)
	if !ok {
		return Undecided, false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	rings, ok, err := readMeridian(budget, rp, axis)
	if err != nil || !ok {
		return Undecided, false, err
	}
	solved, ok := revolveLocal(p, rp, axis)
	if !ok {
		return Undecided, false, nil
	}
	z, x, y := solved[0], solved[1], solved[2]
	radial := new(big.Rat).Add(new(big.Rat).Mul(x, x), new(big.Rat).Mul(y, y))
	// The axis point at axial coordinate z, and the plane offset of unit
	// radial coordinate: Q± = A + (z·d ± √R·e)/|d|², e = (−d_v, d_u).
	scale := new(big.Rat).Quo(z, axis.d2)
	base := planePoint{
		u: new(big.Rat).Add(axis.a.u, new(big.Rat).Mul(scale, axis.d.u)),
		v: new(big.Rat).Add(axis.a.v, new(big.Rat).Mul(scale, axis.d.v)),
	}
	if radial.Sign() == 0 {
		class, err := rings.locate(budget, base)
		if err != nil {
			return Undecided, false, err
		}
		result := regionLocation(class)
		if result == OnBoundary && axisPiecesOnly(rings, base, axis) {
			result = Inside
		}
		return result, result != Undecided, nil
	}
	plus := surdPoint{
		u0: base.u, u1: new(big.Rat).Quo(new(big.Rat).Neg(axis.d.v), axis.d2),
		v0: base.v, v1: new(big.Rat).Quo(axis.d.u, axis.d2),
		r: radial,
	}
	near, err := rings.locate(budget, plus)
	if err != nil {
		return Undecided, false, err
	}
	far, err := rings.locate(budget, plus.mirrored())
	if err != nil {
		return Undecided, false, err
	}
	result := join(regionLocation(near), regionLocation(far))
	return result, result != Undecided, nil
}

// revolveAxis is the resolved axis read exactly: anchor a, direction d, its
// squared length d2 and the build's snap tolerance.
type revolveAxis struct {
	a, d     planePoint
	d2, snap *big.Rat
}

// rho is the exact radial coordinate d × (q − a) of a plane point.
func (ax revolveAxis) rho(q planePoint) *big.Rat {
	o := q.sub(ax.a)
	return new(big.Rat).Sub(new(big.Rat).Mul(ax.d.u, o.v), new(big.Rat).Mul(ax.d.v, o.u))
}

// exactRevolveAxis admits a full solid revolution whose record is its own
// set (§10.1): no section displacement, a sweep denoting exactly one turn,
// and a resolved axis with zero anchor and direction bounds, zero radial
// admission allowance and zero snap allowance.
func exactRevolveAxis(rp revolveplan.Payload) (revolveAxis, bool) {
	ax := rp.Ax
	if !rp.Full || rp.SurfaceResult || rp.SectionDelta != 0 || !fullTurn(rp.Den) ||
		ax.AUBound != 0 || ax.AVBound != 0 || ax.DUBound != 0 || ax.DVBound != 0 ||
		ax.RadialAdmitAllow != 0 || ax.Snap != (revolveaxis.SnapAllow{}) {
		return revolveAxis{}, false
	}
	au, av, du, dv, snap := proof.FloatRat(ax.AU), proof.FloatRat(ax.AV),
		proof.FloatRat(ax.DU), proof.FloatRat(ax.DV), proof.FloatRat(ax.SnapTol)
	if au == nil || av == nil || du == nil || dv == nil || snap == nil || snap.Sign() < 0 {
		return revolveAxis{}, false
	}
	// A zero direction makes W zero, so revolveLocal's map is singular and
	// no quotient by d2 is reached.
	d := planePoint{u: du, v: dv}
	return revolveAxis{a: planePoint{u: au, v: av}, d: d, d2: d.dot(d), snap: snap}, true
}

// fullTurn reports a sweep whose two ends, stated as rad + 2π·turn, denote
// exactly one turn apart. A radian-stated sweep snapped to full, an
// atan2-derived span and a sweep with no denotation report false.
func fullTurn(den revolveangle.Sweep) bool {
	lo, hi := den.Phi0, den.Phi1
	if lo.Span != nil || hi.Span != nil || lo.Rad == nil || lo.Turn == nil || hi.Rad == nil || hi.Turn == nil {
		return false
	}
	return lo.Rad.Cmp(hi.Rad) == 0 && new(big.Rat).Sub(hi.Turn, lo.Turn).Cmp(big.NewRat(1, 1)) == 0
}

// readMeridian reads the recorded meridian into exact rings (§10.1): every
// segment whole, every junction closing at two equal ends, and every
// recorded point and centre on the material side of the axis, either on it
// or clear of the build's snap band. Any other meridian reports false.
func readMeridian(budget *proofbound.WorkBudget, rp revolveplan.Payload, axis revolveAxis) (profileRing, bool, error) {
	for _, loop := range append([]sectionrecord.LoopRecord{rp.Profile.Outer}, rp.Profile.Holes...) {
		for _, segment := range loop.Segments {
			if line, ok := segment.(sectionrecord.LineSeg); ok && !isWholeRange(line.TStart, line.TEnd) {
				return profileRing{}, false, nil
			}
		}
	}
	rings, ok, err := readProfile(budget, rp.Profile)
	if err != nil || !ok {
		return profileRing{}, false, err
	}
	if !rings.closed() {
		return profileRing{}, false, nil
	}
	for _, ring := range append([]loopRing{rings.outer}, rings.holes...) {
		var points []planePoint
		if ring.circle != nil {
			center, ok := recordPoint(ring.circle.Center)
			if !ok {
				return profileRing{}, false, nil
			}
			points = append(points, center)
		}
		for _, edge := range ring.edges {
			points = append(points, edge.from, edge.to)
		}
		for _, arc := range ring.arcs {
			points = append(points, arc.center)
		}
		for _, q := range points {
			rho := axis.rho(q)
			if rho.Sign() < 0 || (rho.Sign() > 0 && rho.Cmp(axis.snap) <= 0) {
				return profileRing{}, false, nil
			}
		}
	}
	return rings, true, nil
}

// revolveLocal solves the record's denoted map G(z, x, y) = T + B·(A3 +
// W·z + E0·x + E1·y) for (z, x, y) (§10.2): A3 = O + U·a_u + V·a_v,
// W = U·d_u + V·d_v, E0 = V·d_u − U·d_v and E1 = W × E0 over the frame's
// held origin and axes, and B and T the placement's held basis and
// translation, every leaf read exactly. It reports false for a non-finite
// leaf or a singular map.
func revolveLocal(p r3.Vec, rp revolveplan.Payload, axis revolveAxis) (ratVec, bool) {
	b, err := massmoment.PlacementRotation(rp.Xform)
	if err != nil {
		return ratVec{}, false
	}
	o, u, v := exactVec(rp.Frame.Origin()), exactVec(rp.Frame.U()), exactVec(rp.Frame.V())
	translation, point := exactVec(rp.Xform.Translation()), exactVec(p)
	if !finiteRatVec(o) || !finiteRatVec(u) || !finiteRatVec(v) || !finiteRatVec(translation) || !finiteRatVec(point) {
		return ratVec{}, false
	}
	combine := func(su, sv *big.Rat) ratVec {
		var out ratVec
		for i := range 3 {
			out[i] = new(big.Rat).Add(new(big.Rat).Mul(u[i], su), new(big.Rat).Mul(v[i], sv))
		}
		return out
	}
	a3 := combine(axis.a.u, axis.a.v)
	for i := range 3 {
		a3[i].Add(a3[i], o[i])
	}
	w := combine(axis.d.u, axis.d.v)
	e0 := combine(new(big.Rat).Neg(axis.d.v), axis.d.u)
	e1 := ratVec{
		new(big.Rat).Sub(new(big.Rat).Mul(w[1], e0[2]), new(big.Rat).Mul(w[2], e0[1])),
		new(big.Rat).Sub(new(big.Rat).Mul(w[2], e0[0]), new(big.Rat).Mul(w[0], e0[2])),
		new(big.Rat).Sub(new(big.Rat).Mul(w[0], e0[1]), new(big.Rat).Mul(w[1], e0[0])),
	}
	place := func(x ratVec) ratVec {
		var out ratVec
		for i := range 3 {
			out[i] = new(big.Rat)
			for k := range 3 {
				out[i].Add(out[i], new(big.Rat).Mul(b[i][k], x[k]))
			}
		}
		return out
	}
	var m [3][3]*big.Rat
	for k, column := range [...]ratVec{place(w), place(e0), place(e1)} {
		for i := range 3 {
			m[i][k] = column[i]
		}
	}
	anchor := place(a3)
	var rhs ratVec
	for i := range 3 {
		rhs[i] = new(big.Rat).Sub(point[i], new(big.Rat).Add(translation[i], anchor[i]))
	}
	return cramerSolve(m, rhs)
}

// axisPiecesOnly reports whether every boundary piece of the meridian
// through the axis point q is a line with both ends exactly on the axis
// line (§10.3's axis rule). A circle or arc through q, or a line leaving the
// axis, reports false.
func axisPiecesOnly(rings profileRing, q planePoint, axis revolveAxis) bool {
	for _, ring := range append([]loopRing{rings.outer}, rings.holes...) {
		if ring.circle != nil && locateInCircle(*ring.circle, q) == regionBoundary {
			return false
		}
		for _, arc := range ring.arcs {
			if arc.holds(q) == regionBoundary {
				return false
			}
		}
		for _, edge := range ring.edges {
			if edge.chord || !onSegment(edge.from, edge.to, q) {
				continue
			}
			if axis.rho(edge.from).Sign() != 0 || axis.rho(edge.to).Sign() != 0 {
				return false
			}
		}
	}
	return true
}
