package meshbool

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// This file reads a local cap side (agent-docs/coplanar-contact-design.md §20.2): an operand that states no cap side
// of its own, such as a mesh Cut result, still lies on one side of the other operand's cap plane inside a box that
// holds the other operand, when every face near that box either lies in the plane, keeps clear of it by more than its
// bound, or is a straight wall that leaves the plane along its rim.

// localSide reads a cap side of operand k on the plane of cb, a cap side of the other operand, facing the other way
// and restricted to the box that holds the other operand's true solid (§20.2). It tries each face of operand k whose
// held facets lie in the plane, face out of the operand and are carrier-exact, in source order, and returns the first
// that the faces near the box prove a side for.
func (ff *FlushFaces) localSide(k int, cb CapSide) (CapSide, bool, error) {
	op, other := ff.ops[k], ff.ops[1-k]
	region, ok := heldRegion(other)
	if !ok {
		return CapSide{}, false, nil
	}
	side := CapSide{O: cb.O, N: ratNeg(cb.N), Region: &region}
	for _, s := range op.sourceOrder() {
		if err := ff.budget.Step(); err != nil {
			return CapSide{}, false, err
		}
		f := op.head(s)
		if f == nil || f.source == nil || !f.planar {
			continue
		}
		on, err := op.faceOnSide(ff.budget, f, side)
		if err != nil {
			return CapSide{}, false, err
		}
		if !on {
			continue
		}
		carrier, err := op.carrierOf(ff.budget, f)
		if err != nil {
			return CapSide{}, false, err
		}
		if !carrier {
			continue
		}
		side.Face = f.source
		ok, err := op.localTruth(ff.budget, side, s)
		if err != nil {
			return CapSide{}, false, err
		}
		if ok {
			return side, true, nil
		}
	}
	return CapSide{}, false, nil
}

// heldRegion is the box that holds an operand's true solid: its held vertices' box, widened by the largest bound any
// of its faces states for how far the true face lies from its held facets, padded by a relative 1e-9 and 64 ulps of
// the coordinate scale. A solid lies inside the box of its boundary.
func heldRegion(op *flushOperand) ([2]r3.Vec, bool) {
	bm := op.bm
	if len(bm.Verts) == 0 {
		return [2]r3.Vec{}, false
	}
	lo, hi := bm.Verts[0], bm.Verts[0]
	scale := 1.0
	for _, v := range bm.Verts {
		if !proofbound.FiniteVec(v) {
			return [2]r3.Vec{}, false
		}
		lo = r3.Vec{X: math.Min(lo.X, v.X), Y: math.Min(lo.Y, v.Y), Z: math.Min(lo.Z, v.Z)}
		hi = r3.Vec{X: math.Max(hi.X, v.X), Y: math.Max(hi.Y, v.Y), Z: math.Max(hi.Z, v.Z)}
		scale = max(scale, math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
	}
	reach := 0.0
	seen := map[*topo.Face]struct{}{}
	for t := range bm.Tris {
		reach = max(reach, bm.FacetBound[t])
		if t >= len(op.m.Source) || op.m.Source[t] == nil {
			return [2]r3.Vec{}, false
		}
		f := op.m.Source[t]
		if _, done := seen[f]; done {
			continue
		}
		seen[f] = struct{}{}
		delta, stated := FaceGateDelta(op.m, bm.FaceSurface, f)
		if !stated || proofbound.IsNonFinite(delta) {
			return [2]r3.Vec{}, false
		}
		reach = max(reach, delta)
	}
	pad := proofbound.AbsSumUpper(reach, proofbound.ProductUpper(reach, 1e-9), 64*0x1p-52*scale)
	if proofbound.IsNonFinite(pad) {
		return [2]r3.Vec{}, false
	}
	return [2]r3.Vec{{X: lo.X - pad, Y: lo.Y - pad, Z: lo.Z - pad}, {X: hi.X + pad, Y: hi.Y + pad, Z: hi.Z + pad}}, true
}

// sourceOrder lists the operand's source ids in the order their first facets appear.
func (op *flushOperand) sourceOrder() []int {
	var out []int
	seen := map[int]struct{}{}
	for _, s := range op.bm.Src {
		if _, ok := seen[s]; ok {
			continue
		}
		seen[s] = struct{}{}
		out = append(out, s)
	}
	return out
}

// localTruth decides §20.2's claim for side, whose Face is source x of the operand: the operand's true solid meets
// the open far side of the plane nowhere inside side.Region. Every face other than x whose held facets come within the
// face's bound of the region must lie below the plane by more than that bound (clearBelow) or be a straight wall that
// leaves the plane along its rim (leavingWall).
func (op *flushOperand) localTruth(budget *proofbound.WorkBudget, side CapSide, x int) (bool, error) {
	below := map[int]bool{}
	clearOf := func(s int) (bool, error) {
		if v, ok := below[s]; ok {
			return v, nil
		}
		v, err := op.clearBelow(budget, side, s)
		if err != nil {
			return false, err
		}
		below[s] = v
		return v, nil
	}
	for _, s := range op.sourceOrder() {
		if s == x {
			continue
		}
		f := op.head(s)
		if f == nil {
			continue
		}
		var delta float64
		if f.source != nil {
			d, stated := FaceGateDelta(op.m, op.bm.FaceSurface, f.source)
			if !stated {
				return false, nil
			}
			delta = d
		}
		for _, t := range f.facets {
			delta = max(delta, op.bm.FacetBound[t])
		}
		slack := proofbound.AbsSumUpper(delta, proofbound.ProductUpper(delta, 1e-9))
		if proofbound.IsNonFinite(slack) {
			return false, nil
		}
		near := false
		for _, t := range f.facets {
			if err := budget.Step(); err != nil {
				return false, err
			}
			if BoxesWithin(op.bm.Boxes[t], *side.Region, slack) {
				near = true
				break
			}
		}
		if !near {
			continue
		}
		ok, err := clearOf(s)
		if err != nil {
			return false, err
		}
		if ok {
			continue
		}
		if f.source == nil {
			return false, nil
		}
		ok, err = op.leavingWall(budget, side, f, clearOf)
		if err != nil || !ok {
			return false, err
		}
	}
	return true, nil
}

// clearBelow reports whether every held vertex of source s lies strictly inside side's half-space, farther from its
// plane than the face's bound: the true face, within that bound of its facets, then lies in the open half-space.
func (op *flushOperand) clearBelow(budget *proofbound.WorkBudget, side CapSide, s int) (bool, error) {
	f := op.head(s)
	if f == nil || f.source == nil {
		return false, nil
	}
	delta, stated := FaceGateDelta(op.m, op.bm.FaceSurface, f.source)
	if !stated {
		return false, nil
	}
	for _, t := range f.facets {
		delta = max(delta, op.bm.FacetBound[t])
	}
	bound, ok := ratBoundUp(delta)
	if !ok {
		return false, nil
	}
	norm := ratDot(side.N, side.N)
	seen := map[int]struct{}{}
	for _, t := range f.facets {
		for _, v := range op.bm.Tris[t] {
			if _, done := seen[v]; done {
				continue
			}
			seen[v] = struct{}{}
			if err := budget.Step(); err != nil {
				return false, err
			}
			if !strictlyBelow(side, op.bm.Verts[v], bound, norm) {
				return false, nil
			}
		}
	}
	return true, nil
}

// strictlyBelow reports whether p lies inside side's half-space at a distance from its plane above bound: with
// d = N·(p − O), d < 0 and d² > bound²·|N|², exactly. norm is N·N.
func strictlyBelow(side CapSide, p r3.Vec, bound, norm *big.Rat) bool {
	if !proofbound.FiniteVec(p) {
		return false
	}
	d := ratDot(side.N, ratSub(ratVec(p), side.O))
	if d.Sign() >= 0 {
		return false
	}
	lhs := new(big.Rat).Mul(d, d)
	rhs := new(big.Rat).Mul(new(big.Rat).Mul(bound, bound), norm)
	return lhs.Cmp(rhs) > 0
}

// ratBoundUp is bound widened by a relative 1e-9 as an exact rational, or false for a bound that is not finite.
func ratBoundUp(bound float64) (*big.Rat, bool) {
	up := proofbound.AbsSumUpper(bound, proofbound.ProductUpper(bound, 1e-9))
	if proofbound.IsNonFinite(up) || up < 0 {
		return nil, false
	}
	return new(big.Rat).SetFloat64(up), true
}

// leavingWall reports whether face f is a straight wall that leaves side's plane along its rim (§20.2 claim L1): its
// true surface is a retained Plane or Cylinder tag that is its true surface, ruled along a direction that crosses the
// plane, every held vertex lies in the closed half-space, and every edge is shared with side's face, with a face clear
// below the plane, or with another such tag along a straight seam that runs from the plane or from below it to below
// it.
func (op *flushOperand) leavingWall(budget *proofbound.WorkBudget, side CapSide, f *flushFace,
	clearOf func(int) (bool, error)) (bool, error) {
	face := f.source
	tag, ok := exactRuledTag(face)
	if !ok || !tag.crosses(side.N) {
		return false, nil
	}
	seen := map[int]struct{}{}
	for _, t := range f.facets {
		for _, v := range op.bm.Tris[t] {
			if _, done := seen[v]; done {
				continue
			}
			seen[v] = struct{}{}
			if err := budget.Step(); err != nil {
				return false, err
			}
			if !proofbound.FiniteVec(op.bm.Verts[v]) || sideOf(side, op.bm.Verts[v]) > 0 {
				return false, nil
			}
		}
	}
	top := map[*topo.Vertex]struct{}{}
	for _, l := range side.Face.Loops {
		for _, ce := range l.CoEdges {
			if ce.Edge == nil || ce.Edge.Start == nil || ce.Edge.End == nil {
				return false, nil
			}
			top[ce.Edge.Start], top[ce.Edge.End] = struct{}{}, struct{}{}
		}
	}
	norm := ratDot(side.N, side.N)
	under := func(v *topo.Vertex) bool {
		bound, ok := ratBoundUp(v.Bound.Base())
		return ok && strictlyBelow(side, v.Position, bound, norm)
	}
	for _, l := range face.Loops {
		for _, ce := range l.CoEdges {
			if err := budget.Step(); err != nil {
				return false, err
			}
			e := ce.Edge
			if e == nil || len(e.Faces) != 2 || e.Faces[0] == e.Faces[1] {
				return false, nil
			}
			g := e.Faces[0]
			if g == face {
				g = e.Faces[1]
			} else if e.Faces[1] != face {
				return false, nil
			}
			if g == side.Face {
				continue
			}
			if gs, ok := op.srcOf(g); ok {
				below, err := clearOf(gs)
				if err != nil {
					return false, err
				}
				if below {
					continue
				}
			}
			other, ok := exactRuledTag(g)
			if !ok {
				return false, nil
			}
			d, ok := tag.seam(other)
			if !ok || ratDot(d, side.N).Sign() == 0 {
				return false, nil
			}
			s, t := e.Start, e.End
			if s == nil || t == nil || s == t {
				return false, nil
			}
			_, topS := top[s]
			_, topT := top[t]
			if topS && topT || !topS && !under(s) || !topT && !under(t) {
				return false, nil
			}
		}
	}
	return true, nil
}

// ruledTag is a retained Plane or Cylinder tag read in exact rationals: a plane's normal, or a cylinder's axis point,
// direction and radius.
type ruledTag struct {
	cylinder bool
	normal   [3]*big.Rat // the plane's U × V
	origin   [3]*big.Rat // a point on the cylinder's axis
	axis     [3]*big.Rat
	radius   *big.Rat
}

// exactRuledTag reads face's tag when the tag is the face's true surface (agent-docs/faceted-source-retention-design.md
// Claim R): a retained Plane or Cylinder that states no departure, denotes no other patch and carries no
// displacement.
func exactRuledTag(face *topo.Face) (ruledTag, bool) {
	if face == nil || !face.RetainedTag || face.TagDeparture != 0 || face.Denoted != nil || face.AxialDelta != 0 ||
		face.ExtentDelta != 0 || face.ViewCharge || face.FlatPatch {
		return ruledTag{}, false
	}
	switch s := face.Surface.(type) {
	case surfacegeom.Plane:
		if !proofbound.FiniteVec(s.Frame.U()) || !proofbound.FiniteVec(s.Frame.V()) {
			return ruledTag{}, false
		}
		n := ratCross(ratVec(s.Frame.U()), ratVec(s.Frame.V()))
		if ratIsZero(n) {
			return ruledTag{}, false
		}
		return ruledTag{normal: n}, true
	case surfacegeom.Cylinder:
		r := s.Radius.Base()
		if !proofbound.FiniteVec(s.Origin) || !proofbound.FiniteVec(s.Axis) || math.IsNaN(r) || math.IsInf(r, 0) || r <= 0 {
			return ruledTag{}, false
		}
		a := ratVec(s.Axis)
		if ratIsZero(a) {
			return ruledTag{}, false
		}
		return ruledTag{cylinder: true, origin: ratVec(s.Origin), axis: a, radius: new(big.Rat).SetFloat64(r)}, true
	}
	return ruledTag{}, false
}

// crosses reports whether every ruling of the tag crosses the plane of normal n, so that the tag's part strictly on
// either side of the plane is connected and unbounded: a cylinder's axis is not parallel to the plane, and a plane is
// not parallel to it.
func (r ruledTag) crosses(n [3]*big.Rat) bool {
	if r.cylinder {
		return ratDot(r.axis, n).Sign() != 0
	}
	return !ratIsZero(ratCross(r.normal, n))
}

// seam returns a direction d along which both tags are invariant, so that two distinct tags meet in lines parallel to
// d: a plane pair's crossing direction, a cylinder's axis when the plane contains it, or the common axis of two parallel
// cylinders that are not one surface. It is false for every other pair.
func (r ruledTag) seam(o ruledTag) ([3]*big.Rat, bool) {
	switch {
	case !r.cylinder && !o.cylinder:
		d := ratCross(r.normal, o.normal)
		return d, !ratIsZero(d)
	case r.cylinder && !o.cylinder:
		return r.axis, ratDot(o.normal, r.axis).Sign() == 0
	case !r.cylinder && o.cylinder:
		return o.axis, ratDot(r.normal, o.axis).Sign() == 0
	}
	if !ratIsZero(ratCross(r.axis, o.axis)) {
		return [3]*big.Rat{}, false
	}
	if ratIsZero(ratCross(ratSub(o.origin, r.origin), r.axis)) && r.radius.Cmp(o.radius) == 0 {
		return [3]*big.Rat{}, false
	}
	return r.axis, true
}
