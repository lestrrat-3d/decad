package brepmodify

import (
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// viewTilt is what the stacked view states on an oblique swept wall's retained
// Plane tag: theta bounds the distance between the unit normal of the true wall
// and the tag's (the tag's NormalBound), and extent bounds how far any point of
// the true patch lies from the tag plane (the tag's ExtentDelta).
type viewTilt struct{ theta, extent float64 }

// maxTiltVertices caps the boundary vertices a wall may have for the tilt to be
// measured: the largest distance between two of them is read pair by pair.
const maxTiltVertices = 64

// stackedViewTilt lists, by index in topobuild.Faces order, the swept planar
// walls of b whose view keeps the Plane tag and states its tilt
// (agent-docs/charged-window-chamfer-design.md §2.2). It is nil when the record
// bp is exact (no section displacement and no carrier charge), or when that
// displacement is not a finite positive number.
//
// A swept wall is rewritten between canonical vertices, so its plane passes
// through the walked ends it was held at, which lie within e of the points the
// record denotes. The true wall is swept along the record face's frame normal
// (the sweep axis a, read through the placement in exact rationals). Where the
// tag's frame normal n has n · a = 0 exactly, a point's distance from the tag
// plane does not change along a, so E, the largest distance of a boundary
// vertex from the tag plus e, bounds every point of the true patch, and the
// true trace is tilted from the tag's by at most asin(2E / (L − 2e)), L being
// the largest distance of two boundary vertices at right angles to a. The two
// unit normals then lie θ = √2 · 2E / (L − 2e) apart (Claim T). A face with no
// such certificate has no entry: n · a ≠ 0, L ≤ 4E, a value that is not
// finite, a boundary whose walked ends the loop does not state in walk order,
// or a tag whose outward normal does not turn the way the record's rule does.
// Every check only refuses: no entry leaves the face untagged in the view.
func stackedViewTilt(b *topo.Body, bp breprecord.Payload) map[int]viewTilt {
	w := max(bp.SectionDelta(), bp.CarrierCharge)
	if !(w > 0) || math.IsInf(w, 0) || !bp.Xform.IsValid() {
		return nil
	}
	var out map[int]viewTilt
	for i, f := range topobuild.Faces(b) {
		t, ok := wallTilt(b, bp, f)
		if !ok {
			continue
		}
		if out == nil {
			out = map[int]viewTilt{}
		}
		out[i] = t
	}
	return out
}

// wallTilt is stackedViewTilt's reading of one face.
func wallTilt(b *topo.Body, bp breprecord.Payload, f *topo.Face) (viewTilt, bool) {
	plane, ok := f.Surface.(surfacegeom.Plane)
	if !ok || recordPlanarFace(b, f) {
		return viewTilt{}, false
	}
	k, ok := recordWallIndex(b, bp, f)
	if !ok {
		return viewTilt{}, false
	}
	n := plane.Frame.N()
	if !proofbound.FiniteVec(n) || !proofbound.FiniteVec(plane.Frame.Origin()) {
		return viewTilt{}, false
	}
	axis, ok := sweepAxis(bp, k)
	if !ok {
		return viewTilt{}, false
	}
	ratN := ratVecOf(n)
	// The exact gate: only then is a point's distance from the tag independent
	// of its height along the sweep.
	if ratN.dot(axis).Sign() != 0 {
		return viewTilt{}, false
	}
	nn := ratN.dot(ratN)
	nLower := proofbound.RatSqrtDown(nn)
	if !(nLower > 0) {
		return viewTilt{}, false
	}
	verts, e, ok := boundaryVertices(f)
	if !ok {
		return viewTilt{}, false
	}
	p, q, ok := walkedEnds(f)
	if !ok {
		return viewTilt{}, false
	}
	aa := axis.dot(axis)
	perp := func(d ratVec) *big.Rat {
		along := d.dot(axis)
		along.Mul(along, along)
		along.Quo(along, aa)
		return new(big.Rat).Sub(d.dot(d), along)
	}
	// The walked ends are the wall's rim: they differ at right angles to the
	// sweep by more than the vertices' own bounds. A side line does not.
	pos := [2]ratVec{ratVecOf(p.Position), ratVecOf(q.Position)}
	trace := pos[1].sub(pos[0])
	if !(proofbound.RatSqrtDown(perp(trace)) > proofbound.ProductUpper(2, e)) {
		return viewTilt{}, false
	}
	// Orientation, exact: the tag's outward normal must turn the way the
	// record's rule turns the walked trace about the sweep axis.
	outward := ratN
	if f.Reversed {
		outward = ratN.neg()
	}
	turn := trace.cross(axis)
	if bp.Xform.IsReflection() {
		turn = turn.neg()
	}
	if outward.dot(turn).Sign() <= 0 {
		return viewTilt{}, false
	}
	origin := ratVecOf(plane.Frame.Origin())
	var dist float64
	for _, v := range verts {
		off := new(big.Rat).Abs(ratN.dot(ratVecOf(v.Position).sub(origin)))
		dist = max(dist, proofbound.DivUpper(proofbound.RatFloatUp(off), nLower))
	}
	extent := proofbound.AbsSumUpper(dist, e)
	longest := new(big.Rat)
	for i, v := range verts {
		for _, u := range verts[i+1:] {
			d := ratVecOf(v.Position).sub(ratVecOf(u.Position))
			if s := perp(d); s.Cmp(longest) > 0 {
				longest = s
			}
		}
	}
	span := proofbound.RatSqrtDown(longest)
	if proofbound.IsNonFinite(extent) || proofbound.IsNonFinite(span) ||
		!(span > proofbound.ProductUpper(4, extent)) {
		return viewTilt{}, false
	}
	room := math.Nextafter(span-proofbound.ProductUpper(2, e), math.Inf(-1))
	x := proofbound.DivUpper(proofbound.ProductUpper(2, extent), room)
	theta := min(2, proofbound.ProductUpper(math.Nextafter(math.Sqrt2, math.Inf(1)), x))
	if proofbound.IsNonFinite(theta) {
		return viewTilt{}, false
	}
	return viewTilt{theta: theta, extent: extent}, true
}

// recordWallIndex returns the index k of the one swept record face of b that f
// carries a wall(k) origin of, under the record's producer.
func recordWallIndex(b *topo.Body, bp breprecord.Payload, f *topo.Face) (int, bool) {
	found := -1
	for _, origin := range f.Origins {
		if origin.Producer != b.Origin.Producer {
			continue
		}
		var k int
		if _, err := fmt.Sscanf(origin.Role, "wall(%d)", &k); err != nil ||
			origin.Role != fmt.Sprintf("wall(%d)", k) {
			continue
		}
		if k < 0 || k >= len(bp.Faces) || bp.Faces[k].Planar() || bp.Faces[k].Role != origin.Role {
			return 0, false
		}
		if found >= 0 && found != k {
			return 0, false
		}
		found = k
	}
	return found, found >= 0
}

// sweepAxis is the exact image, under the record's placement, of swept record
// face k's frame normal: the direction its wall sweeps along.
func sweepAxis(bp breprecord.Payload, k int) (ratVec, bool) {
	basis := bp.Xform.Basis()
	n := bp.Faces[k].Frame.N()
	if !proofbound.FiniteVec(n) || !proofbound.FiniteVec(basis.EX) || !proofbound.FiniteVec(basis.EY) ||
		!proofbound.FiniteVec(basis.EZ) {
		return ratVec{}, false
	}
	x, y, z := ratVecOf(basis.EX), ratVecOf(basis.EY), ratVecOf(basis.EZ)
	var out ratVec
	for i := range out {
		out[i] = new(big.Rat).Add(new(big.Rat).Mul(x[i], ratOf(n.X)),
			new(big.Rat).Add(new(big.Rat).Mul(y[i], ratOf(n.Y)), new(big.Rat).Mul(z[i], ratOf(n.Z))))
	}
	if out.dot(out).Sign() == 0 {
		return ratVec{}, false
	}
	return out, true
}

// boundaryVertices lists the distinct vertices of f's loops and the largest
// published bound among them, a length. ok is false when f has none, too many,
// or a position or bound that is not finite.
func boundaryVertices(f *topo.Face) ([]*topo.Vertex, float64, bool) {
	var verts []*topo.Vertex
	add := func(v *topo.Vertex) {
		if v != nil && !slices.Contains(verts, v) {
			verts = append(verts, v)
		}
	}
	for _, loop := range f.Loops {
		for _, ce := range loop.CoEdges {
			if ce.Edge == nil {
				return nil, 0, false
			}
			add(ce.Edge.Start)
			add(ce.Edge.End)
		}
	}
	if len(verts) < 2 || len(verts) > maxTiltVertices {
		return nil, 0, false
	}
	var e float64
	for _, v := range verts {
		bound := v.Bound.Base()
		if !proofbound.FiniteVec(v.Position) || !(bound >= 0) || math.IsInf(bound, 0) {
			return nil, 0, false
		}
		e = max(e, bound)
	}
	return verts, e, true
}

// walkedEnds returns the walked start and end of f's wall in walk order. The
// record builds a swept face's loop as rim0, side1, rim1, side0
// (breprecord.Topology FaceUses), and rim0 is the wall's bottom edge whose
// owner use runs from the walk's start to its end, so its first coedge reads
// the ends in walk order. A face whose first coedge is not a straight edge of a
// single loop has none.
func walkedEnds(f *topo.Face) (*topo.Vertex, *topo.Vertex, bool) {
	if len(f.Loops) != 1 || len(f.Loops[0].CoEdges) < 4 {
		return nil, nil, false
	}
	first := f.Loops[0].CoEdges[0]
	e := first.Edge
	if e == nil || e.Start == nil || e.End == nil || e.Start == e.End {
		return nil, nil, false
	}
	if _, line := e.Curve.(surfacegeom.Line3); !line {
		return nil, nil, false
	}
	if first.Forward {
		return e.Start, e.End, true
	}
	return e.End, e.Start, true
}

// applyViewTilt states each tilt on the group of its face: a retained Plane tag
// that states no departure keeps its surface and takes NormalBound = theta and
// ExtentDelta = extent; any other group in tilt is untagged. It reports false
// when an index lies outside groups.
func applyViewTilt(groups []facetedrecord.Group, tilt map[int]viewTilt) bool {
	for gi, t := range tilt {
		if gi < 0 || gi >= len(groups) {
			return false
		}
		g := groups[gi]
		_, plane := g.Surface.(surfacegeom.Plane)
		if !plane || !g.Retained || g.Denoted != nil || g.NormalBound != 0 || g.HasAxialDelta ||
			g.CapRecord != nil || g.ExtentDelta != 0 {
			groups[gi] = untagged(g)
			continue
		}
		g.NormalBound, g.ExtentDelta = t.theta, t.extent
		groups[gi] = g
	}
	return true
}

// ratVec is an exact three-vector.
type ratVec [3]*big.Rat

func ratOf(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }

func ratVecOf(v r3.Vec) ratVec { return ratVec{ratOf(v.X), ratOf(v.Y), ratOf(v.Z)} }

func (a ratVec) sub(b ratVec) ratVec {
	var out ratVec
	for i := range out {
		out[i] = new(big.Rat).Sub(a[i], b[i])
	}
	return out
}

func (a ratVec) neg() ratVec {
	var out ratVec
	for i := range out {
		out[i] = new(big.Rat).Neg(a[i])
	}
	return out
}

func (a ratVec) dot(b ratVec) *big.Rat {
	sum := new(big.Rat)
	for i := range a {
		sum.Add(sum, new(big.Rat).Mul(a[i], b[i]))
	}
	return sum
}

func (a ratVec) cross(b ratVec) ratVec {
	term := func(i, j int) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Mul(a[i], b[j]), new(big.Rat).Mul(a[j], b[i]))
	}
	return ratVec{term(1, 2), term(2, 0), term(0, 1)}
}
