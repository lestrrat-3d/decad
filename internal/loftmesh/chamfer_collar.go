package loftmesh

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ChamferMiddleOffset translates a retained middle facet inward along its
// horizontal normal and returns the translated facet and the shift.
func ChamferMiddleOffset(base [3]proofarith.Xpt,
	distance, clipBound float64) ([3]proofarith.Xpt, proofarith.Xpt, error) {
	var q [3]proofarith.Xpt
	n := proofarith.Xcross(proofarith.Xsub(base[1], base[0]),
		proofarith.Xsub(base[2], base[0]))
	f := n.Vec()
	length := math.Hypot(f.X, f.Y)
	if length <= 0 || proofbound.IsNonFinite(length) {
		return q, proofarith.Xpt{},
			fmt.Errorf("%w: retained middle wall has no horizontal normal", decaderr.ErrUnsupported)
	}
	shiftFloat := r3.Vec{X: -distance * f.X / length, Y: -distance * f.Y / length}
	if !proofbound.FiniteVec(shiftFloat) {
		return q, proofarith.Xpt{},
			fmt.Errorf("%w: retained middle collar has no finite direction", decaderr.ErrUnsupported)
	}
	shift := proofarith.XptOf(shiftFloat)
	allowance := 0.1 - clipBound
	if allowance <= 0 || proofbound.IsNonFinite(allowance) {
		return q, proofarith.Xpt{},
			fmt.Errorf("%w: retained middle collar has no boundary allowance", decaderr.ErrUnsupported)
	}
	limit := proofarith.FloatRat(math.Nextafter(allowance, math.Inf(-1)))
	limit.Mul(limit, limit)
	if proofarith.XdotSign(n, shift) >= 0 ||
		proofarith.XdotRat(shift, shift).Cmp(limit) >= 0 {
		return q, proofarith.Xpt{},
			fmt.Errorf("%w: retained middle collar exceeds its boundary allowance", decaderr.ErrUnsupported)
	}
	for i := range q {
		q[i] = chamferTranslate(base[i], shift)
	}
	return q, shift, nil
}

func chamferTranslate(p, shift proofarith.Xpt) proofarith.Xpt {
	den := new(big.Int).Mul(p.W, shift.W)
	coord := func(a, b *big.Int) *big.Int {
		return new(big.Int).Add(new(big.Int).Mul(a, shift.W), new(big.Int).Mul(b, p.W))
	}
	return proofarith.Xpt(proofarith.XhpStripTwosOwned(proofarith.Xhp{
		X: coord(p.X, shift.X), Y: coord(p.Y, shift.Y),
		Z: coord(p.Z, shift.Z), W: den,
	}))
}

// ChamferMiddleBlank reports whether every vertex of q lies strictly inside
// the blank disk of the given radius.
func ChamferMiddleBlank(q [3]proofarith.Xpt, radius float64) bool {
	if radius <= 0 || proofbound.IsNonFinite(radius) {
		return false
	}
	r := proofarith.FloatRat(radius)
	r2 := new(big.Rat).Mul(r, r)
	for _, p := range q {
		x, y := meshbool.RatCoordOf(p, 0), meshbool.RatCoordOf(p, 1)
		q2 := new(big.Rat).Add(new(big.Rat).Mul(x, x), new(big.Rat).Mul(y, y))
		if q2.Cmp(r2) >= 0 {
			return false
		}
	}
	return true
}

// ChamferMiddleHeldRootInside reports whether the held root wall facets of a
// Loft tooth, clipped to the slab between zLo and zHi, stay strictly inside the
// disk of the given radius. walls bounds the wall facets of tris.
func ChamferMiddleHeldRootInside(verts []r3.Vec, tris [][3]int, walls, root int,
	zLo, zHi *big.Rat, radius float64) bool {
	if root < 0 || walls > len(tris) || zLo == nil || zHi == nil ||
		zLo.Cmp(zHi) >= 0 || radius <= 0 || proofbound.IsNonFinite(radius) {
		return false
	}
	r := proofarith.FloatRat(radius)
	rn2 := new(big.Int).Mul(r.Num(), r.Num())
	rd2 := new(big.Int).Mul(r.Denom(), r.Denom())
	loN, loD := new(big.Int).Set(zLo.Num()), new(big.Int).Set(zLo.Denom())
	hiN, hiD := new(big.Int).Set(zHi.Num()), new(big.Int).Set(zHi.Denom())
	var lhs, rhs, t big.Int
	for j := 2 * root; j < walls; j++ {
		tri := tris[j]
		poly := []proofarith.Xpt{
			proofarith.XptOf(verts[tri[0]]),
			proofarith.XptOf(verts[tri[1]]),
			proofarith.XptOf(verts[tri[2]]),
		}
		poly = chamferMiddleClipZ(poly, loN, loD, true)
		poly = chamferMiddleClipZ(poly, hiN, hiD, false)
		for _, p := range poly {
			lhs.Mul(p.X, p.X)
			t.Mul(p.Y, p.Y)
			lhs.Add(&lhs, &t)
			lhs.Mul(&lhs, rd2)
			rhs.Mul(p.W, p.W)
			rhs.Mul(&rhs, rn2)
			if lhs.Cmp(&rhs) >= 0 {
				return false
			}
		}
	}
	return true
}

// ChamferHeldRootMemo remembers where ChamferMiddleHeldRootInside has
// answered true for one tooth: the same verts, tris, walls, root and radius
// on every call. It keeps one closed interval [lo, hi], the union of
// answered slabs, each overlapping the interval when it was added, so every
// point of the interval lies in an answered slab.
//
// The predicate is containment in one fixed convex region R, the open disk:
// every wall triangle's part inside the slab must lie in R. The clip of a
// triangle to a slab has that part as its convex hull and R is convex, so a
// true answer proves the whole part inside R. A slab is a z interval, so
// overlapping or touching slabs have exactly the interval [min lo, max hi]
// as their union, which is also their convex hull in z. Each triangle's part
// inside a slab within that interval is a subset of the union of the parts
// inside the answered slabs, and each of those parts was proved inside R.
// Containment in R needs no convexity of the union, only that every piece
// was proved inside R. Inside returns true for such a slab without the clip.
// A gap between answered slabs is never inside the interval, so a query
// spanning one runs the exact check.
//
// Every other query is computed. A memo belongs to one tooth in one proof
// call: one worker state of one reach proof, never across proofs. The tooth
// meshes (verts and tris) must not change for the memo's lifetime; the memo
// compares slice identity and lengths, never contents, so an in-place change
// could return true where the direct check returns false. It computes
// directly when the tooth arguments' identity, walls, root or radius differ.
// It is not safe for concurrent use.
type ChamferHeldRootMemo struct {
	verts  []r3.Vec
	tris   [][3]int
	walls  int
	root   int
	radius float64
	lo, hi *big.Rat
}

// Inside returns ChamferMiddleHeldRootInside(verts, tris, walls, root, zLo,
// zHi, radius).
func (m *ChamferHeldRootMemo) Inside(verts []r3.Vec, tris [][3]int, walls, root int,
	zLo, zHi *big.Rat, radius float64) bool {
	same := m.lo != nil && m.sameTooth(verts, tris, walls, root, radius)
	if same && zLo != nil && zHi != nil && zLo.Cmp(zHi) < 0 &&
		zLo.Cmp(m.lo) >= 0 && zHi.Cmp(m.hi) <= 0 {
		return true
	}
	if !ChamferMiddleHeldRootInside(verts, tris, walls, root, zLo, zHi, radius) {
		return false
	}
	switch {
	case m.lo == nil:
		m.verts, m.tris, m.walls, m.root, m.radius = verts, tris, walls, root, radius
		m.lo, m.hi = new(big.Rat).Set(zLo), new(big.Rat).Set(zHi)
	case same && zLo.Cmp(m.hi) <= 0 && zHi.Cmp(m.lo) >= 0:
		if zLo.Cmp(m.lo) < 0 {
			m.lo = new(big.Rat).Set(zLo)
		}
		if zHi.Cmp(m.hi) > 0 {
			m.hi = new(big.Rat).Set(zHi)
		}
	}
	return true
}

// sameTooth reports whether the tooth arguments are the ones the memo's
// interval was answered for: the same backing arrays, lengths, wall range
// and radius.
func (m *ChamferHeldRootMemo) sameTooth(verts []r3.Vec, tris [][3]int, walls, root int,
	radius float64) bool {
	return len(verts) == len(m.verts) && len(tris) == len(m.tris) &&
		len(verts) > 0 && len(tris) > 0 && &verts[0] == &m.verts[0] && &tris[0] == &m.tris[0] &&
		walls == m.walls && root == m.root && radius == m.radius
}

// chamferMiddleZCmp is the sign of p.z − n/d for a positive d.
func chamferMiddleZCmp(p proofarith.Xpt, n, d *big.Int) int {
	a := new(big.Int).Mul(p.Z, d)
	b := new(big.Int).Mul(n, p.W)
	return a.Cmp(b)
}

func chamferMiddleClipZ(poly []proofarith.Xpt, n, d *big.Int,
	keepAbove bool) []proofarith.Xpt {
	if len(poly) == 0 {
		return nil
	}
	inside := func(p proofarith.Xpt) bool {
		cmp := chamferMiddleZCmp(p, n, d)
		return keepAbove && cmp >= 0 || !keepAbove && cmp <= 0
	}
	var out []proofarith.Xpt
	for i, a := range poly {
		b := poly[(i+1)%len(poly)]
		inA, inB := inside(a), inside(b)
		if inA != inB {
			// t = (n/d − za)/(zb − za) = (n·Wa − Za·d)·Wb / (d·(Zb·Wa − Za·Wb)).
			tn := new(big.Int).Mul(n, a.W)
			tn.Sub(tn, new(big.Int).Mul(a.Z, d))
			tn.Mul(tn, b.W)
			td := new(big.Int).Mul(b.Z, a.W)
			td.Sub(td, new(big.Int).Mul(a.Z, b.W))
			td.Mul(td, d)
			out = append(out, proofarith.Xlerp(a, b, tn, td))
		}
		if inB {
			out = append(out, b)
		}
	}
	return out
}

// A strict separating line through the axis puts an entire convex q facet
// outside the exact bore circle. This is deliberately a refusal-only test.
func ChamferMiddleBeyondBore(q [3]proofarith.Xpt, radius float64) bool {
	if radius == 0 {
		return true
	}
	if radius < 0 || proofbound.IsNonFinite(radius) {
		return false
	}
	c := meshbool.XCentroid(q[0], q[1], q[2])
	axis := proofarith.XptOf(r3.Vec{X: c.Vec().X, Y: c.Vec().Y})
	norm2 := proofarith.XdotRat(axis, axis)
	if norm2.Sign() == 0 {
		return false
	}
	r := proofarith.FloatRat(radius)
	threshold := new(big.Rat).Mul(new(big.Rat).Mul(r, r), norm2)
	for _, p := range q {
		dot := proofarith.XdotRat(axis, p)
		if dot.Sign() <= 0 || new(big.Rat).Mul(dot, dot).Cmp(threshold) <= 0 {
			return false
		}
	}
	return true
}

// ChamferExactTriBox returns the outward-rounded float box of an exact
// triangle, or false when the box is not finite.
func ChamferExactTriBox(tri [3]proofarith.Xpt) ([2]r3.Vec, bool) {
	var box [2]r3.Vec
	for axis := range 3 {
		lo, hi := math.Inf(1), math.Inf(-1)
		for _, p := range tri {
			coord := meshbool.RatCoordOf(p, axis)
			lo = min(lo, proofbound.RatFloatDown(coord))
			hi = max(hi, proofbound.RatFloatUp(coord))
		}
		if proofbound.IsNonFinite(lo) || proofbound.IsNonFinite(hi) || lo > hi {
			return box, false
		}
		switch axis {
		case 0:
			box[0].X, box[1].X = lo, hi
		case 1:
			box[0].Y, box[1].Y = lo, hi
		default:
			box[0].Z, box[1].Z = lo, hi
		}
	}
	return box, true
}

// ChamferCorner is one exact point's outward-rounded float box and its
// float image, as ChamferExactTriBox and Xpt.Vec compute them. Box[0] holds
// the per-axis downward roundings and Box[1] the upward ones.
type ChamferCorner struct {
	Box   [2]r3.Vec
	Float r3.Vec
}

// ChamferCornerOf rounds one exact point once.
func ChamferCornerOf(p proofarith.Xpt) ChamferCorner {
	var c ChamferCorner
	for axis := range 3 {
		coord := meshbool.RatCoordOf(p, axis)
		lo, hi := proofbound.RatFloatDown(coord), proofbound.RatFloatUp(coord)
		switch axis {
		case 0:
			c.Box[0].X, c.Box[1].X = lo, hi
		case 1:
			c.Box[0].Y, c.Box[1].Y = lo, hi
		default:
			c.Box[0].Z, c.Box[1].Z = lo, hi
		}
	}
	c.Float = p.Vec()
	return c
}

// ChamferCornersBox is ChamferExactTriBox of the triangle with these
// corners: the same box when accepted, and false exactly when that returns
// false. The box returned with a refusal is unspecified and must be ignored.
func ChamferCornersBox(a, b, c ChamferCorner) ([2]r3.Vec, bool) {
	box := [2]r3.Vec{
		{X: min(a.Box[0].X, b.Box[0].X, c.Box[0].X), Y: min(a.Box[0].Y, b.Box[0].Y, c.Box[0].Y),
			Z: min(a.Box[0].Z, b.Box[0].Z, c.Box[0].Z)},
		{X: max(a.Box[1].X, b.Box[1].X, c.Box[1].X), Y: max(a.Box[1].Y, b.Box[1].Y, c.Box[1].Y),
			Z: max(a.Box[1].Z, b.Box[1].Z, c.Box[1].Z)},
	}
	for _, pair := range [3][2]float64{{box[0].X, box[1].X}, {box[0].Y, box[1].Y}, {box[0].Z, box[1].Z}} {
		if proofbound.IsNonFinite(pair[0]) || proofbound.IsNonFinite(pair[1]) || pair[0] > pair[1] {
			return [2]r3.Vec{}, false
		}
	}
	return box, true
}

// chamferPrismFaceCorners lists, for each face ChamferMiddlePrism builds, the
// indices into its corners array of that face's three corners: 0..2 are
// base[0..2] and 3..5 are q[0..2]. Face 0 is q; faces 1+2i and 2+2i, with
// j = (i+1) % 3, are {base[i], base[j], q[j]} and {base[i], q[j], q[i]}.
var chamferPrismFaceCorners = [7][3]int{
	{3, 4, 5},
	{0, 1, 4}, {0, 4, 3},
	{1, 2, 5}, {1, 5, 4},
	{2, 0, 3}, {2, 3, 5},
}

// The seven non-base faces bound one triangular prism. Exact contacts with
// the old shell may occur only on its original base facet. The old shell has
// no vertex inside the strict middle slab, so it cannot hide a separate
// boundary component inside this prism. One strict q-face witness then puts
// every open inward segment in the old held solid. The corners array holds
// the rounded corners of base[0..2] at indices 0..2 and of q[0..2] at 3..5.
func ChamferMiddlePrism(ctx context.Context, triCount int,
	old *LoftAuditData, oldBoxes [][2]r3.Vec, oldGrid *ChamferBoxGrid,
	oldParity *meshbool.ParityMesh,
	base, q [3]proofarith.Xpt, shift proofarith.Xpt,
	corners [6]ChamferCorner) error {
	if len(oldBoxes) != triCount || oldGrid == nil ||
		meshbool.RatCoordOf(shift, 2).Sign() != 0 {
		return fmt.Errorf("%w: retained middle prism has incomplete source geometry", decaderr.ErrUnsupported)
	}
	var faces [7][3]proofarith.Xpt
	faces[0] = q
	for i := range 3 {
		j := (i + 1) % 3
		faces[1+2*i] = [3]proofarith.Xpt{base[i], base[j], q[j]}
		faces[2+2*i] = [3]proofarith.Xpt{base[i], q[j], q[i]}
	}
	baseNormal := proofarith.Xcross(proofarith.Xsub(base[1], base[0]),
		proofarith.Xsub(base[2], base[0]))
	if proofarith.XdotSign(baseNormal, baseNormal) == 0 ||
		proofarith.XdotSign(baseNormal, shift) >= 0 {
		return fmt.Errorf("%w: retained middle collar has no inward prism", decaderr.ErrUnsupported)
	}
	for i, face := range faces {
		if i%2 == 0 && ctx.Err() != nil {
			return ctx.Err()
		}
		normal := proofarith.Xcross(proofarith.Xsub(face[1], face[0]),
			proofarith.Xsub(face[2], face[0]))
		if proofarith.XdotSign(normal, normal) == 0 {
			return fmt.Errorf("%w: retained middle collar has a flat side", decaderr.ErrUnsupported)
		}
		fc := chamferPrismFaceCorners[i]
		box, ok := ChamferCornersBox(corners[fc[0]], corners[fc[1]], corners[fc[2]])
		if !ok {
			return fmt.Errorf("%w: retained middle collar has an unbounded box", decaderr.ErrUnsupported)
		}
		floatFace := [3]r3.Vec{corners[fc[0]].Float, corners[fc[1]].Float, corners[fc[2]].Float}
		plane := NewLoftExactPlane(face[0], normal)
		candidates, err := oldGrid.candidates(box)
		if err != nil {
			return err
		}
		for _, j := range candidates {
			if !meshbool.BoxesOverlap(box, oldBoxes[j]) {
				continue
			}
			signsA := TrianglePlaneSigns(old.Planes[j], face)
			signsB := TrianglePlaneSigns(plane, old.Xtris[j])
			contact, err := meshbool.TriTriContactSpanCore(floatFace, old.Corners[j],
				face, old.Xtris[j], normal, old.Norms[j], nil, nil,
				&signsA, &signsB, false, r3.Vec{}, r3.Vec{}, false)
			if err != nil {
				return err
			}
			if contact.Kind == meshbool.ContactNone {
				continue
			}
			if contact.Kind == meshbool.ContactRegion ||
				proofarith.OrientSignExact(base[0], base[1], base[2], contact.P0) != 0 ||
				!meshbool.PointOnTri(contact.P0, base, baseNormal) {
				return fmt.Errorf("%w: retained middle collar crosses the old wall at facet %d",
					decaderr.ErrUnsupported, j)
			}
			if contact.Kind == meshbool.ContactSegment &&
				(proofarith.OrientSignExact(base[0], base[1], base[2], contact.P1) != 0 ||
					!meshbool.PointOnTri(contact.P1, base, baseNormal)) {
				return fmt.Errorf("%w: retained middle collar crosses the old wall at facet %d",
					decaderr.ErrUnsupported, j)
			}
		}
	}
	witness := meshbool.XCentroid(q[0], q[1], q[2])
	inside, on, err := meshbool.MeshParityAllContext(ctx, witness, oldParity)
	if err != nil || !inside || on {
		return fmt.Errorf("%w: retained middle collar ends outside the old solid", decaderr.ErrUnsupported)
	}
	return nil
}
