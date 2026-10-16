package loftcylinder

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// XY is an exact rational point in the cylinder's cross-section plane.
type XY struct{ X, Y *big.Rat }

// AuditAnnulus checks the exact planar area and directed edge
// balance of one rebuilt cap. The old disk audit supplies the outer loop; the
// bore ring and side-clearance gates supply a disjoint inner loop. The whole
// shell crossing audit excludes overlapping or folded cap facets.
func AuditAnnulus(ctx context.Context, verts []r3.Vec, tris [][3]int, src []int,
	outer, inner []int, group int, z float64, bottom bool) bool {
	if len(outer) < 3 || len(inner) < 3 {
		return false
	}
	allowed := make(map[int]struct{}, len(outer)+len(inner))
	loopArea := func(loop []int) *big.Rat {
		area := new(big.Rat)
		for i, a := range loop {
			b := loop[(i+1)%len(loop)]
			if a < 0 || a >= len(verts) || b < 0 || b >= len(verts) ||
				verts[a].Z != z || verts[b].Z != z {
				return nil
			}
			if _, exists := allowed[a]; exists {
				return nil
			}
			allowed[a] = struct{}{}
			pa, pb := ExactXY(verts[a]), ExactXY(verts[b])
			area.Add(area, proofarith.SubRat(new(big.Rat),
				proofarith.MulRat(new(big.Rat), pa.X, pb.Y), proofarith.MulRat(new(big.Rat), pa.Y, pb.X)))
		}
		return area
	}
	outerArea, innerArea := loopArea(outer), loopArea(inner)
	if outerArea == nil || innerArea == nil || outerArea.Sign() <= 0 ||
		innerArea.Sign() <= 0 || outerArea.Cmp(innerArea) <= 0 {
		return false
	}
	expected := proofarith.SubRat(new(big.Rat), outerArea, innerArea)
	actual := new(big.Rat)
	edges := make(map[[2]int]int)
	count := 0
	for i, tri := range tris {
		if i%256 == 0 && ctx.Err() != nil {
			return false
		}
		if src[i] != group {
			continue
		}
		count++
		for _, vertex := range tri {
			if _, ok := allowed[vertex]; !ok || verts[vertex].Z != z {
				return false
			}
		}
		a := ExactXY(verts[tri[0]])
		b := ExactXY(verts[tri[1]])
		c := ExactXY(verts[tri[2]])
		twice := Orient(a, b, c)
		if bottom {
			if twice.Sign() >= 0 {
				return false
			}
			proofarith.SubRat(actual, actual, twice)
		} else {
			if twice.Sign() <= 0 {
				return false
			}
			proofarith.AddRat(actual, actual, twice)
		}
		for k := range 3 {
			edges[[2]int{tri[k], tri[(k+1)%3]}]++
		}
	}
	if count == 0 || actual.Cmp(expected) != 0 {
		return false
	}
	consumeBoundary := func(loop []int, reverse bool) bool {
		for i, a := range loop {
			b := loop[(i+1)%len(loop)]
			if reverse {
				a, b = b, a
			}
			if edges[[2]int{a, b}]-edges[[2]int{b, a}] != 1 {
				return false
			}
			delete(edges, [2]int{a, b})
			delete(edges, [2]int{b, a})
		}
		return true
	}
	if !consumeBoundary(outer, bottom) || !consumeBoundary(inner, !bottom) {
		return false
	}
	for edge, n := range edges {
		if n != 1 || edges[[2]int{edge[1], edge[0]}] != 1 {
			return false
		}
	}
	return true
}

// ExactXY returns the exact X and Y of a held vertex.
func ExactXY(p r3.Vec) XY {
	return XY{proofarith.FloatRat(p.X), proofarith.FloatRat(p.Y)}
}

// Orient returns twice the signed area of triangle abc.
func Orient(a, b, c XY) *big.Rat {
	abx := proofarith.SubRat(new(big.Rat), b.X, a.X)
	aby := proofarith.SubRat(new(big.Rat), b.Y, a.Y)
	acx := proofarith.SubRat(new(big.Rat), c.X, a.X)
	acy := proofarith.SubRat(new(big.Rat), c.Y, a.Y)
	return proofarith.SubRat(new(big.Rat), proofarith.MulRat(new(big.Rat), abx, acy), proofarith.MulRat(new(big.Rat), aby, acx))
}

// edgesMeetXP decides the same closed-segment contact as
// EdgesMeet. Strict separation of rounded float bounds implies
// strict separation of the rational bounds. Other decisions use exact
// rational comparisons or meshbool.Cross2xSign's certified sign filter. A
// point without a finite float image takes the rational body.
func edgesMeetXP(a, b, c, d meshbool.Xp2) bool {
	if !a.FloatFinite || !b.FloatFinite || !c.FloatFinite || !d.FloatFinite {
		return EdgesMeet(XY{a.U, a.V}, XY{b.U, b.V},
			XY{c.U, c.V}, XY{d.U, d.V})
	}
	between := func(a, b, p *big.Rat) bool {
		return (a.Cmp(p) <= 0 && p.Cmp(b) <= 0) ||
			(b.Cmp(p) <= 0 && p.Cmp(a) <= 0)
	}
	if a.Fu > c.Fu && a.Fu > d.Fu && b.Fu > c.Fu && b.Fu > d.Fu ||
		a.Fu < c.Fu && a.Fu < d.Fu && b.Fu < c.Fu && b.Fu < d.Fu ||
		a.Fv > c.Fv && a.Fv > d.Fv && b.Fv > c.Fv && b.Fv > d.Fv ||
		a.Fv < c.Fv && a.Fv < d.Fv && b.Fv < c.Fv && b.Fv < d.Fv {
		return false
	}
	abc, abd := meshbool.Cross2xSign(a, b, c), meshbool.Cross2xSign(a, b, d)
	cda, cdb := meshbool.Cross2xSign(c, d, a), meshbool.Cross2xSign(c, d, b)
	if abc == 0 && between(a.U, b.U, c.U) && between(a.V, b.V, c.V) ||
		abd == 0 && between(a.U, b.U, d.U) && between(a.V, b.V, d.V) ||
		cda == 0 && between(c.U, d.U, a.U) && between(c.V, d.V, a.V) ||
		cdb == 0 && between(c.U, d.U, b.U) && between(c.V, d.V, b.V) {
		return true
	}
	return abc != abd && cda != cdb
}

// EdgesMeet reports whether two closed segments share a point, by exact comparisons.
func EdgesMeet(a, b, c, d XY) bool {
	between := func(a, b, p *big.Rat) bool {
		return (proofarith.CmpRat(a, p) <= 0 && proofarith.CmpRat(p, b) <= 0) ||
			(proofarith.CmpRat(b, p) <= 0 && proofarith.CmpRat(p, a) <= 0)
	}
	if proofarith.CmpRat(a.X, c.X) > 0 && proofarith.CmpRat(a.X, d.X) > 0 &&
		proofarith.CmpRat(b.X, c.X) > 0 && proofarith.CmpRat(b.X, d.X) > 0 ||
		proofarith.CmpRat(a.X, c.X) < 0 && proofarith.CmpRat(a.X, d.X) < 0 &&
			proofarith.CmpRat(b.X, c.X) < 0 && proofarith.CmpRat(b.X, d.X) < 0 ||
		proofarith.CmpRat(a.Y, c.Y) > 0 && proofarith.CmpRat(a.Y, d.Y) > 0 &&
			proofarith.CmpRat(b.Y, c.Y) > 0 && proofarith.CmpRat(b.Y, d.Y) > 0 ||
		proofarith.CmpRat(a.Y, c.Y) < 0 && proofarith.CmpRat(a.Y, d.Y) < 0 &&
			proofarith.CmpRat(b.Y, c.Y) < 0 && proofarith.CmpRat(b.Y, d.Y) < 0 {
		return false
	}
	abc, abd := Orient(a, b, c).Sign(), Orient(a, b, d).Sign()
	cda, cdb := Orient(c, d, a).Sign(), Orient(c, d, b).Sign()
	if abc == 0 && between(a.X, b.X, c.X) && between(a.Y, b.Y, c.Y) ||
		abd == 0 && between(a.X, b.X, d.X) && between(a.Y, b.Y, d.Y) ||
		cda == 0 && between(c.X, d.X, a.X) && between(c.Y, d.Y, a.Y) ||
		cdb == 0 && between(c.X, d.X, b.X) && between(c.Y, d.Y, b.Y) {
		return true
	}
	return abc != abd && cda != cdb
}

// AuditMaxScans is the crossing-audit scan ceiling for a structural join. The
// join has at most 32 separately certified Loft teeth. Its final crossing
// audit and the matched centered bore may scan twice the generic mesh
// ceiling; exact pair classifications remain unchanged.
const AuditMaxScans = 16_000_000

// The cap group must tile exactly one simple CCW outline, with the origin
// strictly inside. A later centered bore may use that outline as its source.
func AuditCap(ctx context.Context, verts []r3.Vec, tris [][3]int, src []int,
	outline []int, group int, z float64, bottom bool) bool {
	n := len(outline)
	if n < 3 {
		return false
	}
	points := make([]XY, n)
	xpoints := make([]meshbool.Xp2, n)
	seen := make(map[int]struct{}, n)
	for i, index := range outline {
		if index < 0 || index >= len(verts) || verts[index].Z != z {
			return false
		}
		if _, duplicate := seen[index]; duplicate {
			return false
		}
		seen[index] = struct{}{}
		points[i] = ExactXY(verts[index])
		xpoints[i] = meshbool.NewXP2(points[i].X, points[i].Y)
	}
	polyTwice := new(big.Rat)
	inside := false
	zero := new(big.Rat)
	for i := range points {
		a, b := points[i], points[(i+1)%n]
		if a.X.Cmp(b.X) == 0 && a.Y.Cmp(b.Y) == 0 {
			return false
		}
		polyTwice.Add(polyTwice, proofarith.SubRat(new(big.Rat),
			proofarith.MulRat(new(big.Rat), a.X, b.Y), proofarith.MulRat(new(big.Rat), a.Y, b.X)))
		if a.Y.Sign() == 0 && a.X.Sign() == 0 {
			return false
		}
		if (a.Y.Sign() > 0) != (b.Y.Sign() > 0) {
			x := proofarith.SubRat(new(big.Rat), a.X,
				new(big.Rat).Quo(proofarith.MulRat(new(big.Rat), a.Y,
					proofarith.SubRat(new(big.Rat), b.X, a.X)), proofarith.SubRat(new(big.Rat), b.Y, a.Y)))
			if x.Sign() == 0 {
				return false
			}
			if x.Sign() > 0 {
				inside = !inside
			}
		}
		for j := i + 1; j < n; j++ {
			if j == i+1 || i == 0 && j == n-1 {
				continue
			}
			if j%256 == 0 && ctx.Err() != nil {
				return false
			}
			if edgesMeetXP(xpoints[i], xpoints[(i+1)%n], xpoints[j], xpoints[(j+1)%n]) {
				return false
			}
		}
	}
	if polyTwice.Sign() <= 0 || !inside {
		return false
	}
	areaTwice := new(big.Rat)
	edges := make(map[[2]int]int)
	for i, tri := range tris {
		if src[i] != group {
			continue
		}
		for _, index := range tri {
			if _, ok := seen[index]; !ok || verts[index].Z != z {
				return false
			}
		}
		a := ExactXY(verts[tri[0]])
		b := ExactXY(verts[tri[1]])
		c := ExactXY(verts[tri[2]])
		twice := Orient(a, b, c)
		if bottom {
			if twice.Sign() >= 0 {
				return false
			}
			proofarith.SubRat(areaTwice, areaTwice, twice)
		} else {
			if twice.Sign() <= 0 {
				return false
			}
			proofarith.AddRat(areaTwice, areaTwice, twice)
		}
		for k := range 3 {
			edges[[2]int{tri[k], tri[(k+1)%3]}]++
		}
	}
	if areaTwice.Cmp(polyTwice) != 0 {
		return false
	}
	for i := range outline {
		a, b := outline[i], outline[(i+1)%n]
		if bottom {
			a, b = b, a
		}
		if edges[[2]int{a, b}]-edges[[2]int{b, a}] != 1 {
			return false
		}
		delete(edges, [2]int{a, b})
		delete(edges, [2]int{b, a})
	}
	for edge, count := range edges {
		if count != 1 || edges[[2]int{edge[1], edge[0]}] != 1 {
			return false
		}
	}
	return areaTwice.Cmp(zero) > 0
}

// The circular stations must bound one disk around the axis. Combined with
// the radial shell bound, this makes the polygonal bore differ from the exact
// cylinder only inside that shell, including its occupied volume.
func BoreRingSimple(ctx context.Context, verts []r3.Vec, ring []int) bool {
	n := len(ring)
	if n < 3 {
		return false
	}
	points := make([]XY, n)
	filtered := make([]meshbool.Xp2, n)
	for i, index := range ring {
		points[i] = ExactXY(verts[index])
		filtered[i] = meshbool.NewXP2(points[i].X, points[i].Y)
	}
	area2 := new(big.Rat)
	inside := false
	for i := range points {
		if i%32 == 0 && ctx.Err() != nil {
			return false
		}
		a, b := points[i], points[(i+1)%n]
		area2.Add(area2, proofarith.SubRat(new(big.Rat),
			proofarith.MulRat(new(big.Rat), a.X, b.Y), proofarith.MulRat(new(big.Rat), a.Y, b.X)))
		if (a.Y.Sign() > 0) != (b.Y.Sign() > 0) {
			x := proofarith.SubRat(new(big.Rat), a.X, new(big.Rat).Quo(proofarith.MulRat(new(big.Rat), a.Y,
				proofarith.SubRat(new(big.Rat), b.X, a.X)), proofarith.SubRat(new(big.Rat), b.Y, a.Y)))
			if x.Sign() == 0 {
				return false
			}
			if x.Sign() > 0 {
				inside = !inside
			}
		}
		for j := i + 1; j < n; j++ {
			if j == i+1 || i == 0 && j == n-1 {
				continue
			}
			if edgesMeetXP(filtered[i], filtered[(i+1)%n],
				filtered[j], filtered[(j+1)%n]) {
				return false
			}
		}
	}
	return area2.Sign() > 0 && inside
}
