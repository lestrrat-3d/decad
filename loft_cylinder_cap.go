package decad

import (
	"context"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

type loftCylinderXY struct{ x, y *big.Rat }

func loftCylinderExactXY(p r3.Vec) loftCylinderXY {
	return loftCylinderXY{proofarith.FloatRat(p.X), proofarith.FloatRat(p.Y)}
}

func loftCylinderOrient(a, b, c loftCylinderXY) *big.Rat {
	abx := new(big.Rat).Sub(b.x, a.x)
	aby := new(big.Rat).Sub(b.y, a.y)
	acx := new(big.Rat).Sub(c.x, a.x)
	acy := new(big.Rat).Sub(c.y, a.y)
	return new(big.Rat).Sub(new(big.Rat).Mul(abx, acy), new(big.Rat).Mul(aby, acx))
}

func loftCylinderEdgesMeet(a, b, c, d loftCylinderXY) bool {
	between := func(a, b, p *big.Rat) bool {
		return (a.Cmp(p) <= 0 && p.Cmp(b) <= 0) ||
			(b.Cmp(p) <= 0 && p.Cmp(a) <= 0)
	}
	if a.x.Cmp(c.x) > 0 && a.x.Cmp(d.x) > 0 &&
		b.x.Cmp(c.x) > 0 && b.x.Cmp(d.x) > 0 ||
		a.x.Cmp(c.x) < 0 && a.x.Cmp(d.x) < 0 &&
			b.x.Cmp(c.x) < 0 && b.x.Cmp(d.x) < 0 ||
		a.y.Cmp(c.y) > 0 && a.y.Cmp(d.y) > 0 &&
			b.y.Cmp(c.y) > 0 && b.y.Cmp(d.y) > 0 ||
		a.y.Cmp(c.y) < 0 && a.y.Cmp(d.y) < 0 &&
			b.y.Cmp(c.y) < 0 && b.y.Cmp(d.y) < 0 {
		return false
	}
	abc, abd := loftCylinderOrient(a, b, c).Sign(), loftCylinderOrient(a, b, d).Sign()
	cda, cdb := loftCylinderOrient(c, d, a).Sign(), loftCylinderOrient(c, d, b).Sign()
	if abc == 0 && between(a.x, b.x, c.x) && between(a.y, b.y, c.y) ||
		abd == 0 && between(a.x, b.x, d.x) && between(a.y, b.y, d.y) ||
		cda == 0 && between(c.x, d.x, a.x) && between(c.y, d.y, a.y) ||
		cdb == 0 && between(c.x, d.x, b.x) && between(c.y, d.y, b.y) {
		return true
	}
	return abc != abd && cda != cdb
}

// The cap group must tile exactly one simple CCW outline, with the origin
// strictly inside. A later centered bore may use that outline as its source.
func loftCylinderAuditCap(ctx context.Context, mesh *bevelJoinedMesh,
	outline []int, group int, z float64, bottom bool) bool {
	n := len(outline)
	if n < 3 {
		return false
	}
	points := make([]loftCylinderXY, n)
	seen := make(map[int]struct{}, n)
	for i, index := range outline {
		if index < 0 || index >= len(mesh.verts) || mesh.verts[index].Z != z {
			return false
		}
		if _, duplicate := seen[index]; duplicate {
			return false
		}
		seen[index] = struct{}{}
		points[i] = loftCylinderExactXY(mesh.verts[index])
	}
	polyTwice := new(big.Rat)
	inside := false
	zero := new(big.Rat)
	for i := range points {
		a, b := points[i], points[(i+1)%n]
		if a.x.Cmp(b.x) == 0 && a.y.Cmp(b.y) == 0 {
			return false
		}
		polyTwice.Add(polyTwice, new(big.Rat).Sub(
			new(big.Rat).Mul(a.x, b.y), new(big.Rat).Mul(a.y, b.x)))
		if a.y.Sign() == 0 && a.x.Sign() == 0 {
			return false
		}
		if (a.y.Sign() > 0) != (b.y.Sign() > 0) {
			x := new(big.Rat).Sub(a.x,
				new(big.Rat).Quo(new(big.Rat).Mul(a.y,
					new(big.Rat).Sub(b.x, a.x)), new(big.Rat).Sub(b.y, a.y)))
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
			if loftCylinderEdgesMeet(a, b, points[j], points[(j+1)%n]) {
				return false
			}
		}
	}
	if polyTwice.Sign() <= 0 || !inside {
		return false
	}
	areaTwice := new(big.Rat)
	edges := make(map[[2]int]int)
	for i, tri := range mesh.tris {
		if mesh.src[i] != group {
			continue
		}
		for _, index := range tri {
			if _, ok := seen[index]; !ok || mesh.verts[index].Z != z {
				return false
			}
		}
		a := loftCylinderExactXY(mesh.verts[tri[0]])
		b := loftCylinderExactXY(mesh.verts[tri[1]])
		c := loftCylinderExactXY(mesh.verts[tri[2]])
		twice := loftCylinderOrient(a, b, c)
		if bottom {
			if twice.Sign() >= 0 {
				return false
			}
			areaTwice.Sub(areaTwice, twice)
		} else {
			if twice.Sign() <= 0 {
				return false
			}
			areaTwice.Add(areaTwice, twice)
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
