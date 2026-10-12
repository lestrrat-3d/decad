package decad

import (
	"context"
	"fmt"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

type loftCylinderXY struct{ x, y *big.Rat }

// loftCylinderTagCaps gives the two already audited cap groups the result's
// own cap roles and the source blank's exact planar surface. The joined cap
// audit proves each disk tiles its outline; the bore path audits each annulus
// before calling this helper. The mesh's remaining groups keep their
// faceted surface identity.
func loftCylinderTagCaps(pp *facetedPayload, blank *Body, groups [2]int,
	levels [2]float64, ref producerID) error {
	if blank == nil || len(pp.tris) != len(pp.src) || groups[0] == groups[1] {
		return fmt.Errorf("%w: loft/cylinder caps have no separate source groups", ErrUnsupported)
	}
	roles := [2]string{roleCapStart, roleCapEnd}
	for cap, group := range groups {
		if group < 0 || group >= len(pp.groups) {
			return fmt.Errorf("%w: loft/cylinder cap names no source group", ErrUnsupported)
		}
		var source *Face
		for _, face := range blank.Faces() {
			for _, origin := range face.Origins() {
				if origin != (FeatureRef{producer: blank.origin.producer, Role: roles[cap]}) {
					continue
				}
				if source != nil {
					return fmt.Errorf("%w: loft/cylinder blank has duplicate cap roles", ErrUnsupported)
				}
				source = face
			}
		}
		if source == nil {
			return fmt.Errorf("%w: loft/cylinder blank has no cap source", ErrUnsupported)
		}
		plane, ok := source.Surface().(Plane)
		if !ok || plane.Frame.Origin().Z != levels[cap] {
			return fmt.Errorf("%w: loft/cylinder blank cap has no matching plane", ErrUnsupported)
		}
		want := float64(1)
		if cap == 0 {
			want = -1
		}
		if plane.Frame.N() != (r3.Vec{Z: want}) {
			return fmt.Errorf("%w: loft/cylinder blank cap has no axial normal", ErrUnsupported)
		}
		count := 0
		for i, tri := range pp.tris {
			if pp.src[i] != group {
				continue
			}
			count++
			for _, index := range tri {
				if index < 0 || index >= len(pp.verts) || pp.verts[index].Z != levels[cap] {
					return fmt.Errorf("%w: loft/cylinder cap facet leaves its source plane", ErrUnsupported)
				}
			}
		}
		if count == 0 {
			return fmt.Errorf("%w: loft/cylinder cap group has no facets", ErrUnsupported)
		}
		g := &pp.groups[group]
		g.origins = append(append([]FeatureRef(nil), g.origins...),
			FeatureRef{producer: ref, Role: roles[cap]})
		g.surface = plane
		g.planar = true
		g.normalBound = source.normalBound
		g.reversed = source.reversed
	}
	return nil
}

// loftCylinderTagBridgedTop replaces the blank's exact top Plane tag when
// an original tooth ends below the held cap. The cap role remains intact;
// buildFacetedBody supplies the source-to-held displacement on its surface.
func loftCylinderTagBridgedTop(pp *facetedPayload, topGroup int, axialBridge []float64) {
	for _, shift := range axialBridge {
		if shift > 0 {
			top := &pp.groups[topGroup]
			top.surface = nil
			top.planar = false
			return
		}
	}
}

// loftCylinderAuditAnnulus checks the exact planar area and directed edge
// balance of one rebuilt cap. The old disk audit supplies the outer loop; the
// bore ring and side-clearance gates supply a disjoint inner loop. The whole
// shell crossing audit excludes overlapping or folded cap facets.
func loftCylinderAuditAnnulus(ctx context.Context, mesh *joinedMesh,
	outer, inner []int, group int, z float64, bottom bool) bool {
	if len(outer) < 3 || len(inner) < 3 {
		return false
	}
	allowed := make(map[int]struct{}, len(outer)+len(inner))
	loopArea := func(loop []int) *big.Rat {
		area := new(big.Rat)
		for i, a := range loop {
			b := loop[(i+1)%len(loop)]
			if a < 0 || a >= len(mesh.verts) || b < 0 || b >= len(mesh.verts) ||
				mesh.verts[a].Z != z || mesh.verts[b].Z != z {
				return nil
			}
			if _, exists := allowed[a]; exists {
				return nil
			}
			allowed[a] = struct{}{}
			pa, pb := loftCylinderExactXY(mesh.verts[a]), loftCylinderExactXY(mesh.verts[b])
			area.Add(area, new(big.Rat).Sub(
				new(big.Rat).Mul(pa.x, pb.y), new(big.Rat).Mul(pa.y, pb.x)))
		}
		return area
	}
	outerArea, innerArea := loopArea(outer), loopArea(inner)
	if outerArea == nil || innerArea == nil || outerArea.Sign() <= 0 ||
		innerArea.Sign() <= 0 || outerArea.Cmp(innerArea) <= 0 {
		return false
	}
	expected := new(big.Rat).Sub(outerArea, innerArea)
	actual := new(big.Rat)
	edges := make(map[[2]int]int)
	count := 0
	for i, tri := range mesh.tris {
		if i%256 == 0 && ctx.Err() != nil {
			return false
		}
		if mesh.src[i] != group {
			continue
		}
		count++
		for _, vertex := range tri {
			if _, ok := allowed[vertex]; !ok || mesh.verts[vertex].Z != z {
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
			actual.Sub(actual, twice)
		} else {
			if twice.Sign() <= 0 {
				return false
			}
			actual.Add(actual, twice)
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
func loftCylinderAuditCap(ctx context.Context, mesh *joinedMesh,
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
