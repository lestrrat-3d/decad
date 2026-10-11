package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// A retained middle facet is an exact subset of an old held wall. An inward
// horizontal collar ending inside the source solid gives every point of that
// facet a source-cut boundary point within the collar's length.
func loftCylinderChamferMiddleSideReach(ctx context.Context, fp facetedPayload,
	mesh *bevelJoinedMesh, source loftCylinderChamferSource, ideal []proofarith.Xpt,
	middle [2]int, clipBound float64) error {
	if middle[0] < 0 || middle[1] > len(mesh.tris) || middle[0] >= middle[1] ||
		len(fp.src) != len(fp.tris) || proofbound.IsNonFinite(clipBound) {
		return fmt.Errorf("%w: cap chamfer has no retained middle side", ErrUnsupported)
	}
	faces, err := loftCylinderChamferToothFaces(source)
	if err != nil {
		return err
	}
	old := loftmesh.NewLoftAuditData(fp.verts, fp.tris)
	oldBoxes := make([][2]r3.Vec, len(fp.tris))
	for i, tri := range fp.tris {
		oldBoxes[i] = meshbool.TriBox(fp.verts, tri)
	}
	oldGrid, err := loftCylinderChamferBoxGridOf(oldBoxes, source.z0, source.z1)
	if err != nil {
		return err
	}
	// A whole old boundary component cannot hide inside a middle collar:
	// every old vertex is on a cap plane, outside every strict middle prism.
	for _, p := range fp.verts {
		if p.Z != source.z0 && p.Z != source.z1 {
			return fmt.Errorf("%w: joined source has an interior-height vertex", ErrUnsupported)
		}
	}
	oldParity := meshbool.NewParityMesh(fp.verts, fp.tris)
	oldIndices := make([]int, len(fp.tris))
	for i := range oldIndices {
		oldIndices[i] = i
	}
	toothParity := make(map[producerID]*meshbool.ParityMesh)
	toothIndices := make(map[producerID][]int)
	for i := middle[0]; i < middle[1]; i++ {
		if i%32 == 0 && ctx.Err() != nil {
			return ctx.Err()
		}
		group := mesh.src[i]
		if group < 0 || group >= len(mesh.groups) || len(mesh.groups[group].origins) == 0 {
			return fmt.Errorf("%w: retained middle facet lost its source group", ErrUnsupported)
		}
		origin := mesh.groups[group].origins[0]
		var base [3]proofarith.Xpt
		for j, index := range mesh.tris[i] {
			if index < 0 || index >= len(ideal) {
				return fmt.Errorf("%w: retained middle facet has an invalid vertex", ErrUnsupported)
			}
			base[j] = ideal[index]
			z := meshbool.RatCoordOf(base[j], 2)
			if z.Cmp(proofarith.FloatRat(source.z0)) <= 0 ||
				z.Cmp(proofarith.FloatRat(source.z1)) >= 0 {
				return fmt.Errorf("%w: retained middle facet reaches a source cap", ErrUnsupported)
			}
		}
		accepted := false
		last := "no inward source witness"
		for _, distance := range [...]float64{0.08, 0.06, 0.04, 0.02, 0.01} {
			q, shift, err := loftCylinderChamferMiddleOffset(base, distance, clipBound)
			if err != nil {
				return err
			}
			if !loftCylinderChamferMiddleBeyondBore(q, source.boreRadius) {
				last = "inward patch reaches the source bore"
				continue
			}
			if !loftCylinderChamferMiddleBlank(q, source.blankRadius) {
				face, ok := faces[origin]
				if !ok {
					last = "source face has no authenticated tooth"
					continue
				}
				key := origin.producer
				parity, ok := toothParity[key]
				if !ok {
					parity = meshbool.NewParityMesh(face.loft.verts, face.loft.tris)
					toothParity[key] = parity
					indices := make([]int, len(face.loft.tris))
					for j := range indices {
						indices[j] = j
					}
					toothIndices[key] = indices
				}
				if err := loftCylinderChamferMiddleToothSource(ctx, q, face,
					parity, toothIndices[key], source.blankRadius); err != nil {
					last = err.Error()
					continue
				}
			}
			if err := loftCylinderChamferMiddlePrism(ctx, fp, old, oldBoxes, oldGrid,
				oldParity, oldIndices, base, q, shift); err != nil {
				if ctx.Err() != nil {
					return ctx.Err()
				}
				last = err.Error()
				continue
			}
			accepted = true
			break
		}
		if !accepted {
			return fmt.Errorf("%w: retained middle facet %d (origin %v) has no certified inward collar: %s",
				ErrUnsupported, i, origin, last)
		}
	}
	return nil
}

func loftCylinderChamferMiddleOffset(base [3]proofarith.Xpt,
	distance, clipBound float64) ([3]proofarith.Xpt, proofarith.Xpt, error) {
	var q [3]proofarith.Xpt
	n := proofarith.Xcross(proofarith.Xsub(base[1], base[0]),
		proofarith.Xsub(base[2], base[0]))
	f := n.Vec()
	length := math.Hypot(f.X, f.Y)
	if length <= 0 || proofbound.IsNonFinite(length) {
		return q, proofarith.Xpt{},
			fmt.Errorf("%w: retained middle wall has no horizontal normal", ErrUnsupported)
	}
	shiftFloat := r3.Vec{X: -distance * f.X / length, Y: -distance * f.Y / length}
	if !proofbound.FiniteVec(shiftFloat) {
		return q, proofarith.Xpt{},
			fmt.Errorf("%w: retained middle collar has no finite direction", ErrUnsupported)
	}
	shift := proofarith.XptOf(shiftFloat)
	allowance := 0.1 - clipBound
	if allowance <= 0 || proofbound.IsNonFinite(allowance) {
		return q, proofarith.Xpt{},
			fmt.Errorf("%w: retained middle collar has no boundary allowance", ErrUnsupported)
	}
	limit := proofarith.FloatRat(math.Nextafter(allowance, math.Inf(-1)))
	limit.Mul(limit, limit)
	if proofarith.XdotSign(n, shift) >= 0 ||
		proofarith.XdotRat(shift, shift).Cmp(limit) >= 0 {
		return q, proofarith.Xpt{},
			fmt.Errorf("%w: retained middle collar exceeds its boundary allowance", ErrUnsupported)
	}
	for i := range q {
		q[i] = loftCylinderChamferTranslate(base[i], shift)
	}
	return q, shift, nil
}

func loftCylinderChamferTranslate(p, shift proofarith.Xpt) proofarith.Xpt {
	den := new(big.Int).Mul(p.W, shift.W)
	coord := func(a, b *big.Int) *big.Int {
		return new(big.Int).Add(new(big.Int).Mul(a, shift.W), new(big.Int).Mul(b, p.W))
	}
	return proofarith.Xpt(proofarith.XhpStripTwosOwned(proofarith.Xhp{
		X: coord(p.X, shift.X), Y: coord(p.Y, shift.Y),
		Z: coord(p.Z, shift.Z), W: den,
	}))
}

func loftCylinderChamferMiddleBlank(q [3]proofarith.Xpt, radius float64) bool {
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

// A whole inward patch is in the true Loft tooth when it misses every moving
// wall by its matched departure and one held-tooth point is inside. If only
// hidden root walls meet it, use the exact blank disk for that part instead.
func loftCylinderChamferMiddleToothSource(ctx context.Context, q [3]proofarith.Xpt,
	face loftCylinderChamferToothFace, parity *meshbool.ParityMesh, indices []int,
	blankRadius float64) error {
	lp := face.loft
	if lp.walls <= 0 || lp.walls > len(lp.tris) || face.root < 3 ||
		face.scale == nil || face.scale.Sign() <= 0 ||
		2*face.root >= lp.walls || face.bound < 0 || proofbound.IsNonFinite(face.bound) {
		return fmt.Errorf("%w: inward patch lacks a bounded Loft wall", ErrUnsupported)
	}
	fullClear := true
	for j := range lp.walls {
		if j%64 == 0 && ctx.Err() != nil {
			return ctx.Err()
		}
		tri := lp.tris[j]
		other := [3]proofarith.Xpt{
			proofarith.XptOf(lp.verts[tri[0]]),
			proofarith.XptOf(lp.verts[tri[1]]),
			proofarith.XptOf(lp.verts[tri[2]]),
		}
		if loftCylinderChamferTriangleGapSplit(q, other, face.bound, 2) ||
			meshbool.DisjointTriDistanceAbove(q, other, face.bound) {
			continue
		}
		if j < 2*face.root {
			return fmt.Errorf("%w: inward patch approaches exterior tooth facet %d", ErrUnsupported, j)
		}
		fullClear = false
	}
	if fullClear {
		witness := meshbool.XCentroid(q[0], q[1], q[2])
		inside, on, err := meshbool.MeshParityPreparedContext(ctx, witness, parity, indices)
		if err != nil {
			return err
		}
		if inside && !on {
			return nil
		}
	}
	// The original root arc is hidden by the blank over this middle slab.
	// Both matched held and true root patches must remain in the convex disk;
	// their entire same-height homotopy then stays in that disk too.
	circle, err := loftCylinderSourceCircleOf(lp, blankRadius)
	if err != nil {
		return err
	}
	zLo, zHi := meshbool.RatCoordOf(q[0], 2), meshbool.RatCoordOf(q[0], 2)
	for _, p := range q[1:] {
		z := meshbool.RatCoordOf(p, 2)
		if z.Cmp(zLo) < 0 {
			zLo = z
		}
		if z.Cmp(zHi) > 0 {
			zHi = z
		}
	}
	mappedLo := new(big.Rat).Quo(zLo, face.scale)
	mappedHi := new(big.Rat).Quo(zHi, face.scale)
	if mappedLo.Cmp(circle.safeBottom) <= 0 || mappedHi.Cmp(circle.safeTop) >= 0 ||
		!loftCylinderChamferMiddleHeldRootInside(lp, face.root, mappedLo, mappedHi, blankRadius) {
		return fmt.Errorf("%w: inward patch has no hidden root-wall interval", ErrUnsupported)
	}
	r := proofarith.FloatRat(blankRadius)
	r2 := new(big.Rat).Mul(r, r)
	outside := 0
	for _, p := range q {
		x, y := meshbool.RatCoordOf(p, 0), meshbool.RatCoordOf(p, 1)
		diff := new(big.Rat).Add(new(big.Rat).Mul(x, x), new(big.Rat).Mul(y, y))
		if diff.Cmp(r2) <= 0 {
			continue
		}
		outside++
		inside, on, err := meshbool.MeshParityPreparedContext(ctx, p, parity, indices)
		if err != nil {
			return err
		}
		if !inside || on {
			return fmt.Errorf("%w: inward patch exterior vertex leaves its held tooth", ErrUnsupported)
		}
	}
	if outside == 0 {
		return fmt.Errorf("%w: inward patch root split has no exterior witness", ErrUnsupported)
	}
	// Each connected part of a triangle outside a centered convex disk reaches
	// an outside vertex: from x choose a vertex v with x·v >= |x|², and the
	// segment x--v stays outside. No source wall meets that part, so its
	// inside vertex fixes occupancy. The remaining patch is inside the blank.
	return nil
}

func loftCylinderChamferMiddleHeldRootInside(lp loftPayload, root int,
	zLo, zHi *big.Rat, radius float64) bool {
	if root < 0 || lp.walls > len(lp.tris) || zLo == nil || zHi == nil ||
		zLo.Cmp(zHi) >= 0 || radius <= 0 || proofbound.IsNonFinite(radius) {
		return false
	}
	r := proofarith.FloatRat(radius)
	r2 := new(big.Rat).Mul(r, r)
	for j := 2 * root; j < lp.walls; j++ {
		tri := lp.tris[j]
		poly := []proofarith.Xpt{
			proofarith.XptOf(lp.verts[tri[0]]),
			proofarith.XptOf(lp.verts[tri[1]]),
			proofarith.XptOf(lp.verts[tri[2]]),
		}
		poly = loftCylinderChamferMiddleClipZ(poly, zLo, true)
		poly = loftCylinderChamferMiddleClipZ(poly, zHi, false)
		for _, p := range poly {
			x, y := meshbool.RatCoordOf(p, 0), meshbool.RatCoordOf(p, 1)
			radius2 := new(big.Rat).Add(new(big.Rat).Mul(x, x), new(big.Rat).Mul(y, y))
			if radius2.Cmp(r2) >= 0 {
				return false
			}
		}
	}
	return true
}

func loftCylinderChamferMiddleClipZ(poly []proofarith.Xpt, level *big.Rat,
	keepAbove bool) []proofarith.Xpt {
	if len(poly) == 0 {
		return nil
	}
	inside := func(p proofarith.Xpt) bool {
		cmp := meshbool.RatCoordOf(p, 2).Cmp(level)
		return keepAbove && cmp >= 0 || !keepAbove && cmp <= 0
	}
	var out []proofarith.Xpt
	for i, a := range poly {
		b := poly[(i+1)%len(poly)]
		inA, inB := inside(a), inside(b)
		if inA != inB {
			za, zb := meshbool.RatCoordOf(a, 2), meshbool.RatCoordOf(b, 2)
			t := new(big.Rat).Quo(new(big.Rat).Sub(level, za), new(big.Rat).Sub(zb, za))
			out = append(out, proofarith.Xlerp(a, b, t.Num(), t.Denom()))
		}
		if inB {
			out = append(out, b)
		}
	}
	return out
}

// A strict separating line through the axis puts an entire convex q facet
// outside the exact bore circle. This is deliberately a refusal-only test.
func loftCylinderChamferMiddleBeyondBore(q [3]proofarith.Xpt, radius float64) bool {
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

func loftCylinderChamferExactTriBox(tri [3]proofarith.Xpt) ([2]r3.Vec, bool) {
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

// The seven non-base faces bound one triangular prism. Exact contacts with
// the old shell may occur only on its original base facet. The old shell has
// no vertex inside the strict middle slab, so it cannot hide a separate
// boundary component inside this prism. One strict q-face witness then puts
// every open inward segment in the old held solid.
func loftCylinderChamferMiddlePrism(ctx context.Context, fp facetedPayload,
	old *loftmesh.LoftAuditData, oldBoxes [][2]r3.Vec, oldGrid *loftCylinderChamferBoxGrid,
	oldParity *meshbool.ParityMesh,
	oldIndices []int, base, q [3]proofarith.Xpt, shift proofarith.Xpt) error {
	if len(oldBoxes) != len(fp.tris) || oldGrid == nil ||
		meshbool.RatCoordOf(shift, 2).Sign() != 0 {
		return fmt.Errorf("%w: retained middle prism has incomplete source geometry", ErrUnsupported)
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
		return fmt.Errorf("%w: retained middle collar has no inward prism", ErrUnsupported)
	}
	for i, face := range faces {
		if i%2 == 0 && ctx.Err() != nil {
			return ctx.Err()
		}
		normal := proofarith.Xcross(proofarith.Xsub(face[1], face[0]),
			proofarith.Xsub(face[2], face[0]))
		if proofarith.XdotSign(normal, normal) == 0 {
			return fmt.Errorf("%w: retained middle collar has a flat side", ErrUnsupported)
		}
		box, ok := loftCylinderChamferExactTriBox(face)
		if !ok {
			return fmt.Errorf("%w: retained middle collar has an unbounded box", ErrUnsupported)
		}
		floatFace := [3]r3.Vec{face[0].Vec(), face[1].Vec(), face[2].Vec()}
		plane := loftmesh.NewLoftExactPlane(face[0], normal)
		candidates, err := oldGrid.candidates(box)
		if err != nil {
			return err
		}
		for _, j := range candidates {
			if !meshbool.BoxesOverlap(box, oldBoxes[j]) {
				continue
			}
			signsA := loftmesh.TrianglePlaneSigns(old.Planes[j], face)
			signsB := loftmesh.TrianglePlaneSigns(plane, old.Xtris[j])
			contact, err := meshbool.TriTriClassifyCore(floatFace, old.Corners[j],
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
					ErrUnsupported, j)
			}
			if contact.Kind == meshbool.ContactSegment &&
				(proofarith.OrientSignExact(base[0], base[1], base[2], contact.P1) != 0 ||
					!meshbool.PointOnTri(contact.P1, base, baseNormal)) {
				return fmt.Errorf("%w: retained middle collar crosses the old wall at facet %d",
					ErrUnsupported, j)
			}
		}
	}
	witness := meshbool.XCentroid(q[0], q[1], q[2])
	inside, on, err := meshbool.MeshParityPreparedContext(ctx, witness, oldParity, oldIndices)
	if err != nil || !inside || on {
		return fmt.Errorf("%w: retained middle collar ends outside the old solid", ErrUnsupported)
	}
	return nil
}
