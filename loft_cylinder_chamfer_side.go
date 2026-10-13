package decad

import (
	"context"
	"fmt"
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
	mesh *joinedMesh, source loftCylinderChamferSource, ideal []proofarith.Xpt,
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
	oldGrid, err := loftmesh.ChamferBoxGridOf(oldBoxes, source.z0, source.z1)
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
	toothParity := make(map[*Body]*meshbool.ParityMesh)
	toothIndices := make(map[*Body][]int)
	toothCircles := make(map[*Body]loftCylinderSourceCircle)
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
			q, shift, err := loftmesh.ChamferMiddleOffset(base, distance, clipBound)
			if err != nil {
				return err
			}
			if !loftmesh.ChamferMiddleBeyondBore(q, source.boreRadius) {
				last = "inward patch reaches the source bore"
				continue
			}
			if !loftmesh.ChamferMiddleBlank(q, source.blankRadius) {
				candidates := faces[origin]
				if len(candidates) == 0 {
					last = "source face has no authenticated tooth"
					continue
				}
				certified := false
				for _, face := range candidates {
					parity, ok := toothParity[face.body]
					if !ok {
						parity = meshbool.NewParityMesh(face.loft.verts, face.loft.tris)
						toothParity[face.body] = parity
						indices := make([]int, len(face.loft.tris))
						for j := range indices {
							indices[j] = j
						}
						toothIndices[face.body] = indices
					}
					if err := loftCylinderChamferMiddleToothSource(ctx, q, face,
						parity, toothIndices[face.body], source.blankRadius, toothCircles); err != nil {
						last = err.Error()
						continue
					}
					certified = true
					break
				}
				if !certified {
					continue
				}
			}
			if err := loftmesh.ChamferMiddlePrism(ctx, len(fp.tris), old, oldBoxes, oldGrid,
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

// A whole inward patch is in the true Loft tooth when it misses every moving
// wall by its matched departure and one held-tooth point is inside. If only
// hidden root walls meet it, use the exact blank disk for that part instead.
func loftCylinderChamferMiddleToothSource(ctx context.Context, q [3]proofarith.Xpt,
	face loftCylinderChamferToothFace, parity *meshbool.ParityMesh, indices []int,
	blankRadius float64, circles map[*Body]loftCylinderSourceCircle) error {
	lp := face.loft
	if lp.walls <= 0 || lp.walls > len(lp.tris) || face.root < 3 ||
		face.scale == nil || face.scale.Sign() <= 0 ||
		2*face.root >= lp.walls || face.bound < 0 || proofbound.IsNonFinite(face.bound) {
		return fmt.Errorf("%w: inward patch lacks a bounded Loft wall", ErrUnsupported)
	}
	qBox, boxed := loftmesh.ChamferExactTriBox(q)
	fullClear := true
	for j := range lp.walls {
		if j%64 == 0 && ctx.Err() != nil {
			return ctx.Err()
		}
		tri := lp.tris[j]
		if boxed && loftmesh.ChamferBoxGapAbove(qBox, meshbool.TriBox(lp.verts, tri), face.bound) {
			continue
		}
		other := [3]proofarith.Xpt{
			proofarith.XptOf(lp.verts[tri[0]]),
			proofarith.XptOf(lp.verts[tri[1]]),
			proofarith.XptOf(lp.verts[tri[2]]),
		}
		if loftmesh.ChamferTriangleGapSplit(q, other, face.bound, 2) ||
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
	circle, ok := circles[face.body]
	if !ok {
		var err error
		circle, err = loftCylinderSourceCircleOf(lp, blankRadius)
		if err != nil {
			return err
		}
		circles[face.body] = circle
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
		!loftmesh.ChamferMiddleHeldRootInside(lp.verts, lp.tris, lp.walls, face.root, mappedLo, mappedHi, blankRadius) {
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
