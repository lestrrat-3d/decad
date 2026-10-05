package decad

import (
	"context"

	"github.com/lestrrat-3d/r3"
)

// facetedAxisSupport is one complete rectangular extremal face of an exact
// faceted solid. The coordinates and footprint are exact dyadics; face is the
// original body's Face, not a face of a transient placed body.
type facetedAxisSupport struct {
	face             *Face
	axis, side       int
	plane            dyadic
	footLo, footHi   [2]dyadic
	outerLo, outerHi [3]dyadic
	corners          [4]dyV3
	normal           r3.Vec
}

// sourceFacetedAxisSupport proves that every point of b lies on the material
// side of one axis plane and that its complete contact set on that plane is
// one rectangular, outward-facing source Face. It accepts only a faceted
// payload whose held boundary and occupied volume equal the denoted solid.
// A zero displacement without zero occupied-volume difference is insufficient.
// The boolean evaluator has already audited closure, embedding, and material
// orientation; this reader checks the exact held triangles and their Face map.
func sourceFacetedAxisSupport(ctx context.Context, b *Body, pose r3.Transform,
	axis, side int) (facetedAxisSupport, bool, error) {
	if b == nil || axis < 0 || axis > 2 || (side != 0 && side != 1) ||
		!b.solid || b.kind != BodySolid || !signedAxisTransform(pose) {
		return facetedAxisSupport{}, false, nil
	}
	pp, ok := b.payload.(facetedPayload)
	if !ok || pp.meshBound != 0 || pp.volSymDiff != 0 ||
		len(pp.verts) == 0 || len(pp.tris) == 0 || len(pp.faceOf) != len(pp.tris) {
		return facetedAxisSupport{}, false, nil
	}
	budget := newWorkBudget(ctx)
	if err := budget.err(); err != nil {
		return facetedAxisSupport{}, false, err
	}
	placed := make([]dyV3, len(pp.verts))
	var proof facetedAxisSupport
	proof.axis, proof.side = axis, side
	for i, v := range pp.verts {
		if err := budget.step(); err != nil {
			return facetedAxisSupport{}, false, err
		}
		if !finiteVec(v) {
			return facetedAxisSupport{}, false, nil
		}
		placed[i] = exactContactTransform(pose, dyVec(v))
		for j := range 3 {
			if i == 0 || dyCmp(placed[i][j], proof.outerLo[j]) < 0 {
				proof.outerLo[j] = placed[i][j]
			}
			if i == 0 || dyCmp(placed[i][j], proof.outerHi[j]) > 0 {
				proof.outerHi[j] = placed[i][j]
			}
		}
	}
	proof.plane = proof.outerLo[axis]
	sign := -1
	if side == 1 {
		proof.plane, sign = proof.outerHi[axis], 1
	}
	windingSign := sign
	if pose.IsReflection() {
		windingSign = -windingSign
	}
	var projected [2]int
	for j, n := 0, 0; j < 3; j++ {
		if j != axis {
			projected[n] = j
			n++
		}
	}
	faces := b.Faces()
	covered := make([]bool, len(placed))
	var area2 dyadic
	first := true
	for i, tri := range pp.tris {
		if err := budget.step(); err != nil {
			return facetedAxisSupport{}, false, err
		}
		for _, vertex := range tri {
			if vertex < 0 || vertex >= len(placed) {
				return facetedAxisSupport{}, false, nil
			}
		}
		faceIndex := pp.faceOf[i]
		if faceIndex < 0 || faceIndex >= len(faces) {
			return facetedAxisSupport{}, false, nil
		}
		if dyCmp(placed[tri[0]][axis], proof.plane) != 0 ||
			dyCmp(placed[tri[1]][axis], proof.plane) != 0 ||
			dyCmp(placed[tri[2]][axis], proof.plane) != 0 {
			continue
		}
		face := faces[faceIndex]
		if !face.heldPlanar || face.surface.Kind() != KindFaceted ||
			(!first && proof.face != face) {
			return facetedAxisSupport{}, false, nil
		}
		proof.face = face
		cross := dvCross(dvSub(placed[tri[1]], placed[tri[0]]),
			dvSub(placed[tri[2]], placed[tri[0]]))
		if cross[axis].sign() != windingSign ||
			!cross[projected[0]].isZero() || !cross[projected[1]].isZero() {
			return facetedAxisSupport{}, false, nil
		}
		if windingSign < 0 {
			area2 = dySubScalar(area2, cross[axis])
		} else {
			area2 = dyAdd(area2, cross[axis])
		}
		for _, vertex := range tri {
			covered[vertex] = true
			for j, coord := range projected {
				if first || dyCmp(placed[vertex][coord], proof.footLo[j]) < 0 {
					proof.footLo[j] = placed[vertex][coord]
				}
				if first || dyCmp(placed[vertex][coord], proof.footHi[j]) > 0 {
					proof.footHi[j] = placed[vertex][coord]
				}
			}
			first = false
		}
	}
	if proof.face == nil || dyCmp(proof.footLo[0], proof.footHi[0]) >= 0 ||
		dyCmp(proof.footLo[1], proof.footHi[1]) >= 0 {
		return facetedAxisSupport{}, false, nil
	}
	// The source Face must name this support patch in full, rather than also
	// naming a different-level facet that the solver would falsely include.
	for i, tri := range pp.tris {
		if err := budget.step(); err != nil {
			return facetedAxisSupport{}, false, err
		}
		if faces[pp.faceOf[i]] != proof.face {
			continue
		}
		for _, vertex := range tri {
			if dyCmp(placed[vertex][axis], proof.plane) != 0 {
				return facetedAxisSupport{}, false, nil
			}
		}
	}
	for i := range placed {
		if err := budget.step(); err != nil {
			return facetedAxisSupport{}, false, err
		}
		if dyCmp(placed[i][axis], proof.plane) == 0 && !covered[i] {
			return facetedAxisSupport{}, false, nil
		}
	}
	width := dySubScalar(proof.footHi[0], proof.footLo[0])
	height := dySubScalar(proof.footHi[1], proof.footLo[1])
	if dyCmp(area2, dyMul(mustDyOf(2), dyMul(width, height))) != 0 {
		return facetedAxisSupport{}, false, nil
	}
	for i, corner := range [][2]int{{0, 0}, {1, 0}, {1, 1}, {0, 1}} {
		proof.corners[i][axis] = proof.plane
		for j, coord := range projected {
			proof.corners[i][coord] = proof.footLo[j]
			if corner[j] == 1 {
				proof.corners[i][coord] = proof.footHi[j]
			}
		}
	}
	switch axis {
	case 0:
		proof.normal.X = float64(sign)
	case 1:
		proof.normal.Y = float64(sign)
	case 2:
		proof.normal.Z = float64(sign)
	}
	return proof, true, budget.err()
}
