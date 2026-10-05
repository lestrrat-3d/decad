package decad

import (
	"context"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// boundedFacetedExtent encloses the true boundary by the held vertex extrema
// and the payload's two-sided boundary displacement. It does not identify a
// support face, so callers may use it only for strict separation.
type boundedFacetedExtent struct {
	box   sourceBoxContactProof
	bound proofarith.Dyadic
}

func sourceBoundedFacetedExtent(ctx context.Context, b *Body,
	pose r3.Transform) (boundedFacetedExtent, bool, error) {
	if b == nil || !b.solid || b.kind != BodySolid || !signedAxisTransform(pose) {
		return boundedFacetedExtent{}, false, nil
	}
	pp, ok := b.payload.(facetedPayload)
	if !ok || len(pp.verts) == 0 || !finiteMeasurementValues(pp.meshBound, pp.volSymDiff) ||
		pp.meshBound < 0 || pp.volSymDiff < 0 {
		return boundedFacetedExtent{}, false, nil
	}
	budget := newWorkBudget(ctx)
	var proof boundedFacetedExtent
	proof.bound = proofarith.MustDyOf(pp.meshBound)
	for i, v := range pp.verts {
		if err := budget.step(); err != nil {
			return boundedFacetedExtent{}, false, err
		}
		if !finiteVec(v) {
			return boundedFacetedExtent{}, false, nil
		}
		placed := exactContactTransform(pose, proofarith.DyVec(v))
		for axis := range 3 {
			if i == 0 || proofarith.DyCmp(placed[axis], proof.box.lo[axis]) < 0 {
				proof.box.lo[axis] = placed[axis]
			}
			if i == 0 || proofarith.DyCmp(placed[axis], proof.box.hi[axis]) > 0 {
				proof.box.hi[axis] = placed[axis]
			}
		}
	}
	return proof, true, budget.err()
}

func boundedFacetedInsideFloor(extent boundedFacetedExtent, floor sourceBoxContactProof) bool {
	for axis := range 2 {
		if proofarith.DyCmp(proofarith.DySubScalar(extent.box.lo[axis], extent.bound), floor.lo[axis]) <= 0 ||
			proofarith.DyCmp(proofarith.DyAdd(extent.box.hi[axis], extent.bound), floor.hi[axis]) >= 0 {
			return false
		}
	}
	return true
}

func boundedFacetedFloorGap(extent boundedFacetedExtent,
	floor sourceBoxContactProof) (Measurement, bool) {
	held := proofarith.DySubScalar(extent.box.lo[2], floor.hi[2])
	if proofarith.DyCmp(held, extent.bound) <= 0 {
		return Measurement{}, false
	}
	reading, ok := sourceBoxSignedReading(held)
	if !ok {
		return Measurement{}, false
	}
	boundaryBound, _ := extent.bound.Float64()
	bound := absSumUpper(reading.Bound.Base(), boundaryBound)
	if !finiteMeasurementValues(bound) || reading.Value.Base()-bound <= 0 {
		return Measurement{}, false
	}
	reading.Bound = units.Millimeters(bound)
	reading.Exactness = exactnessOf(bound)
	return reading, true
}

// facetedAxisSupport is one complete rectangular extremal face of a faceted
// solid whose true support geometry is exact. The coordinates and footprint
// are exact dyadics; face belongs to the caller's body.
type facetedAxisSupport struct {
	face             *Face
	axis, side       int
	plane            proofarith.Dyadic
	footLo, footHi   [2]proofarith.Dyadic
	outerLo, outerHi [3]proofarith.Dyadic
	corners          [4]proofarith.DyV3
	normal           r3.Vec
}

// sourceFacetedAxisSupport proves that every point of b lies on the material
// side of one axis plane and that its complete contact set on that plane is
// one rectangular, outward-facing source Face. A zero-bound Boolean is read
// directly. A translation-only placement may instead read its saved exact
// source mesh, after checking that the rebuilt mesh stays within its bound.
// Other positive-bound meshes cannot identify a true support plane.
func sourceFacetedAxisSupport(ctx context.Context, b *Body, pose r3.Transform,
	axis, side int) (facetedAxisSupport, bool, error) {
	if b == nil || axis < 0 || axis > 2 || (side != 0 && side != 1) ||
		!b.solid || b.kind != BodySolid || !signedAxisTransform(pose) {
		return facetedAxisSupport{}, false, nil
	}
	pp, ok := b.payload.(facetedPayload)
	if !ok || !finiteMeasurementValues(pp.meshBound, pp.volSymDiff) ||
		pp.meshBound < 0 || pp.volSymDiff < 0 ||
		len(pp.verts) == 0 || len(pp.tris) == 0 || len(pp.faceOf) != len(pp.tris) {
		return facetedAxisSupport{}, false, nil
	}
	sourceVerts := pp.verts
	placedFromSource := false
	if pp.meshBound != 0 || pp.volSymDiff != 0 {
		if !facetedTranslationOnly(pp.xform) ||
			len(pp.exactSourceVerts) != len(pp.verts) ||
			len(pp.exactSourceTris) != len(pp.tris) {
			return facetedAxisSupport{}, false, nil
		}
		for i, tri := range pp.tris {
			if tri != pp.exactSourceTris[i] {
				return facetedAxisSupport{}, false, nil
			}
		}
		sourceVerts, placedFromSource = pp.exactSourceVerts, true
	}
	budget := newWorkBudget(ctx)
	if err := budget.err(); err != nil {
		return facetedAxisSupport{}, false, err
	}
	placed := make([]proofarith.DyV3, len(pp.verts))
	var proof facetedAxisSupport
	proof.axis, proof.side = axis, side
	for i, v := range sourceVerts {
		if err := budget.step(); err != nil {
			return facetedAxisSupport{}, false, err
		}
		if !finiteVec(v) {
			return facetedAxisSupport{}, false, nil
		}
		source := proofarith.DyVec(v)
		if placedFromSource {
			if !finiteVec(pp.verts[i]) {
				return facetedAxisSupport{}, false, nil
			}
			source = exactContactTransform(pp.xform, source)
			difference := proofarith.DvSub(source, proofarith.DyVec(pp.verts[i]))
			bound := proofarith.MustDyOf(pp.meshBound)
			if proofarith.DyCmp(proofarith.DvDot(difference, difference), proofarith.DyMul(bound, bound)) > 0 {
				return facetedAxisSupport{}, false, nil
			}
		}
		placed[i] = exactContactTransform(pose, source)
		for j := range 3 {
			if i == 0 || proofarith.DyCmp(placed[i][j], proof.outerLo[j]) < 0 {
				proof.outerLo[j] = placed[i][j]
			}
			if i == 0 || proofarith.DyCmp(placed[i][j], proof.outerHi[j]) > 0 {
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
	var area2 proofarith.Dyadic
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
		if proofarith.DyCmp(placed[tri[0]][axis], proof.plane) != 0 ||
			proofarith.DyCmp(placed[tri[1]][axis], proof.plane) != 0 ||
			proofarith.DyCmp(placed[tri[2]][axis], proof.plane) != 0 {
			continue
		}
		face := faces[faceIndex]
		if !face.heldPlanar || face.surface.Kind() != KindFaceted ||
			(!first && proof.face != face) {
			return facetedAxisSupport{}, false, nil
		}
		proof.face = face
		cross := proofarith.DvCross(proofarith.DvSub(placed[tri[1]], placed[tri[0]]),
			proofarith.DvSub(placed[tri[2]], placed[tri[0]]))
		if cross[axis].Sign() != windingSign ||
			!cross[projected[0]].IsZero() || !cross[projected[1]].IsZero() {
			return facetedAxisSupport{}, false, nil
		}
		if windingSign < 0 {
			area2 = proofarith.DySubScalar(area2, cross[axis])
		} else {
			area2 = proofarith.DyAdd(area2, cross[axis])
		}
		for _, vertex := range tri {
			covered[vertex] = true
			for j, coord := range projected {
				if first || proofarith.DyCmp(placed[vertex][coord], proof.footLo[j]) < 0 {
					proof.footLo[j] = placed[vertex][coord]
				}
				if first || proofarith.DyCmp(placed[vertex][coord], proof.footHi[j]) > 0 {
					proof.footHi[j] = placed[vertex][coord]
				}
			}
			first = false
		}
	}
	if proof.face == nil || proofarith.DyCmp(proof.footLo[0], proof.footHi[0]) >= 0 ||
		proofarith.DyCmp(proof.footLo[1], proof.footHi[1]) >= 0 {
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
			if proofarith.DyCmp(placed[vertex][axis], proof.plane) != 0 {
				return facetedAxisSupport{}, false, nil
			}
		}
	}
	for i := range placed {
		if err := budget.step(); err != nil {
			return facetedAxisSupport{}, false, err
		}
		if proofarith.DyCmp(placed[i][axis], proof.plane) == 0 && !covered[i] {
			return facetedAxisSupport{}, false, nil
		}
	}
	width := proofarith.DySubScalar(proof.footHi[0], proof.footLo[0])
	height := proofarith.DySubScalar(proof.footHi[1], proof.footLo[1])
	if proofarith.DyCmp(area2, proofarith.DyMul(proofarith.MustDyOf(2), proofarith.DyMul(width, height))) != 0 {
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

// classifyFacetedFloorBox admits a faceted body only when its complete lower
// support patch lies strictly inside a source-box floor's upper face. The
// certified support plane separates the whole occupied sets at and above it.
func classifyFacetedFloorBox(ctx context.Context, report *ContactReport, faceted *Body,
	pose r3.Transform, floor sourceBoxContactProof, facetedIsA bool) error {
	report.Reason = ContactPayloadUnsupported
	support, ok, err := sourceFacetedAxisSupport(ctx, faceted, pose, 2, 0)
	if err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	if !ok {
		extent, bounded, err := sourceBoundedFacetedExtent(ctx, faceted, pose)
		if err != nil {
			return err
		}
		if bounded && extent.bound.Sign() > 0 && boundedFacetedInsideFloor(extent, floor) {
			if gap, measured := boundedFacetedFloorGap(extent, floor); measured {
				report.Relation, report.Gap, report.Reason = ContactSeparated, &gap, ContactNoReason
			}
		}
		return nil
	}
	for j := range 2 {
		if proofarith.DyCmp(floor.lo[j], support.footLo[j]) >= 0 ||
			proofarith.DyCmp(support.footHi[j], floor.hi[j]) >= 0 {
			return nil
		}
	}
	gap := proofarith.DySubScalar(support.plane, floor.hi[2])
	if gap.Sign() < 0 {
		return nil
	}
	if gap.Sign() > 0 {
		var gaps [3]proofarith.Dyadic
		gaps[2] = gap
		m, measured := sourceBoxGap(gaps)
		if !measured {
			report.Reason = ContactNoGapProof
			return nil
		}
		report.Relation, report.Gap, report.Reason = ContactSeparated, &m, ContactNoReason
		return nil
	}
	report.Relation = ContactTouching
	exactZero := Measurement{Value: units.Millimeters(0), Exactness: Exact, Bound: units.Millimeters(0)}
	report.Gap = &exactZero
	patch := sourceBoxContactProof{}
	patch.lo[2], patch.hi[2] = support.plane, support.outerHi[2]
	patch.faces[2][0] = support.face
	for j := range 2 {
		patch.lo[j], patch.hi[j] = support.footLo[j], support.footHi[j]
	}
	if facetedIsA {
		publishSourceBoxPatch(report, patch, floor, 2, -1, proofarith.DyZero())
	} else {
		publishSourceBoxPatch(report, floor, patch, 2, 1, proofarith.DyZero())
	}
	if report.Manifold != nil {
		report.Reason = ContactNoReason
	}
	return nil
}
