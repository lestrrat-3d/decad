package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

func certifyFacetedUnionLowerSupport(ctx context.Context, result, a, b *Body) error {
	pp, ok := result.payload.(facetedPayload)
	if !ok || pp.meshBound <= 0 || len(pp.exactSourceVerts) != 0 {
		return nil
	}
	for candidate, operands := range [][2]*Body{{a, b}, {b, a}} {
		base, upper := operands[0], operands[1]
		box, exact := sourceBoxAtPose(base, r3.Identity())
		if !exact {
			continue
		}
		bounds, err := upper.Bounds()
		if err != nil || bounds.Bound.Kind() != units.Length ||
			!finiteMeasurementValues(bounds.Min.Z, bounds.Bound.Base()) || bounds.Bound.Base() < 0 ||
			proofarith.DyCmp(proofarith.DySubScalar(proofarith.MustDyOf(bounds.Min.Z),
				proofarith.MustDyOf(bounds.Bound.Base())), box.lo[2]) <= 0 {
			continue
		}
		faces := base.Faces()
		for i, face := range faces {
			if face != box.faces[2][0] {
				continue
			}
			sourceGroup := i + candidate*len(a.Faces())
			pp.lowerSupport = &facetproof.LowerSupport{Plane: box.lo[2], SourceGroup: sourceGroup,
				FootLo: [2]proofarith.Dyadic{box.lo[0], box.lo[1]},
				FootHi: [2]proofarith.Dyadic{box.hi[0], box.hi[1]}}
			result.payload = pp
			_, proven, err := sourceFacetedAxisSupport(ctx, result, r3.Identity(), 2, 0)
			if err != nil {
				return err
			}
			if proven {
				return nil
			}
			pp.lowerSupport = nil
			result.payload = pp
			break
		}
	}
	return nil
}

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
	budget := proofbound.NewWorkBudget(ctx)
	var proof boundedFacetedExtent
	proof.bound = proofarith.MustDyOf(pp.meshBound)
	for i, v := range pp.verts {
		if err := budget.Step(); err != nil {
			return boundedFacetedExtent{}, false, err
		}
		if !proofbound.FiniteVec(v) {
			return boundedFacetedExtent{}, false, nil
		}
		placed := proofarith.DvTransform(pose, proofarith.DyVec(v))
		for axis := range 3 {
			if i == 0 || proofarith.DyCmp(placed[axis], proof.box.lo[axis]) < 0 {
				proof.box.lo[axis] = placed[axis]
			}
			if i == 0 || proofarith.DyCmp(placed[axis], proof.box.hi[axis]) > 0 {
				proof.box.hi[axis] = placed[axis]
			}
		}
	}
	return proof, true, budget.Err()
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
	bound := proofbound.AbsSumUpper(reading.Bound.Base(), boundaryBound)
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
// A positive-bound Union can also use its exact source-box lower face when
// its other operand is certified strictly above that face.
func sourceFacetedAxisSupport(ctx context.Context, b *Body, pose r3.Transform,
	axis, side int) (facetedAxisSupport, bool, error) {
	if b == nil || axis < 0 || axis > 2 || (side != 0 && side != 1) ||
		!b.solid || b.kind != BodySolid || !signedAxisTransform(pose) {
		return facetedAxisSupport{}, false, nil
	}
	pp, ok := b.payload.(facetedPayload)
	if !ok {
		return facetedAxisSupport{}, false, nil
	}
	faces := b.Faces()
	valid := make([]bool, len(faces))
	for i, face := range faces {
		valid[i] = face.heldPlanar && face.surface.Kind() == KindFaceted
	}
	result, proven, err := facetproof.ProveAxisSupport(ctx, facetproof.AxisSupportInput{
		Verts: pp.verts, ExactSourceVerts: pp.exactSourceVerts,
		Tris: pp.tris, ExactSourceTris: pp.exactSourceTris,
		FaceOf: pp.faceOf, SourceGroup: pp.src, FaceValid: valid,
		GroupCount: len(pp.groups), MeshBound: pp.meshBound,
		VolSymDiff: pp.volSymDiff, Xform: pp.xform, Pose: pose,
		Lower: pp.lowerSupport, Axis: axis, Side: side,
	})
	if err != nil || !proven {
		return facetedAxisSupport{}, false, err
	}
	return facetedAxisSupport{
		face: faces[result.FaceIndex], axis: result.Axis, side: result.Side,
		plane: result.Plane, footLo: result.FootLo, footHi: result.FootHi,
		outerLo: result.OuterLo, outerHi: result.OuterHi,
		corners: result.Corners, normal: result.Normal,
	}, true, nil
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
