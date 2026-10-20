package contactfaceted

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/contactbox"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/measurement"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

func CertifyUnionLowerSupport(ctx context.Context, result, a, b *topo.Body) error {
	pp, ok := result.Payload.(facetedrecord.Payload)
	if !ok || pp.MeshBound <= 0 || len(pp.ExactSourceVerts) != 0 {
		return nil
	}
	for candidate, operands := range [][2]*topo.Body{{a, b}, {b, a}} {
		base, upper := operands[0], operands[1]
		box, exact := contactbox.SourceBoxAtPose(base, r3.Identity())
		if !exact {
			continue
		}
		bounds := upper.Bounds
		if bounds.Bound.Kind() != units.Length ||
			!measurement.FiniteValues(bounds.Min.Z, bounds.Bound.Base()) || bounds.Bound.Base() < 0 ||
			proofarith.DyCmp(proofarith.DySubScalar(proofarith.MustDyOf(bounds.Min.Z),
				proofarith.MustDyOf(bounds.Bound.Base())), box.Lo[2]) <= 0 {
			continue
		}
		faces := topobuild.Faces(base)
		for i, face := range faces {
			if face != box.Faces[2][0] {
				continue
			}
			sourceGroup := i + candidate*len(topobuild.Faces(a))
			pp.LowerSupport = &facetproof.LowerSupport{Plane: box.Lo[2], SourceGroup: sourceGroup,
				FootLo: [2]proofarith.Dyadic{box.Lo[0], box.Lo[1]},
				FootHi: [2]proofarith.Dyadic{box.Hi[0], box.Hi[1]}}
			result.Payload = pp
			_, proven, err := AxisSupportAtPose(ctx, result, r3.Identity())
			if err != nil {
				return err
			}
			if proven {
				return nil
			}
			pp.LowerSupport = nil
			result.Payload = pp
			break
		}
	}
	return nil
}

// BoundedExtent encloses the true boundary by the held vertex extrema
// and the payload's two-sided boundary displacement. It does not identify a
// support face, so callers may use it only for strict separation.
type BoundedExtent struct {
	Box   contactbox.SourceBox
	Bound proofarith.Dyadic
}

func BoundedExtentAtPose(ctx context.Context, b *topo.Body,
	pose r3.Transform) (BoundedExtent, bool, error) {
	if b == nil || !b.Solid || b.Kind != topo.BodySolid || !pairbox.SignedAxisTransform(pose) {
		return BoundedExtent{}, false, nil
	}
	pp, ok := b.Payload.(facetedrecord.Payload)
	if !ok || len(pp.Verts) == 0 || !measurement.FiniteValues(pp.MeshBound, pp.VolSymDiff) ||
		pp.MeshBound < 0 || pp.VolSymDiff < 0 {
		return BoundedExtent{}, false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	var proof BoundedExtent
	proof.Bound = proofarith.MustDyOf(pp.MeshBound)
	for i, v := range pp.Verts {
		if err := budget.Step(); err != nil {
			return BoundedExtent{}, false, err
		}
		if !proofbound.FiniteVec(v) {
			return BoundedExtent{}, false, nil
		}
		placed := proofarith.DvTransform(pose, proofarith.DyVec(v))
		for axis := range 3 {
			if i == 0 || proofarith.DyCmp(placed[axis], proof.Box.Lo[axis]) < 0 {
				proof.Box.Lo[axis] = placed[axis]
			}
			if i == 0 || proofarith.DyCmp(placed[axis], proof.Box.Hi[axis]) > 0 {
				proof.Box.Hi[axis] = placed[axis]
			}
		}
	}
	return proof, true, budget.Err()
}

func InsideFloor(extent BoundedExtent, floor contactbox.SourceBox) bool {
	for axis := range 2 {
		if proofarith.DyCmp(proofarith.DySubScalar(extent.Box.Lo[axis], extent.Bound), floor.Lo[axis]) <= 0 ||
			proofarith.DyCmp(proofarith.DyAdd(extent.Box.Hi[axis], extent.Bound), floor.Hi[axis]) >= 0 {
			return false
		}
	}
	return true
}

func FloorGap(extent BoundedExtent,
	floor contactbox.SourceBox) (measurement.Measurement, bool) {
	held := proofarith.DySubScalar(extent.Box.Lo[2], floor.Hi[2])
	if proofarith.DyCmp(held, extent.Bound) <= 0 {
		return measurement.Measurement{}, false
	}
	reading, ok := pairbox.SignedMeasurement(held)
	if !ok {
		return measurement.Measurement{}, false
	}
	boundaryBound, _ := extent.Bound.Float64()
	bound := proofbound.AbsSumUpper(reading.Bound.Base(), boundaryBound)
	if !measurement.FiniteValues(bound) || reading.Value.Base()-bound <= 0 {
		return measurement.Measurement{}, false
	}
	reading.Bound = units.Millimeters(bound)
	reading.Exactness = measurement.ExactnessOf(bound)
	return reading, true
}

// AxisSupport is one complete rectangular extremal face of a faceted
// solid whose true support geometry is exact. The coordinates and footprint
// are exact dyadics; face belongs to the caller's body.
type AxisSupport struct {
	Face             *topo.Face
	Axis, Side       int
	Plane            proofarith.Dyadic
	FootLo, FootHi   [2]proofarith.Dyadic
	OuterLo, OuterHi [3]proofarith.Dyadic
	Corners          [4]proofarith.DyV3
	Normal           r3.Vec
}

// AxisSupportAtPose proves that every point of b lies on the material
// side of one axis plane and that its complete contact set on that plane is
// one rectangular, outward-facing source Face. A zero-bound Boolean is read
// directly. A translation-only placement may instead read its saved exact
// source mesh, after checking that the rebuilt mesh stays within its bound.
// A positive-bound Union can also use its exact source-box lower face when
// its other operand is certified strictly above that face.
func AxisSupportAtPose(ctx context.Context, b *topo.Body, pose r3.Transform) (AxisSupport, bool, error) {
	const (
		axis = 2
		side = 0
	)
	if b == nil ||
		!b.Solid || b.Kind != topo.BodySolid || !pairbox.SignedAxisTransform(pose) {
		return AxisSupport{}, false, nil
	}
	pp, ok := b.Payload.(facetedrecord.Payload)
	if !ok {
		return AxisSupport{}, false, nil
	}
	faces := topobuild.Faces(b)
	valid := make([]bool, len(faces))
	for i, face := range faces {
		valid[i] = face.HeldPlanar && (face.Surface.Kind() == surfacegeom.KindFaceted || face.RetainedTag)
	}
	result, proven, err := facetproof.ProveAxisSupport(ctx, facetproof.AxisSupportInput{
		Verts: pp.Verts, ExactSourceVerts: pp.ExactSourceVerts,
		Tris: pp.Tris, ExactSourceTris: pp.ExactSourceTris,
		FaceOf: pp.FaceOf, SourceGroup: pp.Src, FaceValid: valid,
		GroupCount: len(pp.Groups), MeshBound: pp.MeshBound,
		VolSymDiff: pp.VolSymDiff, Xform: pp.Xform, Pose: pose,
		Lower: pp.LowerSupport, Axis: axis, Side: side,
	})
	if err != nil || !proven {
		return AxisSupport{}, false, err
	}
	return AxisSupport{
		Face: faces[result.FaceIndex], Axis: result.Axis, Side: result.Side,
		Plane: result.Plane, FootLo: result.FootLo, FootHi: result.FootHi,
		OuterLo: result.OuterLo, OuterHi: result.OuterHi,
		Corners: result.Corners, Normal: result.Normal,
	}, true, nil
}

// ClassifyFloorBox admits a faceted body only when its complete lower
// support patch lies strictly inside a source-box floor's upper face. The
// certified support plane separates the whole occupied sets at and above it.
func ClassifyFloorBox(ctx context.Context, report *reportvocab.ContactReport, faceted *topo.Body,
	pose r3.Transform, floor contactbox.SourceBox, facetedIsA bool) error {
	report.Reason = reportvocab.ContactPayloadUnsupported
	support, ok, err := AxisSupportAtPose(ctx, faceted, pose)
	if err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	if !ok {
		extent, bounded, err := BoundedExtentAtPose(ctx, faceted, pose)
		if err != nil {
			return err
		}
		if bounded && extent.Bound.Sign() > 0 && InsideFloor(extent, floor) {
			if gap, measured := FloorGap(extent, floor); measured {
				report.Relation, report.Gap, report.Reason = reportvocab.ContactSeparated, &gap, reportvocab.ContactNoReason
			}
		}
		return nil
	}
	for j := range 2 {
		if proofarith.DyCmp(floor.Lo[j], support.FootLo[j]) >= 0 ||
			proofarith.DyCmp(support.FootHi[j], floor.Hi[j]) >= 0 {
			return nil
		}
	}
	gap := proofarith.DySubScalar(support.Plane, floor.Hi[2])
	if gap.Sign() < 0 {
		return nil
	}
	if gap.Sign() > 0 {
		var gaps [3]proofarith.Dyadic
		gaps[2] = gap
		m, measured := pairbox.GapMeasurement(gaps)
		if !measured {
			report.Reason = reportvocab.ContactNoGapProof
			return nil
		}
		report.Relation, report.Gap, report.Reason = reportvocab.ContactSeparated, &m, reportvocab.ContactNoReason
		return nil
	}
	report.Relation = reportvocab.ContactTouching
	exactZero := measurement.Measurement{Value: units.Millimeters(0), Exactness: measurement.Exact, Bound: units.Millimeters(0)}
	report.Gap = &exactZero
	patch := contactbox.SourceBox{}
	patch.Lo[2], patch.Hi[2] = support.Plane, support.OuterHi[2]
	patch.Faces[2][0] = support.Face
	for j := range 2 {
		patch.Lo[j], patch.Hi[j] = support.FootLo[j], support.FootHi[j]
	}
	if facetedIsA {
		contactbox.PublishSourceBoxPatch(report, patch, floor, 2, -1, proofarith.DyZero())
	} else {
		contactbox.PublishSourceBoxPatch(report, floor, patch, 2, 1, proofarith.DyZero())
	}
	if report.Manifold != nil {
		report.Reason = reportvocab.ContactNoReason
	}
	return nil
}
