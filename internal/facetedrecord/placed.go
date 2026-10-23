package facetedrecord

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfaceplace"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// Placed re-evaluates the held mesh under the composed motion: the vertices
// move through the delta motion (float rounding is folded into the proven
// bounds — the geometry is never silently trusted), the moved mesh is kept
// embedded (facetproof.PlaceMesh), a reflection flips the windings, and the
// topology and measurements rebuild from the moved mesh.
func (fp Payload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	return fp.PlacedWith(ctx, env, composed, nil)
}

// PlacedWith is Placed for a copy that denotes the exact rotation exact, or
// the float placement alone when exact is nil. With a record the result
// denotes Q*S + t_Q, where S is fp's true solid: the held mesh is placed by
// the float delta as ever, charged for its distance to Q, and every record
// that describes the float placement rather than Q is dropped (every group
// surface tag and denoted patch, PointSection, PointPlane, PointSimilarity,
// RevolveLoftSector and LoftMirrorCopy). A dropped record is never rebuilt
// from a later float placement.
func (fp Payload) PlacedWith(ctx context.Context, env topo.Env, composed r3.Transform,
	exact *exactrotation.Rotation) (*topo.Body, error) {
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	delta, err := PlacementDelta(fp.Xform, composed)
	if err != nil {
		return nil, err
	}
	next := fp
	next.SmoothRectSection = nil
	next.SmoothHomotheticSection = nil
	next.AxialSectionJoin = false
	next.PrismPatternJoin = nil
	next.Xform = composed
	next.LowerSupport = nil
	next.PointCone = nil
	next.LoftFacesCap = nil
	next.LoftFacesToe = nil
	next.LoftFacesTrim = nil
	next.CopyLineage = nil
	next.SpiralTools = nil
	// A rigid copy keeps the immutable Split source for certified face
	// centroid ranking, but cannot become a new structural Split source.
	next.PointPlaneSourceEligible = false
	next.RevolveLoftJoin = nil
	next.ObliqueBoss = nil
	next.PrismUnionFillet = nil
	next.CrossCylinderBore = nil
	next.LoftCylinderJoin = nil
	next.LoftCylinderBore = nil
	next.LoftMirrorJoin = nil
	next.LoftMirrorCopy = nil
	next.LoftMirrorCylinderJoin = nil
	next.LoftMirrorCylinderBore = nil
	next.RevolveLoftSector = nil
	// The true solid follows each exact reading of delta, even when the
	// accumulated float placement later returns to identity.
	if delta != r3.Identity() {
		next.PointSection = nil
	}
	if fp.LoftMirrorJoin != nil && fp.Xform == r3.Identity() {
		next.LoftMirrorCopy = &LoftMirrorPlacementRecord{Source: fp.LoftMirrorJoin, Motion: composed}
	} else if fp.LoftMirrorCopy != nil {
		next.LoftMirrorCopy = &LoftMirrorPlacementRecord{Source: fp.LoftMirrorCopy.Source, Motion: composed}
	}
	if fp.PointCone != nil && fp.Xform == r3.Identity() {
		next.RevolveLoftSector = &RevolveLoftSectorPlacement{Source: fp.PointCone, Motion: composed}
	}
	if exact != nil {
		next.LoftMirrorCopy = nil
		next.RevolveLoftSector = nil
		next.PointSection = nil
		next.PointPlane = nil
		next.PointSimilarity = nil
		next.PointPlaneCapDisplacement = 0
		next.ExactPlaced = true
	}
	next.Groups = append([]Group(nil), fp.Groups...)
	if fp.LoftMirrorJoin != nil && fp.MirrorCapGroups == nil {
		return nil, fmt.Errorf("%w: a loft mirror join lost its outer-cap groups", decaderr.ErrUnsupported)
	}
	if fp.MirrorCapGroups != nil {
		for cap, group := range fp.MirrorCapGroups {
			if group < 0 || group >= len(next.Groups) {
				return nil, fmt.Errorf("%w: placed mirror join lost an outer cap", decaderr.ErrUnsupported)
			}
			role := provenance.RoleCapStart
			if cap == 1 {
				role = provenance.RoleCapEnd
			}
			origins := append([]provenance.Ref(nil), next.Groups[group].Origins...)
			next.Groups[group].Origins = append(origins, provenance.Ref{Producer: env.Ref, Role: role})
		}
	}
	reach := fp.MeshBound
	for _, beta := range fp.VertexBound {
		reach = max(reach, beta)
	}
	reach = proofbound.AbsSumUpper(reach, fp.MeshBound)
	for i := range next.Groups {
		if exact != nil {
			if err := ExactGroupPlaced(&next.Groups[i], exact); err != nil {
				return nil, err
			}
			continue
		}
		if next.Groups[i].HasCentroid {
			reading, err := PlacedCapCentroid(next.Groups[i].Centroid, delta)
			if err != nil {
				return nil, err
			}
			next.Groups[i].Centroid = reading
		}
		// A later structural trim requires the unplaced source. Its retained
		// public surface identity still moves with this faceted restatement.
		// The moved tag is the float image of the source tag, a bounded
		// stand-in. NormalAt proves a denoted group against the denoted
		// patch's exact image and never reads NormalBound. A retained group adds
		// the departure of PlacedTagDeparture to its own, or the largest distance
		// of two unit normals where that states none. Any other group is charged
		// that distance.
		if tag := next.Groups[i].Surface; tag != nil && delta != r3.Identity() {
			switch tag.(type) {
			case surfacegeom.Plane, surfacegeom.Cylinder, surfacegeom.Cone, surfacegeom.Sphere, surfacegeom.Torus:
				moved, err := surfaceplace.TransformSurface(tag, delta)
				if err != nil {
					return nil, err
				}
				next.Groups[i].Surface = moved
				next.Groups[i].NormalBound = 2
				if next.Groups[i].Retained {
					add, stated := PlacedTagDeparture(tag, moved, delta, fp.Verts, reach)
					if stated {
						next.Groups[i].NormalBound = min(2, proofbound.AbsSumUpper(fp.Groups[i].NormalBound, add))
					}
				}
				next.Groups[i].CapRecord = next.Groups[i].CapRecord.Under(delta)
				if next.Groups[i].CapRecord == nil {
					next.Groups[i].AxialDelta, next.Groups[i].HasAxialDelta = 0, false
				}
				// Only a Plane normal is U x V, which a reflection negates.
				if _, planar := tag.(surfacegeom.Plane); planar && delta.IsReflection() {
					next.Groups[i].Reversed = !next.Groups[i].Reversed
				}
			}
		}
		if next.Groups[i].Denoted != nil {
			denoted := next.Groups[i].Denoted.Transformed(delta)
			next.Groups[i].Denoted = &denoted
		}
	}
	if exact != nil && !exact.IsIdentity() {
		next.ExactSourceVerts, next.ExactSourceTris = nil, nil
	} else if !facetproof.TranslationOnly(delta) || !facetproof.TranslationOnly(composed) {
		next.ExactSourceVerts, next.ExactSourceTris = nil, nil
	} else if len(fp.ExactSourceVerts) == 0 && fp.MeshBound == 0 && fp.VolSymDiff == 0 &&
		fp.Xform == r3.Identity() {
		next.ExactSourceVerts = append([]r3.Vec(nil), fp.Verts...)
		next.ExactSourceTris = append([][3]int(nil), fp.Tris...)
	}
	var charge massmoment.MapCharge
	var placed facetproof.PlacedMesh
	if exact != nil {
		linearGap, translationGap, ok := exact.Gap(delta)
		if !ok {
			return nil, fmt.Errorf("%w: the exact rotation has no finite gap to the float placement", decaderr.ErrUnsupported)
		}
		placed, err = facetproof.PlaceMeshDenoting(ctx, budget, fp.Verts, fp.Tris, fp.VertexBound,
			fp.VolSymDiff, fp.AreaSlack, delta, linearGap, translationGap)
	} else {
		var linear [3][3]*big.Rat
		linear, err = massmoment.PlacementRotation(delta)
		if err != nil {
			return nil, err
		}
		charge, err = massmoment.MapChargeOf(linear)
		if err != nil {
			return nil, err
		}
		placed, err = facetproof.PlaceMesh(ctx, budget, fp.Verts, fp.Tris, fp.VertexBound,
			fp.VolSymDiff, fp.AreaSlack, charge.Stretch, charge.Volume, delta)
	}
	if err != nil {
		return nil, err
	}
	next.Verts, next.Tris = placed.Verts, placed.Tris
	next.VertexBound, next.MeshBound, next.VolSymDiff = placed.VertexBound, placed.MeshBound, placed.VolSymDiff
	next.AreaSlack = placed.AreaSlack
	next.HeldFloor = placedHeldFloor(fp, placed.VertexBound)
	next.DPair = SourceDiameterUpper(next.Verts, next.MeshBound)
	if proofbound.IsNonFinite(next.DPair) {
		return nil, fmt.Errorf("%w: the placed pair diameter has no finite proof", decaderr.ErrUnsupported)
	}
	if fp.PointPlane != nil && exact == nil {
		// PlaceMesh's MeshBound includes the held-coordinate rounding and any
		// embedding correction, but does not stretch the previous source gap.
		// Add both terms; the possible double charge is conservative.
		next.PointPlaneCapDisplacement = proofbound.AbsSumUpper(
			proofbound.ProductUpper(fp.PointPlaneCapDisplacement,
				proofbound.AbsSumUpper(1, charge.Stretch)), placed.MeshBound)
	}
	if fp.PointPlane != nil && exact == nil {
		record := PointPlaneSimilarityRecord{Slice: fp.PointPlane}
		if fp.PointSimilarity != nil {
			record.Steps = append(record.Steps, fp.PointSimilarity.Steps...)
		}
		record.Steps = append(record.Steps, PointPlaneSimilarityStep{Motion: delta,
			MeshBound: next.MeshBound, CapDisplacement: next.PointPlaneCapDisplacement})
		next.PointSimilarity = &record
	}
	return Build(ctx, env, next)
}

// placedHeldFloor carries fp's held floor through a rigid placement: every
// vertex bound grows by the placement's charge, so the floor grows by the
// largest growth. The growth is read as a float difference and widened by a
// relative 1e-9, far past its own rounding.
func placedHeldFloor(fp Payload, placed []float64) float64 {
	if fp.HeldFloor <= 0 || len(placed) != len(fp.VertexBound) {
		return 0
	}
	growth := 0.0
	for i, bound := range placed {
		growth = max(growth, bound-fp.VertexBound[i])
	}
	growth = proofbound.ProductUpper(growth, 1+1e-9)
	return proofbound.AbsSumUpper(fp.HeldFloor, growth)
}
