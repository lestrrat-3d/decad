package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/smoothloft"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfaceplace"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file turns a stitched boolean mesh into a Faceted Body
// (docs/evaluator-design.md §9, core §6.1): facets grouped into one Face per
// CONNECTED PATCH of a source analytic face — the source's origins ride onto
// every patch of it, so provenance survives the boolean, while a source the
// boolean cut into disconnected pieces reports each piece as the separate face
// it now is — real Face/Loop/Edge/Vertex topology chained along the face
// boundaries, shells and voids decided by exact signed volume and exact
// containment parity, and measurements integrated exactly over the held mesh,
// reported Approximate with the proven composed bounds. An ordinary Boolean
// face is Faceted; a structural source with its own surface proof can retain
// that analytic tag while the polygons remain the held boundary.

// facetedPayload is the evaluator's own record of a mesh-built body: the
// held mesh, its per-facet source grouping, and the proven error terms the
// measurements compose from. It is what Transform re-evaluates under a composed
// motion (docs/evaluator-design.md §8).
type facetedPayload struct {
	// SmoothRectSection is the immutable exact height curve of an original
	// rectangular LoftSections result. Placement and similarity drop it.
	SmoothRectSection *smoothloft.RectangularSection
	// AxialSectionJoin marks a proven XY section swept through one axial
	// interval. Its held vertical boundary chains retain a Z direction for
	// selectors even though their curve type remains bounded FacetedCurve.
	AxialSectionJoin bool
	// PrismPatternJoin retains immutable source and shell records for a later
	// certified local corner rewrite. Placement drops this source relation.
	PrismPatternJoin *prismPatternJoinRecord
	// SimilaritySource marks a certified similar image of a prism, full
	// Revolve, or exact line-only stack/BRep. Rigid copies retain it;
	// booleans and modify rewrites do not.
	SimilaritySource bool
	// BooleanSimilaritySource marks the certified held mesh and source groups
	// produced by the generic mesh Boolean after its topology and volume audit.
	// Rigid placement and positive similarity retain this certificate.
	BooleanSimilaritySource bool
	// ObliqueBoss retains the two authenticated prism sources for one complete
	// plate-to-cylinder junction. A later Fillet rebuilds from these records.
	ObliqueBoss       *obliqueBossRecord
	PrismUnionFillet  *prismUnionFilletRecord
	CrossCylinderBore *crossCylinderBoreRecord
	LoftCylinderJoin  *loftCylinderJoinRecord
	LoftCylinderBore  *loftCylinderBoreRecord
	LoftMirrorJoin    *loftMirrorJoinRecord
	LoftMirrorCopy    *loftMirrorPlacementRecord
	// MirrorCapGroups names the two outer-cap groups of a loft mirror join.
	// It is provenance only: every placement keeps it, so each copy mints the
	// cap roles under its own identity, whether or not the LoftMirrorCopy
	// proof survives the placement.
	MirrorCapGroups        *[2]int
	LoftMirrorCylinderJoin *loftMirrorCylinderJoinRecord
	LoftMirrorCylinderBore *loftMirrorCylinderBoreRecord
	// ContactAudited records a whole-mesh exact contact audit at construction.
	// PlaceMesh preserves it by checking every rounded facet after placement.
	ContactAudited bool
	// PointSection retains the authenticated Sketch source of a point loft.
	// Boolean results have nil here; ordinary placement keeps it only for an
	// exactly identity delta, and exact rotation copies drop it.
	PointSection *pointSectionRecord
	PointCone    *pointConeTrimRecord
	PointPlane   *pointPlaneSliceRecord
	// PointSimilarity records ordered images of the immutable Split source.
	PointSimilarity *pointPlaneSimilarityRecord
	// PointPlaneCapDisplacement bounds the true source cap contour against
	// each held cap ring, including every placed-basis stretch and rounding.
	PointPlaneCapDisplacement float64
	// PointPlaneSourceEligible belongs only to an original point loft or
	// freshly built Split slab. Rigid copies retain PointPlane for queries,
	// but even a round-trip copy cannot become a new structural Split source.
	PointPlaneSourceEligible bool
	// LoftFacesPrimitive identifies one certified LoftFaces held mesh. Rigid
	// placement preserves this source; a Boolean result starts a new chain.
	LoftFacesPrimitive bool
	// LoftFacesCap records the authenticated final cap map of an original
	// LoftFaces body. A Cut result receives only the one-use cone trim record.
	LoftFacesCap  *loftFacesCapRecord
	LoftFacesToe  *loftFacesToeRecord
	LoftFacesTrim *loftFacesTrimRecord
	// ExactPlaced marks a body whose current true solid is the image of an
	// exact-rotation placement (placedWith with a record), at any angle and
	// whether the copy was direct or not. The pose model of
	// motionbound.PoseDeviation does not describe re-placing such a solid by
	// a float transform, so motionRun.place refuses it. Every later placement,
	// Copy, Transform and ScaledAt builds its payload from the receiver's and
	// MUST keep the mark, whatever happens to CopyLineage or to the source's
	// liveness. A future placement path that does not start from the
	// receiver's payload must carry it over by hand. Only an exact placement
	// sets it; a fresh payload never has it.
	ExactPlaced bool
	// CopyLineage names the live faceted source and its recorded rigid image.
	// A Boolean result has no copy lineage; Copy and placement issue it.
	CopyLineage *facetedCopyLineage
	// SpiralTools exists only on a transient disjoint-tool operand built by
	// A multi-body Union. It maps its source faces back to the live input tools.
	SpiralTools *spiralCombinedTools
	// RevolveLoftJoin retains a geometry-certified section/sector source.
	RevolveLoftJoin *revolveLoftJoinRecord
	// RevolveLoftSector identifies one rigid copy of a trimmed point loft.
	RevolveLoftSector *revolveLoftSectorPlacement
	Verts             []r3.Vec
	// VertexBound is β(v) per held vertex (docs/faceted-vertex-bounds-design.md
	// §2, §4.1), composed by the boolean that built the payload (§3) and
	// carried through every placement. The boolean always writes it; a nil
	// record is an invariant failure, never a fallback.
	VertexBound []float64
	Tris        [][3]int
	Src         []int // per-facet source-group id
	Groups      []facetGroup

	// FaceOf maps each facet to its face's index in the built body's
	// Faces() order; buildFacetedBody sets it, Tessellate reads it.
	FaceOf []int

	// ExactSourceVerts and ExactSourceTris name the zero-bound Boolean mesh
	// before translation-only placement rounded its held coordinates. They
	// certify its true occupied boundary under Xform; ordinary positive-bound
	// Boolean results and non-translation placements carry neither record.
	ExactSourceVerts []r3.Vec
	ExactSourceTris  [][3]int
	// LowerSupport survives one mesh Union only when an exact source box owns
	// the lower face and the other operand is certified strictly above it.
	// Placement drops this proof until source-frame transfer is certified.
	LowerSupport *facetproof.LowerSupport

	// MeshBound is the proven vertex-level bound (mm): no point of the true
	// result boundary is farther than this from the held mesh's
	// corresponding piece. It is the largest facet bound δ(t) over the held
	// facets, each the largest of its corners' VertexBound. VolSymDiff bounds the volume of the symmetric
	// difference between the held solid and the true result. AreaSlack
	// bounds the area the held mesh cannot report: the chord-length deficit
	// the operand tessellations carry, plus the area of any facet the final
	// weld collapsed out of the mesh.
	// DPair bounds the held and true result diameter for the centroid
	// allowance. Some sources supply the broader operand-pair diameter.
	MeshBound  float64
	VolSymDiff float64
	AreaSlack  float64
	DPair      float64

	// Diameter is the held Faceted body's own diameter for Verify's
	// tolerance gate. It is computed from every payload vertex, including
	// interior tessellation vertices that have no B-rep Vertex, and rebuilt
	// after every placement. It is read through the one witness-maximum reader
	// every gate diameter is published through (diameter.PointsWithBudget,
	// verify.go), so it sits at or below the held boundary's own diameter.
	Diameter float64

	Xform r3.Transform
}

// Transform is the accumulated rigid placement.
func (fp facetedPayload) Transform() r3.Transform { return fp.Xform }

// Placed re-evaluates the held mesh under the composed motion: the vertices
// move through the delta motion (float rounding is folded into the proven
// bounds — the geometry is never silently trusted), the moved mesh is kept
// embedded (facetproof.PlaceMesh), a reflection flips the windings, and the
// topology and measurements rebuild from the moved mesh.
func (fp facetedPayload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	d, ref := documentHandle(env.Doc), env.Ref
	return placedNode(fp.placedWith(ctx, d, ref, composed, nil))
}

// placedWith is placed for a copy that denotes the exact rotation exact, or
// the float placement alone when exact is nil. With a record the result
// denotes Q*S + t_Q, where S is fp's true solid: the held mesh is placed by
// the float delta as ever, charged for its distance to Q, and every record
// that describes the float placement rather than Q is dropped (every group
// surface tag and denoted patch, PointSection, PointPlane, PointSimilarity,
// RevolveLoftSector and LoftMirrorCopy). A dropped record is never rebuilt
// from a later float placement.
func (fp facetedPayload) placedWith(ctx context.Context, d *Document, ref producerID, composed r3.Transform,
	exact *exactrotation.Rotation) (*Body, error) {
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	delta, err := facetedrecord.PlacementDelta(fp.Xform, composed)
	if err != nil {
		return nil, err
	}
	next := fp
	next.SmoothRectSection = nil
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
		next.LoftMirrorCopy = &loftMirrorPlacementRecord{Source: fp.LoftMirrorJoin, Motion: composed}
	} else if fp.LoftMirrorCopy != nil {
		next.LoftMirrorCopy = &loftMirrorPlacementRecord{Source: fp.LoftMirrorCopy.Source, Motion: composed}
	}
	if fp.PointCone != nil && fp.Xform == r3.Identity() {
		next.RevolveLoftSector = &revolveLoftSectorPlacement{Source: fp.PointCone, Motion: composed}
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
	next.Groups = append([]facetGroup(nil), fp.Groups...)
	if fp.LoftMirrorJoin != nil && fp.MirrorCapGroups == nil {
		return nil, fmt.Errorf("%w: a loft mirror join lost its outer-cap groups", ErrUnsupported)
	}
	if fp.MirrorCapGroups != nil {
		for cap, group := range fp.MirrorCapGroups {
			if group < 0 || group >= len(next.Groups) {
				return nil, fmt.Errorf("%w: placed mirror join lost an outer cap", ErrUnsupported)
			}
			role := provenance.RoleCapStart
			if cap == 1 {
				role = provenance.RoleCapEnd
			}
			origins := append([]provenance.Ref(nil), next.Groups[group].Origins...)
			next.Groups[group].Origins = append(origins, provenance.Ref{Producer: ref, Role: role})
		}
	}
	for i := range next.Groups {
		if exact != nil {
			if err := facetedrecord.ExactGroupPlaced(&next.Groups[i], exact); err != nil {
				return nil, err
			}
			continue
		}
		if next.Groups[i].HasCentroid {
			reading, err := facetedrecord.PlacedCapCentroid(next.Groups[i].Centroid, delta)
			if err != nil {
				return nil, err
			}
			next.Groups[i].Centroid = reading
		}
		// A later structural trim requires the unplaced source. Its retained
		// public surface identity still moves with this faceted restatement.
		// The moved tag is the float image of the source tag, a bounded
		// stand-in. NormalAt proves a denoted group against the denoted
		// patch's exact image and never reads NormalBound. Any other group is
		// charged the largest distance of two unit normals.
		if tag := next.Groups[i].Surface; tag != nil && delta != r3.Identity() {
			switch tag.(type) {
			case surfacegeom.Plane, surfacegeom.Cylinder, surfacegeom.Cone, surfacegeom.Sphere, surfacegeom.Torus:
				moved, err := surfaceplace.TransformSurface(tag, delta)
				if err != nil {
					return nil, err
				}
				next.Groups[i].Surface = moved
				next.Groups[i].NormalBound = 2
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
			return nil, fmt.Errorf("%w: the exact rotation has no finite gap to the float placement", ErrUnsupported)
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
	next.DPair = facetedrecord.SourceDiameterUpper(next.Verts, next.MeshBound)
	if proofbound.IsNonFinite(next.DPair) {
		return nil, fmt.Errorf("%w: the placed pair diameter has no finite proof", ErrUnsupported)
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
		record := pointPlaneSimilarityRecord{Slice: fp.PointPlane}
		if fp.PointSimilarity != nil {
			record.Steps = append(record.Steps, fp.PointSimilarity.Steps...)
		}
		record.Steps = append(record.Steps, pointPlaneSimilarityStep{Motion: delta,
			MeshBound: next.MeshBound, CapDisplacement: next.PointPlaneCapDisplacement})
		next.PointSimilarity = &record
	}
	return buildFacetedBody(ctx, d, ref, next)
}

// buildFacetedBody assembles the Faceted body from the payload: exact
// component/void analysis, per-source-face topology, and measurements with
// the composed proven bounds.
func buildFacetedBody(ctx context.Context, d *Document, ref producerID, pp facetedPayload) (*Body, error) {
	return buildFacetedBodyWithProof(ctx, d, ref, pp, nil, Measurement{}, nil)
}

// buildFacetedBodyWithProof reuses a proof only for the evaluator's unchanged
// result mesh. Placement calls buildFacetedBody and proves its moved mesh anew.
func buildFacetedBodyWithProof(ctx context.Context, d *Document, ref producerID, pp facetedPayload,
	audit *facetproof.MeshAudit, volume Measurement, volRat *big.Rat) (*Body, error) {
	in := facetedrecord.Mesh{Verts: pp.Verts, VertexBound: pp.VertexBound, Tris: pp.Tris, Src: pp.Src,
		Groups: pp.Groups, AxialSectionJoin: pp.AxialSectionJoin, SpiralTools: pp.SpiralTools != nil,
		VolSymDiff: pp.VolSymDiff, AreaSlack: pp.AreaSlack, DPair: pp.DPair}
	body, faceOf, diameterReading, err := facetedrecord.AssembleBody(ctx, envOf(documentNode(d), ref), in,
		audit, measurementToInternal(volume), volRat)
	if err != nil {
		return nil, err
	}
	pp.Diameter = diameterReading
	pp.FaceOf = faceOf
	body.Payload = pp
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return bodyHandle(body), nil
}

// meshVolumeMeasurement reads the audited mesh's exact component volumes and
// composes the symmetric-difference allowance with final float rounding.
func meshVolumeMeasurement(ctx context.Context, audit *facetproof.MeshAudit, volSymDiff float64) (Measurement, *big.Rat, error) {
	if err := ctx.Err(); err != nil {
		return Measurement{}, nil, err
	}
	volF, bound, volRat, err := facetproof.VolumeFromAudit(audit, volSymDiff)
	if ctxErr := ctx.Err(); ctxErr != nil {
		return Measurement{}, nil, ctxErr
	}
	if err != nil {
		return Measurement{}, nil, err
	}
	return Measurement{
		Value:     units.CubicMillimeters(volF),
		Exactness: exactnessOf(bound),
		Bound:     units.CubicMillimeters(bound),
	}, volRat, nil
}

// exactnessOf keys a measurement's exactness off its proven bound: zero is the
// truth, anything else is Approximate.
//
// A zero bound is therefore a CLAIM that the reported value is exactly
// representable — and only a value proven so may reach it: analytic features
// with explicit rational/error evaluation, and boolean rational integrals
// carrying their own rounding term. A value a float loop COMPUTED is never among
// them: its bound comes from internal/proofbound/bounds.go, whose helpers are never zero for a
// nonzero float-computed quantity (proofbound.SumSlop, proofbound.ChainLengthBound), so it can never
// arrive here claiming Exact.
func exactnessOf(bound float64) Exactness {
	if bound == 0 {
		return Exact
	}
	return Approximate
}
