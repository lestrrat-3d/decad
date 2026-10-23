package facetedrecord

import (
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/smoothloft"
	"github.com/lestrrat-3d/r3"
)

// Payload is the evaluator's own record of a mesh-built body: the
// held mesh, its per-facet source grouping, and the proven error terms the
// measurements compose from. It is what Transform re-evaluates under a composed
// motion (agent-docs/evaluator-design.md §8).
type Payload struct {
	// SmoothRectSection is the immutable exact height curve of an original
	// rectangular LoftSections result. Placement and similarity drop it.
	SmoothRectSection *smoothloft.RectangularSection
	// SmoothHomotheticSection retains the exact base walk and height scale of
	// an original three-section whole-line LoftSections result.
	SmoothHomotheticSection *smoothloft.HomotheticSection
	// AxialSectionJoin marks a proven XY section swept through one axial
	// interval. Its held vertical boundary chains retain a Z direction for
	// selectors even though their curve type remains bounded FacetedCurve.
	AxialSectionJoin bool
	// PrismPatternJoin retains immutable source and shell records for a later
	// certified local corner rewrite. Placement drops this source relation.
	PrismPatternJoin *PrismPatternJoinRecord
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
	ObliqueBoss       *ObliqueBossRecord
	PrismUnionFillet  *PrismUnionFilletRecord
	CrossCylinderBore *CrossCylinderBoreRecord
	LoftCylinderJoin  *LoftCylinderJoinRecord
	LoftCylinderBore  *LoftCylinderBoreRecord
	LoftMirrorJoin    *LoftMirrorJoinRecord
	LoftMirrorCopy    *LoftMirrorPlacementRecord
	// MirrorCapGroups names the two outer-cap groups of a loft mirror join.
	// It is provenance only: every placement keeps it, so each copy mints the
	// cap roles under its own identity, whether or not the LoftMirrorCopy
	// proof survives the placement.
	MirrorCapGroups        *[2]int
	LoftMirrorCylinderJoin *LoftMirrorCylinderJoinRecord
	LoftMirrorCylinderBore *LoftMirrorCylinderBoreRecord
	// ContactAudited records a whole-mesh exact contact audit at construction.
	// PlaceMesh preserves it by checking every rounded facet after placement.
	ContactAudited bool
	// PointSection retains the authenticated Sketch source of a point loft.
	// Boolean results have nil here; ordinary placement keeps it only for an
	// exactly identity delta, and exact rotation copies drop it.
	PointSection *PointSectionRecord
	PointCone    *PointConeTrimRecord
	PointPlane   *PointPlaneSliceRecord
	// PointSimilarity records ordered images of the immutable Split source.
	PointSimilarity *PointPlaneSimilarityRecord
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
	// HeldFloor is the largest bound, in millimetres, of facets this body keeps
	// from fixed held meshes that cannot be refined: a certified primitive's
	// own mesh, carried through Booleans and rigid placement. Facets a Boolean
	// creates are not counted. A later pair's chord tolerance rises to it, as
	// for the primitive itself, so the chain-depth gate admits those facets.
	HeldFloor float64
	// LoftFacesCap records the authenticated final cap map of an original
	// LoftFaces body. A Cut result receives only the one-use cone trim record.
	LoftFacesCap  *LoftFacesCapRecord
	LoftFacesToe  *LoftFacesToeRecord
	LoftFacesTrim *LoftFacesTrimRecord
	// ExactPlaced marks a body whose current true solid is the image of an
	// exact-rotation placement (PlacedWith with a record), at any angle and
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
	CopyLineage *FacetedCopyLineage
	// SpiralTools exists only on a transient disjoint-tool operand built by
	// A multi-body Union. It maps its source faces back to the live input tools.
	SpiralTools *SpiralCombinedTools
	// RevolveLoftJoin retains a geometry-certified section/sector source.
	RevolveLoftJoin *RevolveLoftJoinRecord
	// RevolveLoftSector identifies one rigid copy of a trimmed point loft.
	RevolveLoftSector *RevolveLoftSectorPlacement
	Verts             []r3.Vec
	// VertexBound is β(v) per held vertex (agent-docs/faceted-vertex-bounds-design.md
	// §2, §4.1), composed by the boolean that built the payload (§3) and
	// carried through every placement. The boolean always writes it; a nil
	// record is an invariant failure, never a fallback.
	VertexBound []float64
	Tris        [][3]int
	Src         []int // per-facet source-group id
	Groups      []Group

	// FaceOf maps each facet to its face's index in the built body's
	// Faces() order; BuildWithProof sets it, Tessellate reads it.
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
func (fp Payload) Transform() r3.Transform { return fp.Xform }

// SurveyClass names the payload class a staged survey diagnostic and the
// tessellate refusal report: the root type name this payload replaced.
func (Payload) SurveyClass() string { return "facetedPayload" }

// CertifiedBooleanSimilaritySource checks the private Boolean result's
// geometry, proof terms, and source-group mapping before ScaleMesh consumes it.
func CertifiedBooleanSimilaritySource(fp Payload, faceCount int) bool {
	if len(fp.Tris) == 0 || len(fp.Tris) != len(fp.Src) ||
		len(fp.Tris) != len(fp.FaceOf) || len(fp.Groups) == 0 || faceCount == 0 ||
		fp.MeshBound < 0 || fp.VolSymDiff < 0 || fp.AreaSlack < 0 || fp.DPair <= 0 ||
		proofbound.IsNonFinite(fp.MeshBound) || proofbound.IsNonFinite(fp.VolSymDiff) ||
		proofbound.IsNonFinite(fp.AreaSlack) || proofbound.IsNonFinite(fp.DPair) {
		return false
	}
	for _, beta := range fp.VertexBound {
		if beta < 0 || proofbound.IsNonFinite(beta) {
			return false
		}
	}
	if fp.MeshBound < facetproof.FacetBoundMax(fp.Tris, fp.VertexBound) {
		return false
	}
	for _, source := range fp.Src {
		if source < 0 || source >= len(fp.Groups) {
			return false
		}
	}
	for _, face := range fp.FaceOf {
		if face < 0 || face >= faceCount {
			return false
		}
	}
	for _, group := range fp.Groups {
		if len(group.Origins) == 0 {
			return false
		}
	}
	return true
}
