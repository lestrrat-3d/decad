package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/diameter"
	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/facetedtopology"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/smoothloft"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
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

// facetGroup is one source face's provenance, carried into the payload so a
// rebuild (Transform) reproduces the same origins.
type facetGroup struct {
	origins []provenance.Ref
	// centroid is available only when this entire source group has a separate
	// certificate. A disconnected group cannot give that reading to a patch.
	centroid    VecMeasurement
	hasCentroid bool
	// surface is set by a source whose authenticated surface is retained,
	// including structural trims and the smooth three-section loft. Ordinary
	// mesh booleans leave it nil.
	surface surfacegeom.Surface
	denoted *surfacenormal.Revolved
	// normalBound covers the tag's normal departure from its denoted patch.
	// A placed planar tag conservatively uses the maximum distance of two
	// unit normals when its rounded frame has no tighter certificate.
	normalBound float64
	// reversed records a retained analytic surface whose natural normal
	// points into the result, as on the cone exposed by a Cut.
	reversed bool
	// planar records whether the source analytic face is a plane: the rim
	// between two planar sources is a straight line, whose chord length is
	// exact; any curved source makes the rim's true length unboundable
	// without curvature knowledge, and Edge.Length must refuse.
	planar bool
}

// facetedPayload is the evaluator's own record of a mesh-built body: the
// held mesh, its per-facet source grouping, and the proven error terms the
// measurements compose from. It is what Transform re-evaluates under a composed
// motion (docs/evaluator-design.md §8).
type facetedPayload struct {
	// smoothRectSection is the immutable exact height curve of an original
	// rectangular LoftSections result. Placement and similarity drop it.
	smoothRectSection *smoothloft.RectangularSection
	// axialSectionJoin marks a proven XY section swept through one axial
	// interval. Its held vertical boundary chains retain a Z direction for
	// selectors even though their curve type remains bounded FacetedCurve.
	axialSectionJoin bool
	// prismPatternJoin retains immutable source and shell records for a later
	// certified local corner rewrite. Placement drops this source relation.
	prismPatternJoin *prismPatternJoinRecord
	// similaritySource marks a certified similar image of a prism, full
	// Revolve, or exact line-only stack/BRep. Rigid copies retain it;
	// booleans and modify rewrites do not.
	similaritySource bool
	// booleanSimilaritySource marks the certified held mesh and source groups
	// produced by the generic mesh Boolean after its topology and volume audit.
	// Rigid placement and positive similarity retain this certificate.
	booleanSimilaritySource bool
	// obliqueBoss retains the two authenticated prism sources for one complete
	// plate-to-cylinder junction. A later Fillet rebuilds from these records.
	obliqueBoss       *obliqueBossRecord
	prismUnionFillet  *prismUnionFilletRecord
	crossCylinderBore *crossCylinderBoreRecord
	loftCylinderJoin  *loftCylinderJoinRecord
	loftCylinderBore  *loftCylinderBoreRecord
	loftMirrorJoin    *loftMirrorJoinRecord
	loftMirrorCopy    *loftMirrorPlacementRecord
	// mirrorCapGroups names the two outer-cap groups of a loft mirror join.
	// It is provenance only: every placement keeps it, so each copy mints the
	// cap roles under its own identity, whether or not the loftMirrorCopy
	// proof survives the placement.
	mirrorCapGroups        *[2]int
	loftMirrorCylinderJoin *loftMirrorCylinderJoinRecord
	loftMirrorCylinderBore *loftMirrorCylinderBoreRecord
	// contactAudited records a whole-mesh exact contact audit at construction.
	// PlaceMesh preserves it by checking every rounded facet after placement.
	contactAudited bool
	// pointSection retains the authenticated Sketch source of a point loft.
	// Boolean results have nil here; ordinary placement keeps it only for an
	// exactly identity delta, and exact rotation copies drop it.
	pointSection *pointSectionRecord
	pointCone    *pointConeTrimRecord
	pointPlane   *pointPlaneSliceRecord
	// pointSimilarity records ordered images of the immutable Split source.
	pointSimilarity *pointPlaneSimilarityRecord
	// pointPlaneCapDisplacement bounds the true source cap contour against
	// each held cap ring, including every placed-basis stretch and rounding.
	pointPlaneCapDisplacement float64
	// pointPlaneSourceEligible belongs only to an original point loft or
	// freshly built Split slab. Rigid copies retain pointPlane for queries,
	// but even a round-trip copy cannot become a new structural Split source.
	pointPlaneSourceEligible bool
	// LoftFacesPrimitive identifies one certified LoftFaces held mesh. Rigid
	// placement preserves this source; a Boolean result starts a new chain.
	LoftFacesPrimitive bool
	// loftFacesCap records the authenticated final cap map of an original
	// LoftFaces body. A Cut result receives only the one-use cone trim record.
	loftFacesCap  *loftFacesCapRecord
	loftFacesToe  *loftFacesToeRecord
	loftFacesTrim *loftFacesTrimRecord
	// exactPlaced marks a body whose current true solid is the image of an
	// exact-rotation placement (placedWith with a record), at any angle and
	// whether the copy was direct or not. The pose model of
	// motionbound.PoseDeviation does not describe re-placing such a solid by
	// a float transform, so motionRun.place refuses it. Every later placement,
	// Copy, Transform and ScaledAt builds its payload from the receiver's and
	// MUST keep the mark, whatever happens to copyLineage or to the source's
	// liveness. A future placement path that does not start from the
	// receiver's payload must carry it over by hand. Only an exact placement
	// sets it; a fresh payload never has it.
	exactPlaced bool
	// copyLineage names the live faceted source and its recorded rigid image.
	// A Boolean result has no copy lineage; Copy and placement issue it.
	copyLineage *facetedCopyLineage
	// spiralTools exists only on a transient disjoint-tool operand built by
	// A multi-body Union. It maps its source faces back to the live input tools.
	spiralTools *spiralCombinedTools
	// revolveLoftJoin retains a geometry-certified section/sector source.
	revolveLoftJoin *revolveLoftJoinRecord
	// revolveLoftSector identifies one rigid copy of a trimmed point loft.
	revolveLoftSector *revolveLoftSectorPlacement
	Verts             []r3.Vec
	// vertexBound is β(v) per held vertex (docs/faceted-vertex-bounds-design.md
	// §2, §4.1), composed by the boolean that built the payload (§3) and
	// carried through every placement. The boolean always writes it; a nil
	// record is an invariant failure, never a fallback.
	vertexBound []float64
	Tris        [][3]int
	Src         []int // per-facet source-group id
	groups      []facetGroup

	// FaceOf maps each facet to its face's index in the built body's
	// Faces() order; buildFacetedBody sets it, Tessellate reads it.
	FaceOf []int

	// exactSourceVerts and exactSourceTris name the zero-bound Boolean mesh
	// before translation-only placement rounded its held coordinates. They
	// certify its true occupied boundary under xform; ordinary positive-bound
	// Boolean results and non-translation placements carry neither record.
	exactSourceVerts []r3.Vec
	exactSourceTris  [][3]int
	// lowerSupport survives one mesh Union only when an exact source box owns
	// the lower face and the other operand is certified strictly above it.
	// Placement drops this proof until source-frame transfer is certified.
	lowerSupport *facetproof.LowerSupport

	// meshBound is the proven vertex-level bound (mm): no point of the true
	// result boundary is farther than this from the held mesh's
	// corresponding piece. It is the largest facet bound δ(t) over the held
	// facets, each the largest of its corners' vertexBound. volSymDiff bounds the volume of the symmetric
	// difference between the held solid and the true result. areaSlack
	// bounds the area the held mesh cannot report: the chord-length deficit
	// the operand tessellations carry, plus the area of any facet the final
	// weld collapsed out of the mesh.
	// dPair bounds the held and true result diameter for the centroid
	// allowance. Some sources supply the broader operand-pair diameter.
	meshBound  float64
	volSymDiff float64
	areaSlack  float64
	dPair      float64

	// diameter is the held Faceted body's own diameter for Verify's
	// tolerance gate. It is computed from every payload vertex, including
	// interior tessellation vertices that have no B-rep Vertex, and rebuilt
	// after every placement. It is read through the one witness-maximum reader
	// every gate diameter is published through (diameter.PointsWithBudget,
	// verify.go), so it sits at or below the held boundary's own diameter.
	diameter float64

	xform r3.Transform
}

// Transform is the accumulated rigid placement.
func (fp facetedPayload) Transform() r3.Transform { return fp.xform }

// Placed re-evaluates the held mesh under the composed motion: the vertices
// move through the delta motion (float rounding is folded into the proven
// bounds — the geometry is never silently trusted), the moved mesh is kept
// embedded (facetproof.PlaceMesh), a reflection flips the windings, and the
// topology and measurements rebuild from the moved mesh.
func (fp facetedPayload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	d, ref := documentHandle(env.Doc), env.Ref
	return placedNode(fp.placedWith(ctx, d, ref, composed, nil))
}

// facetedPlacementDelta is the float map applied to the held mesh. Motion
// verification must charge this same map against its ideal pose.
func facetedPlacementDelta(xform, composed r3.Transform) (r3.Transform, error) {
	inv, err := xform.Inverse()
	if err != nil {
		return r3.Transform{}, fmt.Errorf(`decad: inverting the accumulated placement failed: %w`, err)
	}
	delta, err := inv.Then(composed)
	if err != nil {
		return r3.Transform{}, fmt.Errorf(`decad: composing the placement failed: %w`, err)
	}
	return delta, nil
}

// placedWith is placed for a copy that denotes the exact rotation exact, or
// the float placement alone when exact is nil. With a record the result
// denotes Q*S + t_Q, where S is fp's true solid: the held mesh is placed by
// the float delta as ever, charged for its distance to Q, and every record
// that describes the float placement rather than Q is dropped (every group
// surface tag and denoted patch, pointSection, pointPlane, pointSimilarity,
// revolveLoftSector and loftMirrorCopy). A dropped record is never rebuilt
// from a later float placement.
func (fp facetedPayload) placedWith(ctx context.Context, d *Document, ref producerID, composed r3.Transform,
	exact *exactrotation.Rotation) (*Body, error) {
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	delta, err := facetedPlacementDelta(fp.xform, composed)
	if err != nil {
		return nil, err
	}
	next := fp
	next.smoothRectSection = nil
	next.axialSectionJoin = false
	next.prismPatternJoin = nil
	next.xform = composed
	next.lowerSupport = nil
	next.pointCone = nil
	next.loftFacesCap = nil
	next.loftFacesToe = nil
	next.loftFacesTrim = nil
	next.copyLineage = nil
	next.spiralTools = nil
	// A rigid copy keeps the immutable Split source for certified face
	// centroid ranking, but cannot become a new structural Split source.
	next.pointPlaneSourceEligible = false
	next.revolveLoftJoin = nil
	next.obliqueBoss = nil
	next.prismUnionFillet = nil
	next.crossCylinderBore = nil
	next.loftCylinderJoin = nil
	next.loftCylinderBore = nil
	next.loftMirrorJoin = nil
	next.loftMirrorCopy = nil
	next.loftMirrorCylinderJoin = nil
	next.loftMirrorCylinderBore = nil
	next.revolveLoftSector = nil
	// The true solid follows each exact reading of delta, even when the
	// accumulated float placement later returns to identity.
	if delta != r3.Identity() {
		next.pointSection = nil
	}
	if fp.loftMirrorJoin != nil && fp.xform == r3.Identity() {
		next.loftMirrorCopy = &loftMirrorPlacementRecord{source: fp.loftMirrorJoin, motion: composed}
	} else if fp.loftMirrorCopy != nil {
		next.loftMirrorCopy = &loftMirrorPlacementRecord{source: fp.loftMirrorCopy.source, motion: composed}
	}
	if fp.pointCone != nil && fp.xform == r3.Identity() {
		next.revolveLoftSector = &revolveLoftSectorPlacement{source: fp.pointCone, motion: composed}
	}
	if exact != nil {
		next.loftMirrorCopy = nil
		next.revolveLoftSector = nil
		next.pointSection = nil
		next.pointPlane = nil
		next.pointSimilarity = nil
		next.pointPlaneCapDisplacement = 0
		next.exactPlaced = true
	}
	next.groups = append([]facetGroup(nil), fp.groups...)
	if fp.loftMirrorJoin != nil && fp.mirrorCapGroups == nil {
		return nil, fmt.Errorf("%w: a loft mirror join lost its outer-cap groups", ErrUnsupported)
	}
	if fp.mirrorCapGroups != nil {
		for cap, group := range fp.mirrorCapGroups {
			if group < 0 || group >= len(next.groups) {
				return nil, fmt.Errorf("%w: placed mirror join lost an outer cap", ErrUnsupported)
			}
			role := provenance.RoleCapStart
			if cap == 1 {
				role = provenance.RoleCapEnd
			}
			origins := append([]provenance.Ref(nil), next.groups[group].origins...)
			next.groups[group].origins = append(origins, provenance.Ref{Producer: ref, Role: role})
		}
	}
	for i := range next.groups {
		if exact != nil {
			if err := exactGroupPlaced(&next.groups[i], exact); err != nil {
				return nil, err
			}
			continue
		}
		if next.groups[i].hasCentroid {
			reading, err := placedCapCentroid(next.groups[i].centroid, delta)
			if err != nil {
				return nil, err
			}
			next.groups[i].centroid = reading
		}
		// A later structural trim requires the unplaced source. Its retained
		// public surface identity still moves with this faceted restatement.
		// The moved tag is the float image of the source tag, a bounded
		// stand-in. NormalAt proves a denoted group against the denoted
		// patch's exact image and never reads normalBound. Any other group is
		// charged the largest distance of two unit normals.
		if tag := next.groups[i].surface; tag != nil && delta != r3.Identity() {
			switch tag.(type) {
			case surfacegeom.Plane, surfacegeom.Cylinder, surfacegeom.Cone, surfacegeom.Sphere, surfacegeom.Torus:
				moved, err := surfaceplace.TransformSurface(tag, delta)
				if err != nil {
					return nil, err
				}
				next.groups[i].surface = moved
				next.groups[i].normalBound = 2
				// Only a Plane normal is U x V, which a reflection negates.
				if _, planar := tag.(surfacegeom.Plane); planar && delta.IsReflection() {
					next.groups[i].reversed = !next.groups[i].reversed
				}
			}
		}
		if next.groups[i].denoted != nil {
			denoted := next.groups[i].denoted.Transformed(delta)
			next.groups[i].denoted = &denoted
		}
	}
	if exact != nil && !exact.IsIdentity() {
		next.exactSourceVerts, next.exactSourceTris = nil, nil
	} else if !facetproof.TranslationOnly(delta) || !facetproof.TranslationOnly(composed) {
		next.exactSourceVerts, next.exactSourceTris = nil, nil
	} else if len(fp.exactSourceVerts) == 0 && fp.meshBound == 0 && fp.volSymDiff == 0 &&
		fp.xform == r3.Identity() {
		next.exactSourceVerts = append([]r3.Vec(nil), fp.Verts...)
		next.exactSourceTris = append([][3]int(nil), fp.Tris...)
	}
	var charge massmoment.MapCharge
	var placed facetproof.PlacedMesh
	if exact != nil {
		linearGap, translationGap, ok := exact.Gap(delta)
		if !ok {
			return nil, fmt.Errorf("%w: the exact rotation has no finite gap to the float placement", ErrUnsupported)
		}
		placed, err = facetproof.PlaceMeshDenoting(ctx, budget, fp.Verts, fp.Tris, fp.vertexBound,
			fp.volSymDiff, fp.areaSlack, delta, linearGap, translationGap)
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
		placed, err = facetproof.PlaceMesh(ctx, budget, fp.Verts, fp.Tris, fp.vertexBound,
			fp.volSymDiff, fp.areaSlack, charge.Stretch, charge.Volume, delta)
	}
	if err != nil {
		return nil, err
	}
	next.Verts, next.Tris = placed.Verts, placed.Tris
	next.vertexBound, next.meshBound, next.volSymDiff = placed.VertexBound, placed.MeshBound, placed.VolSymDiff
	next.areaSlack = placed.AreaSlack
	next.dPair = facetedSourceDiameterUpper(next.Verts, next.meshBound)
	if proofbound.IsNonFinite(next.dPair) {
		return nil, fmt.Errorf("%w: the placed pair diameter has no finite proof", ErrUnsupported)
	}
	if fp.pointPlane != nil && exact == nil {
		// PlaceMesh's MeshBound includes the held-coordinate rounding and any
		// embedding correction, but does not stretch the previous source gap.
		// Add both terms; the possible double charge is conservative.
		next.pointPlaneCapDisplacement = proofbound.AbsSumUpper(
			proofbound.ProductUpper(fp.pointPlaneCapDisplacement,
				proofbound.AbsSumUpper(1, charge.Stretch)), placed.MeshBound)
	}
	if fp.pointPlane != nil && exact == nil {
		record := pointPlaneSimilarityRecord{slice: fp.pointPlane}
		if fp.pointSimilarity != nil {
			record.steps = append(record.steps, fp.pointSimilarity.steps...)
		}
		record.steps = append(record.steps, pointPlaneSimilarityStep{motion: delta,
			meshBound: next.meshBound, capDisplacement: next.pointPlaneCapDisplacement})
		next.pointSimilarity = &record
	}
	return buildFacetedBody(ctx, d, ref, next)
}

// facetedFacePerimeterUpper reads every loop's edge lengths and error bounds
// and returns proofbound.PerimeterUpper over them.
func facetedFacePerimeterUpper(f *Face, budget *proofbound.WorkBudget) (float64, error) {
	var lengths, bounds []float64
	for _, l := range f.n.Loops {
		for _, ce := range l.CoEdges {
			lengths = append(lengths, ce.Edge.Length)
			bounds = append(bounds, ce.Edge.LengthBound)
		}
	}
	return proofbound.PerimeterUpper(budget, lengths, bounds)
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
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	verts, tris := pp.Verts, pp.Tris
	var diameterReading float64
	var ok bool
	var err error
	if pp.spiralTools != nil {
		// The transient multi-tool operand needs only a lower witness. Its
		// coordinate-extreme vertices give one without a quadratic all-pairs
		// scan over every patterned copy.
		diameterReading, ok, err = diameter.AxisExtremesContext(ctx, verts)
	} else {
		diameterReading, ok, err = diameter.PointsContext(ctx, verts)
	}
	if err != nil {
		return nil, err
	}
	if !ok {
		return nil, fmt.Errorf(`%w: the held boundary has no usable diameter`, ErrBooleanFailed)
	}
	pp.diameter = diameterReading
	if audit == nil {
		audit, err = facetproof.AuditFacetedMesh(ctx, verts, tris)
		if err != nil {
			return nil, err
		}
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	comp, adj := audit.Comp, audit.Adj
	members, compVol, contains := audit.Members, audit.CompVol, audit.Contains

	body := bornBody(&Body{n: topo.Body{Solid: true, Origin: provenance.Ref{Producer: ref, Role: provenance.RoleBody}, Doc: documentNode(d)}})
	if len(pp.vertexBound) != len(verts) {
		return nil, fmt.Errorf(`%w: a faceted payload carries %d vertex bounds for %d vertices`, ErrBooleanFailed, len(pp.vertexBound), len(verts))
	}
	// facetDelta is each held facet's δ(t), the largest of its corners' β
	// (docs/faceted-vertex-bounds-design.md §2).
	facetDelta := facetproof.FacetDeltas(tris, pp.vertexBound)

	for _, s := range pp.Src {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		if s < 0 || s >= len(pp.groups) {
			return nil, fmt.Errorf(`%w: a facet names no source group`, ErrBooleanFailed)
		}
	}

	// Faces: one per connected PATCH of a source group, in facet order.
	//
	// The source face is NOT the face. A boolean can cut one source face into
	// SEVERAL pieces that no longer touch — a blind trench crosses a cap and
	// leaves two separate strips of it standing — and each piece is its own
	// Face, bounded from outside by its own loop. Keying only by source would
	// hand both strips to one Face, which then has two outer boundaries and
	// can call only one of them outer: the other would be reported as a HOLE
	// in a patch it is not even part of. So the key is the patch: the facets
	// of one source group reachable from each other ACROSS SHARED EDGES.
	//
	// Two facets sharing an edge always share a component, so a patch never
	// spans two components and the component need not be keyed on separately.
	// The partition is exactly edge-connectivity, so it adds no face boundary
	// that was not already there: an edge whose two facets differ in patch also
	// differs in source group, and was a boundary before the split.
	patch, err := facetproof.SourcePatches(budget, len(tris), pp.Src, adj)
	if err != nil {
		return nil, err
	}
	groupPatch := make(map[int]int)
	for i, group := range pp.Src {
		if first, exists := groupPatch[group]; !exists {
			groupPatch[group] = patch[i]
		} else if first != patch[i] {
			groupPatch[group] = -1
		}
	}

	faceIdx := map[int]*Face{}
	facePlanar := map[*Face]bool{}
	faceGroup := map[*Face]int{}
	facetFace := make([]*Face, len(tris))
	compFaces := make([][]*Face, len(members))
	// faceTermSlop is each face's summed per-facet evaluation charge
	// (internal/proofbound/bounds.go, proofbound.FacetAreaTermSlop): a sliver
	// facet's cross product cancels, and its rounding tracks its edge products
	// rather than the area SumSlop's relative charge reads.
	faceTermSlop := map[*Face]float64{}
	for i, t := range tris {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		f, ok := faceIdx[patch[i]]
		if !ok {
			// Every patch of one source face carries that face's OWN origins:
			// both strips of a split cap came from the cap, and both must
			// still say so, or FaceCreatedBy and the surveys lose them.
			f = bornFace(&Face{
				n: topo.Face{
					HeldPlanar:  pp.groups[pp.Src[i]].planar,
					Denoted:     pp.groups[pp.Src[i]].denoted,
					NormalBound: pp.groups[pp.Src[i]].normalBound,
					Reversed:    pp.groups[pp.Src[i]].reversed,
					Origins:     append([]provenance.Ref(nil), pp.groups[pp.Src[i]].origins...),
					Body:        bodyNode(body),
				},
			})
			group := pp.groups[pp.Src[i]]
			if group.hasCentroid && groupPatch[pp.Src[i]] == patch[i] {
				f.n.Centroid, f.n.HasCentroid = vecMeasurementToInternal(group.centroid), true
			}
			faceIdx[patch[i]] = f
			facePlanar[f] = pp.groups[pp.Src[i]].planar
			faceGroup[f] = pp.Src[i]
			compFaces[comp[i]] = append(compFaces[comp[i]], f)
		}
		a, b, c := verts[t[0]], verts[t[1]], verts[t[2]]
		f.n.Area += b.Sub(a).Cross(c.Sub(a)).Len() / 2
		faceTermSlop[f] = proofbound.AbsSumUpper(faceTermSlop[f], proofbound.FacetAreaTermSlop(a, b, c))
		facetFace[i] = f
	}
	// A face's bound is the largest δ(t) over its own facets, and its
	// per-facet area allowance sums each facet's perturbation at its own δ(t)
	// (§4.2).
	faceDelta, faceFacetArea, err := facetproof.FaceProofs(budget, verts, tris, facetFace, facetDelta)
	if err != nil {
		return nil, err
	}
	for f, delta := range faceDelta {
		group := pp.groups[faceGroup[f]]
		if group.surface != nil {
			f.n.Surface = group.surface
		} else {
			f.n.Surface = surfacegeom.Faceted{Bound: units.Millimeters(delta)}
		}
	}

	if err := buildFacetedTopology(ctx, verts, tris, facetFace, facePlanar,
		pp.vertexBound, pp.axialSectionJoin); err != nil {
		return nil, err
	}

	// Per-face area bounds: the smaller of two proven geometric terms — the
	// §4 shape, the face's own displacement times its own bounding edges, and
	// docs/faceted-vertex-bounds-design.md §4.2's sum of each facet's
	// perturbation at its own δ(t) — plus the operands' chord-length slack, the
	// PROVEN slop of the float sum that produced the area, and each facet's own
	// evaluation charge. The perimeter the first multiplies is the UPPER one:
	// an edge's held chord length is
	// itself only known within its own bound (internal/proofbound/bounds.go,
	// proofbound.ChainLengthBound).
	facetsOf := map[*Face]int{}
	for _, f := range facetFace {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		facetsOf[f]++
	}
	// bodyGeom sums every face's own geometric term: a rim that moves perturbs
	// BOTH faces it separates, and each face charges it.
	bodyGeom := 0.0
	for _, faces := range compFaces {
		for _, f := range faces {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			loopLen, err := facetedFacePerimeterUpper(f, budget)
			if err != nil {
				return nil, err
			}
			geom := facetproof.FacetedAreaGeom(faceDelta[f], loopLen, faceFacetArea[f])
			bodyGeom = proofbound.AbsSumUpper(bodyGeom, geom)
			f.n.AreaBound = proofbound.AbsSumUpper(geom, pp.areaSlack, proofbound.SumSlop(facetsOf[f], f.n.Area), faceTermSlop[f])
		}
	}

	// Shells and lumps: positive components are outer shells (one lump
	// each); a negative component is a void of its innermost positive
	// container.
	shells := make([]*FaceShell, len(members))
	lumpOf := map[int]*Lump{}
	var lumps []*Lump
	for ci := range members {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		shells[ci] = bornFaceShell(&FaceShell{n: topo.FaceShell{Void: compVol[ci].Sign() < 0, Faces: faceNodes(compFaces[ci])}})
		if compVol[ci].Sign() > 0 {
			l := bornLump(&Lump{n: topo.Lump{Shells: []*topo.FaceShell{faceShellNode(shells[ci])}}})
			lumpOf[ci] = l
			lumps = append(lumps, l)
		}
	}
	parents, err := facetproof.VoidParents(budget, compVol, contains)
	if err != nil {
		return nil, err
	}
	for ci, parent := range parents {
		if parent >= 0 {
			lumpOf[parent].n.Shells = append(lumpOf[parent].n.Shells, faceShellNode(shells[ci]))
		}
	}
	body.n.Lumps = lumpNodes(lumps)

	// Measurements: exact rational integrals over the held mesh, reported
	// with the composed proven bounds (§9: the bound shapes of the
	// verification design, composed from the operands' chord errors). The
	// volume helper is also the read-only interference evaluator's one source
	// of volume truth.
	if volRat == nil {
		var volF, volBound float64
		volF, volBound, volRat, err = facetproof.VolumeFromAudit(audit, pp.volSymDiff)
		volume = Measurement{Value: units.CubicMillimeters(volF),
			Exactness: exactnessOf(volBound), Bound: units.CubicMillimeters(volBound)}
		if err != nil {
			return nil, err
		}
	}
	body.n.Volume = measurementToInternal(volume)
	moments, err := facetproof.FirstMomentsFromAudit(budget, audit, tris)
	if err != nil {
		return nil, err
	}

	areaF, termSlop := 0.0, 0.0
	for _, t := range tris {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		a, b, c := verts[t[0]], verts[t[1]], verts[t[2]]
		areaF += b.Sub(a).Cross(c.Sub(a)).Len() / 2
		termSlop = proofbound.AbsSumUpper(termSlop, proofbound.FacetAreaTermSlop(a, b, c))
	}
	// The float area accumulation itself rounds, and the loop above sums
	// NAIVELY: charge the proven naive-summation bound, never a pairwise one
	// (internal/proofbound/bounds.go, proofbound.SumSlop), beside each term's
	// own evaluation charge, which a sliver facet's cancelling cross product
	// needs at the scale of its edge products (proofbound.FacetAreaTermSlop).
	// Both are ulp-scale in the total, so a genuinely tiny-bound planar boolean
	// stays tiny — and never zero, which would claim an exactness a float sum of
	// square roots does not have.
	areaBound := proofbound.AbsSumUpper(bodyGeom, pp.areaSlack, proofbound.SumSlop(len(tris), areaF), termSlop)
	body.n.Area = measurement.Measurement{
		Value:     units.SquareMillimeters(areaF),
		Exactness: measurement.ExactnessOf(areaBound),
		Bound:     units.SquareMillimeters(areaBound),
	}

	// Centroid: the exact first moment over the exact volume; the bound is
	// the symmetric-difference mass displaced to the pair's own diameter,
	// with the diameter itself as the honest ceiling.
	tf := big.NewRat(1, 24)
	cx := centroidCoord(moments[0], tf, volRat)
	cy := centroidCoord(moments[1], tf, volRat)
	cz := centroidCoord(moments[2], tf, volRat)
	cenBound := facetproof.FacetedCentroidAllowance(pp.volSymDiff, pp.dPair, volFloor(volRat, pp.volSymDiff))
	cxF, _ := cx.Float64()
	cyF, _ := cy.Float64()
	czF, _ := cz.Float64()
	// The three coordinates round independently, and the VecMeasurement bound
	// is a 3D radius (internal/proofbound/bounds.go, proofbound.Radius3D).
	cenRound := proofbound.Radius3D(math.Max(proofbound.RatAbsDiff(cx, cxF), math.Max(proofbound.RatAbsDiff(cy, cyF), proofbound.RatAbsDiff(cz, czF))))
	centroidBound := proofbound.AbsSumUpper(cenBound, cenRound)
	body.n.Centroid = measurement.VecMeasurement{
		Value:     r3.Vec{X: cxF, Y: cyF, Z: czF},
		Exactness: measurement.ExactnessOf(centroidBound),
		Bound:     units.Millimeters(centroidBound),
	}

	lo, hi := verts[0], verts[0]
	for _, v := range verts[1:] {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		lo = r3.Vec{X: math.Min(lo.X, v.X), Y: math.Min(lo.Y, v.Y), Z: math.Min(lo.Z, v.Z)}
		hi = r3.Vec{X: math.Max(hi.X, v.X), Y: math.Max(hi.Y, v.Y), Z: math.Max(hi.Z, v.Z)}
	}
	// Min and Max are POSITIONS, and Bound is the error bound on them: a 3D
	// radius, not a per-axis extent (core §5.2). Each of the six extremes is
	// off by at most facetproof.FacetedExtremeError's per-coordinate figure, which reads
	// each vertex's own facet bound; the corner can be off on every axis at
	// once, so the radius is √3 of the largest (internal/proofbound/bounds.go,
	// proofbound.Radius3D).
	vertexFacetDelta := facetproof.VertexFacetDeltas(len(verts), tris, facetDelta)
	extremeErr, err := facetproof.FacetedExtremeError(budget, verts, pp.vertexBound, vertexFacetDelta, lo, hi)
	if err != nil {
		return nil, err
	}
	boxBound := proofbound.Radius3D(extremeErr)
	body.n.Bounds = measurement.Box{
		Min: lo, Max: hi,
		Exactness: measurement.ExactnessOf(boxBound),
		Bound:     units.Millimeters(boxBound),
	}

	// The facet → face mapping, in the body's Faces() order, for Tessellate.
	if err := budget.Err(); err != nil {
		return nil, err
	}
	faceOf, err := facetproof.FaceIndices(ctx, body.Faces(), facetFace)
	if err != nil {
		return nil, err
	}
	pp.FaceOf = faceOf
	body.n.Payload = pp
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return body, nil
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

// centroidCoord is (moment/24) / volume, exact.
func centroidCoord(m, tf, vol *big.Rat) *big.Rat {
	c := new(big.Rat).Mul(m, tf)
	return c.Quo(c, vol)
}

// volFloor is the proven lower bound on the true volume: the held volume minus
// the symmetric-difference bound, floored at zero. The rational→float rounding
// can go UP, which would overstate the floor and understate every bound divided
// by it, so it is nudged back down first.
func volFloor(vol *big.Rat, sym float64) float64 {
	v, _ := vol.Float64()
	v = math.Nextafter(v, math.Inf(-1))
	rem := v - sym
	if rem <= 0 {
		return 0
	}
	return rem
}

// facetedSourceDiameterUpper bounds any pair of held or true boundary
// points. Every true point lies within departure of the held surface, and
// the held surface lies inside its vertices' coordinate box.
func facetedSourceDiameterUpper(verts []r3.Vec, departure float64) float64 {
	lo, hi := [3]float64{math.Inf(1), math.Inf(1), math.Inf(1)},
		[3]float64{math.Inf(-1), math.Inf(-1), math.Inf(-1)}
	for _, vertex := range verts {
		coords := [3]float64{vertex.X, vertex.Y, vertex.Z}
		for i, v := range coords {
			lo[i] = min(lo[i], v)
			hi[i] = max(hi[i], v)
		}
	}
	return proofbound.AbsSumUpper(proofbound.DvLenUpper(proofbound.HeldDelta(
		r3.NewVec(hi[0], hi[1], hi[2]), r3.NewVec(lo[0], lo[1], lo[2]))),
		proofbound.ProductUpper(2, departure))
}

// buildFacetedTopology chains the face-boundary mesh edges into topological
// Edges (split where the adjacent face pair or the exact hinge convexity
// changes) and builds each face's loops by walking the facet fans. Each
// Vertex reports its own held bound vertexBound[v], and each Edge the largest
// of its chain's vertices' bounds (docs/faceted-vertex-bounds-design.md
// §4.2).
func buildFacetedTopology(
	ctx context.Context,
	verts []r3.Vec,
	tris [][3]int,
	facetFace []*Face,
	facePlanar map[*Face]bool,
	vertexBound []float64,
	axialSectionJoin bool,
) error {
	faceID := map[*Face]int{}
	faces := []*Face{}
	facetID := make([]int, len(facetFace))
	planarID := map[int]bool{}
	for i, f := range facetFace {
		id, ok := faceID[f]
		if !ok {
			id = len(faces)
			faceID[f] = id
			faces = append(faces, f)
			planarID[id] = facePlanar[f]
		}
		facetID[i] = id
	}
	plan, err := facetedtopology.Build(ctx, verts, tris, facetID, planarID, vertexBound)
	if err != nil {
		return err
	}
	vertexObj := map[int]*Vertex{}
	vertexOf := func(vi int) *Vertex {
		if v, ok := vertexObj[vi]; ok {
			return v
		}
		v := bornVertex(&Vertex{n: topo.Vertex{Position: verts[vi], Bound: units.Millimeters(vertexBound[vi])}})
		vertexObj[vi] = v
		return v
	}
	edgeObj := map[*facetedtopology.Edge]*Edge{}
	for _, chain := range plan.Edges {
		e := bornEdge(&Edge{
			n: topo.Edge{
				Convex:          chain.Convex,
				Length:          chain.Length,
				LengthBound:     chain.LengthBound,
				LengthUnbounded: chain.LengthUnbounded,
				Curve:           surfacegeom.FacetedCurve{Bound: units.Millimeters(chain.Bound)},
				Start:           vertexNode(vertexOf(chain.Vertices[0])),
				End:             vertexNode(vertexOf(chain.Vertices[len(chain.Vertices)-1])),
				Faces:           []*topo.Face{faceNode(faces[chain.Faces[0]]), faceNode(faces[chain.Faces[1]])},
			},
		})
		if axialSectionJoin {
			e.n.SelectorDirection, e.n.SelectorDirectionProven =
				facetedtopology.AxialChainDirection(verts, chain.Vertices)
		}
		edgeObj[chain] = e
	}
	for fi, loops := range plan.FaceLoops {
		f := faces[fi]
		for _, planned := range loops {
			loop := bornLoop(&Loop{n: topo.Loop{Outer: planned.Outer}})
			for _, ce := range planned.Coedges {
				loop.n.CoEdges = append(loop.n.CoEdges, topo.CoEdge{Edge: edgeNode(edgeObj[ce.Edge]), Forward: ce.Forward})
			}
			f.n.Loops = append(f.n.Loops, loopNode(loop))
		}
	}
	return nil
}

// exactGroupPlaced restates one source group for a copy that denotes the exact
// rotation q. The cap centroid moves by q itself, which is exactly orthogonal,
// so the placed reading adds no stretch charge. The analytic surface tag and
// its denoted patch describe the float placement, not q, so both are dropped;
// the group's faces then read as Faceted with their held bound. planar,
// reversed and origins describe the source face, which q maps rigidly, and
// stay.
func exactGroupPlaced(g *facetGroup, q *exactrotation.Rotation) error {
	if g.hasCentroid {
		reading, err := placedCapCentroidBy(g.centroid, q.Linear(), q.Translation())
		if err != nil {
			return err
		}
		g.centroid = reading
	}
	g.surface, g.denoted, g.normalBound = nil, nil, 0
	return nil
}
