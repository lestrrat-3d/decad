package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedtopology"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/proof"

	"github.com/lestrrat-3d/decad/internal/proofbound"

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
// reported Approximate with the proven composed bounds. A Faceted face IS
// exactly its polygons; what it approximates is which surface it stands for.

// facetGroup is one source face's provenance, carried into the payload so a
// rebuild (Placed) reproduces the same origins.
type facetGroup struct {
	origins []FeatureRef
	// planar records whether the source analytic face is a plane: the rim
	// between two planar sources is a straight line, whose chord length is
	// exact; any curved source makes the rim's true length unboundable
	// without curvature knowledge, and Edge.Length must refuse.
	planar bool
}

// facetedPayload is the evaluator's own record of a boolean-built body: the
// held mesh, its per-facet source grouping, and the proven error terms the
// measurements compose from. It is what Placed re-evaluates under a composed
// motion (docs/evaluator-design.md §8).
type facetedPayload struct {
	verts []r3.Vec
	// vertexBound is β(v) per held vertex (docs/faceted-vertex-bounds-design.md
	// §2, §4.1), composed by the boolean that built the payload (§3) and
	// carried through every placement. The boolean always writes it; a nil
	// record is an invariant failure, never a fallback.
	vertexBound []float64
	tris        [][3]int
	src         []int // per-facet source-group id
	groups      []facetGroup

	// faceOf maps each facet to its face's index in the built body's
	// Faces() order; buildFacetedBody sets it, Tessellate reads it.
	faceOf []int

	// exactSourceVerts and exactSourceTris name the zero-bound Boolean mesh
	// before translation-only placement rounded its held coordinates. They
	// certify its true occupied boundary under xform; ordinary positive-bound
	// Boolean results and non-translation placements carry neither record.
	exactSourceVerts []r3.Vec
	exactSourceTris  [][3]int
	// lowerSupport survives one mesh Union only when an exact source box owns
	// the lower face and the other operand is certified strictly above it.
	// Placement drops this proof until source-frame transfer is certified.
	lowerSupport *facetedLowerSupport

	// meshBound is the proven vertex-level bound (mm): no point of the true
	// result boundary is farther than this from the held mesh's
	// corresponding piece. It is the largest facet bound δ(t) over the held
	// facets, each the largest of its corners' vertexBound. volSymDiff bounds the volume of the symmetric
	// difference between the held solid and the true result. areaSlack
	// bounds the area the held mesh cannot report: the chord-length deficit
	// the operand tessellations carry, plus the area of any facet the final
	// weld collapsed out of the mesh.
	// dPair is the operand pair's diameter, the centroid bound's yardstick.
	meshBound  float64
	volSymDiff float64
	areaSlack  float64
	dPair      float64

	// diameter is the held Faceted body's own diameter for Verify's
	// tolerance gate. It is computed from every payload vertex, including
	// interior tessellation vertices that have no B-rep Vertex, and rebuilt
	// after every placement. It is read through the one witness-maximum reader
	// every gate diameter is published through (pointSetDiameterWithBudget,
	// verify.go), so it sits at or below the held boundary's own diameter.
	diameter float64

	xform r3.Transform
}

// transform is the accumulated rigid placement.
func (fp facetedPayload) transform() r3.Transform { return fp.xform }

func facetedTranslationOnly(t r3.Transform) bool {
	return t.IsValid() && proofbound.FiniteVec(t.Translation()) && t.Basis() == r3.Identity().Basis()
}

// placed re-evaluates the held mesh under the composed motion: the vertices
// move through the delta motion (float rounding is folded into the proven
// bounds — the geometry is never silently trusted), the moved mesh is kept
// embedded (facetproof.PlaceMesh), a reflection flips the windings, and the
// topology and measurements rebuild from the moved mesh.
func (fp facetedPayload) placed(ctx context.Context, d *Document, ref producerID, composed r3.Transform) (*Body, error) {
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	inv, err := fp.xform.Inverse()
	if err != nil {
		return nil, fmt.Errorf(`decad: inverting the accumulated placement failed: %w`, err)
	}
	delta, err := inv.Then(composed)
	if err != nil {
		return nil, fmt.Errorf(`decad: composing the placement failed: %w`, err)
	}
	next := fp
	next.xform = composed
	next.lowerSupport = nil
	if !facetedTranslationOnly(delta) || !facetedTranslationOnly(composed) {
		next.exactSourceVerts, next.exactSourceTris = nil, nil
	} else if len(fp.exactSourceVerts) == 0 && fp.meshBound == 0 && fp.volSymDiff == 0 &&
		fp.xform == r3.Identity() {
		next.exactSourceVerts = append([]r3.Vec(nil), fp.verts...)
		next.exactSourceTris = append([][3]int(nil), fp.tris...)
	}
	placed, err := facetproof.PlaceMesh(ctx, budget, fp.verts, fp.tris, fp.vertexBound, fp.volSymDiff, delta)
	if err != nil {
		return nil, err
	}
	next.verts, next.tris = placed.Verts, placed.Tris
	next.vertexBound, next.meshBound, next.volSymDiff = placed.VertexBound, placed.MeshBound, placed.VolSymDiff
	return buildFacetedBody(ctx, d, ref, next)
}

// meshAreaUpper keeps root callers on the shared facet area proof.
func meshAreaUpper(verts []r3.Vec, tris [][3]int) float64 {
	return facetproof.MeshAreaUpper(verts, tris)
}

// facetedFacePerimeterUpper includes each edge's length error and rounds each
// addition outward before the perimeter enters an area bound.
func facetedFacePerimeterUpper(f *Face, budget *proofbound.WorkBudget) (float64, error) {
	perimeter := 0.0
	for _, l := range f.loops {
		for _, ce := range l.coedges {
			if err := budget.Step(); err != nil {
				return 0, err
			}
			perimeter = proofbound.AbsSumUpper(perimeter, ce.edge.length, ce.edge.lengthBound)
		}
	}
	return perimeter, nil
}

// facetedAreaGeom keeps root callers on the shared area allowance.
func facetedAreaGeom(delta, perimeterUpper, facetAllow float64) float64 {
	return facetproof.FacetedAreaGeom(delta, perimeterUpper, facetAllow)
}

// facetedExtremeError reads the shared per-coordinate bound.
func facetedExtremeError(budget *proofbound.WorkBudget, verts []r3.Vec, beta, reach []float64, lo, hi r3.Vec) (float64, error) {
	return facetproof.FacetedExtremeError(budget, verts, beta, reach, lo, hi)
}

// facetedMeshAudit is the shared geometry-only shell certificate.
type facetedMeshAudit = facetproof.MeshAudit

func auditFacetedMesh(ctx context.Context, verts []r3.Vec, tris [][3]int) (*facetedMeshAudit, error) {
	return facetproof.AuditFacetedMesh(ctx, verts, tris)
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
	audit *facetedMeshAudit, volume Measurement, volRat *big.Rat) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	verts, tris := pp.verts, pp.tris
	diameter, ok, err := pointSetDiameterContext(ctx, verts)
	if err != nil {
		return nil, err
	}
	if !ok {
		return nil, fmt.Errorf(`%w: the held boundary has no usable diameter`, ErrBooleanFailed)
	}
	pp.diameter = diameter
	if audit == nil {
		audit, err = auditFacetedMesh(ctx, verts, tris)
		if err != nil {
			return nil, err
		}
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	xverts, comp, adj := audit.XVerts, audit.Comp, audit.Adj
	members, compVol, contains := audit.Members, audit.CompVol, audit.Contains

	body := &Body{doc: d, origin: FeatureRef{producer: ref, Role: roleBody}, solid: true}
	if len(pp.vertexBound) != len(verts) {
		return nil, fmt.Errorf(`%w: a faceted payload carries %d vertex bounds for %d vertices`, ErrBooleanFailed, len(pp.vertexBound), len(verts))
	}
	// facetDelta is each held facet's δ(t), the largest of its corners' β
	// (docs/faceted-vertex-bounds-design.md §2).
	facetDelta := facetproof.FacetDeltas(tris, pp.vertexBound)

	for _, s := range pp.src {
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
	patch, err := facetproof.SourcePatches(budget, len(tris), pp.src, adj)
	if err != nil {
		return nil, err
	}

	faceIdx := map[int]*Face{}
	facePlanar := map[*Face]bool{}
	facetFace := make([]*Face, len(tris))
	compFaces := make([][]*Face, len(members))
	for i, t := range tris {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		f, ok := faceIdx[patch[i]]
		if !ok {
			// Every patch of one source face carries that face's OWN origins:
			// both strips of a split cap came from the cap, and both must
			// still say so, or FaceCreatedBy and the surveys lose them.
			f = &Face{
				origins:    append([]FeatureRef(nil), pp.groups[pp.src[i]].origins...),
				body:       body,
				heldPlanar: pp.groups[pp.src[i]].planar,
			}
			faceIdx[patch[i]] = f
			facePlanar[f] = pp.groups[pp.src[i]].planar
			compFaces[comp[i]] = append(compFaces[comp[i]], f)
		}
		a, b, c := verts[t[0]], verts[t[1]], verts[t[2]]
		f.area += b.Sub(a).Cross(c.Sub(a)).Len() / 2
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
		f.surface = Faceted{Bound: units.Millimeters(delta)}
	}

	if err := buildFacetedTopology(ctx, verts, tris, facetFace, facePlanar, pp.vertexBound); err != nil {
		return nil, err
	}

	// Per-face area bounds: the smaller of two proven geometric terms — the
	// §4 shape, the face's own displacement times its own bounding edges, and
	// docs/faceted-vertex-bounds-design.md §4.2's sum of each facet's
	// perturbation at its own δ(t) — plus the operands' chord-length slack and
	// the PROVEN slop of the float sum that produced the area. The perimeter
	// the first multiplies is the UPPER one: an edge's held chord length is
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
			geom := facetedAreaGeom(faceDelta[f], loopLen, faceFacetArea[f])
			bodyGeom = proofbound.AbsSumUpper(bodyGeom, geom)
			f.areaBound = proofbound.AbsSumUpper(geom, pp.areaSlack, proofbound.SumSlop(facetsOf[f], f.area))
		}
	}

	// Shells and lumps: positive components are outer shells (one lump
	// each); a negative component is a void of its innermost positive
	// container.
	shells := make([]*Shell, len(members))
	lumpOf := map[int]*Lump{}
	var lumps []*Lump
	for ci := range members {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		shells[ci] = &Shell{faces: compFaces[ci], void: compVol[ci].Sign() < 0}
		if compVol[ci].Sign() > 0 {
			l := &Lump{shells: []*Shell{shells[ci]}}
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
			lumpOf[parent].shells = append(lumpOf[parent].shells, shells[ci])
		}
	}
	body.lumps = lumps

	// Measurements: exact rational integrals over the held mesh, reported
	// with the composed proven bounds (§9: the bound shapes of the
	// verification design, composed from the operands' chord errors). The
	// volume helper is also the read-only interference evaluator's one source
	// of volume truth.
	if volRat == nil {
		volume, volRat, err = meshVolumeMeasurement(ctx, xverts, tris, pp.volSymDiff)
		if err != nil {
			return nil, err
		}
	}
	body.volume = volume
	moments, err := facetproof.FirstMoments(budget, xverts, tris)
	if err != nil {
		return nil, err
	}

	areaF := 0.0
	for _, t := range tris {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		a, b, c := verts[t[0]], verts[t[1]], verts[t[2]]
		areaF += b.Sub(a).Cross(c.Sub(a)).Len() / 2
	}
	// The float area accumulation itself rounds, and the loop above sums
	// NAIVELY: charge the proven naive-summation bound, never a pairwise one
	// (internal/proofbound/bounds.go, proofbound.SumSlop). It is ulp-scale in the total, so a genuinely
	// tiny-bound planar boolean stays tiny — and never zero, which would claim
	// an exactness a float sum of square roots does not have.
	areaBound := proofbound.AbsSumUpper(bodyGeom, pp.areaSlack, proofbound.SumSlop(len(tris), areaF))
	body.area = Measurement{
		Value:     units.SquareMillimeters(areaF),
		Exactness: exactnessOf(areaBound),
		Bound:     units.SquareMillimeters(areaBound),
	}

	// Centroid: the exact first moment over the exact volume; the bound is
	// the symmetric-difference mass displaced to the pair's own diameter,
	// with the diameter itself as the honest ceiling.
	tf := big.NewRat(1, 24)
	cx := centroidCoord(moments[0], tf, volRat)
	cy := centroidCoord(moments[1], tf, volRat)
	cz := centroidCoord(moments[2], tf, volRat)
	cenBound := facetedCentroidAllowance(pp.volSymDiff, pp.dPair, volFloor(volRat, pp.volSymDiff))
	cxF, _ := cx.Float64()
	cyF, _ := cy.Float64()
	czF, _ := cz.Float64()
	// The three coordinates round independently, and the VecMeasurement bound
	// is a 3D radius (internal/proofbound/bounds.go, proofbound.Radius3D).
	cenRound := proofbound.Radius3D(math.Max(proofbound.RatAbsDiff(cx, cxF), math.Max(proofbound.RatAbsDiff(cy, cyF), proofbound.RatAbsDiff(cz, czF))))
	centroidBound := proofbound.AbsSumUpper(cenBound, cenRound)
	body.centroid = VecMeasurement{
		Value:     r3.Vec{X: cxF, Y: cyF, Z: czF},
		Exactness: exactnessOf(centroidBound),
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
	// off by at most facetedExtremeError's per-coordinate figure, which reads
	// each vertex's own facet bound; the corner can be off on every axis at
	// once, so the radius is √3 of the largest (internal/proofbound/bounds.go,
	// proofbound.Radius3D).
	vertexFacetDelta := facetproof.VertexFacetDeltas(len(verts), tris, facetDelta)
	extremeErr, err := facetedExtremeError(budget, verts, pp.vertexBound, vertexFacetDelta, lo, hi)
	if err != nil {
		return nil, err
	}
	boxBound := proofbound.Radius3D(extremeErr)
	body.bounds = Box{
		Min: lo, Max: hi,
		Exactness: exactnessOf(boxBound),
		Bound:     units.Millimeters(boxBound),
	}

	// The facet → face mapping, in the body's Faces() order, for Tessellate.
	if err := budget.Err(); err != nil {
		return nil, err
	}
	faceOf, err := facetFaceIndices(ctx, body.Faces(), facetFace)
	if err != nil {
		return nil, err
	}
	pp.faceOf = faceOf
	body.payload = pp
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return body, nil
}

// facetFaceIndices maps each facet to its face's index in the built body's
// Faces() order — the lookup Tessellate reads. Every facetFace entry is a face
// this build attached to the body's own topology, so on a consistent build the
// lookup cannot miss. A miss is an invariant failure (docs/interference-design.md
// §7.1), returned as ErrBooleanFailed rather than silently attributing the facet
// to face 0.
func facetFaceIndices(ctx context.Context, faces, facetFace []*Face) ([]int, error) {
	return facetproof.FaceIndices(ctx, faces, facetFace)
}

// meshVolumeMeasurement integrates one stitched, oriented, closed mesh in
// exact rational arithmetic and composes the shared symmetric-difference
// allowance with the final rational-to-float rounding. It reuses the audit's
// exact vertices while retaining the original facet-order sum.
func meshVolumeMeasurement(ctx context.Context, xverts []proof.Xpt, tris [][3]int, volSymDiff float64) (Measurement, *big.Rat, error) {
	volF, bound, volRat, err := facetproof.Volume(ctx, xverts, tris, volSymDiff)
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

// facetedCentroidAllowance keeps a positive occupied-volume displacement
// positive even when its product with the pair diameter underflows.
func facetedCentroidAllowance(sym, diameter, volumeFloor float64) float64 {
	if volumeFloor <= 0 {
		return diameter
	}
	return math.Min(diameter, proofbound.DivUpper(proofbound.ProductUpper(sym, diameter), volumeFloor))
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
		v := &Vertex{position: verts[vi], bound: units.Millimeters(vertexBound[vi])}
		vertexObj[vi] = v
		return v
	}
	edgeObj := map[*facetedtopology.Edge]*Edge{}
	for _, chain := range plan.Edges {
		e := &Edge{
			curve:           FacetedCurve{Bound: units.Millimeters(chain.Bound)},
			start:           vertexOf(chain.Vertices[0]),
			end:             vertexOf(chain.Vertices[len(chain.Vertices)-1]),
			faces:           []*Face{faces[chain.Faces[0]], faces[chain.Faces[1]]},
			convex:          chain.Convex,
			length:          chain.Length,
			lengthBound:     chain.LengthBound,
			lengthUnbounded: chain.LengthUnbounded,
		}
		edgeObj[chain] = e
	}
	for fi, loops := range plan.FaceLoops {
		f := faces[fi]
		for _, planned := range loops {
			loop := &Loop{outer: planned.Outer}
			for _, ce := range planned.Coedges {
				loop.coedges = append(loop.coedges, coedge{edge: edgeObj[ce.Edge], forward: ce.Forward})
			}
			f.loops = append(f.loops, loop)
		}
	}
	return nil
}

// refuseRimPastPair refuses a boolean whose largest rim bound has stopped
// bounding anything. A rim vertex is not a point of either operand's surface:
// it is the exact crossing of an operand A facet's PLANE with an operand B
// facet's, and the true intersection curve lies anywhere within δ(t_A) of the
// one and δ(t_B) of the other — a tube of half-width (δ(t_A) + δ(t_B))/sin θ
// about the crossing line (meshbool.RimBound), which grows without limit as
// the two facets approach tangency. maxRim is the largest such bound any rim
// vertex of the operation takes. Once it is not finite, or reaches the pair's
// own diameter, the operation is refused (ErrUnsupported) rather than
// reported with a bound nobody can use — decad never understates a bound, and
// never fakes one (docs/faceted-vertex-bounds-design.md §3.2).
func refuseRimPastPair(maxRim, dPair float64) error {
	if proofbound.IsNonFinite(maxRim) {
		return fmt.Errorf(`%w: the operands' facets meet at an angle this evaluator cannot bound`, ErrUnsupported)
	}
	if maxRim > 0 && maxRim >= dPair {
		return fmt.Errorf(`%w: the operands cross too shallowly — the rim's proven displacement bound reaches the pair's own diameter, so no measurement of the result would be trustworthy`, ErrUnsupported)
	}
	return nil
}
