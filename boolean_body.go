package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/meshbool"

	"github.com/lestrrat-3d/decad/internal/survey2d"

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
// embedded (facetproof.KeepPlacedEmbedded), a reflection flips the windings, and the
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
	next.verts = make([]r3.Vec, len(fp.verts))
	// The rounding a rigid motion commits is committed at the magnitude of the
	// INPUT coordinate and of the translation — inside the products and sums —
	// never at the magnitude of the result: a body built far from the origin
	// and moved back rounds at the far magnitude. Charge it there (internal/proofbound/bounds.go).
	// Each vertex's own rounding reads that vertex's inputs alone, so each is
	// charged at its own magnitude (docs/faceted-vertex-bounds-design.md §4.3).
	tr := delta.Translation()
	maxTrans := math.Max(math.Abs(tr.X), math.Max(math.Abs(tr.Y), math.Abs(tr.Z)))
	vertexAllow := make([]float64, len(fp.verts))
	for i, v := range fp.verts {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		vertexAllow[i] = proofbound.RigidRoundAllow(math.Max(math.Abs(v.X), math.Max(math.Abs(v.Y), math.Abs(v.Z))), maxTrans)
		next.verts[i] = delta.Apply(v)
	}
	if err := budget.Err(); err != nil {
		return nil, err
	}
	if delta.IsReflection() {
		next.tris = make([][3]int, len(fp.tris))
		for i, t := range fp.tris {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			next.tris[i] = [3]int{t[0], t[2], t[1]}
		}
	}
	moved, err := facetproof.KeepPlacedEmbedded(ctx, fp.verts, next.verts, next.tris, delta)
	if err != nil {
		return nil, err
	}
	// A vertex the embedding check moved is charged the larger of its own
	// rounding allowance and its distance from its exact image; allow, the
	// largest of them, is what the volume's swept allowance charges.
	allow := 0.0
	next.vertexBound = make([]float64, len(fp.vertexBound))
	for i, beta := range fp.vertexBound {
		a := vertexAllow[i]
		if moved != nil {
			a = math.Max(a, moved[i])
		}
		allow = math.Max(allow, a)
		next.vertexBound[i] = proofbound.AbsSumUpper(beta, a)
	}
	next.meshBound = facetBoundMax(next.tris, next.vertexBound)
	areaUpper, err := proofbound.PerturbedAreaUpperContext(ctx, next.verts, next.tris, allow)
	if err != nil {
		return nil, err
	}
	next.volSymDiff = proofbound.AbsSumUpper(next.volSymDiff, proofbound.SweptVolumeAllow(allow, areaUpper))
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
	facetDelta := make([]float64, len(tris))
	for i, t := range tris {
		facetDelta[i] = max(pp.vertexBound[t[0]], pp.vertexBound[t[1]], pp.vertexBound[t[2]])
	}

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
	patch := make([]int, len(tris))
	for i := range patch {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		patch[i] = -1
	}
	nPatch := 0
	for i := range tris {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		if patch[i] != -1 {
			continue
		}
		id := nPatch
		nPatch++
		patch[i] = id
		queue := []int{i}
		for len(queue) > 0 {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			f := queue[0]
			queue = queue[1:]
			for _, nb := range adj[f] {
				if patch[nb] != -1 || pp.src[nb] != pp.src[f] {
					continue
				}
				patch[nb] = id
				queue = append(queue, nb)
			}
		}
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
	faceDelta := map[*Face]float64{}
	faceFacetArea := map[*Face]float64{}
	for i, f := range facetFace {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		faceDelta[f] = max(faceDelta[f], facetDelta[i])
		t := tris[i]
		faceFacetArea[f] = proofbound.AbsSumUpper(faceFacetArea[f],
			proofbound.PerturbedTriangleAreaAllow(verts[t[0]], verts[t[1]], verts[t[2]], facetDelta[i]))
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
	for ci := range members {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		if compVol[ci].Sign() > 0 {
			continue
		}
		parent := -1
		for outer := range members {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			if outer == ci || compVol[outer].Sign() <= 0 || !contains[outer][ci] {
				continue
			}
			if parent == -1 {
				parent = outer
				continue
			}
			// The innermost container is contained by every other candidate.
			if contains[parent][outer] {
				parent = outer
			}
		}
		if parent == -1 {
			return nil, fmt.Errorf(`%w: a void shell has no containing shell`, ErrBooleanFailed)
		}
		lumpOf[parent].shells = append(lumpOf[parent].shells, shells[ci])
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
	var mx, my, mz = new(big.Rat), new(big.Rat), new(big.Rat)
	for _, t := range tris {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		a, b, c := xverts[t[0]], xverts[t[1]], xverts[t[2]]
		det := proofbound.XdotRat(a, meshbool.Xcross(b, c))
		ax, ay, az := meshbool.XhpRat(proofbound.Xhp(a))
		bx, by, bz := meshbool.XhpRat(proofbound.Xhp(b))
		cx, cy, cz := meshbool.XhpRat(proofbound.Xhp(c))
		mx.Add(mx, new(big.Rat).Mul(det, new(big.Rat).Add(new(big.Rat).Add(ax, bx), cx)))
		my.Add(my, new(big.Rat).Mul(det, new(big.Rat).Add(new(big.Rat).Add(ay, by), cy)))
		mz.Add(mz, new(big.Rat).Mul(det, new(big.Rat).Add(new(big.Rat).Add(az, bz), cz)))
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
	cx := centroidCoord(mx, tf, volRat)
	cy := centroidCoord(my, tf, volRat)
	cz := centroidCoord(mz, tf, volRat)
	cenBound := facetedCentroidAllowance(pp.volSymDiff, pp.dPair, volFloor(volRat, pp.volSymDiff))
	cxF, _ := cx.Float64()
	cyF, _ := cy.Float64()
	czF, _ := cz.Float64()
	// The three coordinates round independently, and the VecMeasurement bound
	// is a 3D radius (internal/proofbound/bounds.go, proofbound.Radius3D).
	cenRound := proofbound.Radius3D(math.Max(survey2d.RatAbsDiff(cx, cxF), math.Max(survey2d.RatAbsDiff(cy, cyF), survey2d.RatAbsDiff(cz, czF))))
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
	vertexFacetDelta := make([]float64, len(verts))
	for i, t := range tris {
		for _, v := range t {
			vertexFacetDelta[v] = max(vertexFacetDelta[v], facetDelta[i])
		}
	}
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
	budget := proofbound.NewWorkBudget(ctx)
	flat := make(map[*Face]int, len(faces))
	for i, f := range faces {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		flat[f] = i
	}
	out := make([]int, len(facetFace))
	for i, f := range facetFace {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		idx, ok := flat[f]
		if !ok {
			return nil, fmt.Errorf(`%w: a facet maps to a face absent from the built body`, ErrBooleanFailed)
		}
		out[i] = idx
	}
	return out, budget.Err()
}

// meshVolumeMeasurement integrates one stitched, oriented, closed mesh in
// exact rational arithmetic and composes the shared symmetric-difference
// allowance with the final rational-to-float rounding. It reuses the audit's
// exact vertices while retaining the original facet-order sum.
func meshVolumeMeasurement(ctx context.Context, xverts []proofbound.Xpt, tris [][3]int, volSymDiff float64) (Measurement, *big.Rat, error) {
	total := new(big.Rat)
	for i, t := range tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return Measurement{}, nil, err
			}
		}
		a, b, c := xverts[t[0]], xverts[t[1]], xverts[t[2]]
		total.Add(total, proofbound.XdotRat(a, meshbool.Xcross(b, c)))
	}
	volRat := new(big.Rat).Mul(total, big.NewRat(1, 6))
	if volRat.Sign() <= 0 {
		return Measurement{}, nil, fmt.Errorf(`%w: the boolean result encloses no volume`, ErrBooleanFailed)
	}
	volF, _ := volRat.Float64()
	bound := proofbound.AbsSumUpper(volSymDiff, survey2d.RatAbsDiff(volRat, volF))
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
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return err
	}
	if len(vertexBound) != len(verts) {
		return fmt.Errorf(`%w: a faceted body carries %d vertex bounds for %d vertices`, ErrBooleanFailed, len(vertexBound), len(verts))
	}

	// Directed halfedge → facet, and the boundary predicate: the twin facet
	// belongs to a different face.
	halfOwner := map[[2]int]int{}
	for i, t := range tris {
		for k := range 3 {
			if err := budget.Step(); err != nil {
				return err
			}
			halfOwner[[2]int{t[k], t[(k+1)%3]}] = i
		}
	}
	isBoundary := func(u, v int) bool {
		mine, ok := halfOwner[[2]int{u, v}]
		if !ok {
			return false
		}
		twin, ok := halfOwner[[2]int{v, u}]
		if !ok {
			return false
		}
		return facetFace[mine] != facetFace[twin]
	}

	// Undirected boundary edges with their face pair and exact hinge sign.
	type hingeInfo struct {
		fa, fb *Face
		sign   int
	}
	hinges := map[[2]int]hingeInfo{}
	var boundaryEdges [][2]int
	for i, t := range tris {
		for k := range 3 {
			if err := budget.Step(); err != nil {
				return err
			}
			u, v := t[k], t[(k+1)%3]
			if u > v || !isBoundary(u, v) {
				continue
			}
			mine := i
			twin := halfOwner[[2]int{v, u}]
			// The hinge: the twin's opposite vertex against this facet's
			// plane — below (negative) is material bending away: convex.
			tw := tris[twin]
			opp := tw[0] + tw[1] + tw[2] - u - v
			s := meshbool.OrientSign(verts[t[0]], verts[t[1]], verts[t[2]], verts[opp])
			hinges[[2]int{u, v}] = hingeInfo{fa: facetFace[mine], fb: facetFace[twin], sign: s}
			boundaryEdges = append(boundaryEdges, [2]int{u, v})
		}
	}
	if len(boundaryEdges) == 0 {
		return fmt.Errorf(`%w: a faceted body carries no face boundaries`, ErrBooleanFailed)
	}

	// Chain the boundary edges: a vertex continues a chain when exactly two
	// boundary edges meet there with the same face pair and hinge sign.
	incident := map[int][][2]int{}
	for _, e := range boundaryEdges {
		if err := budget.Step(); err != nil {
			return err
		}
		incident[e[0]] = append(incident[e[0]], e)
		incident[e[1]] = append(incident[e[1]], e)
	}
	samePair := func(a, b hingeInfo) bool {
		return a.sign == b.sign &&
			(a.fa == b.fa && a.fb == b.fb || a.fa == b.fb && a.fb == b.fa)
	}
	// A chain breaks at any vertex that does not join exactly two boundary
	// edges of the same face pair and hinge sign.
	breakAt := func(v int) bool {
		list := incident[v]
		if len(list) != 2 {
			return true
		}
		ka := [2]int{min(list[0][0], list[0][1]), max(list[0][0], list[0][1])}
		kb := [2]int{min(list[1][0], list[1][1]), max(list[1][0], list[1][1])}
		return !samePair(hinges[ka], hinges[kb])
	}
	ukey := func(e [2]int) [2]int { return [2]int{min(e[0], e[1]), max(e[0], e[1])} }

	type chainRec struct {
		verts []int
		edge  *Edge
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
	chainOf := map[[2]int]int{} // undirected mesh edge → chain index
	posInChain := map[[2]int]int{}
	var chains []chainRec
	usedEdge := map[[2]int]struct{}{}
	otherAt := func(v int, notKey [2]int) [2]int {
		for _, e := range incident[v] {
			if ukey(e) != notKey {
				return e
			}
		}
		return notKey
	}
	commitChain := func(path []int) error {
		info := hinges[ukey([2]int{path[0], path[1]})]
		length := 0.0
		nSegs := 0
		bound := vertexBound[path[0]]
		for k := 0; k+1 < len(path); k++ {
			if err := budget.Step(); err != nil {
				return err
			}
			key := ukey([2]int{path[k], path[k+1]})
			chainOf[key] = len(chains)
			posInChain[key] = k
			length += verts[path[k+1]].Sub(verts[path[k]]).Len()
			bound = max(bound, vertexBound[path[k+1]])
			nSegs++
		}
		// The chain holds nSegs chords, and BOTH endpoints of each move: the
		// error accumulates over N, it is not δ (internal/proofbound/bounds.go, proofbound.ChainLengthBound).
		// It is never zero either — the held length is a float sum of square
		// roots, and the last ulp is not free — so an all-planar rim reports
		// Approximate rather than an Exact it cannot back.
		lengthBound := proofbound.ChainLengthBound(nSegs, bound, length)
		e := &Edge{
			curve:       FacetedCurve{Bound: units.Millimeters(bound)},
			start:       vertexOf(path[0]),
			end:         vertexOf(path[len(path)-1]),
			faces:       []*Face{info.fa, info.fb},
			convex:      info.sign < 0,
			length:      length,
			lengthBound: lengthBound,
			// A rim between two PLANAR sources is a straight line, so its
			// chord length is honest; any curved source leaves the true
			// curve's length excess over the chords unboundable without
			// curvature knowledge, and Length must refuse rather than
			// understate (never a silent pass).
			lengthUnbounded: !facePlanar[info.fa] || !facePlanar[info.fb],
		}
		chains = append(chains, chainRec{verts: path, edge: e})
		return nil
	}
	walkFrom := func(v int, e [2]int) ([]int, error) {
		path := []int{v}
		cur, curEdge := v, e
		for {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			usedEdge[ukey(curEdge)] = struct{}{}
			nxt := curEdge[0] + curEdge[1] - cur
			path = append(path, nxt)
			if nxt == path[0] || breakAt(nxt) {
				return path, nil
			}
			nextEdge := otherAt(nxt, ukey(curEdge))
			if ukey(nextEdge) == ukey(curEdge) {
				return path, nil
			}
			if _, seen := usedEdge[ukey(nextEdge)]; seen {
				return path, nil
			}
			cur, curEdge = nxt, nextEdge
		}
	}
	// Open chains first: start at every break vertex, in edge order.
	for _, e := range boundaryEdges {
		for _, v := range []int{e[0], e[1]} {
			if err := budget.Step(); err != nil {
				return err
			}
			if !breakAt(v) {
				continue
			}
			if _, ok := usedEdge[ukey(e)]; ok {
				break
			}
			path, err := walkFrom(v, e)
			if err != nil {
				return err
			}
			if err := commitChain(path); err != nil {
				return err
			}
			break
		}
	}
	// The rest are closed uniform cycles: anchor each at its smallest
	// vertex, deterministically.
	for _, e := range boundaryEdges {
		if err := budget.Step(); err != nil {
			return err
		}
		if _, ok := usedEdge[ukey(e)]; ok {
			continue
		}
		cycle, err := walkFrom(e[0], e)
		if err != nil {
			return err
		}
		if cycle[0] != cycle[len(cycle)-1] {
			return fmt.Errorf(`%w: a face boundary chain did not close`, ErrBooleanFailed)
		}
		ring := cycle[:len(cycle)-1]
		anchor := 0
		for i, v := range ring {
			if v < ring[anchor] {
				anchor = i
			}
		}
		rotated := make([]int, 0, len(cycle))
		for i := range ring {
			if err := budget.Step(); err != nil {
				return err
			}
			rotated = append(rotated, ring[(anchor+i)%len(ring)])
		}
		rotated = append(rotated, ring[anchor])
		// Re-commit with the rotated ordering (walkFrom already marked the
		// edges used).
		if err := commitChain(rotated); err != nil {
			return err
		}
	}

	// The area-weighted normal of each face's own facets. A face is ONE patch,
	// and a patch of a PLANAR source is coplanar, so this is that plane's
	// outward normal (scaled by twice the patch's area).
	faceNormal := map[*Face]r3.Vec{}
	for i, t := range tris {
		if err := budget.Step(); err != nil {
			return err
		}
		a, b, c := verts[t[0]], verts[t[1]], verts[t[2]]
		f := facetFace[i]
		faceNormal[f] = faceNormal[f].Add(b.Sub(a).Cross(c.Sub(a)))
	}

	// Face loops: walk each face's directed boundary cycles through the
	// facet fans, then group consecutive halfedges by chain into coedges.
	// loopMoment is each loop's closed-polygon area vector Σ vᵢ × vᵢ₊₁ (twice
	// the signed area), which is what decides outer from hole on a planar face.
	loopMoment := map[*Loop]r3.Vec{}
	visited := map[[2]int]struct{}{}
	nextBoundary := func(h [2]int) ([2]int, error) {
		cur := h
		for range len(tris)*3 + 3 {
			if err := budget.Step(); err != nil {
				return [2]int{}, err
			}
			f := halfOwner[cur]
			t := tris[f]
			var follow [2]int
			switch {
			case t[0] == cur[0] && t[1] == cur[1]:
				follow = [2]int{t[1], t[2]}
			case t[1] == cur[0] && t[2] == cur[1]:
				follow = [2]int{t[2], t[0]}
			case t[2] == cur[0] && t[0] == cur[1]:
				follow = [2]int{t[0], t[1]}
			default:
				return [2]int{}, fmt.Errorf(`%w: a halfedge lost its facet`, ErrBooleanFailed)
			}
			if isBoundary(follow[0], follow[1]) {
				return follow, nil
			}
			cur = [2]int{follow[1], follow[0]}
		}
		return [2]int{}, fmt.Errorf(`%w: a face boundary walk did not close`, ErrBooleanFailed)
	}
	for i, t := range tris {
		for k := range 3 {
			if err := budget.Step(); err != nil {
				return err
			}
			h := [2]int{t[k], t[(k+1)%3]}
			if !isBoundary(h[0], h[1]) {
				continue
			}
			if _, ok := visited[h]; ok {
				continue
			}
			face := facetFace[i]
			var cycle [][2]int
			cur := h
			// A boundary walk must consume each boundary halfedge at most once.
			// Keep the walk bounded even if malformed adjacency causes nextBoundary
			// to cycle without returning to its start.
			for steps := 0; ; steps++ {
				if err := budget.Step(); err != nil {
					return err
				}
				if steps >= len(tris)*3+1 {
					return fmt.Errorf(`%w: a face boundary walk exceeded its halfedges`, ErrBooleanFailed)
				}
				visited[cur] = struct{}{}
				cycle = append(cycle, cur)
				nxt, err := nextBoundary(cur)
				if err != nil {
					return err
				}
				if nxt == h {
					break
				}
				cur = nxt
			}
			loop := &Loop{}
			firstChain, lastChain := -1, -1
			for _, he := range cycle {
				if err := budget.Step(); err != nil {
					return err
				}
				key := [2]int{min(he[0], he[1]), max(he[0], he[1])}
				ci, ok := chainOf[key]
				if !ok {
					return fmt.Errorf(`%w: a boundary halfedge belongs to no chain`, ErrBooleanFailed)
				}
				if ci == lastChain {
					continue
				}
				lastChain = ci
				if firstChain == -1 {
					firstChain = ci
				}
				forward := chains[ci].verts[posInChain[key]] == he[0]
				loop.coedges = append(loop.coedges, coedge{edge: chains[ci].edge, forward: forward})
			}
			// The walk starts at whichever halfedge the facet scan reached
			// first, which can sit in the MIDDLE of a chain — and then that one
			// chain is met twice, once at each end of the cycle, and would be
			// listed as two coedges of one edge. The cycle is closed, so the
			// tail is the head's own chain resumed: drop it. (A chain that is
			// the loop's ONLY one already collapsed to a single coedge above.)
			if n := len(loop.coedges); n > 1 && lastChain == firstChain {
				loop.coedges = loop.coedges[:n-1]
			}
			mom := r3.Vec{}
			for _, he := range cycle {
				if err := budget.Step(); err != nil {
					return err
				}
				mom = mom.Add(verts[he[0]].Cross(verts[he[1]]))
			}
			loopMoment[loop] = mom
			face.loops = append(face.loops, loop)
		}
	}

	// Pick each face's outer loop. A face is ONE connected patch, so exactly one
	// of its loops bounds it from outside and the rest are holes in it.
	//
	// On a PLANAR patch that is decided, not guessed: the boundary is walked
	// with the material on its left, so about the patch's own outward normal
	// the outer loop turns positive and every hole turns negative — and the
	// outer loop's area vector is the patch's area PLUS its holes', so it is
	// the largest. A longest-perimeter pick would not do: a long serpentine
	// slot can out-measure the boundary it is cut into, and crowning it outer
	// would report the face's true outer boundary as a hole in its own slot.
	//
	// A CURVED patch has no such plane, and no loop of it is a hole in another
	// (a hole wall's two rims bound a tube). There the longest boundary stands
	// as the deterministic bookkeeping choice: validity never reads it, and
	// Faceted faces expose their polygons through Tessellate, not through loop
	// nesting.
	for f := range collectFaces(facetFace) {
		if err := budget.Step(); err != nil {
			return err
		}
		if len(f.loops) == 0 {
			return fmt.Errorf(`%w: a faceted face has no boundary loop`, ErrBooleanFailed)
		}
		li := -1
		if facePlanar[f] {
			n := faceNormal[f]
			best := 0.0
			for idx, l := range f.loops {
				if err := budget.Step(); err != nil {
					return err
				}
				if s := loopMoment[l].Dot(n); s > best {
					best, li = s, idx
				}
			}
		}
		if li == -1 {
			longest := -1.0
			li = 0
			for idx, l := range f.loops {
				if err := budget.Step(); err != nil {
					return err
				}
				total := 0.0
				for _, ce := range l.coedges {
					if err := budget.Step(); err != nil {
						return err
					}
					total += ce.edge.length
				}
				if total > longest {
					longest, li = total, idx
				}
			}
		}
		f.loops[0], f.loops[li] = f.loops[li], f.loops[0]
		f.loops[0].outer = true
	}
	return budget.Err()
}

func collectFaces(facetFace []*Face) map[*Face]struct{} {
	out := map[*Face]struct{}{}
	for _, f := range facetFace {
		out[f] = struct{}{}
	}
	return out
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
