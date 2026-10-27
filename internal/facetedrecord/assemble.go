package facetedrecord

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/diameter"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Mesh is the part of a faceted payload AssembleBody reads: the held mesh,
// its per-facet source grouping and the proven error terms the measurements
// compose from.
type Mesh struct {
	Verts []r3.Vec
	// VertexBound is β(v) per held vertex (agent-docs/faceted-vertex-bounds-design.md
	// §2, §4.1).
	VertexBound []float64
	Tris        [][3]int
	Src         []int // per-facet source-group id
	Groups      []Group
	// AxialSectionJoin marks a proven XY section swept through one axial
	// interval. Its held vertical boundary chains retain a Z direction for
	// selectors even though their curve type remains bounded FacetedCurve.
	AxialSectionJoin bool
	VolSymDiff       float64
	AreaSlack        float64
	DPair            float64
}

// AssembleBody assembles the Faceted body from the held mesh: exact
// component/void analysis, per-source-face topology, and measurements with
// the composed proven bounds. It reuses audit, volume and volRat only for the
// evaluator's unchanged result mesh; a nil audit or volRat is proved here. It
// returns the body, each facet's face index in the body's face order, and the
// held boundary's diameter; the caller stores the payload on the body.
func AssembleBody(ctx context.Context, env topo.Env, in Mesh,
	audit *facetproof.MeshAudit, volume measurement.Measurement, volRat *big.Rat) (*topo.Body, []int, float64, error) {
	return assembleBody(ctx, nil, env, in, audit, volume, volRat)
}

func assembleBody(ctx context.Context, shared *proofbound.WorkBudget, env topo.Env, in Mesh,
	audit *facetproof.MeshAudit, volume measurement.Measurement, volRat *big.Rat) (*topo.Body, []int, float64, error) {
	if err := ctx.Err(); err != nil {
		return nil, nil, 0, err
	}
	verts, tris := in.Verts, in.Tris
	var diameterReading float64
	var ok bool
	var err error
	if shared != nil {
		diameterReading, ok, err = diameter.PointsWithBudget(shared, verts)
	} else {
		diameterReading, ok, err = diameter.PointsContext(ctx, verts)
	}
	if err != nil {
		return nil, nil, 0, err
	}
	if !ok {
		return nil, nil, 0, fmt.Errorf(`%w: the held boundary has no usable diameter`, decaderr.ErrBooleanFailed)
	}
	if audit == nil {
		audit, err = facetproof.AuditFacetedMesh(ctx, verts, tris)
		if err != nil {
			return nil, nil, 0, err
		}
	}
	budget := shared
	if budget == nil {
		budget = proofbound.NewWorkBudget(ctx)
	}
	if err := budget.Err(); err != nil {
		return nil, nil, 0, err
	}
	comp, adj := audit.Comp, audit.Adj
	members, compVol, contains := audit.Members, audit.CompVol, audit.Contains

	body := env.Alloc.NewBody()
	body.Solid = true
	body.Origin = provenance.Ref{Producer: env.Ref, Role: provenance.RoleBody}
	body.Doc = env.Doc
	if len(in.VertexBound) != len(verts) {
		return nil, nil, 0, fmt.Errorf(`%w: a faceted payload carries %d vertex bounds for %d vertices`, decaderr.ErrBooleanFailed, len(in.VertexBound), len(verts))
	}
	// facetDelta is each held facet's δ(t), the largest of its corners' β
	// (agent-docs/faceted-vertex-bounds-design.md §2).
	facetDelta := facetproof.FacetDeltas(tris, in.VertexBound)

	for _, s := range in.Src {
		if err := budget.Step(); err != nil {
			return nil, nil, 0, err
		}
		if s < 0 || s >= len(in.Groups) {
			return nil, nil, 0, fmt.Errorf(`%w: a facet names no source group`, decaderr.ErrBooleanFailed)
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
	patch, err := facetproof.SourcePatches(budget, len(tris), in.Src, adj)
	if err != nil {
		return nil, nil, 0, err
	}
	groupPatch := make(map[int]int)
	for i, group := range in.Src {
		if first, exists := groupPatch[group]; !exists {
			groupPatch[group] = patch[i]
		} else if first != patch[i] {
			groupPatch[group] = -1
		}
	}

	faceIdx := map[int]*topo.Face{}
	facePlanar := map[*topo.Face]bool{}
	faceGroup := map[*topo.Face]int{}
	facetFace := make([]*topo.Face, len(tris))
	compFaces := make([][]*topo.Face, len(members))
	// faceTermSlop is each face's summed per-facet evaluation charge
	// (internal/proofbound/bounds.go, proofbound.FacetAreaTermSlop): a sliver
	// facet's cross product cancels, and its rounding tracks its edge products
	// rather than the area SumSlop's relative charge reads.
	faceTermSlop := map[*topo.Face]float64{}
	// facetArea and facetSlop hold each facet's area term and evaluation
	// charge for the body totals below, which sum the same per-facet values in
	// the same facet order.
	facetArea := make([]float64, len(tris))
	facetSlop := make([]float64, len(tris))
	for i, t := range tris {
		if err := budget.Step(); err != nil {
			return nil, nil, 0, err
		}
		f, ok := faceIdx[patch[i]]
		if !ok {
			// Every patch of one source face carries that face's OWN origins:
			// both strips of a split cap came from the cap, and both must
			// still say so, or FaceCreatedBy and the surveys lose them.
			f = topobuild.NewFace(env.Alloc, topo.Face{
				HeldPlanar:  in.Groups[in.Src[i]].Planar,
				Denoted:     in.Groups[in.Src[i]].Denoted,
				NormalBound: in.Groups[in.Src[i]].NormalBound,
				Reversed:    in.Groups[in.Src[i]].Reversed,
				Origins:     append([]provenance.Ref(nil), in.Groups[in.Src[i]].Origins...),
				Source:      in.Groups[in.Src[i]].Source,
				Body:        body,
			})
			group := in.Groups[in.Src[i]]
			if group.HasCentroid && groupPatch[in.Src[i]] == patch[i] {
				f.Centroid, f.HasCentroid = group.Centroid, true
			}
			faceIdx[patch[i]] = f
			facePlanar[f] = in.Groups[in.Src[i]].Planar
			faceGroup[f] = in.Src[i]
			compFaces[comp[i]] = append(compFaces[comp[i]], f)
		}
		a, b, c := verts[t[0]], verts[t[1]], verts[t[2]]
		facetArea[i] = b.Sub(a).Cross(c.Sub(a)).Len() / 2
		facetSlop[i] = proofbound.FacetAreaTermSlop(a, b, c)
		f.Area += facetArea[i]
		faceTermSlop[f] = proofbound.AbsSumUpper(faceTermSlop[f], facetSlop[i])
		facetFace[i] = f
	}
	// A face's bound is the largest δ(t) over its own facets, and its
	// per-facet area allowance sums each facet's perturbation at its own δ(t)
	// (§4.2).
	faceDelta, faceFacetArea, err := facetproof.FaceProofs(budget, verts, tris, facetFace, facetDelta)
	if err != nil {
		return nil, nil, 0, err
	}
	coneAxial := coneAxialMin(in.Groups, faceGroup, verts, tris, facetFace)
	for f, delta := range faceDelta {
		group := in.Groups[faceGroup[f]]
		if group.Retained {
			rho := 0.0
			if cone, isCone := group.Surface.(surfacegeom.Cone); isCone {
				rho = ConeRhoLower(cone, coneAxial[f], delta)
			}
			if term, ok := HeldNormalTerm(group.Surface, delta, rho); ok {
				f.Surface = group.Surface
				f.RetainedTag = true
				f.CapRecord, f.AxialDelta, f.HasAxialDelta = group.CapRecord, group.AxialDelta, group.HasAxialDelta
				if group.Denoted != nil {
					// The denoted proof replaces the tag's departure; the held
					// displacement adds on top of it (topobuild.NormalAt).
					f.NormalBound, f.HeldNormalBound = group.NormalBound, term
				} else {
					f.TagDeparture = group.NormalBound
					f.NormalBound = math.Nextafter(group.NormalBound+term, math.Inf(1))
					if term == 0 {
						f.NormalBound = group.NormalBound
					}
				}
				continue
			}
			f.NormalBound, f.Reversed, f.Denoted = 0, false, nil
			f.Surface = surfacegeom.Faceted{Bound: units.Millimeters(delta)}
			continue
		}
		if group.Surface != nil {
			f.Surface = group.Surface
		} else {
			f.Surface = surfacegeom.Faceted{Bound: units.Millimeters(delta)}
		}
	}

	if err := buildTopologyWithBudget(ctx, shared, env.Alloc, verts, tris, facetFace, facePlanar,
		in.VertexBound, in.AxialSectionJoin); err != nil {
		return nil, nil, 0, err
	}
	// A retained tag on a face with no loop would read as a closed analytic
	// surface to the boundary audit (A3).
	for f := range faceDelta {
		if f.RetainedTag && len(f.Loops) == 0 {
			f.Surface = surfacegeom.Faceted{Bound: units.Millimeters(faceDelta[f])}
			f.RetainedTag, f.TagDeparture, f.NormalBound, f.HeldNormalBound, f.Reversed = false, 0, 0, 0, false
			f.Denoted = nil
			f.CapRecord, f.AxialDelta, f.HasAxialDelta = nil, 0, false
		}
	}

	// Per-face area bounds: the smaller of two proven geometric terms — the
	// §4 shape, the face's own displacement times its own bounding edges, and
	// agent-docs/faceted-vertex-bounds-design.md §4.2's sum of each facet's
	// perturbation at its own δ(t) — plus the operands' chord-length slack, the
	// PROVEN slop of the float sum that produced the area, and each facet's own
	// evaluation charge. The perimeter the first multiplies is the UPPER one:
	// an edge's held chord length is
	// itself only known within its own bound (internal/proofbound/bounds.go,
	// proofbound.ChainLengthBound).
	facetsOf := map[*topo.Face]int{}
	for _, f := range facetFace {
		if err := budget.Step(); err != nil {
			return nil, nil, 0, err
		}
		facetsOf[f]++
	}
	// bodyGeom sums every face's own geometric term: a rim that moves perturbs
	// BOTH faces it separates, and each face charges it.
	bodyGeom := 0.0
	for _, faces := range compFaces {
		for _, f := range faces {
			if err := budget.Step(); err != nil {
				return nil, nil, 0, err
			}
			loopLen, err := facePerimeterUpper(f, budget)
			if err != nil {
				return nil, nil, 0, err
			}
			geom := facetproof.FacetedAreaGeom(faceDelta[f], loopLen, faceFacetArea[f])
			bodyGeom = proofbound.AbsSumUpper(bodyGeom, geom)
			f.AreaBound = proofbound.AbsSumUpper(geom, in.AreaSlack, proofbound.SumSlop(facetsOf[f], f.Area), faceTermSlop[f])
		}
	}

	// Shells and lumps: positive components are outer shells (one lump
	// each); a negative component is a void of its innermost positive
	// container.
	shells := make([]*topo.FaceShell, len(members))
	lumpOf := map[int]*topo.Lump{}
	var lumps []*topo.Lump
	for ci := range members {
		if err := budget.Step(); err != nil {
			return nil, nil, 0, err
		}
		shells[ci] = env.Alloc.NewFaceShell()
		shells[ci].Void, shells[ci].Faces = compVol[ci].Sign() < 0, compFaces[ci]
		if compVol[ci].Sign() > 0 {
			l := env.Alloc.NewLump()
			l.Shells = []*topo.FaceShell{shells[ci]}
			lumpOf[ci] = l
			lumps = append(lumps, l)
		}
	}
	parents, err := facetproof.VoidParents(budget, compVol, contains)
	if err != nil {
		return nil, nil, 0, err
	}
	for ci, parent := range parents {
		if parent >= 0 {
			lumpOf[parent].Shells = append(lumpOf[parent].Shells, shells[ci])
		}
	}
	body.Lumps = lumps

	// Measurements: exact rational integrals over the held mesh, reported
	// with the composed proven bounds (§9: the bound shapes of the
	// verification design, composed from the operands' chord errors). The
	// volume helper is also the read-only interference evaluator's one source
	// of volume truth.
	if volRat == nil {
		var volF, volBound float64
		volF, volBound, volRat, err = facetproof.VolumeFromAudit(audit, in.VolSymDiff)
		volume = measurement.Measurement{Value: units.CubicMillimeters(volF),
			Exactness: measurement.ExactnessOf(volBound), Bound: units.CubicMillimeters(volBound)}
		if err != nil {
			return nil, nil, 0, err
		}
	}
	body.Volume = volume
	moments, err := facetproof.FirstMomentsFromAudit(budget, audit, tris)
	if err != nil {
		return nil, nil, 0, err
	}

	areaF, termSlop := 0.0, 0.0
	for i := range tris {
		if err := budget.Step(); err != nil {
			return nil, nil, 0, err
		}
		areaF += facetArea[i]
		termSlop = proofbound.AbsSumUpper(termSlop, facetSlop[i])
	}
	// The float area accumulation itself rounds, and the loop above sums
	// NAIVELY: charge the proven naive-summation bound, never a pairwise one
	// (internal/proofbound/bounds.go, proofbound.SumSlop), beside each term's
	// own evaluation charge, which a sliver facet's cancelling cross product
	// needs at the scale of its edge products (proofbound.FacetAreaTermSlop).
	// Both are ulp-scale in the total, so a genuinely tiny-bound planar boolean
	// stays tiny — and never zero, which would claim an exactness a float sum of
	// square roots does not have.
	areaBound := proofbound.AbsSumUpper(bodyGeom, in.AreaSlack, proofbound.SumSlop(len(tris), areaF), termSlop)
	body.Area = measurement.Measurement{
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
	cenBound := facetproof.FacetedCentroidAllowance(in.VolSymDiff, in.DPair, VolFloor(volRat, in.VolSymDiff))
	cxF, _ := cx.Float64()
	cyF, _ := cy.Float64()
	czF, _ := cz.Float64()
	if shared != nil && !proofbound.FiniteVec(r3.Vec{X: cxF, Y: cyF, Z: czF}) {
		return nil, nil, 0, errBoundedArithmetic
	}
	// The three coordinates round independently, and the VecMeasurement bound
	// is a 3D radius (internal/proofbound/bounds.go, proofbound.Radius3D).
	cenRound := proofbound.Radius3D(math.Max(proofbound.RatAbsDiff(cx, cxF), math.Max(proofbound.RatAbsDiff(cy, cyF), proofbound.RatAbsDiff(cz, czF))))
	centroidBound := proofbound.AbsSumUpper(cenBound, cenRound)
	body.Centroid = measurement.VecMeasurement{
		Value:     r3.Vec{X: cxF, Y: cyF, Z: czF},
		Exactness: measurement.ExactnessOf(centroidBound),
		Bound:     units.Millimeters(centroidBound),
	}

	lo, hi := verts[0], verts[0]
	for _, v := range verts[1:] {
		if err := budget.Step(); err != nil {
			return nil, nil, 0, err
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
	extremeErr, err := facetproof.FacetedExtremeError(budget, verts, in.VertexBound, vertexFacetDelta, lo, hi)
	if err != nil {
		return nil, nil, 0, err
	}
	boxBound := proofbound.Radius3D(extremeErr)
	body.Bounds = measurement.Box{
		Min: lo, Max: hi,
		Exactness: measurement.ExactnessOf(boxBound),
		Bound:     units.Millimeters(boxBound),
	}

	// The facet → face mapping, in the body's Faces() order, for Tessellate.
	if err := budget.Err(); err != nil {
		return nil, nil, 0, err
	}
	var faceOf []int
	if shared == nil {
		faceOf, err = facetproof.FaceIndices(ctx, topobuild.Faces(body), facetFace)
	} else {
		faceOf, err = faceIndicesBounded(ctx, shared, topobuild.Faces(body), facetFace)
	}
	if err != nil {
		return nil, nil, 0, err
	}
	return body, faceOf, diameterReading, nil
}

// FaceAreaGeom is the geometric area term AssembleBody charged face f of the
// body it assembled from verts, beta and tris, whose facets are f's own: the
// smaller of its largest facet bound times its upper perimeter and its
// facets' summed perturbation allowance. A caller that replaces f's facets
// reads the term the old body published for f before it rebuilds.
func FaceAreaGeom(budget *proofbound.WorkBudget, f *topo.Face, verts []r3.Vec, beta []float64, tris [][3]int,
	facets []int) (float64, error) {
	own := make([][3]int, len(facets))
	faces := make([]*topo.Face, len(facets))
	for i, t := range facets {
		own[i], faces[i] = tris[t], f
	}
	faceDelta, faceFacetArea, err := facetproof.FaceProofs(budget, verts, own, faces, facetproof.FacetDeltas(own, beta))
	if err != nil {
		return 0, err
	}
	loopLen, err := facePerimeterUpper(f, budget)
	if err != nil {
		return 0, err
	}
	return facetproof.FacetedAreaGeom(faceDelta[f], loopLen, faceFacetArea[f]), nil
}

// facePerimeterUpper reads every loop's edge lengths and error bounds
// and returns proofbound.PerimeterUpper over them.
func facePerimeterUpper(f *topo.Face, budget *proofbound.WorkBudget) (float64, error) {
	var lengths, bounds []float64
	for _, l := range f.Loops {
		for _, ce := range l.CoEdges {
			lengths = append(lengths, ce.Edge.Length)
			bounds = append(bounds, ce.Edge.LengthBound)
		}
	}
	return proofbound.PerimeterUpper(budget, lengths, bounds)
}

// centroidCoord is (moment/24) / volume, exact.
func centroidCoord(m, tf, vol *big.Rat) *big.Rat {
	c := new(big.Rat).Mul(m, tf)
	return c.Quo(c, vol)
}

// VolFloor is the proven lower bound on the true volume: the held volume minus
// the symmetric-difference bound, floored at zero. The rational→float rounding
// can go UP, which would overstate the floor and understate every bound divided
// by it, so it is nudged back down first.
func VolFloor(vol *big.Rat, sym float64) float64 {
	v, _ := vol.Float64()
	v = math.Nextafter(v, math.Inf(-1))
	rem := v - sym
	if rem <= 0 {
		return 0
	}
	return rem
}
