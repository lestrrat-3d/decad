package decad

import (
	"fmt"
	"math"
	"math/big"

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
	verts  []r3.Vec
	tris   [][3]int
	src    []int // per-facet source-group id
	groups []facetGroup

	// faceOf maps each facet to its face's index in the built body's
	// Faces() order; buildFacetedBody sets it, Tessellate reads it.
	faceOf []int

	// meshBound is the proven vertex-level bound (mm): no point of the true
	// result boundary is farther than this from the held mesh's
	// corresponding piece. volSymDiff bounds the volume of the symmetric
	// difference between the held solid and the true result. areaSlack
	// bounds the area the held mesh cannot report: the chord-length deficit
	// the operand tessellations carry, plus the area of any facet the final
	// weld collapsed out of the mesh.
	// dPair is the operand pair's diameter, the centroid bound's yardstick.
	meshBound  float64
	volSymDiff float64
	areaSlack  float64
	dPair      float64

	xform r3.Transform
}

// transform is the accumulated rigid placement.
func (fp facetedPayload) transform() r3.Transform { return fp.xform }

// placed re-evaluates the held mesh under the composed motion: the vertices
// move through the delta motion (float rounding is folded into the proven
// bounds — the geometry is never silently trusted), a reflection flips the
// windings, and the topology and measurements rebuild from the moved mesh.
func (fp facetedPayload) placed(d *Document, ref StepRef, composed r3.Transform) (*Body, error) {
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
	next.verts = make([]r3.Vec, len(fp.verts))
	// The rounding a rigid motion commits is committed at the magnitude of the
	// INPUT coordinate and of the translation — inside the products and sums —
	// never at the magnitude of the result: a body built far from the origin
	// and moved back rounds at the far magnitude. Charge it there (bounds.go).
	maxIn := 0.0
	for i, v := range fp.verts {
		maxIn = math.Max(maxIn, math.Max(math.Abs(v.X), math.Max(math.Abs(v.Y), math.Abs(v.Z))))
		next.verts[i] = delta.Apply(v)
	}
	if delta.IsReflection() {
		next.tris = make([][3]int, len(fp.tris))
		for i, t := range fp.tris {
			next.tris[i] = [3]int{t[0], t[2], t[1]}
		}
	}
	tr := delta.Translation()
	maxTrans := math.Max(math.Abs(tr.X), math.Max(math.Abs(tr.Y), math.Abs(tr.Z)))
	allow := rigidRoundAllow(maxIn, maxTrans)
	next.meshBound += allow
	next.volSymDiff += sweptVolumeAllow(allow, perturbedAreaUpper(next.verts, next.tris, allow))
	return buildFacetedBody(d, ref, next)
}

// ulpOf is the spacing of float64 at magnitude x.
func ulpOf(x float64) float64 {
	ax := math.Abs(x)
	return math.Nextafter(ax, math.Inf(1)) - ax
}

// meshAreaUpper is a proven upper bound on the held mesh's total facet area:
// the float sum padded by the PROVEN naive-summation bound of the very loop
// that computed it (bounds.go, sumSlop) — not a fixed fraction, which no proof
// backs at any facet count.
func meshAreaUpper(verts []r3.Vec, tris [][3]int) float64 {
	total := 0.0
	for _, t := range tris {
		a, b, c := verts[t[0]], verts[t[1]], verts[t[2]]
		total += b.Sub(a).Cross(c.Sub(a)).Len() / 2
	}
	return total + sumSlop(len(tris), total) + 1e-300
}

// buildFacetedBody assembles the Faceted body from the payload: exact
// component/void analysis, per-source-face topology, and measurements with
// the composed proven bounds.
func buildFacetedBody(d *Document, ref StepRef, pp facetedPayload) (*Body, error) {
	verts, tris := pp.verts, pp.tris
	if len(tris) == 0 {
		return nil, fmt.Errorf(`%w: the boolean result holds no boundary`, ErrBooleanFailed)
	}

	// The held mesh must be closed: every directed edge paired with its
	// reverse. The stitcher proved this; the rebuild re-checks it.
	directed := map[[2]int]int{}
	for _, t := range tris {
		for k := range 3 {
			directed[[2]int{t[k], t[(k+1)%3]}]++
		}
	}
	for e, n := range directed {
		if n != 1 || directed[[2]int{e[1], e[0]}] != 1 {
			return nil, fmt.Errorf(`%w: the held boundary does not close`, ErrBooleanFailed)
		}
	}

	xverts := make([]xpt, len(verts))
	for i, v := range verts {
		xverts[i] = xptOf(v)
	}

	// Connected components → candidate shells, with exact signed volumes.
	comp := make([]int, len(tris))
	for i := range comp {
		comp[i] = -1
	}
	adj := facetAdjacency(tris)
	var members [][]int
	for i := range tris {
		if comp[i] != -1 {
			continue
		}
		id := len(members)
		queue := []int{i}
		comp[i] = id
		var list []int
		for len(queue) > 0 {
			f := queue[0]
			queue = queue[1:]
			list = append(list, f)
			for _, nb := range adj[f] {
				if comp[nb] == -1 {
					comp[nb] = id
					queue = append(queue, nb)
				}
			}
		}
		members = append(members, list)
	}

	sixth := big.NewRat(1, 6)
	compVol := make([]*big.Rat, len(members))
	for ci, list := range members {
		v := new(big.Rat)
		for _, fi := range list {
			t := tris[fi]
			v.Add(v, xdot(xverts[t[0]], xcross(xverts[t[1]], xverts[t[2]])))
		}
		compVol[ci] = v.Mul(v, sixth)
		if compVol[ci].Sign() == 0 {
			return nil, fmt.Errorf(`%w: a result shell encloses no volume`, ErrBooleanFailed)
		}
	}

	// Exact containment: which components enclose which.
	contains := make([][]bool, len(members)) // contains[outer][inner]
	for i := range members {
		contains[i] = make([]bool, len(members))
	}
	for inner := range members {
		t := tris[members[inner][0]]
		probe := xCentroid(xverts[t[0]], xverts[t[1]], xverts[t[2]])
		for outer := range members {
			if outer == inner {
				continue
			}
			inside, onBoundary, err := meshParity(probe, verts, tris, members[outer])
			if err != nil {
				return nil, err
			}
			if onBoundary {
				return nil, fmt.Errorf(`%w: two result shells touch`, ErrBooleanFailed)
			}
			contains[outer][inner] = inside
		}
	}

	body := &Body{doc: d, origin: FeatureRef{Step: ref, Role: "body"}, solid: true}
	boundMM := units.Millimeters(pp.meshBound)

	for _, s := range pp.src {
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
		patch[i] = -1
	}
	nPatch := 0
	for i := range tris {
		if patch[i] != -1 {
			continue
		}
		id := nPatch
		nPatch++
		patch[i] = id
		queue := []int{i}
		for len(queue) > 0 {
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
		f, ok := faceIdx[patch[i]]
		if !ok {
			// Every patch of one source face carries that face's OWN origins:
			// both strips of a split cap came from the cap, and both must
			// still say so, or FaceCreatedBy and the surveys lose them.
			f = &Face{
				surface:    Faceted{Bound: boundMM},
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

	if err := buildFacetedTopology(verts, tris, facetFace, facePlanar, pp.meshBound); err != nil {
		return nil, err
	}

	// Per-face area bounds: the §4 shape — the rim's proven displacement times
	// the face's own bounding edges — plus the operands' chord-length slack and
	// the PROVEN slop of the float sum that produced the area. The perimeter it
	// multiplies is the UPPER one: an edge's held chord length is itself only
	// known within its own bound (bounds.go, chainLengthBound).
	facetsOf := map[*Face]int{}
	for _, f := range facetFace {
		facetsOf[f]++
	}
	// facePerimTotal is the sum of the faces' OWN perimeters, which counts every
	// edge TWICE — once per face it bounds. That is the right multiplier for the
	// whole body's area: a rim that moves perturbs BOTH faces it separates, and
	// charging it once would understate the body's area error by up to a factor
	// of two.
	facePerimTotal := 0.0
	for _, faces := range compFaces {
		for _, f := range faces {
			loopLen := 0.0
			for _, l := range f.loops {
				for _, ce := range l.coedges {
					loopLen += ce.edge.length + ce.edge.lengthBound
				}
			}
			facePerimTotal += loopLen
			f.areaBound = upRound(pp.meshBound*loopLen + pp.areaSlack + sumSlop(facetsOf[f], f.area))
		}
	}

	// Shells and lumps: positive components are outer shells (one lump
	// each); a negative component is a void of its innermost positive
	// container.
	shells := make([]*Shell, len(members))
	lumpOf := map[int]*Lump{}
	var lumps []*Lump
	for ci := range members {
		shells[ci] = &Shell{faces: compFaces[ci], void: compVol[ci].Sign() < 0}
		if compVol[ci].Sign() > 0 {
			l := &Lump{shells: []*Shell{shells[ci]}}
			lumpOf[ci] = l
			lumps = append(lumps, l)
		}
	}
	for ci := range members {
		if compVol[ci].Sign() > 0 {
			continue
		}
		parent := -1
		for outer := range members {
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
	// verification design, composed from the operands' chord errors).
	total := new(big.Rat)
	var mx, my, mz = new(big.Rat), new(big.Rat), new(big.Rat)
	for i, t := range tris {
		a, b, c := xverts[t[0]], xverts[t[1]], xverts[t[2]]
		det := xdot(a, xcross(b, c))
		total.Add(total, det)
		mx.Add(mx, new(big.Rat).Mul(det, new(big.Rat).Add(new(big.Rat).Add(a.x, b.x), c.x)))
		my.Add(my, new(big.Rat).Mul(det, new(big.Rat).Add(new(big.Rat).Add(a.y, b.y), c.y)))
		mz.Add(mz, new(big.Rat).Mul(det, new(big.Rat).Add(new(big.Rat).Add(a.z, b.z), c.z)))
		_ = i
	}
	volRat := new(big.Rat).Mul(total, sixth)
	if volRat.Sign() <= 0 {
		return nil, fmt.Errorf(`%w: the boolean result encloses no volume`, ErrBooleanFailed)
	}
	volF, _ := volRat.Float64()
	volRound := ratAbsDiff(volRat, volF)
	body.volume = Measurement{
		Value:     units.CubicMillimeters(volF),
		Exactness: exactnessOf(pp.volSymDiff + volRound),
		Bound:     units.CubicMillimeters(pp.volSymDiff + volRound),
	}

	areaF := 0.0
	for _, t := range tris {
		a, b, c := verts[t[0]], verts[t[1]], verts[t[2]]
		areaF += b.Sub(a).Cross(c.Sub(a)).Len() / 2
	}
	// The float area accumulation itself rounds, and the loop above sums
	// NAIVELY: charge the proven naive-summation bound, never a pairwise one
	// (bounds.go, sumSlop). It is ulp-scale in the total, so a genuinely
	// tiny-bound planar boolean stays tiny — and never zero, which would claim
	// an exactness a float sum of square roots does not have.
	areaBound := upRound(pp.meshBound*facePerimTotal + pp.areaSlack + sumSlop(len(tris), areaF))
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
	cenBound := pp.dPair
	if rem := volFloor(volRat, pp.volSymDiff); rem > 0 {
		cenBound = math.Min(cenBound, pp.volSymDiff*pp.dPair/rem)
	}
	cxF, _ := cx.Float64()
	cyF, _ := cy.Float64()
	czF, _ := cz.Float64()
	// The three coordinates round independently, and the VecMeasurement bound
	// is a 3D radius (bounds.go, radius3D).
	cenRound := radius3D(math.Max(ratAbsDiff(cx, cxF), math.Max(ratAbsDiff(cy, cyF), ratAbsDiff(cz, czF))))
	body.centroid = VecMeasurement{
		Value:     r3.Vec{X: cxF, Y: cyF, Z: czF},
		Exactness: exactnessOf(cenBound + cenRound),
		Bound:     units.Millimeters(cenBound + cenRound),
	}

	lo, hi := verts[0], verts[0]
	for _, v := range verts[1:] {
		lo = r3.Vec{X: math.Min(lo.X, v.X), Y: math.Min(lo.Y, v.Y), Z: math.Min(lo.Z, v.Z)}
		hi = r3.Vec{X: math.Max(hi.X, v.X), Y: math.Max(hi.Y, v.Y), Z: math.Max(hi.Z, v.Z)}
	}
	// Min and Max are POSITIONS, and Bound is the error bound on them: a 3D
	// radius, not a per-axis extent (core §5.2). The held extreme can be one
	// displacement inside the truth on every axis at once, so the corner is up
	// to √3 of it away — and the per-coordinate reading is exactly tight, so
	// there is no headroom to absorb the factor (bounds.go, radius3D).
	boxBound := radius3D(pp.meshBound)
	body.bounds = Box{
		Min: lo, Max: hi,
		Exactness: exactnessOf(boxBound),
		Bound:     units.Millimeters(boxBound),
	}

	// The facet → face mapping, in the body's Faces() order, for Tessellate.
	flat := map[*Face]int{}
	for i, f := range body.Faces() {
		flat[f] = i
	}
	pp.faceOf = make([]int, len(tris))
	for i, f := range facetFace {
		pp.faceOf[i] = flat[f]
	}
	body.payload = pp
	return body, nil
}

// exactnessOf keys a measurement's exactness off its proven bound: zero is the
// truth, anything else is Approximate.
//
// A zero bound is therefore a CLAIM that the reported value is exactly
// representable — and only a value proven so may reach it: the analytic
// features' closed forms, and the boolean's rational integrals carrying their
// own explicit rounding term. A value a float loop COMPUTED is never among
// them: its bound comes from bounds.go, whose helpers are never zero for a
// nonzero float-computed quantity (sumSlop, chainLengthBound), so it can never
// arrive here claiming Exact.
func exactnessOf(bound float64) Exactness {
	if bound == 0 {
		return Exact
	}
	return Approximate
}

// ratAbsDiff is |r − f| rounded up to float64.
func ratAbsDiff(r *big.Rat, f float64) float64 {
	d := new(big.Rat).Sub(r, ratOf(f))
	d.Abs(d)
	out, _ := d.Float64()
	if out > 0 {
		out = math.Nextafter(out, math.Inf(1))
	}
	return out
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

// buildFacetedTopology chains the face-boundary mesh edges into topological
// Edges (split where the adjacent face pair or the exact hinge convexity
// changes) and builds each face's loops by walking the facet fans.
func buildFacetedTopology(verts []r3.Vec, tris [][3]int, facetFace []*Face, facePlanar map[*Face]bool, bound float64) error {
	boundMM := units.Millimeters(bound)

	// Directed halfedge → facet, and the boundary predicate: the twin facet
	// belongs to a different face.
	halfOwner := map[[2]int]int{}
	for i, t := range tris {
		for k := range 3 {
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
			s := orientSign(verts[t[0]], verts[t[1]], verts[t[2]], verts[opp])
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
		v := &Vertex{position: verts[vi], bound: boundMM}
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
	commitChain := func(path []int) {
		info := hinges[ukey([2]int{path[0], path[1]})]
		length := 0.0
		nSegs := 0
		for k := 0; k+1 < len(path); k++ {
			key := ukey([2]int{path[k], path[k+1]})
			chainOf[key] = len(chains)
			posInChain[key] = k
			length += verts[path[k+1]].Sub(verts[path[k]]).Len()
			nSegs++
		}
		// The chain holds nSegs chords, and BOTH endpoints of each move: the
		// error accumulates over N, it is not δ (bounds.go, chainLengthBound).
		// It is never zero either — the held length is a float sum of square
		// roots, and the last ulp is not free — so an all-planar rim reports
		// Approximate rather than an Exact it cannot back.
		lengthBound := chainLengthBound(nSegs, bound, length)
		e := &Edge{
			curve:       FacetedCurve{Bound: boundMM},
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
	}
	walkFrom := func(v int, e [2]int) []int {
		path := []int{v}
		cur, curEdge := v, e
		for {
			usedEdge[ukey(curEdge)] = struct{}{}
			nxt := curEdge[0] + curEdge[1] - cur
			path = append(path, nxt)
			if nxt == path[0] || breakAt(nxt) {
				return path
			}
			nextEdge := otherAt(nxt, ukey(curEdge))
			if ukey(nextEdge) == ukey(curEdge) {
				return path
			}
			if _, seen := usedEdge[ukey(nextEdge)]; seen {
				return path
			}
			cur, curEdge = nxt, nextEdge
		}
	}
	// Open chains first: start at every break vertex, in edge order.
	for _, e := range boundaryEdges {
		for _, v := range []int{e[0], e[1]} {
			if !breakAt(v) {
				continue
			}
			if _, ok := usedEdge[ukey(e)]; ok {
				break
			}
			commitChain(walkFrom(v, e))
			break
		}
	}
	// The rest are closed uniform cycles: anchor each at its smallest
	// vertex, deterministically.
	for _, e := range boundaryEdges {
		if _, ok := usedEdge[ukey(e)]; ok {
			continue
		}
		cycle := walkFrom(e[0], e)
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
			rotated = append(rotated, ring[(anchor+i)%len(ring)])
		}
		rotated = append(rotated, ring[anchor])
		// Re-commit with the rotated ordering (walkFrom already marked the
		// edges used).
		commitChain(rotated)
	}

	// The area-weighted normal of each face's own facets. A face is ONE patch,
	// and a patch of a PLANAR source is coplanar, so this is that plane's
	// outward normal (scaled by twice the patch's area).
	faceNormal := map[*Face]r3.Vec{}
	for i, t := range tris {
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
			for {
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
		if len(f.loops) == 0 {
			return fmt.Errorf(`%w: a faceted face has no boundary loop`, ErrBooleanFailed)
		}
		li := -1
		if facePlanar[f] {
			n := faceNormal[f]
			best := 0.0
			for idx, l := range f.loops {
				if s := loopMoment[l].Dot(n); s > best {
					best, li = s, idx
				}
			}
		}
		if li == -1 {
			longest := -1.0
			li = 0
			for idx, l := range f.loops {
				total := 0.0
				for _, ce := range l.coedges {
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
	return nil
}

func collectFaces(facetFace []*Face) map[*Face]struct{} {
	out := map[*Face]struct{}{}
	for _, f := range facetFace {
		out[f] = struct{}{}
	}
	return out
}
