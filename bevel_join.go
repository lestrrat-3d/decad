package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"slices"
	"sort"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// The one-tooth join replaces exactly the sector of the blank's finite root
// face covered by the point loft. Its cap and the blank share the very same
// root-ring vertex indices. An unrelated Union still uses the usual evaluator.
func tryBevelOneToothUnion(ctx context.Context, op meshbool.OperationKind,
	d *Document, ref producerID, a, b *Body) (*Body, bool, error) {
	if op != meshbool.OpUnion {
		return nil, false, nil
	}
	blank, tooth := a, b
	blankSource, ok := exactBevelBlank(blank)
	if !ok {
		blank, tooth = b, a
		blankSource, ok = exactBevelBlank(blank)
		if !ok {
			return nil, false, nil
		}
	}
	fp, ok := tooth.payload.(facetedPayload)
	if !ok || fp.xform != r3.Identity() || fp.pointCone == nil ||
		fp.pointCone.lower == nil || fp.pointCone.upper == nil ||
		fp.pointCone.base.pointSection == nil ||
		fp.pointCone.base.pointSection.apex != (r3.Vec{}) ||
		fp.pointCone.base.xform != r3.Identity() {
		return nil, false, nil
	}
	record := fp.pointCone
	boundary, ok := certifyBevelToothBoundary(record.base.pointSection,
		blankSource, record.base)
	if !ok {
		return nil, false, nil
	}
	body, err := buildBevelOneToothJoin(ctx, d, ref, blank, blankSource, record, boundary)
	return body, true, err
}

type bevelPathNode struct {
	p            r3.Vec
	groupToNext  int
	crossing     int // -1 except for the two certified connector contacts
	rootBoundary bool
	angle        float64 // exact mesh station shared with the blank ring
}

type bevelCapEdge struct {
	u, v, group int // group -1 is the shared root arc
}

// bevelExteriorPath inserts the live line certificates into the original
// point-loft ring. Only the path through the outside tip arc remains exposed.
func bevelExteriorPath(base facetedPayload, boundary *bevelToothBoundary,
	source *pointSectionSource, blank *bevelBlankSource) ([]bevelPathNode, float64, error) {
	n := len(base.verts) - 1
	if n < 6 || len(base.tris) < n || len(base.src) != len(base.tris) {
		return nil, 0, fmt.Errorf("%w: point-loft fan is incomplete", ErrUnsupported)
	}
	groups := make([]int, n)
	seen := make([]bool, n)
	for i, tri := range base.tris {
		if tri[0] != 0 || tri[1] <= 0 || tri[2] <= 0 {
			continue
		}
		u, v := tri[1]-1, tri[2]-1
		if u < 0 || u >= n || v < 0 || v >= n {
			return nil, 0, fmt.Errorf("%w: point-loft fan has a changed boundary order", ErrUnsupported)
		}
		if v != (u+1)%n {
			u, v = v, u
		}
		if v != (u+1)%n || seen[u] {
			return nil, 0, fmt.Errorf("%w: point-loft fan has a changed boundary order", ErrUnsupported)
		}
		seen[u], groups[u] = true, base.src[i]
	}
	for _, yes := range seen {
		if !yes {
			return nil, 0, fmt.Errorf("%w: point-loft fan lacks an edge", ErrUnsupported)
		}
	}
	var crossing [2]struct {
		p     r3.Vec
		bound float64
		edge  int
	}
	maxCrossBound := 0.0
	for k, cert := range boundary.crossing {
		p, bound, ok := bevelSeatPoint(source, cert, blank, base)
		if !ok {
			return nil, 0, fmt.Errorf("%w: cone crossing has no finite spatial enclosure", ErrUnsupported)
		}
		best, edge := math.Inf(1), -1
		for i := range n {
			if groups[i] != boundary.lineIndex[k] {
				continue
			}
			distance := bevelPointSegmentDistance(p, base.verts[i+1], base.verts[(i+1)%n+1])
			if distance < best {
				best, edge = distance, i
			}
		}
		allow := proofbound.AbsSumUpper(base.meshBound, bound)
		if edge < 0 || !(best <= allow) || !proofbound.FiniteVec(p) {
			return nil, 0, fmt.Errorf("%w: certified connector is outside its source fan edge", ErrUnsupported)
		}
		crossing[k] = struct {
			p     r3.Vec
			bound float64
			edge  int
		}{p: p, bound: bound, edge: edge}
		maxCrossBound = max(maxCrossBound, bound)
	}
	if crossing[0].edge == crossing[1].edge {
		return nil, 0, fmt.Errorf("%w: both connector contacts occupy one fan edge", ErrUnsupported)
	}
	full := make([]bevelPathNode, 0, n+2)
	var crossPosition [2]int
	for i := range n {
		full = append(full, bevelPathNode{p: base.verts[i+1], groupToNext: groups[i], crossing: -1})
		for k := range crossing {
			if crossing[k].edge == i {
				crossPosition[k] = len(full)
				full = append(full, bevelPathNode{p: crossing[k].p,
					groupToNext: groups[i], crossing: k})
			}
		}
	}
	tipArc := -1
	// The source class has two arcs. Their exact entity types were checked by
	// certifyBevelToothBoundary, so the other arc is the exposed tip.
	for i, entity := range source.entities {
		if i == boundary.insideArc {
			continue
		}
		if _, ok := entity.(*sketch.Arc); ok {
			tipArc = i
			break
		}
	}
	if tipArc < 0 {
		return nil, 0, fmt.Errorf("%w: tooth has no exposed tip arc", ErrUnsupported)
	}
	forward := make([]bevelPathNode, 0, len(full))
	for i := crossPosition[0]; ; i = (i + 1) % len(full) {
		forward = append(forward, full[i])
		if i == crossPosition[1] {
			break
		}
	}
	hasTip, hasRoot := false, false
	for i := 0; i+1 < len(forward); i++ {
		hasTip = hasTip || forward[i].groupToNext == tipArc
		hasRoot = hasRoot || forward[i].groupToNext == boundary.insideArc
	}
	if !hasTip || hasRoot {
		other := make([]bevelPathNode, 0, len(full))
		for i := crossPosition[1]; ; i = (i + 1) % len(full) {
			other = append(other, full[i])
			if i == crossPosition[0] {
				break
			}
		}
		forward = other
		hasTip, hasRoot = false, false
		for i := 0; i+1 < len(forward); i++ {
			hasTip = hasTip || forward[i].groupToNext == tipArc
			hasRoot = hasRoot || forward[i].groupToNext == boundary.insideArc
		}
		if !hasTip || hasRoot {
			return nil, 0, fmt.Errorf("%w: source fan has no unique outside path", ErrUnsupported)
		}
	}
	if forward[0].crossing < 0 || forward[len(forward)-1].crossing < 0 {
		return nil, 0, fmt.Errorf("%w: source path lost its cone crossings", ErrUnsupported)
	}
	return forward, maxCrossBound, nil
}

func bevelPointSegmentDistance(p, a, b r3.Vec) float64 {
	d := b.Sub(a)
	den := d.Dot(d)
	if den <= 0 || math.IsInf(den, 0) {
		return math.Inf(1)
	}
	t := max(0, min(1, p.Sub(a).Dot(d)/den))
	return p.Sub(a.Add(d.Scale(t))).Len()
}

func bevelRootAngles(path []bevelPathNode) (float64, float64, bool) {
	a := math.Atan2(path[0].p.Z, path[0].p.Y)
	b := math.Atan2(path[len(path)-1].p.Z, path[len(path)-1].p.Y)
	if !proofbound.FiniteVec(r3.Vec{X: a, Y: b}) || a == b || math.Abs(a-b) >= math.Pi {
		return 0, 0, false
	}
	return min(a, b), max(a, b), true
}

// bevelUniformAngles creates a full turn with the two seam generators added.
// The 256 regular chords give a small fixed angular sag for the admitted
// millimetre-scale source; every later root-edge split is inserted as well.
func bevelUniformAngles(lo, hi float64) []float64 {
	angles := make([]float64, 0, 258)
	for i := -128; i < 128; i++ {
		angles = append(angles, math.Pi*float64(i)/128)
	}
	angles = append(angles, lo, hi)
	sort.Float64s(angles)
	return angles
}

func bevelPolygonArea(points []triangulation.Point2) float64 {
	a := 0.0
	for i := range points {
		j := (i + 1) % len(points)
		a += points[i].U*points[j].V - points[j].U*points[i].V
	}
	return a * 0.5
}

func bevelCapTriangulate(ctx context.Context, nodes []bevelPathNode,
	frame r3.Frame) ([][3]int, error) {
	local := make([]triangulation.Point2, len(nodes))
	loop := make([]int, len(nodes))
	for i, node := range nodes {
		p := frame.ToLocal(node.p)
		if !proofbound.FiniteVec(p) {
			return nil, fmt.Errorf("%w: the tooth cap has a non-finite station", ErrUnsupported)
		}
		local[i], loop[i] = triangulation.Point2{U: p.X, V: p.Y}, i
	}
	if bevelPolygonArea(local) < 0 {
		for i, j := 0, len(loop)-1; i < j; i, j = i+1, j-1 {
			loop[i], loop[j] = loop[j], loop[i]
		}
	}
	return triangulation.Triangulate(ctx, local, [][]int{loop})
}

func bevelFarRootPoint(frame r3.Frame, slope, angle float64) (r3.Vec, bool) {
	normal := frame.U().Cross(frame.V())
	direction := r3.Vec{X: 1, Y: slope * math.Cos(angle), Z: slope * math.Sin(angle)}
	den := normal.Dot(direction)
	if den == 0 {
		return r3.Vec{}, false
	}
	x := normal.Dot(frame.Origin()) / den
	p := direction.Scale(x)
	return p, proofbound.FiniteVec(p) && x > 0
}

func bevelCapBoundary(path []bevelPathNode, frame r3.Frame, slope float64,
	angles []float64, lo, hi float64) ([]bevelPathNode, error) {
	nodes := append([]bevelPathNode(nil), path...)
	nodes[0].rootBoundary = true
	nodes[0].angle = math.Atan2(nodes[0].p.Z, nodes[0].p.Y)
	nodes[len(nodes)-1].rootBoundary = true
	nodes[len(nodes)-1].angle = math.Atan2(nodes[len(nodes)-1].p.Z, nodes[len(nodes)-1].p.Y)
	nodes[len(nodes)-1].groupToNext = -1
	root := make([]float64, 0, len(angles))
	for _, angle := range angles {
		if lo < angle && angle < hi {
			root = append(root, angle)
		}
	}
	lastAngle := math.Atan2(path[len(path)-1].p.Z, path[len(path)-1].p.Y)
	if lastAngle == hi {
		for _, angle := range slices.Backward(root) {
			p, ok := bevelFarRootPoint(frame, slope, angle)
			if !ok {
				return nil, fmt.Errorf("%w: root generator misses the source plane", ErrUnsupported)
			}
			nodes = append(nodes, bevelPathNode{p: p, groupToNext: -1,
				crossing: -1, rootBoundary: true, angle: angle})
		}
	} else {
		for _, angle := range root {
			p, ok := bevelFarRootPoint(frame, slope, angle)
			if !ok {
				return nil, fmt.Errorf("%w: root generator misses the source plane", ErrUnsupported)
			}
			nodes = append(nodes, bevelPathNode{p: p, groupToNext: -1,
				crossing: -1, rootBoundary: true, angle: angle})
		}
	}
	return nodes, nil
}

// bevelSubdivideCap reduces the cone-map interpolation bound. Boundary edge
// labels are carried through every split so the exposed side and root seam
// are reconstructed from the same triangulation, without a guessed contact.
func bevelSubdivideCap(ctx context.Context, nodes []bevelPathNode,
	triangles [][3]int, rounds int) ([]bevelPathNode, [][3]int, []bevelCapEdge, float64, error) {
	edges := make([]bevelCapEdge, len(nodes))
	for i := range nodes {
		edges[i] = bevelCapEdge{u: i, v: (i + 1) % len(nodes), group: nodes[i].groupToNext}
	}
	totalRound := 0.0
	for range rounds {
		midpoints := map[pointConeEdge]int{}
		levelRound := 0.0
		boundary := map[pointConeEdge]int{}
		for _, edge := range edges {
			boundary[orderedPointConeEdge(edge.u, edge.v)] = edge.group
		}
		midpoint := func(u, v int) int {
			key := orderedPointConeEdge(u, v)
			if id, ok := midpoints[key]; ok {
				return id
			}
			p, q := nodes[u].p, nodes[v].p
			held := p.Scale(0.5).Add(q.Scale(0.5))
			// The same held interpolation is used for the two cap maps; its
			// arithmetic departure is charged in the result bound.
			levelRound = max(levelRound, proofbound.ProductUpper(
				proofbound.AbsSumUpper(p.Len(), q.Len()), 8*math.Nextafter(1, 2)*math.Ldexp(1, -53)))
			group, onBoundary := boundary[key]
			id := len(nodes)
			root := onBoundary && group == -1
			angle := 0.0
			if root {
				angle = nodes[u].angle*0.5 + nodes[v].angle*0.5
			}
			nodes = append(nodes, bevelPathNode{p: held, crossing: -1,
				rootBoundary: root, angle: angle})
			midpoints[key] = id
			return id
		}
		next := make([][3]int, 0, 4*len(triangles))
		for _, tri := range triangles {
			if err := ctx.Err(); err != nil {
				return nil, nil, nil, 0, err
			}
			a, b, c := tri[0], tri[1], tri[2]
			ab, bc, ca := midpoint(a, b), midpoint(b, c), midpoint(c, a)
			next = append(next, [3]int{a, ab, ca}, [3]int{ab, b, bc},
				[3]int{ca, bc, c}, [3]int{ab, bc, ca})
		}
		newEdges := make([]bevelCapEdge, 0, len(edges)*2)
		for _, edge := range edges {
			m, ok := midpoints[orderedPointConeEdge(edge.u, edge.v)]
			if !ok {
				return nil, nil, nil, 0, fmt.Errorf("%w: cap boundary was not triangulated", ErrUnsupported)
			}
			newEdges = append(newEdges,
				bevelCapEdge{u: edge.u, v: m, group: edge.group},
				bevelCapEdge{u: m, v: edge.v, group: edge.group})
		}
		triangles, edges = next, newEdges
		totalRound = proofbound.AbsSumUpper(totalRound, levelRound)
	}
	return nodes, triangles, edges, totalRound, nil
}

func bevelMergedAngles(base []float64, nodes []bevelPathNode) ([]float64, []int, error) {
	angles := append([]float64(nil), base...)
	rootNode := make([]int, len(nodes))
	for i, node := range nodes {
		rootNode[i] = -1
		if !node.rootBoundary {
			continue
		}
		angle := node.angle
		if !proofbound.FiniteVec(r3.Vec{X: angle}) {
			return nil, nil, fmt.Errorf("%w: root angle is non-finite", ErrUnsupported)
		}
		angles = append(angles, angle)
	}
	sort.Float64s(angles)
	unique := angles[:0]
	for _, angle := range angles {
		if len(unique) == 0 || angle != unique[len(unique)-1] {
			unique = append(unique, angle)
		}
	}
	for i, node := range nodes {
		if node.rootBoundary {
			angle := node.angle
			j := sort.SearchFloat64s(unique, angle)
			if j == len(unique) || unique[j] != angle {
				return nil, nil, fmt.Errorf("%w: root node has no angular station", ErrUnsupported)
			}
			rootNode[i] = j
		}
	}
	return unique, rootNode, nil
}

func bevelBlankGroups(blank *Body, source *bevelBlankSource) ([6]facetGroup, error) {
	var groups [6]facetGroup
	var found [6]bool
	for _, face := range blank.Faces() {
		for j := 1; j < 6; j++ {
			role := fmt.Sprintf("side(0,%d)", source.edgeIndex[j])
			match := false
			for _, origin := range face.Origins() {
				match = match || origin.Role == role
			}
			if !match {
				continue
			}
			if found[j] {
				return groups, fmt.Errorf("%w: blank has repeated source face", ErrUnsupported)
			}
			_, planar := face.Surface().(Plane)
			groups[j] = facetGroup{origins: face.Origins(), surface: face.Surface(),
				denoted: face.denoted, reversed: face.reversed, planar: planar}
			found[j] = true
		}
	}
	for j := 1; j < 6; j++ {
		if !found[j] {
			return groups, fmt.Errorf("%w: blank source face is missing", ErrUnsupported)
		}
	}
	return groups, nil
}

type bevelJoinedMesh struct {
	verts  []r3.Vec
	tris   [][3]int
	src    []int
	groups []facetGroup
}

func (m *bevelJoinedMesh) add(t [3]int, group int) {
	m.tris = append(m.tris, t)
	m.src = append(m.src, group)
}

type bevelRingMesh struct {
	angles            []float64
	rings             [4][]int // heel outer, dedendum, toe root, toe outer
	toeAxis, heelAxis int
}

func bevelBuildBlankMesh(m *bevelJoinedMesh, blank *bevelBlankSource,
	angles []float64, seamLo, seamHi float64) (bevelRingMesh, float64, error) {
	if len(angles) < 16 || angles[0] >= -math.Pi/2 ||
		angles[len(angles)-1] <= math.Pi/2 {
		return bevelRingMesh{}, 0, fmt.Errorf("%w: blank angular ring is incomplete", ErrUnsupported)
	}
	r := bevelRingMesh{angles: angles}
	r.toeAxis = len(m.verts)
	m.verts = append(m.verts, r3.Vec{X: blank.toeAxis.U})
	r.heelAxis = len(m.verts)
	m.verts = append(m.verts, r3.Vec{X: blank.heelAxis.U})
	stations := [4]Point2{blank.heelOuter, blank.ded, blank.toeRoot, blank.toeOuter}
	radiusMax := 0.0
	cosHeld, sinHeld := make([]float64, len(angles)), make([]float64, len(angles))
	cosIv, sinIv := make([]proofbound.RatInterval, len(angles)),
		make([]proofbound.RatInterval, len(angles))
	for i, angle := range angles {
		var trigOK bool
		sinIv[i], cosIv[i], trigOK = proofbound.RadSinCosInterval(proofarith.FloatRat(angle))
		if !trigOK {
			return bevelRingMesh{}, 0, fmt.Errorf("%w: blank angular station has no trigonometric enclosure", ErrUnsupported)
		}
		cosHeld[i], sinHeld[i] = math.Cos(angle), math.Sin(angle)
	}
	maxRound := 0.0
	for k, station := range stations {
		radiusMax = max(radiusMax, station.V)
		r.rings[k] = make([]int, len(angles))
		for i := range angles {
			p := r3.Vec{X: station.U,
				Y: station.V * cosHeld[i], Z: station.V * sinHeld[i]}
			if !proofbound.FiniteVec(p) {
				return bevelRingMesh{}, 0, fmt.Errorf("%w: blank ring vertex is non-finite", ErrUnsupported)
			}
			rad := proofarith.FloatRat(station.V)
			yError := proofbound.IntervalFloatError(proofbound.IntervalScale(cosIv[i], rad), p.Y)
			zError := proofbound.IntervalFloatError(proofbound.IntervalScale(sinIv[i], rad), p.Z)
			maxRound = max(maxRound, proofbound.Radius3D(max(yError, zError)))
			r.rings[k][i] = len(m.verts)
			m.verts = append(m.verts, p)
		}
	}
	maxStep := 0.0
	for i := range angles {
		j := (i + 1) % len(angles)
		step := new(big.Rat)
		if j == 0 {
			step.Add(proofbound.TwoPiInterval().Hi,
				new(big.Rat).Sub(proofarith.FloatRat(angles[0]), proofarith.FloatRat(angles[i])))
		} else {
			step.Sub(proofarith.FloatRat(angles[j]), proofarith.FloatRat(angles[i]))
		}
		stepUp := proofbound.RatFloatUp(step)
		if step.Sign() <= 0 || proofbound.IsNonFinite(stepUp) || stepUp > 0.025 {
			return bevelRingMesh{}, 0, fmt.Errorf("%w: blank angular interval is not ordered", ErrUnsupported)
		}
		maxStep = max(maxStep, stepUp)
		m.add([3]int{r.heelAxis, r.rings[0][i], r.rings[0][j]}, 1)
		for k, group := range [...]int{2, 3, 4} {
			if group == 3 && j != 0 && angles[i] >= seamLo && angles[j] <= seamHi {
				continue
			}
			m.add([3]int{r.rings[k][i], r.rings[k+1][i], r.rings[k+1][j]}, group)
			m.add([3]int{r.rings[k][i], r.rings[k+1][j], r.rings[k][j]}, group)
		}
		m.add([3]int{r.rings[3][i], r.toeAxis, r.rings[3][j]}, 5)
	}
	// MaxStep is checked against the regular 256-chord grid. The sine
	// inequality 1-cos(t) <= t²/2 bounds every chord's circular departure.
	sag := proofbound.ProductUpper(radiusMax,
		proofbound.ProductUpper(maxStep, maxStep)/8)
	return r, proofbound.AbsSumUpper(sag, maxRound), nil
}

func bevelBuildToothMesh(m *bevelJoinedMesh, ring bevelRingMesh,
	nodes []bevelPathNode, caps [][3]int, edges []bevelCapEdge, rootNode []int,
	record *pointConeTrimRecord) (float64, float64, error) {
	baseGroups := len(m.groups)
	toothGroups := append([]facetGroup(nil), record.base.groups...)
	for i := range len(toothGroups) - 1 {
		toothGroups[i].surface = NURBSSurface{}
	}
	m.groups = append(m.groups, toothGroups...)
	upperGroup := len(m.groups)
	m.groups = append(m.groups, pointConeFacetGroup(record.upper, false))
	lowerGroup := len(m.groups)
	m.groups = append(m.groups, pointConeFacetGroup(record.lower, true))
	upper := make([]int, len(nodes))
	lower := make([]int, len(nodes))
	maxRound, maxSnap := 0.0, 0.0
	for i, node := range nodes {
		up, err := pointConeFactor(node.p, record.upper)
		if err != nil {
			return 0, 0, err
		}
		down, err := pointConeFactor(node.p, record.lower)
		if err != nil {
			return 0, 0, err
		}
		if up.q+up.qError < 1 || down.q-down.qError <= 1 {
			return 0, 0, fmt.Errorf("%w: tooth cap reaches its far source plane", ErrUnsupported)
		}
		if up.q-up.qError <= 1 {
			maxRound = max(maxRound, proofbound.ProductUpper(
				proofbound.AbsSumUpper(math.Abs(up.alpha-1), up.qError), node.p.Len()))
			up.alpha = 1
		}
		maxRound = max(maxRound, up.pointError, down.pointError)
		if rootNode[i] >= 0 {
			j := rootNode[i]
			upper[i], lower[i] = ring.rings[1][j], ring.rings[2][j]
			maxSnap = max(maxSnap,
				proofbound.DvLenUpper(proofbound.HeldDelta(
					node.p.Scale(up.alpha), m.verts[upper[i]])),
				proofbound.DvLenUpper(proofbound.HeldDelta(
					node.p.Scale(down.alpha), m.verts[lower[i]])))
			continue
		}
		upper[i] = len(m.verts)
		m.verts = append(m.verts, node.p.Scale(up.alpha))
		lower[i] = len(m.verts)
		m.verts = append(m.verts, node.p.Scale(down.alpha))
	}
	for _, tri := range caps {
		m.add([3]int{upper[tri[0]], upper[tri[1]], upper[tri[2]]}, upperGroup)
		m.add([3]int{lower[tri[2]], lower[tri[1]], lower[tri[0]]}, lowerGroup)
	}
	for _, edge := range edges {
		if edge.group < 0 {
			continue
		}
		u, v := edge.u, edge.v
		m.add([3]int{upper[u], lower[u], lower[v]}, baseGroups+edge.group)
		m.add([3]int{upper[u], lower[v], upper[v]}, baseGroups+edge.group)
	}
	return maxRound, maxSnap, nil
}

// bevelOrientMesh proves edge pairing and one connected component before the
// geometric audit. Each pair's directed edge is made opposite by propagation.
func bevelOrientMesh(m *bevelJoinedMesh) error {
	type use struct{ tri, direction int }
	edges := map[pointConeEdge][]use{}
	for i, tri := range m.tris {
		if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] {
			return fmt.Errorf("%w: joined mesh has a collapsed triangle", ErrUnsupported)
		}
		for j := range 3 {
			a, b := tri[j], tri[(j+1)%3]
			key := orderedPointConeEdge(a, b)
			direction := 1
			if key.a != a {
				direction = -1
			}
			edges[key] = append(edges[key], use{tri: i, direction: direction})
		}
	}
	adj := make([][]struct{ tri, same int }, len(m.tris))
	for _, pair := range edges {
		if len(pair) != 2 {
			return fmt.Errorf("%w: joined seam does not pair every mesh edge", ErrUnsupported)
		}
		same := 0
		if pair[0].direction == pair[1].direction {
			same = 1
		}
		a, b := pair[0].tri, pair[1].tri
		adj[a] = append(adj[a], struct{ tri, same int }{b, same})
		adj[b] = append(adj[b], struct{ tri, same int }{a, same})
	}
	flip := make([]int, len(m.tris))
	for i := range flip {
		flip[i] = -1
	}
	if len(flip) == 0 {
		return fmt.Errorf("%w: joined mesh is empty", ErrUnsupported)
	}
	flip[0] = 0
	queue := []int{0}
	for len(queue) != 0 {
		i := queue[0]
		queue = queue[1:]
		for _, next := range adj[i] {
			want := flip[i] ^ next.same
			if flip[next.tri] < 0 {
				flip[next.tri] = want
				queue = append(queue, next.tri)
			} else if flip[next.tri] != want {
				return fmt.Errorf("%w: joined mesh cannot be oriented", ErrUnsupported)
			}
		}
	}
	for i, f := range flip {
		if f < 0 {
			return fmt.Errorf("%w: joined mesh has another component", ErrUnsupported)
		}
		if f != 0 {
			m.tris[i][1], m.tris[i][2] = m.tris[i][2], m.tris[i][1]
		}
	}
	if tessellation.OrientationSign(m.verts, m.tris, r3.Vec{}) < 0 {
		for i := range m.tris {
			m.tris[i][1], m.tris[i][2] = m.tris[i][2], m.tris[i][1]
		}
	}
	return nil
}

func buildBevelOneToothJoin(ctx context.Context, d *Document, ref producerID,
	blankBody *Body, blank *bevelBlankSource, record *pointConeTrimRecord,
	boundary *bevelToothBoundary) (*Body, error) {
	base := record.base
	path, crossingBound, err := bevelExteriorPath(base, boundary, base.pointSection.source, blank)
	if err != nil {
		return nil, err
	}
	lo, hi, ok := bevelRootAngles(path)
	if !ok {
		return nil, fmt.Errorf("%w: root contacts do not bound one angular sector", ErrUnsupported)
	}
	regular := bevelUniformAngles(lo, hi)
	capNodes, err := bevelCapBoundary(path, base.pointSection.source.frame,
		blank.coneSection().Slope, regular, lo, hi)
	if err != nil {
		return nil, err
	}
	caps, err := bevelCapTriangulate(ctx, capNodes, base.pointSection.source.frame)
	if err != nil {
		return nil, triangulation.WrapLoftError(err)
	}
	capNodes, caps, edges, farRound, err := bevelSubdivideCap(ctx, capNodes, caps, 3)
	if err != nil {
		return nil, err
	}
	angles, rootNode, err := bevelMergedAngles(regular, capNodes)
	if err != nil {
		return nil, err
	}
	blankGroups, err := bevelBlankGroups(blankBody, blank)
	if err != nil {
		return nil, err
	}
	mesh := &bevelJoinedMesh{groups: append([]facetGroup(nil), blankGroups[:]...)}
	ring, blankSag, err := bevelBuildBlankMesh(mesh, blank, angles, lo, hi)
	if err != nil {
		return nil, err
	}
	nodeRound, snap, err := bevelBuildToothMesh(mesh, ring, capNodes, caps,
		edges, rootNode, record)
	if err != nil {
		return nil, err
	}
	toolGap, ok := bevelToolFaceGap(blank, record)
	if !ok {
		return nil, fmt.Errorf("%w: cone tools do not match blank end meridians", ErrUnsupported)
	}
	// A snap is only admitted when the authenticated source and its own
	// chording bound cover it. This catches a tool cap from a different blank.
	seamAllowance := proofbound.AbsSumUpper(base.meshBound, blankSag,
		blank.coneGap, crossingBound, toolGap)
	if snap > seamAllowance {
		return nil, fmt.Errorf("%w: cone tools do not meet the blank's finite root patch", ErrUnsupported)
	}
	far := make([]r3.Vec, len(capNodes))
	for i, node := range capNodes {
		far[i] = node.p
	}
	upperBound, upperLip, err := pointConeCapBound(far, caps, record.upper)
	if err != nil {
		return nil, err
	}
	lowerBound, lowerLip, err := pointConeCapBound(far, caps, record.lower)
	if err != nil {
		return nil, err
	}
	lip := proofbound.AbsSumUpper(1, upperLip, lowerLip)
	capBound := max(upperBound, lowerBound)
	bound := proofbound.AbsSumUpper(blankSag, snap, nodeRound,
		proofbound.ProductUpper(farRound, lip),
		proofbound.ProductUpper(proofbound.AbsSumUpper(base.meshBound, crossingBound), lip),
		capBound, blank.coneGap, toolGap)
	if proofbound.IsNonFinite(bound) || bound > 0.1 {
		return nil, fmt.Errorf("%w: joined tooth cannot meet the 0.1 mm boundary proof", ErrUnsupported)
	}
	if err := bevelOrientMesh(mesh); err != nil {
		return nil, err
	}
	if err := tessellation.RequireVertexLinks(ctx, len(mesh.verts), mesh.tris); err != nil {
		return nil, err
	}
	if err := loftmesh.LoftCrossingAuditBevelJoin(proofbound.NewWorkBudget(ctx), mesh.verts, mesh.tris); err != nil {
		return nil, err
	}
	// The source fan's own occupied-volume error is already base.volSymDiff.
	// Sweeping it again at base.meshBound across the subdivided output facets
	// would count its chording as if every tiny facet moved independently.
	roundEnvelope := proofbound.AbsSumUpper(blankSag, snap, nodeRound,
		proofbound.ProductUpper(farRound, lip),
		proofbound.ProductUpper(crossingBound, lip), blank.coneGap, toolGap)
	area, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.verts, mesh.tris, roundEnvelope)
	if err != nil {
		return nil, err
	}
	capArea := pointConeCapArea(far, caps)
	capAreaFactor := max(1, proofbound.ProductUpper(upperLip, upperLip),
		proofbound.ProductUpper(lowerLip, lowerLip))
	volumeGap := proofbound.AbsSumUpper(base.volSymDiff,
		proofbound.ProductUpper(proofbound.ProductUpper(capArea,
			proofbound.AbsSumUpper(upperBound, lowerBound)), capAreaFactor),
		proofbound.SweptVolumeAllow(roundEnvelope, area))
	areaForArea, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.verts, mesh.tris, bound)
	if err != nil {
		return nil, err
	}
	baseArea, err := proofbound.PerturbedAreaUpperContext(ctx, base.verts, base.tris, base.meshBound)
	if err != nil {
		return nil, err
	}
	blankArea, err := blankBody.Area()
	if err != nil {
		return nil, err
	}
	// The joined boundary is contained in the blank boundary, the original
	// point-loft side, and the two mapped cap patches. Bounding both its true
	// area and its held facet area also covers removed shared patches.
	areaSlack := proofbound.AbsSumUpper(areaForArea,
		blankArea.Value.Base(), blankArea.Bound.Base(),
		baseArea, base.areaSlack,
		proofbound.ProductUpper(proofbound.AbsSumUpper(1,
			proofbound.ProductUpper(upperLip, upperLip),
			proofbound.ProductUpper(lowerLip, lowerLip)),
			proofbound.AbsSumUpper(capArea, base.areaSlack)))
	if proofbound.IsNonFinite(volumeGap) || proofbound.IsNonFinite(areaSlack) {
		return nil, fmt.Errorf("%w: joined tooth has no finite measure proof", ErrUnsupported)
	}
	vertexBound := make([]float64, len(mesh.verts))
	for i := range vertexBound {
		vertexBound[i] = bound
	}
	pp := facetedPayload{verts: mesh.verts, tris: mesh.tris, src: mesh.src,
		groups: mesh.groups, vertexBound: vertexBound,
		contactAudited: true,
		meshBound:      bound, volSymDiff: volumeGap, areaSlack: areaSlack,
		dPair: base.dPair, xform: r3.Identity()}
	body, err := buildFacetedBody(ctx, d, ref, pp)
	if err != nil {
		return nil, err
	}
	blankVolume, err := blankBody.Volume()
	if err != nil {
		return nil, err
	}
	joinedVolume, err := body.Volume()
	if err != nil {
		return nil, err
	}
	if joinedVolume.Value.Base()-joinedVolume.Bound.Base() <=
		blankVolume.Value.Base()+blankVolume.Bound.Base() {
		return nil, fmt.Errorf("%w: joined tooth has no certified occupied volume", ErrUnsupported)
	}
	return body, nil
}
