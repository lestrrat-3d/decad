package loftcontact

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// AuditOutputSource checks shell against source without repairing it: every
// directed edge pairs once with a compatible owner, the triangles around each
// vertex form one connected fan carrying its recorded incidences, same-level
// cap triangles do not overlap, and side walls meet the middle cap only on its
// boundary.
func AuditOutputSource(budget *proofbound.WorkBudget,
	shell OutputShell, source SourcePartition, checked ...*Arithmetic) (bool, error) {
	arithmetic := ReplayArithmetic(checked)
	if len(shell.Triangles) == 0 || len(shell.Triangles) > 512 ||
		len(shell.Vertices) == 0 || len(shell.Vertices) > 512 ||
		len(shell.Faces) != len(shell.Triangles) ||
		len(shell.TriangleBounds) != len(shell.Triangles) {
		return false, nil
	}
	directed := make(map[[2]int]int, 3*len(shell.Triangles))
	incident := make([][]int, len(shell.Vertices))
	incidences := make([]map[OutputFace]struct{}, len(shell.Vertices))
	for fi, tri := range shell.Triangles {
		if err := budget.Step(); err != nil {
			return false, err
		}
		for k, vertex := range tri {
			if vertex < 0 || vertex >= len(shell.Vertices) {
				return false, nil
			}
			incident[vertex] = append(incident[vertex], fi)
			if incidences[vertex] == nil {
				incidences[vertex] = make(map[OutputFace]struct{})
			}
			incidences[vertex][shell.Faces[fi]] = struct{}{}
			edge := [2]int{vertex, tri[(k+1)%3]}
			if _, duplicate := directed[edge]; duplicate {
				return false, nil
			}
			directed[edge] = fi
		}
		if shell.TriangleBounds[fi] < 0 || proofbound.IsNonFinite(shell.TriangleBounds[fi]) ||
			shell.TriangleBounds[fi] != max(shell.Vertices[tri[0]].Bound,
				shell.Vertices[tri[1]].Bound, shell.Vertices[tri[2]].Bound) {
			return false, nil
		}
		if !OutputNormalsAgree(shell.Vertices[tri[0]],
			shell.Vertices[tri[1]], shell.Vertices[tri[2]]) {
			return false, nil
		}
	}
	for edge, fi := range directed {
		if err := budget.Step(); err != nil {
			return false, err
		}
		peer, found := directed[[2]int{edge[1], edge[0]}]
		if !found || !outputEdgeOwnersCompatible(shell, source, fi, peer, edge, arithmetic) ||
			!arithmetic.OK {
			return false, nil
		}
	}
	for vertex, faces := range incident {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if len(faces) < 3 || len(shell.Vertices[vertex].Incidences) != len(incidences[vertex]) {
			return false, nil
		}
		for face := range incidences[vertex] {
			if err := budget.Step(); err != nil {
				return false, err
			}
			if _, found := shell.Vertices[vertex].Incidences[face]; !found {
				return false, nil
			}
		}
		seen := map[int]struct{}{faces[0]: {}}
		queue := []int{faces[0]}
		for len(queue) != 0 {
			fi := queue[0]
			queue = queue[1:]
			tri := shell.Triangles[fi]
			for k, v := range tri {
				if v != vertex {
					continue
				}
				for _, edge := range [][2]int{{vertex, tri[(k+1)%3]}, {tri[(k+2)%3], vertex}} {
					if err := budget.Step(); err != nil {
						return false, err
					}
					peer := directed[[2]int{edge[1], edge[0]}]
					if _, found := seen[peer]; !found {
						seen[peer] = struct{}{}
						queue = append(queue, peer)
					}
				}
			}
		}
		if len(seen) != len(faces) {
			return false, nil
		}
	}
	for i, a := range shell.Triangles {
		if shell.Faces[i].Kind != 0 {
			continue
		}
		for j := range i {
			if err := budget.Step(); err != nil {
				return false, err
			}
			if shell.Faces[j].Kind != 0 || shell.Faces[j].Level != shell.Faces[i].Level {
				continue
			}
			p, q := [3]Point{}, [3]Point{}
			for k := range 3 {
				p[k] = Point{U: shell.Vertices[a[k]].Point.U, V: shell.Vertices[a[k]].Point.V}
				v := shell.Vertices[shell.Triangles[j][k]].Point
				q[k] = Point{U: v.U, V: v.V}
			}
			if shell.Faces[i].Level == 0 {
				p[1], p[2] = p[2], p[1]
				q[1], q[2] = q[2], q[1]
			}
			overlap, err := TrianglesOverlap(budget, p, q, arithmetic)
			if err != nil || overlap || !arithmetic.OK {
				return false, err
			}
		}
	}
	return OutputMiddleContact(budget, shell, source.CapLevels[1], directed, arithmetic)
}

func outputEdgeOwnersCompatible(shell OutputShell, source SourcePartition,
	fi, peer int, edge [2]int, arithmetic ...*Arithmetic) bool {
	c := ReplayArithmetic(arithmetic)
	a, b := shell.Faces[fi], shell.Faces[peer]
	if a.Kind == b.Kind && a.Owner == b.Owner {
		return true
	}
	p := shell.Vertices[edge[0]].Point
	q := shell.Vertices[edge[1]].Point
	if a.Kind == 0 && b.Kind == 0 && a.Level == b.Level {
		return true // two selected cells share a certified internal cap edge
	}
	if a.Kind == 0 && b.Kind == 1 || a.Kind == 1 && b.Kind == 0 {
		capFace, side := a, b
		if b.Kind == 0 {
			capFace, side = b, a
		}
		if capFace.Level < 0 || capFace.Level >= len(source.CapLevels) ||
			c.Cmp(p.Z, q.Z) != 0 || c.Cmp(p.Z, source.CapLevels[capFace.Level]) != 0 || !c.OK {
			return false
		}
		if capFace.Owner == side.Owner {
			return true
		}
		if capFace.Level != 1 || capFace.Owner != 1 || side.Owner != 0 ||
			side.Segment != source.Carrier || side.Level != 3 {
			return false
		}
		return outputJunctionEdge(p, q, source.Junction[1], source.CapLevels[1])
	}
	if a.Kind == 1 && b.Kind == 1 && a.Owner != b.Owner {
		prism, loft := a, b
		if b.Owner == 0 {
			prism, loft = b, a
		}
		if prism.Owner != 0 || loft.Owner != 1 || prism.Segment != source.Carrier {
			return false
		}
		for k, crossed := range source.Crossing {
			if loft.Segment == crossed && prism.Level == k &&
				outputJunctionEdge(p, q,
					[2]sectionrecord.Point2{source.Junction[0][k], source.Junction[1][k]},
					source.CapLevels[0], source.CapLevels[1]) {
				return true
			}
		}
		return false
	}
	return false
}

func outputJunctionEdge(a, b OutputPoint, junction [2]sectionrecord.Point2,
	levels ...*big.Rat) bool {
	if len(levels) == 1 {
		levels = []*big.Rat{levels[0], levels[0]}
	}
	p, okP := PointOf(junction[0])
	q, okQ := PointOf(junction[1])
	if !okP || !okQ {
		return false
	}
	x, y := OutputP2(p, levels[0]), OutputP2(q, levels[1])
	return OutputSamePoint(a, x) && OutputSamePoint(b, y) ||
		OutputSamePoint(a, y) && OutputSamePoint(b, x)
}

// OutputMiddleContact reports whether each side triangle of shell meets the
// middle cap at height level only on a cap boundary edge it shares, or at a
// cap boundary vertex. directed maps each directed edge of shell to its
// triangle.
func OutputMiddleContact(budget *proofbound.WorkBudget,
	shell OutputShell, level *big.Rat, directed map[[2]int]int,
	arithmetic ...*Arithmetic) (bool, error) {
	c := ReplayArithmetic(arithmetic)
	if !c.Valid(level) {
		return false, nil
	}
	boundary := make(map[string][2]int)
	capVertices := make(map[string]struct{})
	for fi, tri := range shell.Triangles {
		if shell.Faces[fi].Kind != 0 || shell.Faces[fi].Level != 1 {
			continue
		}
		for k, v := range tri {
			if err := budget.Step(); err != nil {
				return false, err
			}
			e := [2]int{v, tri[(k+1)%3]}
			peer := directed[[2]int{e[1], e[0]}]
			if shell.Faces[peer].Kind == 1 {
				boundary[outputEdgeKey(shell, e)] = e
				capVertices[OutputPointKey(shell.Vertices[e[0]].Point)] = struct{}{}
				capVertices[OutputPointKey(shell.Vertices[e[1]].Point)] = struct{}{}
			}
		}
	}
	if len(boundary) == 0 {
		return false, nil
	}
	for fi, tri := range shell.Triangles {
		if shell.Faces[fi].Kind != 1 {
			continue
		}
		trace, coplanar, err := OutputPlaneTrace(budget, shell, tri, level, c)
		if err != nil || coplanar || !c.OK {
			return false, err
		}
		if len(trace) == 0 {
			continue
		}
		for ci, endCap := range shell.Triangles {
			if err := budget.Step(); err != nil {
				return false, err
			}
			if shell.Faces[ci].Kind != 0 || shell.Faces[ci].Level != 1 {
				continue
			}
			lo, hi, hit := outputTraceInCapTriangle(trace, endCap, shell, c)
			if !c.OK {
				return false, nil
			}
			if !hit {
				continue
			}
			if c.Cmp(lo, hi) == 0 {
				if !c.OK {
					return false, nil
				}
				p := trace[0]
				if len(trace) == 2 {
					p = c.Interpolate(trace[0], trace[1], lo)
				}
				if !c.OK {
					return false, nil
				}
				if _, found := capVertices[OutputPointKey(OutputP2(p, level))]; !found {
					return false, nil
				}
				continue
			}
			a, b := c.Interpolate(trace[0], trace[1], lo),
				c.Interpolate(trace[0], trace[1], hi)
			if !c.OK {
				return false, nil
			}
			key := outputP2EdgeKey(a, b, level)
			edge, found := boundary[key]
			if !found || directed[[2]int{edge[1], edge[0]}] != fi {
				return false, nil
			}
		}
	}
	return c.OK, nil
}

func outputEdgeKey(shell OutputShell, edge [2]int) string {
	a, b := shell.Vertices[edge[0]].Point, shell.Vertices[edge[1]].Point
	if OutputPointKey(a) > OutputPointKey(b) {
		a, b = b, a
	}
	return OutputPointKey(a) + ":" + OutputPointKey(b)
}

func outputP2EdgeKey(a, b Point, z *big.Rat) string {
	p, q := OutputP2(a, z), OutputP2(b, z)
	if OutputPointKey(p) > OutputPointKey(q) {
		p, q = q, p
	}
	return OutputPointKey(p) + ":" + OutputPointKey(q)
}

// OutputPlaneTrace returns the distinct plane points where triangle tri of
// shell meets height z, and whether the triangle lies in that plane or
// crosses it in more than two points.
func OutputPlaneTrace(budget *proofbound.WorkBudget, shell OutputShell,
	tri [3]int, z *big.Rat, arithmetic ...*Arithmetic) ([]Point, bool, error) {
	c := ReplayArithmetic(arithmetic)
	var points []Point
	add := func(p Point) {
		for _, q := range points {
			if c.SamePoint(p, q) {
				return
			}
		}
		points = append(points, p)
	}
	onPlane := 0
	for i, vertex := range tri {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		a := shell.Vertices[vertex].Point
		b := shell.Vertices[tri[(i+1)%3]].Point
		as, bs := c.Cmp(a.Z, z), c.Cmp(b.Z, z)
		if !c.OK {
			return nil, false, nil
		}
		if as == 0 {
			onPlane++
			add(Point{U: a.U, V: a.V})
		}
		if as*bs < 0 {
			t := c.Quo(c.Sub(z, a.Z), c.Sub(b.Z, a.Z))
			add(c.Interpolate(Point{U: a.U, V: a.V}, Point{U: b.U, V: b.V}, t))
			if !c.OK {
				return nil, false, nil
			}
		}
	}
	return points, onPlane == 3 || len(points) > 2, nil
}

func outputTraceInCapTriangle(trace []Point,
	capTriangle [3]int, shell OutputShell,
	arithmetic ...*Arithmetic) (*big.Rat, *big.Rat, bool) {
	c := ReplayArithmetic(arithmetic)
	lo, hi := big.NewRat(0, 1), big.NewRat(1, 1)
	if len(trace) == 1 {
		hi = big.NewRat(0, 1)
	}
	for k, v := range capTriangle {
		a, b := shell.Vertices[v].Point, shell.Vertices[capTriangle[(k+1)%3]].Point
		start := c.PointCross(Point{U: a.U, V: a.V},
			Point{U: b.U, V: b.V}, trace[0])
		end := start
		if len(trace) == 2 {
			end = c.PointCross(Point{U: a.U, V: a.V},
				Point{U: b.U, V: b.V}, trace[1])
		}
		if !c.OK {
			return nil, nil, false
		}
		if start.Sign() < 0 && end.Sign() < 0 {
			return nil, nil, false
		}
		if start.Sign() == end.Sign() || start.Sign() >= 0 && end.Sign() >= 0 {
			continue
		}
		t := c.Quo(start, c.Sub(start, end))
		if !c.OK {
			return nil, nil, false
		}
		if start.Sign() < 0 {
			lo = MaxRat(lo, t, c)
		} else {
			hi = MinRat(hi, t, c)
		}
		if c.Cmp(lo, hi) > 0 || !c.OK {
			return nil, nil, false
		}
	}
	return lo, hi, true
}
