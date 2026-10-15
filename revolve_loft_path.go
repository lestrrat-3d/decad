package decad

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// revolveExteriorPath inserts certified root contacts into the original
// point-loft ring. Only the path through a certified outside segment remains
// exposed.
func revolveExteriorPath(base facetedPayload, boundary revolveSectorBoundary,
	source *pointSectionSource, section revolveSectionCertificate) ([]revolveSectorNode, float64, error) {
	n := len(base.Verts) - 1
	if n < 6 || len(base.Tris) < n || len(base.Src) != len(base.Tris) {
		return nil, 0, fmt.Errorf("%w: point-loft fan is incomplete", ErrUnsupported)
	}
	groups := make([]int, n)
	seen := make([]bool, n)
	for i, tri := range base.Tris {
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
		seen[u], groups[u] = true, base.Src[i]
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
	for k := range boundary.contacts {
		var p r3.Vec
		var bound float64
		var root [2]float64
		var ok bool
		edgeIndex := boundary.contacts[k].segment
		if boundary.fitRoute {
			edgeIndex = boundary.contacts[k].segment
			p, bound, root, ok = revolveFitSeatPoint(source, boundary.contacts[k].fit, section)
		} else {
			p, bound, ok = revolveSeatPoint(source, boundary.contacts[k].line, section, base)
		}
		if !ok {
			return nil, 0, fmt.Errorf("%w: cone crossing has no finite spatial enclosure", ErrUnsupported)
		}
		best, edge := math.Inf(1), -1
		if boundary.fitRoute {
			edge, ok = revolveFitStationEdge(base, groups, source, edgeIndex,
				boundary.contacts[k].fit, root)
			if !ok {
				return nil, 0, fmt.Errorf("%w: cone crossing has no unique source fan cell", ErrUnsupported)
			}
			best = revolvePointSegmentDistance(p, base.Verts[edge+1], base.Verts[(edge+1)%n+1])
		} else {
			for i := range n {
				if groups[i] != edgeIndex {
					continue
				}
				distance := revolvePointSegmentDistance(p, base.Verts[i+1], base.Verts[(i+1)%n+1])
				if distance < best {
					best, edge = distance, i
				}
			}
		}
		allow := proofbound.AbsSumUpper(base.meshBound, bound)
		if edge < 0 || !(best <= allow) || !proofbound.FiniteVec(p) {
			return nil, 0, fmt.Errorf("%w: certified root contact is outside its source fan edge", ErrUnsupported)
		}
		// The fan bound admits this source edge for insertion. The contact's
		// distance from the exact source is the seat enclosure alone; Join
		// charges the fan's source-to-mesh bound separately.
		crossing[k] = struct {
			p     r3.Vec
			bound float64
			edge  int
		}{p: p, bound: bound, edge: edge}
		maxCrossBound = max(maxCrossBound, bound)
	}
	if crossing[0].edge == crossing[1].edge {
		return nil, 0, fmt.Errorf("%w: both root contacts occupy one fan edge", ErrUnsupported)
	}
	full := make([]revolveSectorNode, 0, n+2)
	var crossPosition [2]int
	for i := range n {
		full = append(full, revolveSectorNode{point: base.Verts[i+1], groupToNext: groups[i], crossing: -1})
		for k := range crossing {
			if crossing[k].edge == i {
				crossPosition[k] = len(full)
				full = append(full, revolveSectorNode{point: crossing[k].p,
					groupToNext: groups[i], crossing: k})
			}
		}
	}
	witness := boundary.outsideWitness
	if witness < 0 || witness >= len(source.entities) || witness == boundary.insideSegment {
		return nil, 0, fmt.Errorf("%w: trimmed Loft has no certified outside segment", ErrUnsupported)
	}
	forward := make([]revolveSectorNode, 0, len(full))
	for i := crossPosition[0]; ; i = (i + 1) % len(full) {
		forward = append(forward, full[i])
		if i == crossPosition[1] {
			break
		}
	}
	hasTip, hasRoot := false, false
	for i := 0; i+1 < len(forward); i++ {
		hasTip = hasTip || forward[i].groupToNext == witness
		hasRoot = hasRoot || forward[i].groupToNext == boundary.insideSegment
	}
	if !hasTip || hasRoot {
		other := make([]revolveSectorNode, 0, len(full))
		for i := crossPosition[1]; ; i = (i + 1) % len(full) {
			other = append(other, full[i])
			if i == crossPosition[0] {
				break
			}
		}
		forward = other
		hasTip, hasRoot = false, false
		for i := 0; i+1 < len(forward); i++ {
			hasTip = hasTip || forward[i].groupToNext == witness
			hasRoot = hasRoot || forward[i].groupToNext == boundary.insideSegment
		}
		if !hasTip || hasRoot {
			return nil, 0, fmt.Errorf("%w: source fan has no unique outside path", ErrUnsupported)
		}
	}
	if forward[0].crossing < 0 || forward[len(forward)-1].crossing < 0 {
		return nil, 0, fmt.Errorf("%w: source path lost its cone crossings", ErrUnsupported)
	}
	for _, i := range []int{0, len(forward) - 1} {
		forward[i].rootBoundary = true
		forward[i].angle = math.Atan2(forward[i].point.Z, forward[i].point.Y)
	}
	return forward, maxCrossBound, nil
}

func revolvePointSegmentDistance(p, a, b r3.Vec) float64 {
	d := b.Sub(a)
	den := d.Dot(d)
	if den <= 0 || math.IsInf(den, 0) {
		return math.Inf(1)
	}
	t := max(0, min(1, p.Sub(a).Dot(d)/den))
	return p.Sub(a.Add(d.Scale(t))).Len()
}

func revolveRootAngles(path []revolveSectorNode) (float64, float64, bool) {
	if len(path) < 2 || !path[0].rootBoundary || !path[len(path)-1].rootBoundary {
		return 0, 0, false
	}
	a, b := path[0].angle, path[len(path)-1].angle
	if !proofbound.FiniteVec(r3.Vec{X: a, Y: b}) || a == b || math.Abs(a-b) >= math.Pi {
		return 0, 0, false
	}
	return min(a, b), max(a, b), true
}
