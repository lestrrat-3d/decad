package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sweepmitre"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file assembles docs/sweep-design.md §16's mitred sweep into Table BM's
// held triangle set and topology, and publishes §16.6's four readings from
// the rational vertices. Every face is planar because §16.3 proves each wall
// quad planar exactly, so a wall is ONE face whose loop has four edges, held
// as two triangles split along the diagonal a prism's lateral quad uses.

// mitredAssembly is the held triangle set and the bookkeeping the topology
// reads. Face slot 0 is capStart, slot 1 capEnd, and slot 2 + w is wall w in
// (span, loop, segment) order, whose two triangles are tris[2w] and
// tris[2w+1]. tris[walls:walls+capStartCount] is capStart's triangulation and
// the rest capEnd's. Vertex v of section k is index k·stride + v.
type mitredAssembly struct {
	tris          [][3]int
	triFace       []int
	roles         []string
	walls         int
	capStartCount int
	reversed      bool
	spans         int
	stride        int
	loopIdx       [][]int
}

const (
	mitredSlotCapStart = 0
	mitredSlotCapEnd   = 1
	mitredSlotWall0    = 2
)

func (a mitredAssembly) at(k, v int) int { return k*a.stride + v }

// wall is the wall index of span k over the segment that starts at section
// vertex v.
func (a mitredAssembly) wall(k, v int) int { return k*a.stride + v }

// assembleMitredSweep emits Table BM's triangles in their local winding:
// each wall's two triangles in the order a prism's lateral quad uses, then
// capStart's triangulation reversed and capEnd's retained, which is the same
// cap seeding docs/loft-design.md §5 uses. capEnd reuses capStart's
// triangulation under the section map (this file's sibling's doc comment).
func assembleMitredSweep(c mitredConstruction, stride int) mitredAssembly {
	a := mitredAssembly{
		roles:   []string{roleCapStart, roleCapEnd},
		spans:   len(c.sections) - 1,
		stride:  stride,
		loopIdx: c.loopIdx,
	}
	for k := range a.spans {
		for i, idx := range c.loopIdx {
			m := len(idx)
			for j := range m {
				slot := len(a.roles)
				a.roles = append(a.roles, fmt.Sprintf("side(%d,%d,%d)", k, i, j))
				b0, b1 := a.at(k, idx[j]), a.at(k, idx[(j+1)%m])
				t0, t1 := a.at(k+1, idx[j]), a.at(k+1, idx[(j+1)%m])
				a.tris = append(a.tris, [3]int{b0, b1, t1}, [3]int{b0, t1, t0})
				a.triFace = append(a.triFace, slot, slot)
			}
		}
	}
	a.walls = len(a.tris)
	for _, t := range c.capTris {
		a.tris = append(a.tris, [3]int{a.at(0, t[0]), a.at(0, t[2]), a.at(0, t[1])})
		a.triFace = append(a.triFace, mitredSlotCapStart)
	}
	a.capStartCount = len(c.capTris)
	for _, t := range c.capTris {
		a.tris = append(a.tris, [3]int{a.at(a.spans, t[0]), a.at(a.spans, t[1]), a.at(a.spans, t[2])})
		a.triFace = append(a.triFace, mitredSlotCapEnd)
	}
	return a
}

func mitredVolumeMoments(exact []sweepRatVec, tris [][3]int, anchor sweepRatVec) (*big.Rat, [3]*big.Rat) {
	return sweepmitre.VolumeMoments(exact, tris, anchor)
}

func mitredTriangleAreas(ctx context.Context, exact []sweepRatVec, tris [][3]int) ([][2]*big.Rat, error) {
	return sweepmitre.TriangleAreas(ctx, exact, tris)
}

func mitredEnclosure(lo, hi *big.Rat) (float64, float64) {
	return sweepmitre.Enclosure(lo, hi)
}

func mitredOrientSign(a, b, c, d sweepRatVec) int {
	return sweepmitre.OrientSign(a, b, c, d)
}

// mitredJunctionConvex decides a junction edge's convexity the way
// junctionConvex does for a loft rung, over the rational vertices: primary's
// outward-wound triangle against the apex of other's. Zero is a decided flat,
// non-convex edge.
func mitredJunctionConvex(exact []sweepRatVec, primary, other [3]int, a, b int) bool {
	apex := loftmesh.JunctionApex(other, a, b)
	return mitredOrientSign(exact[primary[0]], exact[primary[1]], exact[primary[2]], exact[apex]) < 0
}

// buildMitredTopology builds Table BM's B-rep over the held vertex table:
// one Vertex per section vertex, one section edge per profile segment per
// section, one longitudinal edge per profile vertex per span, one four-edge
// face per wall and the two caps. Every loop is stated in the local winding
// and passed through loftLoopCoedges, which carries the whole-shell reversal
// into the directed boundary.
func buildMitredTopology(ctx context.Context, body *Body, ref producerID, a mitredAssembly, exact []sweepRatVec, verts []r3.Vec, areas [][2]*big.Rat, delta float64) ([]*Face, error) {
	vertexObjs := make([]*Vertex, len(verts))
	for v, p := range verts {
		vertexObjs[v] = loftVertex(p, delta)
	}
	next := make([]int, a.stride)
	isOuter := make([]bool, a.stride)
	for i, idx := range a.loopIdx {
		for j, v := range idx {
			next[v] = idx[(j+1)%len(idx)]
			isOuter[v] = i == 0
		}
	}
	prev := make([]int, a.stride)
	for v, n := range next {
		prev[n] = v
	}
	tri0 := func(k, v int) [3]int { return a.tris[2*a.wall(k, v)] }
	tri1 := func(k, v int) [3]int { return a.tris[2*a.wall(k, v)+1] }

	sec := make([][]*Edge, a.spans+1)
	lon := make([][]*Edge, a.spans)
	for k := range a.spans + 1 {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		sec[k] = make([]*Edge, a.stride)
		for v := range a.stride {
			from, to := a.at(k, v), a.at(k, next[v])
			convex := isOuter[v]
			if k > 0 && k < a.spans {
				convex = mitredJunctionConvex(exact, tri0(k, v), tri1(k-1, v), from, to)
			}
			sec[k][v] = loftEdge(vertexObjs, verts, from, to, convex, delta)
		}
		if k == a.spans {
			break
		}
		lon[k] = make([]*Edge, a.stride)
		for v := range a.stride {
			from, to := a.at(k, v), a.at(k+1, v)
			convex := mitredJunctionConvex(exact, tri0(k, prev[v]), tri1(k, v), from, to)
			lon[k][v] = loftEdge(vertexObjs, verts, from, to, convex, delta)
		}
	}

	// Each face's area is the sum of its own triangles' enclosures.
	slotLo := make([]*big.Rat, len(a.roles))
	slotHi := make([]*big.Rat, len(a.roles))
	for slot := range a.roles {
		slotLo[slot], slotHi[slot] = new(big.Rat), new(big.Rat)
	}
	for t, slot := range a.triFace {
		slotLo[slot].Add(slotLo[slot], areas[t][0])
		slotHi[slot].Add(slotHi[slot], areas[t][1])
	}
	faceArea := func(slot int) (float64, float64) { return mitredEnclosure(slotLo[slot], slotHi[slot]) }

	faces := make([]*Face, 0, len(a.roles))
	var capStartLoops, capEndLoops []*Loop
	for i, idx := range a.loopIdx {
		m := len(idx)
		capStartCo := make([]coedge, m)
		capEndCo := make([]coedge, m)
		for j, v := range idx {
			capStartCo[m-1-j] = coedge{edge: sec[0][v], forward: false}
			capEndCo[j] = coedge{edge: sec[a.spans][v], forward: true}
		}
		capStartLoops = append(capStartLoops, &Loop{outer: i == 0, coedges: loftLoopCoedges(capStartCo, a.reversed)})
		capEndLoops = append(capEndLoops, &Loop{outer: i == 0, coedges: loftLoopCoedges(capEndCo, a.reversed)})
	}
	capStartSurf, err := planeFromTriangle(verts, a.tris[a.walls])
	if err != nil {
		return nil, err
	}
	capEndSurf, err := planeFromTriangle(verts, a.tris[a.walls+a.capStartCount])
	if err != nil {
		return nil, err
	}
	startArea, startBound := faceArea(mitredSlotCapStart)
	endArea, endBound := faceArea(mitredSlotCapEnd)
	faces = append(faces,
		&Face{
			surface: capStartSurf, loops: capStartLoops,
			origins: []FeatureRef{{producer: ref, Role: roleCapStart}},
			body:    body, area: startArea, areaBound: startBound,
			axialDelta: delta, hasAxialDelta: true,
		},
		&Face{
			surface: capEndSurf, loops: capEndLoops,
			origins: []FeatureRef{{producer: ref, Role: roleCapEnd}},
			body:    body, area: endArea, areaBound: endBound,
			axialDelta: delta, hasAxialDelta: true,
		},
	)

	for k := range a.spans {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		for _, idx := range a.loopIdx {
			for _, v := range idx {
				w := a.wall(k, v)
				surf, err := planeFromTriangle(verts, a.tris[2*w])
				if err != nil {
					return nil, err
				}
				slot := mitredSlotWall0 + w
				area, bound := faceArea(slot)
				face := &Face{
					surface:   surf,
					origins:   []FeatureRef{{producer: ref, Role: a.roles[slot]}},
					body:      body,
					area:      area,
					areaBound: bound,
				}
				face.loops = []*Loop{{outer: true, coedges: loftLoopCoedges([]coedge{
					{edge: sec[k][v], forward: true},
					{edge: lon[k][next[v]], forward: true},
					{edge: sec[k+1][v], forward: false},
					{edge: lon[k][v], forward: false},
				}, a.reversed)}}
				faces = append(faces, face)
			}
		}
	}
	return faces, nil
}

// mitredVolume publishes the exact rational volume as its nearest float. It
// is Exact when that float is the rational itself, and otherwise carries the
// one rounding, read exactly.
func mitredVolume(vol6 *big.Rat) Measurement {
	vol := new(big.Rat).Quo(vol6, big.NewRat(6, 1))
	value, _ := vol.Float64()
	bound := proofarith.RationalFloatError(vol, value)
	return Measurement{
		Value:     units.CubicMillimeters(value),
		Exactness: exactnessOf(bound),
		Bound:     units.CubicMillimeters(bound),
	}
}

// mitredCentroid publishes anchor + moment / (4·vol6), each coordinate
// rounded once, with the largest per-coordinate rounding read as a 3D radius.
func mitredCentroid(anchor sweepRatVec, vol6 *big.Rat, moments [3]*big.Rat) (VecMeasurement, error) {
	value, bound, err := sweepmitre.Centroid(anchor, vol6, moments)
	if err != nil {
		return VecMeasurement{}, err
	}
	return VecMeasurement{
		Value:     r3.NewVec(value[0], value[1], value[2]),
		Exactness: exactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}, nil
}

// mitredBounds is the per-axis extreme over the rational vertices, rounded
// outward: the box holds the exact body, and Bound is the largest outward
// rounding, read exactly.
func mitredBounds(exact []sweepRatVec) Box {
	minV, maxV, worst := sweepmitre.Bounds(exact)
	return Box{
		Min:       r3.NewVec(minV[0], minV[1], minV[2]),
		Max:       r3.NewVec(maxV[0], maxV[1], maxV[2]),
		Exactness: exactnessOf(worst),
		Bound:     units.Millimeters(worst),
	}
}
