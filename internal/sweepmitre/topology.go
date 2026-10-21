package sweepmitre

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/measurement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sweeparc"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file assembles agent-docs/sweep-design.md §16's mitred sweep into Table BM's
// held triangle set and topology, and publishes §16.6's four readings from
// the rational vertices. Every face is planar because §16.3 proves each wall
// quad planar exactly, so a wall is ONE face whose loop has four edges, held
// as two triangles split along the diagonal a prism's lateral quad uses.

// junctionConvex decides a junction edge's convexity the way
// loftmesh.JunctionConvex does for a loft rung, over the rational vertices:
// primary's outward-wound triangle against the apex of other's. Zero is a
// decided flat, non-convex edge.
func junctionConvex(exact []sweeparc.RatVec, primary, other [3]int, a, b int) bool {
	apex := loftmesh.JunctionApex(other, a, b)
	return OrientSign(exact[primary[0]], exact[primary[1]], exact[primary[2]], exact[apex]) < 0
}

// buildTopology builds Table BM's B-rep over the held vertex table:
// one Vertex per section vertex, one section edge per profile segment per
// section, one longitudinal edge per profile vertex per span, one four-edge
// face per wall and the two caps. Every loop is stated in the local winding
// and passed through topobuild.LoftLoopCoedges, which carries the whole-shell reversal
// into the directed boundary.
func buildTopology(ctx context.Context, env topo.Env, body *topo.Body, a Assembly, exact []sweeparc.RatVec, verts []r3.Vec, areas [][2]*big.Rat, delta float64) ([]*topo.Face, error) {
	ref := env.Ref
	vertexObjs := make([]*topo.Vertex, len(verts))
	for v, p := range verts {
		vertexObjs[v] = topobuild.LoftVertex(env.Alloc, p, delta)
	}
	next := make([]int, a.Stride)
	isOuter := make([]bool, a.Stride)
	for i, idx := range a.LoopIdx {
		for j, v := range idx {
			next[v] = idx[(j+1)%len(idx)]
			isOuter[v] = i == 0
		}
	}
	prev := make([]int, a.Stride)
	for v, n := range next {
		prev[n] = v
	}
	tri0 := func(k, v int) [3]int { return a.Tris[2*a.Wall(k, v)] }
	tri1 := func(k, v int) [3]int { return a.Tris[2*a.Wall(k, v)+1] }

	sec := make([][]*topo.Edge, a.Spans+1)
	lon := make([][]*topo.Edge, a.Spans)
	for k := range a.Spans + 1 {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		sec[k] = make([]*topo.Edge, a.Stride)
		for v := range a.Stride {
			from, to := a.At(k, v), a.At(k, next[v])
			convex := isOuter[v]
			if k > 0 && k < a.Spans {
				convex = junctionConvex(exact, tri0(k, v), tri1(k-1, v), from, to)
			}
			sec[k][v] = topobuild.LoftEdge(env.Alloc, vertexObjs, verts, from, to, convex, delta)
		}
		if k == a.Spans {
			break
		}
		lon[k] = make([]*topo.Edge, a.Stride)
		for v := range a.Stride {
			from, to := a.At(k, v), a.At(k+1, v)
			convex := junctionConvex(exact, tri0(k, prev[v]), tri1(k, v), from, to)
			lon[k][v] = topobuild.LoftEdge(env.Alloc, vertexObjs, verts, from, to, convex, delta)
		}
	}

	// Each face's area is the sum of its own triangles' enclosures.
	slotLo := make([]*big.Rat, len(a.Roles))
	slotHi := make([]*big.Rat, len(a.Roles))
	for slot := range a.Roles {
		slotLo[slot], slotHi[slot] = new(big.Rat), new(big.Rat)
	}
	for t, slot := range a.TriFace {
		slotLo[slot].Add(slotLo[slot], areas[t][0])
		slotHi[slot].Add(slotHi[slot], areas[t][1])
	}
	faceArea := func(slot int) (float64, float64) { return Enclosure(slotLo[slot], slotHi[slot]) }

	faces := make([]*topo.Face, 0, len(a.Roles))
	var capStartLoops, capEndLoops []*topo.Loop
	for i, idx := range a.LoopIdx {
		m := len(idx)
		capStartCo := make([]topo.CoEdge, m)
		capEndCo := make([]topo.CoEdge, m)
		for j, v := range idx {
			capStartCo[m-1-j] = topo.CoEdge{Edge: sec[0][v], Forward: false}
			capEndCo[j] = topo.CoEdge{Edge: sec[a.Spans][v], Forward: true}
		}
		capStartLoops = append(capStartLoops, topobuild.NewLoop(env.Alloc, topo.Loop{Outer: i == 0, CoEdges: topobuild.LoftLoopCoedges(capStartCo, a.Reversed)}))
		capEndLoops = append(capEndLoops, topobuild.NewLoop(env.Alloc, topo.Loop{Outer: i == 0, CoEdges: topobuild.LoftLoopCoedges(capEndCo, a.Reversed)}))
	}
	capStartSurf, err := topobuild.PlaneFromTriangle(verts, a.Tris[a.Walls])
	if err != nil {
		return nil, err
	}
	capEndSurf, err := topobuild.PlaneFromTriangle(verts, a.Tris[a.Walls+a.CapStartCount])
	if err != nil {
		return nil, err
	}
	startArea, startBound := faceArea(SlotCapStart)
	endArea, endBound := faceArea(SlotCapEnd)
	faces = append(faces,
		topobuild.NewFace(env.Alloc, topo.Face{
			Area:          startArea,
			AreaBound:     startBound,
			AxialDelta:    delta,
			HasAxialDelta: true,
			Surface:       capStartSurf,
			Origins:       []provenance.Ref{{Producer: ref, Role: provenance.RoleCapStart}},
			Body:          body,
			Loops:         capStartLoops,
		}),
		topobuild.NewFace(env.Alloc, topo.Face{
			Area:          endArea,
			AreaBound:     endBound,
			AxialDelta:    delta,
			HasAxialDelta: true,
			Surface:       capEndSurf,
			Origins:       []provenance.Ref{{Producer: ref, Role: provenance.RoleCapEnd}},
			Body:          body,
			Loops:         capEndLoops,
		}),
	)

	for k := range a.Spans {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		for _, idx := range a.LoopIdx {
			for _, v := range idx {
				w := a.Wall(k, v)
				surf, err := topobuild.PlaneFromTriangle(verts, a.Tris[2*w])
				if err != nil {
					return nil, err
				}
				slot := SlotWall0 + w
				area, bound := faceArea(slot)
				face := topobuild.NewFace(env.Alloc, topo.Face{
					Area:      area,
					AreaBound: bound,
					Surface:   surf,
					Origins:   []provenance.Ref{{Producer: ref, Role: a.Roles[slot]}},
					Body:      body,
				})
				face.Loops = []*topo.Loop{topobuild.NewLoop(env.Alloc, topo.Loop{Outer: true, CoEdges: topobuild.LoftLoopCoedges([]topo.CoEdge{
					{Edge: sec[k][v], Forward: true},
					{Edge: lon[k][next[v]], Forward: true},
					{Edge: sec[k+1][v], Forward: false},
					{Edge: lon[k][v], Forward: false},
				}, a.Reversed),
				})}
				faces = append(faces, face)
			}
		}
	}
	return faces, nil
}

// volumeReading publishes the exact rational volume as its nearest float. It
// is Exact when that float is the rational itself, and otherwise carries the
// one rounding, read exactly.
func volumeReading(vol6 *big.Rat) measurement.Measurement {
	vol := new(big.Rat).Quo(vol6, big.NewRat(6, 1))
	value, _ := vol.Float64()
	bound := proofarith.RationalFloatError(vol, value)
	return measurement.Measurement{
		Value:     units.CubicMillimeters(value),
		Exactness: measurement.ExactnessOf(bound),
		Bound:     units.CubicMillimeters(bound),
	}
}

// centroidReading publishes anchor + moment / (4·vol6), each coordinate
// rounded once, with the largest per-coordinate rounding read as a 3D radius.
func centroidReading(anchor sweeparc.RatVec, vol6 *big.Rat, moments [3]*big.Rat) (measurement.VecMeasurement, error) {
	value, bound, err := Centroid(anchor, vol6, moments)
	if err != nil {
		return measurement.VecMeasurement{}, err
	}
	return measurement.VecMeasurement{
		Value:     r3.NewVec(value[0], value[1], value[2]),
		Exactness: measurement.ExactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}, nil
}

// boundsReading is the per-axis extreme over the rational vertices, rounded
// outward: the box holds the exact body, and Bound is the largest outward
// rounding, read exactly.
func boundsReading(exact []sweeparc.RatVec) measurement.Box {
	minV, maxV, worst := Bounds(exact)
	return measurement.Box{
		Min:       r3.NewVec(minV[0], minV[1], minV[2]),
		Max:       r3.NewVec(maxV[0], maxV[1], maxV[2]),
		Exactness: measurement.ExactnessOf(worst),
		Bound:     units.Millimeters(worst),
	}
}
