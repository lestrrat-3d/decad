package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sweepmitre"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file assembles docs/sweep-design.md §16's mitred sweep into Table BM's
// held triangle set and topology, and publishes §16.6's four readings from
// the rational vertices. Every face is planar because §16.3 proves each wall
// quad planar exactly, so a wall is ONE face whose loop has four edges, held
// as two triangles split along the diagonal a prism's lateral quad uses.

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
// loftmesh.JunctionConvex does for a loft rung, over the rational vertices: primary's
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
func buildMitredTopology(ctx context.Context, body *Body, ref producerID, a sweepmitre.Assembly, exact []sweepRatVec, verts []r3.Vec, areas [][2]*big.Rat, delta float64) ([]*Face, error) {
	vertexObjs := make([]*Vertex, len(verts))
	for v, p := range verts {
		vertexObjs[v] = loftVertex(p, delta)
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

	sec := make([][]*Edge, a.Spans+1)
	lon := make([][]*Edge, a.Spans)
	for k := range a.Spans + 1 {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		sec[k] = make([]*Edge, a.Stride)
		for v := range a.Stride {
			from, to := a.At(k, v), a.At(k, next[v])
			convex := isOuter[v]
			if k > 0 && k < a.Spans {
				convex = mitredJunctionConvex(exact, tri0(k, v), tri1(k-1, v), from, to)
			}
			sec[k][v] = loftEdge(vertexObjs, verts, from, to, convex, delta)
		}
		if k == a.Spans {
			break
		}
		lon[k] = make([]*Edge, a.Stride)
		for v := range a.Stride {
			from, to := a.At(k, v), a.At(k+1, v)
			convex := mitredJunctionConvex(exact, tri0(k, prev[v]), tri1(k, v), from, to)
			lon[k][v] = loftEdge(vertexObjs, verts, from, to, convex, delta)
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
	faceArea := func(slot int) (float64, float64) { return mitredEnclosure(slotLo[slot], slotHi[slot]) }

	faces := make([]*Face, 0, len(a.Roles))
	var capStartLoops, capEndLoops []*Loop
	for i, idx := range a.LoopIdx {
		m := len(idx)
		capStartCo := make([]topo.CoEdge, m)
		capEndCo := make([]topo.CoEdge, m)
		for j, v := range idx {
			capStartCo[m-1-j] = topo.CoEdge{Edge: edgeNode(sec[0][v]), Forward: false}
			capEndCo[j] = topo.CoEdge{Edge: edgeNode(sec[a.Spans][v]), Forward: true}
		}
		capStartLoops = append(capStartLoops, &Loop{n: topo.Loop{Outer: i == 0, CoEdges: loftLoopCoedges(capStartCo, a.Reversed)}})
		capEndLoops = append(capEndLoops, &Loop{n: topo.Loop{Outer: i == 0, CoEdges: loftLoopCoedges(capEndCo, a.Reversed)}})
	}
	capStartSurf, err := planeFromTriangle(verts, a.Tris[a.Walls])
	if err != nil {
		return nil, err
	}
	capEndSurf, err := planeFromTriangle(verts, a.Tris[a.Walls+a.CapStartCount])
	if err != nil {
		return nil, err
	}
	startArea, startBound := faceArea(sweepmitre.SlotCapStart)
	endArea, endBound := faceArea(sweepmitre.SlotCapEnd)
	faces = append(faces,
		bornFace(&Face{loops: capStartLoops, n: topo.Face{
			Area:          startArea,
			AreaBound:     startBound,
			AxialDelta:    delta,
			HasAxialDelta: true,
			Surface:       capStartSurf,
			Origins:       []provenance.Ref{{Producer: ref, Role: provenance.RoleCapStart}},
			Body:          bodyNode(body),
		},
		}),
		bornFace(&Face{loops: capEndLoops, n: topo.Face{
			Area:          endArea,
			AreaBound:     endBound,
			AxialDelta:    delta,
			HasAxialDelta: true,
			Surface:       capEndSurf,
			Origins:       []provenance.Ref{{Producer: ref, Role: provenance.RoleCapEnd}},
			Body:          bodyNode(body),
		},
		}),
	)

	for k := range a.Spans {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		for _, idx := range a.LoopIdx {
			for _, v := range idx {
				w := a.Wall(k, v)
				surf, err := planeFromTriangle(verts, a.Tris[2*w])
				if err != nil {
					return nil, err
				}
				slot := sweepmitre.SlotWall0 + w
				area, bound := faceArea(slot)
				face := bornFace(&Face{
					n: topo.Face{
						Area:      area,
						AreaBound: bound,
						Surface:   surf,
						Origins:   []provenance.Ref{{Producer: ref, Role: a.Roles[slot]}},
						Body:      bodyNode(body),
					},
				})
				face.loops = []*Loop{{n: topo.Loop{Outer: true, CoEdges: loftLoopCoedges([]topo.CoEdge{
					{Edge: edgeNode(sec[k][v]), Forward: true},
					{Edge: edgeNode(lon[k][next[v]]), Forward: true},
					{Edge: edgeNode(sec[k+1][v]), Forward: false},
					{Edge: edgeNode(lon[k][v]), Forward: false},
				}, a.Reversed),
				}}}
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
