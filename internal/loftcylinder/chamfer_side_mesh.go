package loftcylinder

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ChamferSideSource carries only the held side-strip readings needed by this mesh stage.
type ChamferSideSource struct {
	Outer     [2][]int
	CapGroups [2]int
	BoreGroup int
	Z0, Z1    float64
}

// ChamferSideResult returns the old strip's exact clipped rings and mesh rounding.
type ChamferSideResult struct {
	Groups    [][2]int
	ClipBound float64
	Rings     [2][]int
	Exact     [2][]proofarith.Xpt
}

func chamferTriangleKey(tri [3]int) [3]int {
	sort.Ints(tri[:])
	return tri
}

// The joined shell is the paired two-triangle loft strip of its recorded cap
// rings. The bore, when present, owns a separate wall group and stays intact.
func ChamferSideGroups(fp facetedrecord.Payload, source ChamferSideSource) ([][2]int, error) {
	n := len(source.Outer[0])
	if n < 3 || len(source.Outer[1]) != n || len(fp.Src) != len(fp.Tris) {
		return nil, fmt.Errorf("%w: loft/cylinder side lacks paired cap rings", decaderr.ErrUnsupported)
	}
	lookup := make(map[[3]int]int, 2*n)
	for i, tri := range fp.Tris {
		group := fp.Src[i]
		if group == source.CapGroups[0] || group == source.CapGroups[1] || group == source.BoreGroup {
			continue
		}
		key := chamferTriangleKey(tri)
		if _, duplicate := lookup[key]; duplicate {
			return nil, fmt.Errorf("%w: loft/cylinder side repeats a triangle", decaderr.ErrUnsupported)
		}
		lookup[key] = group
	}
	groups := make([][2]int, n)
	for i := range n {
		j := (i + 1) % n
		b0, b1 := source.Outer[0][i], source.Outer[0][j]
		t0, t1 := source.Outer[1][i], source.Outer[1][j]
		for _, index := range [4]int{b0, b1, t0, t1} {
			if index < 0 || index >= len(fp.Verts) {
				return nil, fmt.Errorf("%w: loft/cylinder ring index is invalid", decaderr.ErrUnsupported)
			}
		}
		first := chamferTriangleKey([3]int{b0, b1, t1})
		second := chamferTriangleKey([3]int{b0, t1, t0})
		g0, ok0 := lookup[first]
		g1, ok1 := lookup[second]
		if !ok0 || !ok1 {
			return nil, fmt.Errorf("%w: loft/cylinder side lost its two-triangle strip", decaderr.ErrUnsupported)
		}
		groups[i] = [2]int{g0, g1}
		delete(lookup, first)
		delete(lookup, second)
	}
	if len(lookup) != 0 {
		return nil, fmt.Errorf("%w: loft/cylinder side has an unmatched facet", decaderr.ErrUnsupported)
	}
	return groups, nil
}

// Side rings alternate each source ruling with its old triangle diagonal.
// That is the exact horizontal section of the old two-triangle strip before
// each intersection point is rounded into the result mesh.
func ChamferClipSide(ctx context.Context, mesh *facetedrecord.JoinedMesh,
	vertexBound *[]float64, fp facetedrecord.Payload, source ChamferSideSource,
	groups [][2]int, d float64) (ChamferSideResult, error) {
	n := len(groups)
	result := ChamferSideResult{Groups: groups}
	levels := [2]float64{source.Z0 + d, source.Z1 - d}
	trueLevels := [2]*big.Rat{
		new(big.Rat).Add(proofarith.FloatRat(source.Z0), proofarith.FloatRat(d)),
		new(big.Rat).Sub(proofarith.FloatRat(source.Z1), proofarith.FloatRat(d)),
	}
	for endCap, level := range levels {
		result.Rings[endCap] = make([]int, 2*n)
		result.Exact[endCap] = make([]proofarith.Xpt, 2*n)
		for i := range n {
			if i%64 == 0 && ctx.Err() != nil {
				return result, ctx.Err()
			}
			b := source.Outer[0][i]
			t := source.Outer[1][i]
			next := source.Outer[1][(i+1)%n]
			for k, end := range [2]int{t, next} {
				p, exact, round, err := ChamferLerp(fp.Verts[b], fp.Verts[end],
					trueLevels[endCap], level)

				if err != nil {
					return result, err
				}
				result.ClipBound = max(result.ClipBound, round)
				result.Rings[endCap][2*i+k] = len(mesh.Verts)
				result.Exact[endCap][2*i+k] = exact
				mesh.Verts = append(mesh.Verts, p)
				*vertexBound = append(*vertexBound, proofbound.AbsSumUpper(fp.MeshBound, round))
			}
		}
	}
	lo, hi := result.Rings[0], result.Rings[1]
	for i, group := range groups {
		j := (i + 1) % n
		// Original [b_i,b_j,t_j].
		mesh.Add([3]int{lo[2*i+1], lo[2*j], hi[2*j]}, group[0])
		mesh.Add([3]int{lo[2*i+1], hi[2*j], hi[2*i+1]}, group[0])
		// Original [b_i,t_j,t_i].
		mesh.Add([3]int{lo[2*i], lo[2*i+1], hi[2*i+1]}, group[1])
		mesh.Add([3]int{lo[2*i], hi[2*i+1], hi[2*i]}, group[1])
	}
	return result, nil
}

func ChamferContactAudit(ctx context.Context, fp facetedrecord.Payload,
	mesh *facetedrecord.JoinedMesh, source ChamferSideSource, side ChamferSideResult,
	bands [2][2]int, sideEdges [2]map[[2]int]struct{},
	xverts []proofarith.Xpt, setback float64) error {
	oldData := loftmesh.NewLoftAuditData(fp.Verts, fp.Tris)
	oldParity := meshbool.NewParityMesh(fp.Verts, fp.Tris)
	allTris := make([]int, len(fp.Tris))
	for i := range allTris {
		allTris[i] = i
	}
	oldBoxes := make([][2]r3.Vec, len(fp.Tris))
	for i, tri := range fp.Tris {
		oldBoxes[i] = meshbool.TriBox(fp.Verts, tri)
	}
	for endCap, band := range bands {
		if band[0] == band[1] {
			continue
		}
		pointTouches := make(map[string]struct{})
		segmentEnds := make(map[string]struct{})
		sideVertexKeys := make(map[string]struct{}, len(side.Rings[endCap]))
		for _, vertex := range side.Rings[endCap] {
			sideVertexKeys[xverts[vertex].Key()] = struct{}{}
		}
		tri := mesh.Tris[band[0]]
		witness := meshbool.XCentroid(xverts[tri[0]], xverts[tri[1]], xverts[tri[2]])
		inside, on, parityErr := meshbool.MeshParityPreparedContext(ctx, witness, oldParity, allTris)
		if parityErr != nil {
			return parityErr
		}
		if !inside || on {
			return fmt.Errorf("%w: ideal cap band is not inside the old solid", decaderr.ErrUnsupported)
		}
		level := source.Z0 + setback
		trueLevel := new(big.Rat).Add(proofarith.FloatRat(source.Z0), proofarith.FloatRat(setback))
		if endCap == 1 {
			level = source.Z1 - setback
			trueLevel = new(big.Rat).Sub(proofarith.FloatRat(source.Z1), proofarith.FloatRat(setback))
		}
		for i := band[0]; i < band[1]; i++ {
			if i%64 == 0 {
				if err := ctx.Err(); err != nil {
					return err
				}
			}
			box := meshbool.TriBox(mesh.Verts, mesh.Tris[i])
			pad := max(2*side.ClipBound, 1e-12)
			box[0] = r3.Vec{X: math.Nextafter(box[0].X-pad, math.Inf(-1)),
				Y: math.Nextafter(box[0].Y-pad, math.Inf(-1)), Z: math.Nextafter(box[0].Z-pad, math.Inf(-1))}
			box[1] = r3.Vec{X: math.Nextafter(box[1].X+pad, math.Inf(1)),
				Y: math.Nextafter(box[1].Y+pad, math.Inf(1)), Z: math.Nextafter(box[1].Z+pad, math.Inf(1))}
			tri := mesh.Tris[i]
			var xa [3]proofarith.Xpt
			for k, index := range tri {
				xa[k] = xverts[index]
			}
			na := proofarith.Xcross(proofarith.Xsub(xa[1], xa[0]), proofarith.Xsub(xa[2], xa[0]))
			fa := [3]r3.Vec{xa[0].Vec(), xa[1].Vec(), xa[2].Vec()}
			bandPlane := loftmesh.NewLoftExactPlane(xa[0], na)
			for j := range fp.Tris {
				if fp.Src[j] == source.CapGroups[0] || fp.Src[j] == source.CapGroups[1] ||
					fp.Src[j] == source.BoreGroup || !meshbool.BoxesOverlap(box, oldBoxes[j]) {
					continue
				}
				signsA := loftmesh.TrianglePlaneSigns(oldData.Planes[j], xa)
				signsB := loftmesh.TrianglePlaneSigns(bandPlane, oldData.Xtris[j])
				contact, err := meshbool.TriTriClassifyCore(fa, oldData.Corners[j], xa,
					oldData.Xtris[j], na, oldData.Norms[j], nil, nil, &signsA, &signsB,
					false, r3.Vec{}, r3.Vec{}, false)
				if err != nil {
					return err
				}
				if contact.Kind == meshbool.ContactPoint {
					pointTouches[contact.P0.Key()] = struct{}{}
					_, atSideVertex := sideVertexKeys[contact.P0.Key()]
					if meshbool.RatCoordOf(contact.P0, 2).Cmp(trueLevel) != 0 ||
						!contact.P0OnA ||
						(!atSideVertex && !ChamferContactOnSide(
							contact.P0, tri, xverts, sideEdges[endCap])) {
						return fmt.Errorf("%w: ideal cap band touches an old wall away from its side ring: cap=%d band=%d old=%d point=%v tri=%v level=%g",
							decaderr.ErrUnsupported, endCap, i, j, contact.P0.Vec(), tri, level)
					}
					continue
				}
				if contact.Kind == meshbool.ContactRegion {
					return fmt.Errorf("%w: ideal cap band overlaps an old wall", decaderr.ErrUnsupported)
				}
				if contact.Kind != meshbool.ContactSegment {
					continue
				}
				segmentEnds[contact.P0.Key()] = struct{}{}
				segmentEnds[contact.P1.Key()] = struct{}{}
				if contact.EdgeA < 0 ||
					meshbool.RatCoordOf(contact.P0, 2).Cmp(trueLevel) != 0 ||
					meshbool.RatCoordOf(contact.P1, 2).Cmp(trueLevel) != 0 {
					return fmt.Errorf("%w: ideal cap band crosses an old wall away from its side ring", decaderr.ErrUnsupported)
				}
				a, b := tri[contact.EdgeA], tri[(contact.EdgeA+1)%3]
				if _, ok := sideEdges[endCap][ChamferEdge(a, b)]; !ok {
					return fmt.Errorf("%w: ideal cap band touches an old wall on a non-rim edge", decaderr.ErrUnsupported)
				}
			}
		}
		for point := range pointTouches {
			if _, segment := segmentEnds[point]; !segment {
				return fmt.Errorf("%w: loft/cylinder cap has an isolated wall tangency", decaderr.ErrUnsupported)
			}
		}
	}
	return nil
}

// The top of a private one-cap rewrite is an exact subset of the old side.
// The split ring came from the same two-triangle strip as the middle side;
// joining it to the original top stations leaves the middle cap unchanged.
func ChamferRestoreTop(mesh *facetedrecord.JoinedMesh, fp facetedrecord.Payload,
	source ChamferSideSource, groups [][2]int, ring []int) error {
	n := len(groups)
	if len(ring) != 2*n || len(source.Outer[1]) != n {
		return fmt.Errorf("%w: one-cap chamfer lost its top side ring", decaderr.ErrUnsupported)
	}
	for i, pair := range groups {
		j := (i + 1) % n
		topI, topJ := source.Outer[1][i], source.Outer[1][j]
		mesh.Add([3]int{ring[2*i+1], ring[2*j], topJ}, pair[0])
		mesh.Add([3]int{ring[2*i], ring[2*i+1], topJ}, pair[1])
		mesh.Add([3]int{ring[2*i], topJ, topI}, pair[1])
	}
	for i, group := range fp.Src {
		if group == source.CapGroups[1] {
			mesh.Add(fp.Tris[i], group)
		}
	}
	return nil
}
