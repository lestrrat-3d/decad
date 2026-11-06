package loftcylinder

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// MirrorLoftCapChamfer accepts exactly the two full outer cap loops of an
// unplaced, hole-free mirrored Loft join (agent-docs/loft-design.md §19) and
// chamfers them with one equal setback. It reports false, with no error, for
// any other receiver or selection. asymmetric is true when the caller asked
// for unequal setbacks, which this class refuses.
func MirrorLoftCapChamfer(ctx context.Context, env topo.Env, b *topo.Body, fp facetedrecord.Payload,
	edges []*topo.Edge, d, dDelta float64, asymmetric bool) (*topo.Body, bool, error) {
	record := fp.LoftMirrorJoin
	if record == nil || fp.Xform != r3.Identity() || !fp.ContactAudited || !b.Solid {
		return nil, false, nil
	}
	n := record.StationCount
	if n < 3 || len(fp.Verts) != 3*n || len(fp.VertexBound) != len(fp.Verts) || len(fp.Src) != len(fp.Tris) {
		return nil, false, nil
	}
	rings := [3][]int{make([]int, n), make([]int, n), make([]int, n)}
	for i := range n {
		rings[0][i], rings[1][i], rings[2][i] = i, n+i, 2*n+i
	}
	outer := [2][]int{rings[0], rings[2]}
	start := provenance.Ref{Producer: b.Origin.Producer, Role: provenance.RoleCapStart}
	end := provenance.Ref{Producer: b.Origin.Producer, Role: provenance.RoleCapEnd}
	if err := ChamferSelectsOuterLoops(b, start, end, edges, fp.Verts, outer); err != nil {
		// Any other selection is not this route's class; the receiver's own
		// refusal names it.
		return nil, false, nil //nolint:nilerr // a selection outside the class declines the route.
	}
	if asymmetric {
		return nil, true, fmt.Errorf("%w: asymmetric chamfer is not certified for a mirrored loft cap", decaderr.ErrUnsupported)
	}
	z0, z1 := fp.Verts[rings[0][0]].Z, fp.Verts[rings[2][0]].Z
	if err := BindLoftCapSetback(z0, z1, d, dDelta); err != nil {
		return nil, true, err
	}
	side := ChamferSideSource{Outer: outer, CapGroups: record.CapGroups, BoreGroup: -1, Z0: z0, Z1: z1}
	body, err := ChamferMirrorLoftCaps(ctx, env, fp, side, rings[1], d)
	if err != nil {
		return nil, true, fmt.Errorf("mirrored loft cap chamfer: %w", err)
	}
	return body, true, nil
}

// ChamferMirrorLoftCaps chamfers both outer cap loops of a held mirrored Loft
// join: the lower Loft's strip runs from its start ring side.Outer[0] to the
// shared middle ring, and the reflected Loft's from its start ring
// side.Outer[1] to the same ring, each the Loft's two-triangle cells. Each
// strip is clipped once, at the exact level setback inside its outer cap,
// and keeps every piece between that level and the middle ring, so the middle
// ring and the walls around it are exact subsets of the old walls. The insets,
// bands, caps, audits and bounds are ChamferLoftCaps'. The shell has no hole.
func ChamferMirrorLoftCaps(ctx context.Context, env topo.Env, fp facetedrecord.Payload,
	side ChamferSideSource, middle []int, setback float64) (*topo.Body, error) {
	const maxRimVertices = 4_096
	n := len(middle)
	if n < 3 || len(side.Outer[0]) != n || len(side.Outer[1]) != n || n > maxRimVertices || side.BoreGroup != -1 {
		return nil, fmt.Errorf("%w: mirrored loft cap chamfer lacks paired rings", decaderr.ErrUnsupported)
	}
	levels := [2]float64{side.Z0, side.Z1}
	for capIndex := range 2 {
		if !AuditCap(ctx, fp.Verts, fp.Tris, fp.Src, side.Outer[capIndex], side.CapGroups[capIndex],
			levels[capIndex], capIndex == 0) {
			return nil, fmt.Errorf("%w: mirrored loft cap %d does not tile its held outline", decaderr.ErrUnsupported, capIndex)
		}
	}
	lookup := make(map[[3]int]int, len(fp.Tris))
	for i, tri := range fp.Tris {
		group := fp.Src[i]
		if group == side.CapGroups[0] || group == side.CapGroups[1] {
			continue
		}
		key := chamferTriangleKey(tri)
		if _, duplicate := lookup[key]; duplicate {
			return nil, fmt.Errorf("%w: mirrored loft side repeats a triangle", decaderr.ErrUnsupported)
		}
		lookup[key] = group
	}
	mesh := &facetedrecord.JoinedMesh{Verts: append([]r3.Vec(nil), fp.Verts...),
		Groups: append([]facetedrecord.Group(nil), fp.Groups...)}
	bounds := append([]float64(nil), fp.VertexBound...)
	clipped := ChamferSideResult{}
	trueLevels := [2]*big.Rat{
		new(big.Rat).Add(proofarith.FloatRat(side.Z0), proofarith.FloatRat(setback)),
		new(big.Rat).Sub(proofarith.FloatRat(side.Z1), proofarith.FloatRat(setback)),
	}
	heldLevels := [2]float64{side.Z0 + setback, side.Z1 - setback}
	for capIndex, ring := range side.Outer {
		clipped.Rings[capIndex] = make([]int, 2*n)
		clipped.Exact[capIndex] = make([]proofarith.Xpt, 2*n)
		cells := make([][2]int, n)
		for i := range n {
			j := (i + 1) % n
			first, ok0 := lookup[chamferTriangleKey([3]int{ring[i], ring[j], middle[j]})]
			second, ok1 := lookup[chamferTriangleKey([3]int{ring[i], middle[j], middle[i]})]
			if !ok0 || !ok1 {
				return nil, fmt.Errorf("%w: mirrored loft side lost its two-triangle strip", decaderr.ErrUnsupported)
			}
			delete(lookup, chamferTriangleKey([3]int{ring[i], ring[j], middle[j]}))
			delete(lookup, chamferTriangleKey([3]int{ring[i], middle[j], middle[i]}))
			cells[i] = [2]int{first, second}
		}
		for i := range n {
			if i%64 == 0 && ctx.Err() != nil {
				return nil, ctx.Err()
			}
			for k, far := range [2]int{middle[i], middle[(i+1)%n]} {
				p, exact, round, err := ChamferLerp(fp.Verts[ring[i]], fp.Verts[far],
					trueLevels[capIndex], heldLevels[capIndex])
				if err != nil {
					return nil, err
				}
				clipped.ClipBound = max(clipped.ClipBound, round)
				clipped.Rings[capIndex][2*i+k] = len(mesh.Verts)
				clipped.Exact[capIndex][2*i+k] = exact
				mesh.Verts = append(mesh.Verts, p)
				bounds = append(bounds, proofbound.AbsSumUpper(fp.MeshBound, round))
			}
		}
		lo := clipped.Rings[capIndex]
		for i, cell := range cells {
			j := (i + 1) % n
			// The piece of [a_i, a_j, w_j] past the level, then of [a_i, w_j, w_i].
			mesh.Add([3]int{lo[2*i+1], lo[2*j], middle[j]}, cell[0])
			mesh.Add([3]int{lo[2*i], lo[2*i+1], middle[j]}, cell[1])
			mesh.Add([3]int{lo[2*i], middle[j], middle[i]}, cell[1])
		}
	}
	if len(lookup) != 0 {
		return nil, fmt.Errorf("%w: mirrored loft side has an unmatched facet", decaderr.ErrUnsupported)
	}
	return chamferHeldShell(ctx, env, fp, side, [2][]int{}, setback, mesh, bounds, clipped)
}
