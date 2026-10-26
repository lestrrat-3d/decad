package loftcylinder

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetedtopology"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// MirrorChamferFrame returns the reflection through the authenticated source
// Loft's end frame of record's Join, and that frame's middle height.
func MirrorChamferFrame(record *facetedrecord.LoftMirrorCylinderBoreRecord) (r3.Transform, float64, error) {
	if record == nil || record.Join == nil || record.Join.Source == nil ||
		record.Join.Source.Lower == nil {
		return r3.Transform{}, 0, fmt.Errorf("%w: mirrored cap chamfer lost its Loft frame", decaderr.ErrUnsupported)
	}
	loft, ok := record.Join.Source.Lower.Payload.(loftrecord.Payload)
	if !ok || !loft.AuthenticatedReconstruction {
		return r3.Transform{}, 0, fmt.Errorf("%w: mirrored cap chamfer lost its Loft frame", decaderr.ErrUnsupported)
	}
	reflection, err := r3.Reflection(loft.Frame1)
	if err != nil {
		return r3.Transform{}, 0, fmt.Errorf("%w: mirrored cap chamfer has no source reflection: %s",
			decaderr.ErrUnsupported, err)
	}
	return reflection, loft.Frame1.Origin().Z, nil
}

// MirrorChamferReflectionReserve reserves the same certified rounding
// envelope used by placed faceted meshes for reflecting the chamfered lower
// half, with a second copy for any embedding correction. AssembleMirrorChamfer
// checks the reflected half's measured bound against it.
func MirrorChamferReflectionReserve(lower facetedrecord.Payload,
	record *facetedrecord.LoftMirrorCylinderBoreRecord, setback float64) (float64, error) {
	reflection, _, err := MirrorChamferFrame(record)
	if err != nil {
		return 0, err
	}
	maxInput := 0.0
	for _, v := range lower.Verts {
		maxInput = max(maxInput, math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
	}
	// The inset stays inside the old cap; adding the setback also covers the
	// new side stations and their held rounding.
	maxInput = proofbound.AbsSumUpper(maxInput, setback)
	tr := reflection.Translation()
	maxTrans := max(math.Abs(tr.X), math.Abs(tr.Y), math.Abs(tr.Z))
	round := proofbound.RigidRoundAllow(maxInput, maxTrans)
	reserve := proofbound.AbsSumUpper(round, round)
	if reserve <= 0 || reserve >= 0.1 || proofbound.IsNonFinite(reserve) {
		return 0, fmt.Errorf("%w: mirrored cap chamfer reflection has no finite allowance", decaderr.ErrUnsupported)
	}
	return reserve, nil
}

// MirrorChamfer is the assembled mirrored cap Chamfer shell: its proved
// payload, the result's exterior cap groups (bottom, top) and the exact
// blank cap levels those groups tile.
type MirrorChamfer struct {
	Payload   facetedrecord.Payload
	CapGroups [2]int
	CapLevels [2]float64
}

// AssembleMirrorChamfer joins the chamfered lower half lower, built by
// ChamferBottomCap with half, and upper, its reflection through
// MirrorChamferFrame, into the Chamfer of old, the bored mirrored Join that
// record describes. The reflection must stay within reflectionReserve, and
// the upper cap bridge within bridgeReserve. It removes the opposed middle
// annuli, welds the common middle vertices, audits the seam, the vertex
// links, both exterior annuli and the unchanged bore wall, and proves the
// result's boundary, volume, area and diameter bounds under 0.1 mm. The
// returned payload's cap groups have no origins or surface tags yet.
func AssembleMirrorChamfer(ctx context.Context, old, lower, upper facetedrecord.Payload,
	record *facetedrecord.LoftMirrorCylinderBoreRecord, half ChamferHalf,
	reflectionReserve, bridgeReserve, setback float64) (MirrorChamfer, error) {
	if !lower.ContactAudited || lower.Xform != r3.Identity() {
		return MirrorChamfer{}, fmt.Errorf("%w: mirrored cap chamfer lost its audited lower half", decaderr.ErrUnsupported)
	}
	_, middle, err := MirrorChamferFrame(record)
	if err != nil {
		return MirrorChamfer{}, err
	}
	if !upper.ContactAudited || len(lower.Verts) != len(upper.Verts) ||
		len(lower.Tris) != len(upper.Tris) || len(lower.Src) != len(upper.Src) ||
		half.CapGroups[0] < 0 || half.CapGroups[1] < 0 || half.BoreGroup < 0 {
		return MirrorChamfer{}, fmt.Errorf("%w: mirrored cap chamfer halves have no matching held shell",
			decaderr.ErrUnsupported)
	}
	if upper.MeshBound > proofbound.AbsSumUpper(lower.MeshBound, reflectionReserve) {
		return MirrorChamfer{}, fmt.Errorf("%w: reflected cap chamfer exceeds its %g mm placement reserve "+
			"(lower %g, upper %g)", decaderr.ErrUnsupported, reflectionReserve, lower.MeshBound, upper.MeshBound)
	}
	blank, ok := record.Join.Blank.Payload.(prismrecord.Payload)
	if !ok || !MirrorBlank(blank) {
		return MirrorChamfer{}, fmt.Errorf("%w: mirrored cap chamfer lost its exact blank", decaderr.ErrUnsupported)
	}
	bridgeReach, err := mirrorChamferUpperCapBridge(ctx, old, upper, record, blank, half, setback, middle)
	if err != nil {
		return MirrorChamfer{}, err
	}
	if bridgeReach > bridgeReserve {
		return MirrorChamfer{}, fmt.Errorf("%w: reflected cap bridge %g exceeds its %g mm reserve",
			decaderr.ErrUnsupported, bridgeReach, bridgeReserve)
	}
	midVertices := make(map[r3.Vec]int)
	for i, vertex := range lower.Verts {
		if vertex.Z != middle {
			continue
		}
		if _, duplicate := midVertices[vertex]; duplicate {
			return MirrorChamfer{}, fmt.Errorf("%w: mirrored cap chamfer duplicates a middle station",
				decaderr.ErrUnsupported)
		}
		midVertices[vertex] = i
	}
	mapUpper := make([]int, len(upper.Verts))
	verts := append([]r3.Vec(nil), lower.Verts...)
	vertexBound := append([]float64(nil), lower.VertexBound...)
	for i, vertex := range upper.Verts {
		if vertex.Z == middle {
			index, exists := midVertices[vertex]
			if !exists {
				return MirrorChamfer{}, fmt.Errorf("%w: mirrored cap chamfer middle annuli do not match",
					decaderr.ErrUnsupported)
			}
			mapUpper[i] = index
			vertexBound[index] = max(vertexBound[index],
				proofbound.AbsSumUpper(upper.VertexBound[i], bridgeReach))
			continue
		}
		if vertex.Z <= middle {
			return MirrorChamfer{}, fmt.Errorf("%w: mirrored cap chamfer crosses its middle plane",
				decaderr.ErrUnsupported)
		}
		mapUpper[i] = len(verts)
		verts = append(verts, vertex)
		vertexBound = append(vertexBound, proofbound.AbsSumUpper(upper.VertexBound[i], bridgeReach))
	}
	if !facetedtopology.CapsOppose(lower.Tris, lower.Src, upper.Tris, upper.Src, mapUpper,
		half.CapGroups[1], half.CapGroups[1]) {
		return MirrorChamfer{}, fmt.Errorf("%w: mirrored cap chamfer middle annuli do not tile equally",
			decaderr.ErrUnsupported)
	}
	if err := mirrorChamferSeam(ctx, lower, upper, half, middle); err != nil {
		return MirrorChamfer{}, err
	}
	var tris [][3]int
	var src []int
	var groups []facetedrecord.Group
	capGroups := [2]int{-1, -1}
	wallGroups := [2]int{-1, -1}
	for halfIndex, payload := range []facetedrecord.Payload{lower, upper} {
		groupMap := make(map[int]int)
		for i, tri := range payload.Tris {
			if i%256 == 0 && ctx.Err() != nil {
				return MirrorChamfer{}, ctx.Err()
			}
			oldGroup := payload.Src[i]
			if oldGroup == half.CapGroups[1] {
				continue
			}
			group, exists := groupMap[oldGroup]
			if !exists {
				if oldGroup < 0 || oldGroup >= len(payload.Groups) {
					return MirrorChamfer{}, fmt.Errorf("%w: mirrored cap chamfer facet lost its source group",
						decaderr.ErrUnsupported)
				}
				group = len(groups)
				groupMap[oldGroup] = group
				groups = append(groups, payload.Groups[oldGroup])
				if oldGroup == half.CapGroups[0] {
					capGroups[halfIndex] = group
				}
				if oldGroup == half.BoreGroup {
					wallGroups[halfIndex] = group
				}
			}
			if halfIndex == 1 {
				for corner, index := range tri {
					tri[corner] = mapUpper[index]
				}
			}
			tris = append(tris, tri)
			src = append(src, group)
		}
	}
	if capGroups[0] < 0 || capGroups[1] < 0 || wallGroups[0] < 0 || wallGroups[1] < 0 {
		return MirrorChamfer{}, fmt.Errorf("%w: mirrored cap chamfer lost an exterior annulus or bore wall",
			decaderr.ErrUnsupported)
	}
	if err := tessellation.RequireVertexLinks(ctx, len(verts), tris); err != nil {
		return MirrorChamfer{}, fmt.Errorf("%w: mirrored cap chamfer has an invalid vertex link: %s",
			decaderr.ErrUnsupported, err)
	}
	var outer, inner [2][]int
	outer[0], inner[0] = half.Outer[0], half.Inner[0]
	outer[1] = make([]int, len(half.Outer[0]))
	inner[1] = make([]int, len(half.Inner[0]))
	for i, index := range half.Outer[0] {
		outer[1][i] = mapUpper[index]
	}
	for i, index := range half.Inner[0] {
		inner[1][i] = mapUpper[index]
	}
	levels := [2]float64{blank.Z0, blank.Z1}
	for endCap, group := range capGroups {
		if !AuditAnnulus(ctx, verts, tris, src, outer[endCap], inner[endCap], group, levels[endCap], endCap == 0) {
			return MirrorChamfer{}, fmt.Errorf("%w: mirrored cap chamfer does not tile exterior annulus %d",
				decaderr.ErrUnsupported, endCap)
		}
	}
	if !mirrorChamferBoreUnchanged(old, verts, tris, src, record.WallGroups, wallGroups) {
		return MirrorChamfer{}, fmt.Errorf("%w: mirrored cap chamfer changed its bore wall", decaderr.ErrUnsupported)
	}
	bound := max(lower.MeshBound, proofbound.AbsSumUpper(upper.MeshBound, bridgeReach))
	volumeGap := proofbound.AbsSumUpper(lower.VolSymDiff, upper.VolSymDiff,
		record.Join.BridgeVolume)
	areaSlack := proofbound.AbsSumUpper(lower.AreaSlack, upper.AreaSlack,
		proofbound.ProductUpper(2, max(lower.AreaSlack, upper.AreaSlack)))
	diameter := facetedrecord.SourceDiameterUpper(verts, bound)
	if bound > 0.1 || proofbound.IsNonFinite(bound) || proofbound.IsNonFinite(volumeGap) ||
		proofbound.IsNonFinite(areaSlack) || proofbound.IsNonFinite(diameter) {
		return MirrorChamfer{}, fmt.Errorf("%w: mirrored cap chamfer has no finite 0.1 mm proof "+
			"(lower %g, upper %g, volume %g, area %g, diameter %g)",
			decaderr.ErrUnsupported, lower.MeshBound, upper.MeshBound, volumeGap, areaSlack, diameter)
	}
	return MirrorChamfer{
		Payload: facetedrecord.Payload{Verts: verts, Tris: tris, Src: src, Groups: groups,
			VertexBound: vertexBound, MeshBound: bound, VolSymDiff: volumeGap,
			AreaSlack: areaSlack, DPair: diameter, ContactAudited: true, Xform: r3.Identity()},
		CapGroups: capGroups,
		CapLevels: levels,
	}, nil
}

// Each half was audited before reflection. Their only possible geometric
// contact is on the middle plane: all other vertices lie on strict opposite
// sides. The opposed middle annuli are removed, and every remaining facet
// meets that plane only in a vertex or full edge of their shared two loops.
func mirrorChamferSeam(ctx context.Context, lower, upper facetedrecord.Payload,
	half ChamferHalf, middle float64) error {
	if half.CapGroups[1] < 0 || len(half.Outer[1]) < 3 || len(half.Inner[1]) < 3 ||
		!BoreRingSimple(ctx, lower.Verts, half.Outer[1]) ||
		!BoreRingSimple(ctx, lower.Verts, half.Inner[1]) {
		return fmt.Errorf("%w: mirrored cap chamfer has no simple middle loops", decaderr.ErrUnsupported)
	}
	nested, err := ChamferRingNested(ctx, lower.Verts, half.Outer[1], half.Inner[1])
	if err != nil {
		return err
	}
	if !nested {
		return fmt.Errorf("%w: mirrored cap chamfer middle loops are not nested", decaderr.ErrUnsupported)
	}
	vertices := make(map[int]struct{}, len(half.Outer[1])+len(half.Inner[1]))
	edges := make(map[[2]int]struct{}, len(vertices))
	for _, loop := range [][]int{half.Outer[1], half.Inner[1]} {
		for i, index := range loop {
			vertices[index] = struct{}{}
			edges[ChamferEdge(index, loop[(i+1)%len(loop)])] = struct{}{}
		}
	}
	for side, payload := range []facetedrecord.Payload{lower, upper} {
		if !AuditAnnulus(ctx, payload.Verts, payload.Tris, payload.Src, half.Outer[1], half.Inner[1],
			half.CapGroups[1], middle, side == 1) {
			return fmt.Errorf("%w: mirrored cap chamfer half %d has no tiled middle annulus",
				decaderr.ErrUnsupported, side)
		}
		for i, tri := range payload.Tris {
			if i%256 == 0 && ctx.Err() != nil {
				return ctx.Err()
			}
			if payload.Src[i] == half.CapGroups[1] {
				continue
			}
			var on []int
			for _, index := range tri {
				if index < 0 || index >= len(payload.Verts) {
					return fmt.Errorf("%w: mirrored cap chamfer has an invalid middle facet", decaderr.ErrUnsupported)
				}
				z := payload.Verts[index].Z
				if side == 0 && z > middle || side == 1 && z < middle {
					return fmt.Errorf("%w: mirrored cap chamfer half crosses the middle plane", decaderr.ErrUnsupported)
				}
				if z == middle {
					if _, ok := vertices[index]; !ok {
						return fmt.Errorf("%w: mirrored cap chamfer touches off its middle loop", decaderr.ErrUnsupported)
					}
					on = append(on, index)
				}
			}
			if len(on) == 3 {
				return fmt.Errorf("%w: mirrored cap chamfer has a planar seam wall", decaderr.ErrUnsupported)
			}
			if len(on) == 2 {
				if _, ok := edges[ChamferEdge(on[0], on[1])]; !ok {
					return fmt.Errorf("%w: mirrored cap chamfer has an unmatched seam edge", decaderr.ErrUnsupported)
				}
			}
		}
	}
	return nil
}

// mirrorChamferBoreUnchanged reports whether the bore-wall facets of the
// assembled mesh, in newGroups, are exactly old's facets in oldGroups, with
// the same multiplicity.
func mirrorChamferBoreUnchanged(old facetedrecord.Payload, verts []r3.Vec, tris [][3]int, src []int,
	oldGroups, newGroups [2]int) bool {
	type triangle [3]r3.Vec
	collect := func(verts []r3.Vec, tris [][3]int, src []int, groups [2]int) map[triangle]int {
		kept := make(map[triangle]int)
		for i, tri := range tris {
			if src[i] != groups[0] && src[i] != groups[1] {
				continue
			}
			key := triangle{verts[tri[0]], verts[tri[1]], verts[tri[2]]}
			kept[key]++
		}
		return kept
	}
	a := collect(old.Verts, old.Tris, old.Src, oldGroups)
	b := collect(verts, tris, src, newGroups)
	if len(a) != len(b) {
		return false
	}
	for tri, count := range a {
		if b[tri] != count {
			return false
		}
	}
	return true
}

// The upper source is the reflected lower source shifted down by each
// tooth's authenticated bridge. The one-cap split rings were independently
// checked at these physical levels. Its remaining top sliver is covered by
// an exact transverse section of the actual upper cap band.
func mirrorChamferUpperCapBridge(ctx context.Context, old, upper facetedrecord.Payload,
	record *facetedrecord.LoftMirrorCylinderBoreRecord, blank prismrecord.Payload,
	half ChamferHalf, setback, middle float64) (float64, error) {
	shift := record.Join.BridgeBound
	if shift == 0 {
		return 0, nil
	}
	if len(half.Ideal) != len(upper.Verts) || half.Band[0] >= half.Band[1] || len(record.Outer[1]) < 3 {
		return 0, fmt.Errorf("%w: mirrored cap bridge lost its upper band", decaderr.ErrUnsupported)
	}
	twiceMiddle := new(big.Rat).Mul(big.NewRat(2, 1), proofarith.FloatRat(middle))
	ideal := make([]proofarith.Xpt, len(half.Ideal))
	for i, p := range half.Ideal {
		if p.W == nil || p.W.Sign() <= 0 {
			return 0, fmt.Errorf("%w: mirrored cap bridge lost an exact band vertex", decaderr.ErrUnsupported)
		}
		z := new(big.Rat).Sub(twiceMiddle, new(big.Rat).SetFrac(p.Z, p.W))
		ideal[i] = proofarith.Xpt{
			X: new(big.Int).Mul(p.X, z.Denom()),
			Y: new(big.Int).Mul(p.Y, z.Denom()),
			Z: new(big.Int).Mul(p.W, z.Num()),
			W: new(big.Int).Mul(p.W, z.Denom()),
		}
	}
	mesh := &facetedrecord.JoinedMesh{Verts: append([]r3.Vec(nil), upper.Verts...), Tris: upper.Tris}
	outer := make([]int, len(record.Outer[1]))
	for i, index := range record.Outer[1] {
		if index < 0 || index >= len(old.Verts) {
			return 0, fmt.Errorf("%w: mirrored cap bridge lost its old outer rim", decaderr.ErrUnsupported)
		}
		outer[i] = len(mesh.Verts)
		mesh.Verts = append(mesh.Verts, old.Verts[index])
		ideal = append(ideal, proofarith.XptOf(old.Verts[index]))
	}
	topSourceLevel := new(big.Rat).Sub(proofarith.FloatRat(blank.Z1), proofarith.FloatRat(shift))
	return ChamferBridgeCap(ctx, old, mesh, ideal, half.Band,
		half.Outer[0], setback, blank.Z1, shift, topSourceLevel, outer)
}

// BuildMirrorCapChamfer reflects the chamfered lower half lowerBody, built by
// ChamferBottomCap with half, through MirrorChamferFrame in env, has
// AssembleMirrorChamfer assemble and prove the mirrored shell, then gives its
// exterior caps the cap origins of old, the bored mirrored Join, and the
// blank's planar cap tags. env.Ref names the result's cap roles.
func BuildMirrorCapChamfer(ctx context.Context, env topo.Env, old facetedrecord.Payload,
	record *facetedrecord.LoftMirrorCylinderBoreRecord, lowerBody *topo.Body,
	half ChamferHalf, reflectionReserve, bridgeReserve, setback float64) (*topo.Body, error) {
	lower, ok := lowerBody.Payload.(facetedrecord.Payload)
	if !ok {
		return nil, fmt.Errorf("%w: mirrored cap chamfer lost its audited lower half", decaderr.ErrUnsupported)
	}
	reflection, _, err := MirrorChamferFrame(record)
	if err != nil {
		return nil, err
	}
	upperBody, err := lower.Placed(ctx, env, reflection)
	if err != nil {
		return nil, err
	}
	upper, ok := upperBody.Payload.(facetedrecord.Payload)
	if !ok {
		return nil, fmt.Errorf("%w: mirrored cap chamfer halves have no matching held shell", decaderr.ErrUnsupported)
	}
	assembled, err := AssembleMirrorChamfer(ctx, old, lower, upper, record, half,
		reflectionReserve, bridgeReserve, setback)
	if err != nil {
		return nil, err
	}
	result := assembled.Payload
	for endCap, group := range assembled.CapGroups {
		result.Groups[group].Origins = append([]provenance.Ref(nil), old.Groups[record.CapGroups[endCap]].Origins...)
	}
	if err := TagCaps(&result, record.Join.Blank, assembled.CapGroups,
		assembled.CapLevels, env.Ref); err != nil {
		return nil, err
	}
	return buildMirrorWithWarnings(ctx, env, result, lowerBody, upperBody)
}
