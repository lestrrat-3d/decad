package loftcylinder

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// BindLoftCapSetback admits one equal setback d for the two cap bands of a
// held loft between the axial levels z0 and z1. dDelta, the setback's
// asymmetric part, must be zero. The exact band levels z0+d and z1−d must be
// ordered, and d must move each float cap level.
func BindLoftCapSetback(z0, z1, d, dDelta float64) error {
	if dDelta != 0 || d <= 0 || !proofbound.FiniteVec(r3.Vec{X: d, Y: z0, Z: z1}) {
		return fmt.Errorf("%w: loft cap setback has no exact millimeter magnitude", decaderr.ErrUnsupported)
	}
	start := new(big.Rat).Add(proofarith.FloatRat(z0), proofarith.FloatRat(d))
	end := new(big.Rat).Sub(proofarith.FloatRat(z1), proofarith.FloatRat(d))
	if start.Cmp(end) >= 0 || z0+d == z0 || z1-d == z1 {
		return fmt.Errorf("%w: loft cap bands reach each other or round onto a cap", decaderr.ErrUnsupported)
	}
	return nil
}

// ChamferLoftCaps builds the equal-setback chamfer of both outer cap loops of
// a held loft shell (agent-docs/loft-design.md §19). fp is the held mesh of
// a solid Loft with exact horizontal caps at side.Z0 and side.Z1 and the
// Loft's verified boundary, occupied-volume and area allowances. side.Outer
// pairs each start-cap station with its end-cap station, counterclockwise
// from +Z, and the side between two consecutive pairs is the Loft's
// two-triangle cell. side.BoreGroup is -1 for a shell without holes, or the
// one group of a bore wall whose start and end rims are inner[0] and inner[1];
// the bore wall and both rims are kept whole.
//
// The result denotes the solid C cut from the held shell H: its side strips
// are clipped at the exact levels z0+d and z1−d, each cap is replaced by the
// chorded inward offset of its held outline, and a bevel band joins each
// inset loop to its clipped side ring. The audits prove C ⊂ H, so C = H∩C.
// For the true Loft S, (S∩C) △ C lies inside S △ H, so the result's
// occupied-volume bound adds the Loft's to the clip rounding; its boundary
// bound adds the Loft's held departure and the inset's chord departure from
// the exact offset to that rounding, as Loft §18's held-polygon chamfer does.
func ChamferLoftCaps(ctx context.Context, env topo.Env, fp facetedrecord.Payload,
	side ChamferSideSource, inner [2][]int, setback float64) (*topo.Body, error) {
	const (
		maxRimVertices   = 4_096
		maxInsetVertices = 4_096
		maxTriangles     = 32_768
	)
	if (side.BoreGroup < 0) != (len(inner[0]) == 0) || (side.BoreGroup < 0) != (len(inner[1]) == 0) {
		return nil, fmt.Errorf("%w: loft cap chamfer has a bore wall without both rims", decaderr.ErrUnsupported)
	}
	if len(side.Outer[0]) > maxRimVertices || len(side.Outer[1]) > maxRimVertices {
		return nil, fmt.Errorf("%w: loft cap outline exceeds the chamfer work ceiling", decaderr.ErrUnsupported)
	}
	if len(fp.VertexBound) != len(fp.Verts) || len(fp.Src) != len(fp.Tris) {
		return nil, fmt.Errorf("%w: loft source has incomplete mesh proof", decaderr.ErrUnsupported)
	}
	levels := [2]float64{side.Z0, side.Z1}
	for capIndex := range 2 {
		tiles := false
		if len(inner[capIndex]) > 0 {
			tiles = AuditAnnulus(ctx, fp.Verts, fp.Tris, fp.Src, side.Outer[capIndex], inner[capIndex],
				side.CapGroups[capIndex], levels[capIndex], capIndex == 0)
		} else {
			tiles = AuditCap(ctx, fp.Verts, fp.Tris, fp.Src, side.Outer[capIndex], side.CapGroups[capIndex],
				levels[capIndex], capIndex == 0)
		}
		if !tiles {
			return nil, fmt.Errorf("%w: loft cap %d does not tile its held outline", decaderr.ErrUnsupported, capIndex)
		}
	}
	groups, err := ChamferSideGroups(fp, side)
	if err != nil {
		return nil, err
	}
	mesh := &facetedrecord.JoinedMesh{Verts: append([]r3.Vec(nil), fp.Verts...),
		Groups: append([]facetedrecord.Group(nil), fp.Groups...)}
	bounds := append([]float64(nil), fp.VertexBound...)
	for i, tri := range fp.Tris {
		if fp.Src[i] == side.BoreGroup {
			mesh.Add(tri, fp.Src[i])
		}
	}
	middleFirst := len(mesh.Tris)
	clipped, err := ChamferClipSide(ctx, mesh, &bounds, fp, side, groups, setback)
	if err != nil {
		return nil, err
	}
	if middleFirst == len(mesh.Tris) {
		return nil, fmt.Errorf("%w: loft cap chamfer keeps no side", decaderr.ErrUnsupported)
	}
	var inset [2][]int
	var bands [2][2]int
	maxInset := 0.0
	for capIndex := range 2 {
		if !BoreRingSimple(ctx, mesh.Verts, clipped.Rings[capIndex]) {
			return nil, fmt.Errorf("%w: loft side level is not one simple outer ring", decaderr.ErrUnsupported)
		}
		samples, err := ChamferInset(ctx, fp.Verts, side.Outer[capIndex], setback)
		if err != nil {
			return nil, fmt.Errorf("loft cap %d inset: %w", capIndex, err)
		}
		if len(samples.Points) > maxInsetVertices {
			return nil, fmt.Errorf("%w: loft inset exceeds the chamfer work ceiling", decaderr.ErrUnsupported)
		}
		maxInset = max(maxInset, samples.Bound)
		inset[capIndex] = make([]int, len(samples.Points))
		for i, p := range samples.Points {
			p.Z = levels[capIndex]
			inset[capIndex][i] = len(mesh.Verts)
			mesh.Verts = append(mesh.Verts, p)
			bounds = append(bounds, proofbound.AbsSumUpper(fp.MeshBound, samples.Bound))
		}
		if !BoreRingSimple(ctx, mesh.Verts, inset[capIndex]) {
			return nil, fmt.Errorf("%w: loft inset is not one simple outer ring", decaderr.ErrUnsupported)
		}
		pairs := [][2][]int{{side.Outer[capIndex], inset[capIndex]}, {clipped.Rings[capIndex], inset[capIndex]}}
		if len(inner[capIndex]) > 0 {
			pairs = append(pairs, [2][]int{inset[capIndex], inner[capIndex]})
		}
		for _, pair := range pairs {
			nested, err := ChamferRingNested(ctx, mesh.Verts, pair[0], pair[1])
			if err != nil {
				return nil, err
			}
			if !nested {
				return nil, fmt.Errorf("%w: loft inset crosses its cap outline, side ring or bore rim", decaderr.ErrUnsupported)
			}
		}
		if err := ChamferHeldOutlineGap(ctx, mesh.Verts, side.Outer[capIndex], inset[capIndex], fp.MeshBound); err != nil {
			return nil, err
		}
		capGroup := side.CapGroups[capIndex]
		role := provenance.RoleCapStart
		if capIndex == 1 {
			role = provenance.RoleCapEnd
		}
		mesh.Groups[capGroup].Origins = append(append([]provenance.Ref(nil), mesh.Groups[capGroup].Origins...),
			provenance.Ref{Producer: env.Ref, Role: role})
		capTris, err := ChamferCapTriangles(ctx, mesh.Verts, inset[capIndex], inner[capIndex], capIndex == 0)
		if err != nil {
			return nil, fmt.Errorf("loft cap %d: %w", capIndex, err)
		}
		for _, t := range capTris {
			mesh.Add(t, capGroup)
		}
		bandGroup := len(mesh.Groups)
		mesh.Groups = append(mesh.Groups, facetedrecord.Group{Origins: []provenance.Ref{{Producer: env.Ref,
			Role: fmt.Sprintf("chamferCap(%d,0,0)", capIndex)}}})
		bandFirst := len(mesh.Tris)
		bandTris, err := ChamferBandTriangles(clipped.Rings[capIndex], inset[capIndex],
			samples.LineStart, samples.ArcStart, len(samples.Points))
		if err != nil {
			return nil, fmt.Errorf("loft band %d: %w", capIndex, err)
		}
		for _, t := range bandTris {
			mesh.Add(t, bandGroup)
		}
		bands[capIndex] = [2]int{bandFirst, len(mesh.Tris)}
	}
	if err := tessellation.OrientJoinedMesh(mesh.Verts, mesh.Tris); err != nil {
		return nil, err
	}
	if err := tessellation.RequireVertexLinks(ctx, len(mesh.Verts), mesh.Tris); err != nil {
		return nil, err
	}
	ideal := chamferIdealVertices(mesh.Verts, clipped)
	var bandSideEdges [2]map[[2]int]struct{}
	for capIndex := range 2 {
		capLevel := proofarith.FloatRat(side.Z0)
		sideLevel := new(big.Rat).Add(capLevel, proofarith.FloatRat(setback))
		if capIndex == 1 {
			capLevel = proofarith.FloatRat(side.Z1)
			sideLevel = new(big.Rat).Sub(capLevel, proofarith.FloatRat(setback))
		}
		if !ChamferExactRingSimple(ctx, ideal, clipped.Rings[capIndex]) {
			return nil, fmt.Errorf("%w: ideal loft side section is not simple", decaderr.ErrUnsupported)
		}
		nested, err := ChamferExactNested(ctx, ideal, clipped.Rings[capIndex], inset[capIndex])
		if err != nil {
			return nil, err
		}
		if !nested {
			return nil, fmt.Errorf("%w: ideal loft inset crosses its side section", decaderr.ErrUnsupported)
		}
		bandSideEdges[capIndex], err = ChamferBandBoundary(mesh.Tris, bands[capIndex], clipped.Rings[capIndex], inset[capIndex])
		if err != nil {
			return nil, err
		}
		if err := ChamferIdealBandLevels(mesh.Tris, ideal, bands[capIndex], inset[capIndex],
			capLevel, sideLevel); err != nil {
			return nil, err
		}
	}
	if err := ChamferContactAudit(ctx, fp, mesh, side, clipped, bands, bandSideEdges, ideal, setback); err != nil {
		return nil, err
	}
	if err := ChamferIdealBandAudit(ctx, len(mesh.Verts), mesh.Tris, ideal, bands, clipped.ClipBound); err != nil {
		return nil, err
	}
	if err := ChamferIdealOrientation(ctx, mesh.Tris, ideal); err != nil {
		return nil, err
	}
	if len(mesh.Tris) > maxTriangles {
		return nil, fmt.Errorf("%w: loft cap chamfer exceeds the triangle work ceiling", decaderr.ErrUnsupported)
	}
	audit := ChamferSource{Outer: side.Outer, Inner: inner, CapGroups: side.CapGroups, BoreGroup: side.BoreGroup,
		Z0: side.Z0, Z1: side.Z1}
	if err := ChamferCrossingAudit(ctx, mesh, audit, inset, maxTriangles); err != nil {
		return nil, err
	}
	bound := proofbound.AbsSumUpper(fp.MeshBound, max(clipped.ClipBound, maxInset))
	if proofbound.IsNonFinite(bound) {
		return nil, fmt.Errorf("%w: loft cap chamfer has no finite boundary bound", decaderr.ErrUnsupported)
	}
	for i := range bounds {
		bounds[i] = max(bounds[i], bound)
	}
	mesh.Verts, bounds, _ = ChamferCompact(mesh.Verts, mesh.Tris, bounds)
	roundArea, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.Verts, mesh.Tris, clipped.ClipBound)
	if err != nil {
		return nil, err
	}
	// One cutter C clips both the true and the held solid. The rounding of the
	// exact side stations into the stored mesh is charged separately.
	volumeGap := proofbound.AbsSumUpper(fp.VolSymDiff, proofbound.SweptVolumeAllow(clipped.ClipBound, roundArea))
	area, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.Verts, mesh.Tris, bound)
	if err != nil {
		return nil, err
	}
	result := fp
	result.Verts, result.Tris, result.Src, result.Groups = mesh.Verts, mesh.Tris, mesh.Src, mesh.Groups
	result.VertexBound, result.MeshBound = bounds, bound
	result.VolSymDiff, result.AreaSlack = volumeGap, proofbound.AbsSumUpper(fp.AreaSlack, area)
	result.ContactAudited = true
	result.ExactCopyMoments = nil
	return facetedrecord.Build(ctx, env, result)
}
