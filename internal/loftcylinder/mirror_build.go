package loftcylinder

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// BuildMirrorJoin builds the mirrored Loft/cylinder Union from its certified
// lower circular Join lowerHalf: it reflects the lower slab by reflection in
// env, assembles and proves the joined shell, audits both outer cap disks on
// blank's cap levels, gives the cap groups the blank's and every pair's cap
// origins and the blank's planar cap tags, and records blank, the source
// Join, pairs and lowerHalf for a later Union or bore. env.Ref names the
// result's cap roles.
func BuildMirrorJoin(ctx context.Context, env topo.Env, blank *topo.Body, pairs []*topo.Body,
	source *facetedrecord.LoftMirrorJoinRecord, lowerHalf *topo.Body, reflection r3.Transform,
	bridgeBound, bridgeVolume float64) (*topo.Body, error) {
	lower, ok := lowerHalf.Payload.(facetedrecord.Payload)
	if !ok || lower.LoftCylinderJoin == nil || !lower.ContactAudited || lower.Xform != r3.Identity() {
		return nil, fmt.Errorf("%w: mirrored cylinder's lower slab has no audited source", decaderr.ErrUnsupported)
	}
	upperHalf, err := lower.Placed(ctx, env, reflection)
	if err != nil {
		return nil, err
	}
	upper, ok := upperHalf.Payload.(facetedrecord.Payload)
	if !ok || !upper.ContactAudited || len(lower.Verts) != len(upper.Verts) ||
		len(lower.Tris) != len(upper.Tris) || len(lower.Src) != len(upper.Src) {
		return nil, fmt.Errorf("%w: reflected cylinder slab has no matching held mesh", decaderr.ErrUnsupported)
	}
	if caps := lower.LoftCylinderJoin.CapGroups; caps[0] < 0 || caps[1] < 0 || caps[0] == caps[1] {
		return nil, fmt.Errorf("%w: lower circular Join lacks separate cap groups", decaderr.ErrUnsupported)
	}
	shell, err := AssembleMirrorJoin(ctx, lower, upper, bridgeBound, bridgeVolume)
	if err != nil {
		return nil, err
	}
	fullBlank, ok := blank.Payload.(prismrecord.Payload)
	if !ok {
		return nil, fmt.Errorf("%w: mirrored cylinder lost its circular blank", decaderr.ErrUnsupported)
	}
	for cap, outline := range shell.CapOutlines {
		z := fullBlank.Z0
		if cap == 1 {
			z = fullBlank.Z1
		}
		if !AuditCap(ctx, shell.Payload.Verts, shell.Payload.Tris, shell.Payload.Src, outline, shell.CapGroups[cap], z, cap == 0) {
			return nil, fmt.Errorf("%w: mirrored cylinder cap does not tile its recorded outline", decaderr.ErrUnsupported)
		}
	}
	fp := shell.Payload
	fp.LoftMirrorCylinderJoin = &facetedrecord.LoftMirrorCylinderJoinRecord{
		Blank: blank, Source: source, Pairs: append([]*topo.Body(nil), pairs...),
		LowerHalf: lowerHalf, CapGroups: shell.CapGroups, CapOutlines: shell.CapOutlines,
		BridgeBound: bridgeBound, BridgeVolume: bridgeVolume,
	}
	if err := mirrorCapOrigins(&fp, blank, fp.LoftMirrorCylinderJoin.Pairs, shell.CapGroups); err != nil {
		return nil, err
	}
	if err := TagCaps(&fp, blank, shell.CapGroups,
		[2]float64{fullBlank.Z0, fullBlank.Z1}, env.Ref); err != nil {
		return nil, err
	}
	TagBridgedTop(&fp, shell.CapGroups[1], []float64{bridgeBound})
	return facetedrecord.Build(ctx, env, fp)
}

// BuildMirrorBore builds the mirrored circular bore from the bored lower half
// lowerBody of record's Join: it reflects the lower half through the source
// Loft's end frame in env, assembles and proves the bored shell, audits both
// exterior annuli on the blank's cap levels, and gives them the cap origins
// and planar cap tags BuildMirrorJoin gives the Join's caps. env.Ref names
// the result's cap roles.
func BuildMirrorBore(ctx context.Context, env topo.Env, record *facetedrecord.LoftMirrorCylinderJoinRecord,
	lowerBody *topo.Body) (*topo.Body, error) {
	lower, ok := lowerBody.Payload.(facetedrecord.Payload)
	if !ok || lower.LoftCylinderBore == nil || !lower.ContactAudited ||
		lower.Xform != r3.Identity() || record.Source.Lower == nil {
		return nil, fmt.Errorf("%w: mirrored circular bore lost its lower annulus", decaderr.ErrUnsupported)
	}
	loft, ok := record.Source.Lower.Payload.(loftrecord.Payload)
	if !ok || !loft.AuthenticatedReconstruction {
		return nil, fmt.Errorf("%w: mirrored circular bore lost its Loft source", decaderr.ErrUnsupported)
	}
	reflection, err := r3.Reflection(loft.Frame1)
	if err != nil {
		return nil, fmt.Errorf("%w: mirrored circular bore has no source reflection: %s", decaderr.ErrUnsupported, err)
	}
	upperBody, err := lower.Placed(ctx, env, reflection)
	if err != nil {
		return nil, err
	}
	upper, ok := upperBody.Payload.(facetedrecord.Payload)
	if !ok || !upper.ContactAudited || len(lower.Verts) != len(upper.Verts) ||
		len(lower.Tris) != len(upper.Tris) || len(lower.Src) != len(upper.Src) {
		return nil, fmt.Errorf("%w: reflected circular bore lacks a matching held shell", decaderr.ErrUnsupported)
	}
	shell, err := AssembleMirrorBore(ctx, lower, upper,
		record.BridgeBound, record.BridgeVolume)
	if err != nil {
		return nil, err
	}
	fullBlank, ok := record.Blank.Payload.(prismrecord.Payload)
	if !ok || !MirrorBlank(fullBlank) {
		return nil, fmt.Errorf("%w: reflected circular bore lost its exact blank", decaderr.ErrUnsupported)
	}
	for cap, group := range shell.CapGroups {
		z := fullBlank.Z0
		if cap == 1 {
			z = fullBlank.Z1
		}
		if !AuditAnnulus(ctx, shell.Payload.Verts, shell.Payload.Tris, shell.Payload.Src,
			shell.Outer[cap], shell.Inner[cap], group, z, cap == 0) {
			return nil, fmt.Errorf("%w: reflected circular bore cap does not tile its annulus", decaderr.ErrUnsupported)
		}
	}
	fp := shell.Payload
	fp.LoftMirrorCylinderBore = &facetedrecord.LoftMirrorCylinderBoreRecord{
		Join: record, LowerBore: lowerBody, CapGroups: shell.CapGroups,
		Outer: shell.Outer, Inner: shell.Inner, WallGroups: shell.WallGroups,
	}
	if err := mirrorCapOrigins(&fp, record.Blank, record.Pairs, shell.CapGroups); err != nil {
		return nil, err
	}
	if err := TagCaps(&fp, record.Blank, shell.CapGroups,
		[2]float64{fullBlank.Z0, fullBlank.Z1}, env.Ref); err != nil {
		return nil, err
	}
	TagBridgedTop(&fp, shell.CapGroups[1], []float64{record.BridgeBound})
	return facetedrecord.Build(ctx, env, fp)
}

// mirrorCapOrigins gives each cap group of fp, without repeats and in first-
// seen order, every origin of the blank faces that carry the blank's own cap
// role, then every origin of each pair's source cap group.
func mirrorCapOrigins(fp *facetedrecord.Payload, blank *topo.Body, pairs []*topo.Body,
	groups [2]int) error {
	for cap, group := range groups {
		role := provenance.RoleCapStart
		if cap == 1 {
			role = provenance.RoleCapEnd
		}
		seen := make(map[provenance.Ref]struct{})
		var origins []provenance.Ref
		add := func(ref provenance.Ref) {
			if _, exists := seen[ref]; !exists {
				seen[ref] = struct{}{}
				origins = append(origins, ref)
			}
		}
		for _, face := range topobuild.Faces(blank) {
			for _, origin := range face.Origins {
				if origin.Producer == blank.Origin.Producer && origin.Role == role {
					for _, source := range face.Origins {
						add(source)
					}
				}
			}
		}
		for _, pair := range pairs {
			payload, ok := pair.Payload.(facetedrecord.Payload)
			if !ok {
				return fmt.Errorf("%w: mirrored tooth lacks a source cap", decaderr.ErrUnsupported)
			}
			record := payload.LoftMirrorJoin
			if record == nil && payload.LoftMirrorCopy != nil {
				record = payload.LoftMirrorCopy.Source
			}
			if record == nil || record.CapGroups[cap] < 0 ||
				record.CapGroups[cap] >= len(payload.Groups) {
				return fmt.Errorf("%w: mirrored tooth lost a source cap role", decaderr.ErrUnsupported)
			}
			for _, origin := range payload.Groups[record.CapGroups[cap]].Origins {
				add(origin)
			}
		}
		if len(origins) == 0 {
			return fmt.Errorf("%w: mirrored cylinder cap has no source role", decaderr.ErrUnsupported)
		}
		fp.Groups[group].Origins = origins
	}
	return nil
}
