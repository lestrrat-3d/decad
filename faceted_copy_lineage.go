package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/shellseparation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// bindFacetedCopyLineage records a rigid image only while its source remains
// live. Losing an optional copy proof never prevents an otherwise valid Copy
// or Transform from producing a body. direct is true only when image was
// placed from source's own payload.
func bindFacetedCopyLineage(image, source *Body, targetPlacement r3.Transform, direct bool,
	exact *exactrotation.Rotation) {
	if image == nil || source == nil || image.n.Doc != source.n.Doc ||
		documentHandle(source.n.Doc).requireLive(source) != nil {
		return
	}
	base, baseOK := source.n.Payload.(facetedPayload)
	placed, placedOK := image.n.Payload.(facetedPayload)
	if !baseOK || !placedOK || placed.Xform != targetPlacement {
		return
	}
	inverse, err := base.Xform.Inverse()
	if err != nil {
		return
	}
	motion, err := inverse.Then(targetPlacement)
	if err != nil || !motion.IsValid() {
		return
	}
	placed.CopyLineage = &facetedCopyLineage{
		Source: bodyNode(source), SourceProducer: source.n.Origin.Producer,
		SourcePlacement: base.Xform, TargetPlacement: targetPlacement,
		Motion: motion, Direct: direct,
	}
	if direct {
		placed.CopyLineage.Exact = exact
	}
	image.n.Payload = placed
}

func facetedShell(p facetedPayload) shellseparation.Shell {
	return shellseparation.Shell{Verts: p.Verts, Tris: p.Tris,
		VertexBound: p.VertexBound, MeshBound: p.MeshBound}
}

// certifyFacetedCopies consumes live source and Copy outputs. It rechecks
// their recorded transforms and face provenance before using shell separation.
func certifyFacetedCopies(ctx context.Context, tools []*Body) (*facetedCopySet, error) {
	if ctx == nil || len(tools) < 2 || tools[0] == nil || tools[0].n.Doc == nil {
		return nil, fmt.Errorf("%w: copied tools lack a live faceted source", ErrUnsupported)
	}
	source := tools[0]
	d := documentHandle(source.n.Doc)
	first, ok := source.n.Payload.(facetedPayload)
	if !ok || d.requireLive(source) != nil {
		return nil, fmt.Errorf("%w: copied tools lack a live faceted source", ErrUnsupported)
	}
	set := &facetedCopySet{Source: bodyNode(source), Tools: bodyNodes(tools),
		Moves:     make([]r3.Transform, len(tools)),
		Exact:     make([]*exactrotation.Rotation, len(tools)),
		FaceOwner: make([]map[*topo.Face]*topo.Face, len(tools))}
	set.Moves[0] = r3.Identity()
	set.FaceOwner[0] = make(map[*topo.Face]*topo.Face, len(source.Faces()))
	for _, face := range source.Faces() {
		set.FaceOwner[0][faceNode(face)] = faceNode(face)
	}
	shells := make([]shellseparation.Shell, len(tools))
	maps := make([][3][3]*big.Rat, len(tools))
	anyExact := false
	sourceLinked := true
	heldCopies := make([]shellseparation.HeldCopy, len(tools))
	for i, tool := range tools {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if tool == nil || tool.n.Doc != documentNode(d) || d.requireLive(tool) != nil {
			return nil, fmt.Errorf("%w: copied tool %d is not live in the source document", ErrUnsupported, i+1)
		}
		part, partOK := tool.n.Payload.(facetedPayload)
		if !partOK {
			return nil, fmt.Errorf("%w: copied tool %d lost its faceted payload", ErrUnsupported, i+1)
		}
		shells[i] = facetedShell(part)
		heldCopies[i] = shellseparation.HeldCopy{Verts: part.Verts, Tris: part.Tris, Linked: true}
		if i == 0 {
			continue
		}
		record := part.CopyLineage
		if record == nil || record.Source != bodyNode(source) ||
			record.SourceProducer != source.n.Origin.Producer ||
			record.SourcePlacement != first.Xform ||
			record.TargetPlacement != part.Xform || !record.Motion.IsValid() {
			return nil, fmt.Errorf("%w: copied tool %d lost its source placement", ErrUnsupported, i+1)
		}
		inverse, err := first.Xform.Inverse()
		if err != nil {
			return nil, fmt.Errorf("%w: copied tool %d has no source inverse", ErrUnsupported, i+1)
		}
		motion, err := inverse.Then(part.Xform)
		if err != nil || motion != record.Motion {
			return nil, fmt.Errorf("%w: copied tool %d changed its recorded motion", ErrUnsupported, i+1)
		}
		owners, mapped := facetedrecord.CopyFaceOwners(bodyNode(source), bodyNode(tool), first, part)
		if !mapped {
			return nil, fmt.Errorf("%w: copied tool %d lost source face ownership", ErrUnsupported, i+1)
		}
		if record.Exact != nil && !record.Direct {
			return nil, fmt.Errorf("%w: copied tool %d carries an exact rotation without a direct placement",
				ErrUnsupported, i+1)
		}
		set.FaceOwner[i] = owners
		set.Moves[i] = record.Motion
		sourceLinked = sourceLinked && record.Direct
		if record.Exact != nil {
			set.Exact[i] = record.Exact
			maps[i] = record.Exact.Linear()
			anyExact = true
			// Transfer composes no translation, so an exact copy with a
			// translation, even one the float move rounds to zero, turns it off.
			for _, t := range record.Exact.Translation() {
				sourceLinked = sourceLinked && t.Sign() == 0
			}
		}
	}
	if anyExact {
		set.Maps = maps
	}
	for _, axis := range []shellseparation.Axis{
		shellseparation.AxisX, shellseparation.AxisY, shellseparation.AxisZ,
	} {
		// Source-solid transfer needs each copy's true solid to be exactly its
		// recorded motion of the source's, which holds only for direct copies.
		if !shellseparation.Separate(ctx, shells, set.Moves, set.Maps,
			shellseparation.SourceShell, axis, sourceLinked) {
			continue
		}
		held, err := shellseparation.PlacedHeldShells(ctx, heldCopies, set.Moves)
		if err != nil {
			return nil, err
		}
		for i := 1; i < len(tools); i++ {
			part, ok := tools[i].n.Payload.(facetedPayload)
			if !ok {
				return nil, fmt.Errorf("%w: copied tool %d lost its faceted payload", ErrUnsupported, i+1)
			}
			for j, placementBound := range held[i].VertexBound {
				if placementBound > part.VertexBound[j] {
					return nil, fmt.Errorf("%w: copied tool %d lost its held placement bound", ErrUnsupported, i+1)
				}
			}
		}
		if !shellseparation.Separate(ctx, held, set.Moves, nil,
			shellseparation.HeldCopyShell, axis, true) {
			continue
		}
		set.Axis, set.Held = axis, held
		return set, nil
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return nil, fmt.Errorf("%w: complete copied source and held shells are not separate", ErrUnsupported)
}
