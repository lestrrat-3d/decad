package decad

import (
	"context"
	"fmt"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/shellseparation"
	"github.com/lestrrat-3d/r3"
)

// facetedCopyLineage records the source of a faceted body's rigid image.
// Copy issues it after rebuilding the held mesh. A Boolean result does not
// inherit it. The source must still be live when a copied-tool proof uses it.
type facetedCopyLineage struct {
	source          *Body
	sourceProducer  producerID
	sourcePlacement r3.Transform
	targetPlacement r3.Transform
	motion          r3.Transform
	// direct is true only when placed applied motion to the source payload
	// itself, so the image's true solid is exactly motion of the source's.
	direct bool
	// exact is the rational rotation the image denotes, set only for a direct
	// copy made with WithCopyRotation. A Transform, a copy of a copy and every
	// ordinary placement leave it nil.
	exact *exactrotation.Rotation
}

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
	if !baseOK || !placedOK || placed.xform != targetPlacement {
		return
	}
	inverse, err := base.xform.Inverse()
	if err != nil {
		return
	}
	motion, err := inverse.Then(targetPlacement)
	if err != nil || !motion.IsValid() {
		return
	}
	placed.copyLineage = &facetedCopyLineage{
		source: source, sourceProducer: source.n.Origin.Producer,
		sourcePlacement: base.xform, targetPlacement: targetPlacement,
		motion: motion, direct: direct,
	}
	if direct {
		placed.copyLineage.exact = exact
	}
	image.n.Payload = placed
}

// facetedCopySet binds a source and its live rigid copies to their actual
// held shells and one-to-one face owners. The shell proof establishes that
// source solids and held shells are pairwise separate; it says nothing about
// their contact with a separate Union blank.
type facetedCopySet struct {
	source *Body
	tools  []*Body
	moves  []r3.Transform
	// exact holds each copy's exact rotation record (index 0 and every ordinary
	// copy nil). maps holds the matching true linear maps for shellseparation:
	// the record's rotation where one exists, else nil for the float move.
	exact     []*exactrotation.Rotation
	maps      [][3][3]*big.Rat
	axis      shellseparation.Axis
	held      []shellseparation.Shell
	faceOwner []map[*Face]*Face // source face to the corresponding live copy face
}

func facetedShell(p facetedPayload) shellseparation.Shell {
	return shellseparation.Shell{Verts: p.verts, Tris: p.tris,
		VertexBound: p.vertexBound, MeshBound: p.meshBound}
}

func facetedCopyFaceOwners(source, image *Body, first, placed facetedPayload) (map[*Face]*Face, bool) {
	if len(first.tris) != len(placed.tris) || len(first.src) != len(first.tris) ||
		len(placed.src) != len(placed.tris) || len(first.faceOf) != len(first.tris) ||
		len(placed.faceOf) != len(placed.tris) || len(first.verts) != len(placed.verts) {
		return nil, false
	}
	sourceFaces, imageFaces := source.Faces(), image.Faces()
	owners := make(map[*Face]*Face, len(sourceFaces))
	reverse := make(map[*Face]*Face, len(imageFaces))
	for i, triangle := range first.tris {
		want := triangle
		if placed.copyLineage.motion.IsReflection() {
			want[1], want[2] = want[2], want[1]
		}
		if placed.tris[i] != want || first.src[i] != placed.src[i] ||
			first.faceOf[i] < 0 || first.faceOf[i] >= len(sourceFaces) ||
			placed.faceOf[i] < 0 || placed.faceOf[i] >= len(imageFaces) {
			return nil, false
		}
		original, copyFace := sourceFaces[first.faceOf[i]], imageFaces[placed.faceOf[i]]
		if owner := owners[original]; owner != nil && owner != copyFace {
			return nil, false
		}
		if prior := reverse[copyFace]; prior != nil && prior != original {
			return nil, false
		}
		owners[original], reverse[copyFace] = copyFace, original
	}
	if len(owners) != len(sourceFaces) || len(reverse) != len(imageFaces) {
		return nil, false
	}
	for original, copyFace := range owners {
		copyOrigins := copyFace.originRefs()
		for _, sourceOrigin := range original.originRefs() {
			if !slices.Contains(copyOrigins, sourceOrigin) {
				return nil, false
			}
		}
	}
	return owners, true
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
	set := &facetedCopySet{source: source, tools: append([]*Body(nil), tools...),
		moves:     make([]r3.Transform, len(tools)),
		exact:     make([]*exactrotation.Rotation, len(tools)),
		faceOwner: make([]map[*Face]*Face, len(tools))}
	set.moves[0] = r3.Identity()
	set.faceOwner[0] = make(map[*Face]*Face, len(source.Faces()))
	for _, face := range source.Faces() {
		set.faceOwner[0][face] = face
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
		heldCopies[i] = shellseparation.HeldCopy{Verts: part.verts, Tris: part.tris, Linked: true}
		if i == 0 {
			continue
		}
		record := part.copyLineage
		if record == nil || record.source != source ||
			record.sourceProducer != source.n.Origin.Producer ||
			record.sourcePlacement != first.xform ||
			record.targetPlacement != part.xform || !record.motion.IsValid() {
			return nil, fmt.Errorf("%w: copied tool %d lost its source placement", ErrUnsupported, i+1)
		}
		inverse, err := first.xform.Inverse()
		if err != nil {
			return nil, fmt.Errorf("%w: copied tool %d has no source inverse", ErrUnsupported, i+1)
		}
		motion, err := inverse.Then(part.xform)
		if err != nil || motion != record.motion {
			return nil, fmt.Errorf("%w: copied tool %d changed its recorded motion", ErrUnsupported, i+1)
		}
		owners, mapped := facetedCopyFaceOwners(source, tool, first, part)
		if !mapped {
			return nil, fmt.Errorf("%w: copied tool %d lost source face ownership", ErrUnsupported, i+1)
		}
		if record.exact != nil && !record.direct {
			return nil, fmt.Errorf("%w: copied tool %d carries an exact rotation without a direct placement",
				ErrUnsupported, i+1)
		}
		set.faceOwner[i] = owners
		set.moves[i] = record.motion
		sourceLinked = sourceLinked && record.direct
		if record.exact != nil {
			set.exact[i] = record.exact
			maps[i] = record.exact.Linear()
			anyExact = true
			// Transfer composes no translation, so an exact copy with a
			// translation, even one the float move rounds to zero, turns it off.
			for _, t := range record.exact.Translation() {
				sourceLinked = sourceLinked && t.Sign() == 0
			}
		}
	}
	if anyExact {
		set.maps = maps
	}
	for _, axis := range []shellseparation.Axis{
		shellseparation.AxisX, shellseparation.AxisY, shellseparation.AxisZ,
	} {
		// Source-solid transfer needs each copy's true solid to be exactly its
		// recorded motion of the source's, which holds only for direct copies.
		if !shellseparation.Separate(ctx, shells, set.moves, set.maps,
			shellseparation.SourceShell, axis, sourceLinked) {
			continue
		}
		held, err := shellseparation.PlacedHeldShells(ctx, heldCopies, set.moves)
		if err != nil {
			return nil, err
		}
		for i := 1; i < len(tools); i++ {
			part, ok := tools[i].n.Payload.(facetedPayload)
			if !ok {
				return nil, fmt.Errorf("%w: copied tool %d lost its faceted payload", ErrUnsupported, i+1)
			}
			for j, placementBound := range held[i].VertexBound {
				if placementBound > part.vertexBound[j] {
					return nil, fmt.Errorf("%w: copied tool %d lost its held placement bound", ErrUnsupported, i+1)
				}
			}
		}
		if !shellseparation.Separate(ctx, held, set.moves, nil,
			shellseparation.HeldCopyShell, axis, true) {
			continue
		}
		set.axis, set.held = axis, held
		return set, nil
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return nil, fmt.Errorf("%w: complete copied source and held shells are not separate", ErrUnsupported)
}
