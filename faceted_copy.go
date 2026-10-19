package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/proofbound"
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

// placedProxyShells places one source-certified proxy under every recorded
// copy motion and proves its held shells separate. The caller owns the proof
// that proxy's true boundary and volume represent the original source solid.
// This function checks the live source face of every proxy triangle; a mesh
// carrying unrelated faces cannot borrow the copy lineage.
func placedProxyShells(ctx context.Context, set *facetedCopySet,
	proxy *Mesh) ([]facetproof.PlacedMesh, error) {
	if ctx == nil || set == nil || set.Source == nil || proxy == nil ||
		len(set.Tools) < 2 || len(set.Moves) != len(set.Tools) ||
		documentHandle(set.Source.Doc).requireLive(bodyHandle(set.Source)) != nil ||
		!proxy.n.BoundaryOK || !proxy.n.SymDiffOK {
		return nil, fmt.Errorf("%w: copied proxy lacks a live source proof", ErrUnsupported)
	}
	first, ok := set.Source.Payload.(facetedPayload)
	if !ok || len(proxy.n.Vertices) != len(first.Verts) ||
		len(proxy.n.VertexBound) != len(proxy.n.Vertices) ||
		len(proxy.n.Triangles) != len(first.Tris) ||
		len(proxy.n.Source) != len(proxy.n.Triangles) ||
		len(first.FaceOf) != len(first.Tris) {
		return nil, fmt.Errorf("%w: copied proxy lost its source facets", ErrUnsupported)
	}
	faces := bodyHandle(set.Source).Faces()
	for i, tri := range proxy.n.Triangles {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		faceIndex := first.FaceOf[i]
		if tri != first.Tris[i] || faceIndex < 0 || faceIndex >= len(faces) ||
			proxy.n.Source[i] != faceNode(faces[faceIndex]) {
			return nil, fmt.Errorf("%w: copied proxy changed source face ownership", ErrUnsupported)
		}
	}
	budget := proofbound.NewWorkBudget(ctx)
	placed := make([]facetproof.PlacedMesh, len(set.Tools))
	placed[0] = facetproof.PlacedMesh{
		Verts: proxy.n.Vertices, Tris: proxy.n.Triangles,
		VertexBound: proxy.n.VertexBound,
		MeshBound:   facetproof.FacetBoundMax(proxy.n.Triangles, proxy.n.VertexBound),
		VolSymDiff:  proxy.n.VolSymDiff, AreaSlack: proxy.n.AreaSlack,
	}
	copies := make([]shellseparation.HeldCopy, len(set.Tools))
	copies[0] = shellseparation.HeldCopy{Verts: proxy.n.Vertices, Tris: proxy.n.Triangles, Linked: true}
	for i := 1; i < len(set.Tools); i++ {
		tool := set.Tools[i]
		if tool == nil || tool.Doc != set.Source.Doc ||
			documentHandle(set.Source.Doc).requireLive(bodyHandle(tool)) != nil {
			return nil, fmt.Errorf("%w: copied proxy tool %d lost its live placement", ErrUnsupported, i+1)
		}
		part, ok := tool.Payload.(facetedPayload)
		if !ok || part.CopyLineage == nil ||
			part.CopyLineage.Source != set.Source ||
			part.CopyLineage.SourceProducer != set.Source.Origin.Producer ||
			part.CopyLineage.SourcePlacement != first.Xform ||
			part.CopyLineage.TargetPlacement != part.Xform ||
			part.CopyLineage.Motion != set.Moves[i] ||
			len(part.Tris) != len(proxy.n.Triangles) {
			return nil, fmt.Errorf("%w: copied proxy tool %d changed its source", ErrUnsupported, i+1)
		}
		for j, triangle := range part.Tris {
			want := proxy.n.Triangles[j]
			if set.Moves[i].IsReflection() {
				want[1], want[2] = want[2], want[1]
			}
			if triangle != want {
				return nil, fmt.Errorf("%w: copied proxy tool %d changed its source facets", ErrUnsupported, i+1)
			}
		}
		var err error
		placed[i], err = placeProxy(ctx, set, budget, proxy, i, part.CopyLineage.Exact)
		if err != nil {
			return nil, err
		}
		copies[i] = shellseparation.HeldCopy{
			Verts: placed[i].Verts, Tris: placed[i].Tris, Linked: true,
		}
	}
	held, err := shellseparation.PlacedHeldShells(ctx, copies, set.Moves)
	if err != nil {
		return nil, err
	}
	for _, axis := range []shellseparation.Axis{
		set.Axis, shellseparation.AxisX, shellseparation.AxisY, shellseparation.AxisZ,
	} {
		if shellseparation.Separate(ctx, held, set.Moves, nil,
			shellseparation.ProxyShell, axis, true) {
			return placed, nil
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return nil, fmt.Errorf("%w: copied proxy shells are not separate", ErrUnsupported)
}

// placeProxy places the proxy for copy i. An exact copy's true solid is its
// exact rotation of the source's, so the proxy pays the gap between that
// rotation and the float move instead of the float matrix's stretch.
func placeProxy(ctx context.Context, set *facetedCopySet, budget *proofbound.WorkBudget, proxy *Mesh,
	i int, record *exactrotation.Rotation) (facetproof.PlacedMesh, error) {
	if record != set.Exact[i] {
		return facetproof.PlacedMesh{}, fmt.Errorf("%w: copied proxy tool %d changed its exact rotation",
			ErrUnsupported, i+1)
	}
	if record != nil {
		linearGap, translationGap, ok := record.Gap(set.Moves[i])
		if !ok {
			return facetproof.PlacedMesh{}, fmt.Errorf("%w: copied proxy tool %d has no finite rotation gap",
				ErrUnsupported, i+1)
		}
		return facetproof.PlaceMeshDenoting(ctx, budget, proxy.n.Vertices, proxy.n.Triangles,
			proxy.n.VertexBound, proxy.n.VolSymDiff, proxy.n.AreaSlack, set.Moves[i],
			linearGap, translationGap)
	}
	linear, err := massmoment.PlacementRotation(set.Moves[i])
	if err != nil {
		return facetproof.PlacedMesh{}, err
	}
	charge, err := massmoment.MapChargeOf(linear)
	if err != nil {
		return facetproof.PlacedMesh{}, err
	}
	return facetproof.PlaceMesh(ctx, budget, proxy.n.Vertices, proxy.n.Triangles,
		proxy.n.VertexBound, proxy.n.VolSymDiff, proxy.n.AreaSlack,
		charge.Stretch, charge.Volume, set.Moves[i])
}
