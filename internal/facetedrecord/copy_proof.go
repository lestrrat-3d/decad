package facetedrecord

import (
	"context"
	"fmt"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/shellseparation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// liveIn reports whether b belongs to doc and is live in it, the check a
// document applies before it operates on a body.
func liveIn(doc *topo.Document, b *topo.Body) bool {
	return b != nil && b.Doc == doc && doc.IsLive(b)
}

// BindCopyLineage records a rigid image only while its source remains
// live. Losing an optional copy proof never prevents an otherwise valid Copy
// or Transform from producing a body. direct is true only when image was
// placed from source's own payload.
func BindCopyLineage(image, source *topo.Body, targetPlacement r3.Transform, direct bool,
	exact *exactrotation.Rotation) {
	if image == nil || source == nil || image.Doc != source.Doc || !liveIn(source.Doc, source) {
		return
	}
	base, baseOK := source.Payload.(Payload)
	placed, placedOK := image.Payload.(Payload)
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
	placed.CopyLineage = &FacetedCopyLineage{
		Source: source, SourceProducer: source.Origin.Producer,
		SourcePlacement: base.Xform, TargetPlacement: targetPlacement,
		Motion: motion, Direct: direct,
	}
	if direct {
		placed.CopyLineage.Exact = exact
	}
	image.Payload = placed
}

// SeparationShell is p's held shell as shellseparation reads it.
func SeparationShell(p Payload) shellseparation.Shell {
	return shellseparation.Shell{Verts: p.Verts, Tris: p.Tris,
		VertexBound: p.VertexBound, MeshBound: p.MeshBound}
}

// CertifyCopies consumes live source and Copy outputs. It rechecks
// their recorded transforms and face provenance before using shell separation.
func CertifyCopies(ctx context.Context, tools []*topo.Body) (*FacetedCopySet, error) {
	if ctx == nil || len(tools) < 2 || tools[0] == nil || tools[0].Doc == nil {
		return nil, fmt.Errorf("%w: copied tools lack a live faceted source", decaderr.ErrUnsupported)
	}
	source := tools[0]
	doc := source.Doc
	first, ok := source.Payload.(Payload)
	if !ok || !liveIn(doc, source) {
		return nil, fmt.Errorf("%w: copied tools lack a live faceted source", decaderr.ErrUnsupported)
	}
	set := &FacetedCopySet{Source: source, Tools: slices.Clone(tools),
		Moves:     make([]r3.Transform, len(tools)),
		Exact:     make([]*exactrotation.Rotation, len(tools)),
		FaceOwner: make([]map[*topo.Face]*topo.Face, len(tools))}
	set.Moves[0] = r3.Identity()
	sourceFaces := topobuild.Faces(source)
	set.FaceOwner[0] = make(map[*topo.Face]*topo.Face, len(sourceFaces))
	for _, face := range sourceFaces {
		set.FaceOwner[0][face] = face
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
		if tool == nil || !liveIn(doc, tool) {
			return nil, fmt.Errorf("%w: copied tool %d is not live in the source document", decaderr.ErrUnsupported, i+1)
		}
		part, partOK := tool.Payload.(Payload)
		if !partOK {
			return nil, fmt.Errorf("%w: copied tool %d lost its faceted payload", decaderr.ErrUnsupported, i+1)
		}
		shells[i] = SeparationShell(part)
		heldCopies[i] = shellseparation.HeldCopy{Verts: part.Verts, Tris: part.Tris, Linked: true}
		if i == 0 {
			continue
		}
		record := part.CopyLineage
		if record == nil || record.Source != source ||
			record.SourceProducer != source.Origin.Producer ||
			record.SourcePlacement != first.Xform ||
			record.TargetPlacement != part.Xform || !record.Motion.IsValid() {
			return nil, fmt.Errorf("%w: copied tool %d lost its source placement", decaderr.ErrUnsupported, i+1)
		}
		inverse, err := first.Xform.Inverse()
		if err != nil {
			return nil, fmt.Errorf("%w: copied tool %d has no source inverse", decaderr.ErrUnsupported, i+1)
		}
		motion, err := inverse.Then(part.Xform)
		if err != nil || motion != record.Motion {
			return nil, fmt.Errorf("%w: copied tool %d changed its recorded motion", decaderr.ErrUnsupported, i+1)
		}
		owners, mapped := CopyFaceOwners(source, tool, first, part)
		if !mapped {
			return nil, fmt.Errorf("%w: copied tool %d lost source face ownership", decaderr.ErrUnsupported, i+1)
		}
		if record.Exact != nil && !record.Direct {
			return nil, fmt.Errorf("%w: copied tool %d carries an exact rotation without a direct placement",
				decaderr.ErrUnsupported, i+1)
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
			part, ok := tools[i].Payload.(Payload)
			if !ok {
				return nil, fmt.Errorf("%w: copied tool %d lost its faceted payload", decaderr.ErrUnsupported, i+1)
			}
			for j, placementBound := range held[i].VertexBound {
				if placementBound > part.VertexBound[j] {
					return nil, fmt.Errorf("%w: copied tool %d lost its held placement bound", decaderr.ErrUnsupported, i+1)
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
	return nil, fmt.Errorf("%w: complete copied source and held shells are not separate", decaderr.ErrUnsupported)
}

// PlacedProxyShells places one source-certified proxy under every recorded
// copy motion and proves its held shells separate. The caller owns the proof
// that proxy's true boundary and volume represent the original source solid.
// This function checks the live source face of every proxy triangle; a mesh
// carrying unrelated faces cannot borrow the copy lineage.
func PlacedProxyShells(ctx context.Context, set *FacetedCopySet,
	proxy *topo.Mesh) ([]facetproof.PlacedMesh, error) {
	if ctx == nil || set == nil || set.Source == nil || proxy == nil ||
		len(set.Tools) < 2 || len(set.Moves) != len(set.Tools) ||
		!liveIn(set.Source.Doc, set.Source) ||
		!proxy.BoundaryOK || !proxy.SymDiffOK {
		return nil, fmt.Errorf("%w: copied proxy lacks a live source proof", decaderr.ErrUnsupported)
	}
	first, ok := set.Source.Payload.(Payload)
	if !ok || len(proxy.Vertices) != len(first.Verts) ||
		len(proxy.VertexBound) != len(proxy.Vertices) ||
		len(proxy.Triangles) != len(first.Tris) ||
		len(proxy.Source) != len(proxy.Triangles) ||
		len(first.FaceOf) != len(first.Tris) {
		return nil, fmt.Errorf("%w: copied proxy lost its source facets", decaderr.ErrUnsupported)
	}
	faces := topobuild.Faces(set.Source)
	for i, tri := range proxy.Triangles {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		faceIndex := first.FaceOf[i]
		if tri != first.Tris[i] || faceIndex < 0 || faceIndex >= len(faces) ||
			proxy.Source[i] != faces[faceIndex] {
			return nil, fmt.Errorf("%w: copied proxy changed source face ownership", decaderr.ErrUnsupported)
		}
	}
	budget := proofbound.NewWorkBudget(ctx)
	placed := make([]facetproof.PlacedMesh, len(set.Tools))
	placed[0] = facetproof.PlacedMesh{
		Verts: proxy.Vertices, Tris: proxy.Triangles,
		VertexBound: proxy.VertexBound,
		MeshBound:   facetproof.FacetBoundMax(proxy.Triangles, proxy.VertexBound),
		VolSymDiff:  proxy.VolSymDiff, AreaSlack: proxy.AreaSlack,
	}
	copies := make([]shellseparation.HeldCopy, len(set.Tools))
	copies[0] = shellseparation.HeldCopy{Verts: proxy.Vertices, Tris: proxy.Triangles, Linked: true}
	for i := 1; i < len(set.Tools); i++ {
		tool := set.Tools[i]
		if tool == nil || tool.Doc != set.Source.Doc ||
			!liveIn(set.Source.Doc, tool) {
			return nil, fmt.Errorf("%w: copied proxy tool %d lost its live placement", decaderr.ErrUnsupported, i+1)
		}
		part, ok := tool.Payload.(Payload)
		if !ok || part.CopyLineage == nil ||
			part.CopyLineage.Source != set.Source ||
			part.CopyLineage.SourceProducer != set.Source.Origin.Producer ||
			part.CopyLineage.SourcePlacement != first.Xform ||
			part.CopyLineage.TargetPlacement != part.Xform ||
			part.CopyLineage.Motion != set.Moves[i] ||
			len(part.Tris) != len(proxy.Triangles) {
			return nil, fmt.Errorf("%w: copied proxy tool %d changed its source", decaderr.ErrUnsupported, i+1)
		}
		for j, triangle := range part.Tris {
			want := proxy.Triangles[j]
			if set.Moves[i].IsReflection() {
				want[1], want[2] = want[2], want[1]
			}
			if triangle != want {
				return nil, fmt.Errorf("%w: copied proxy tool %d changed its source facets", decaderr.ErrUnsupported, i+1)
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
	return nil, fmt.Errorf("%w: copied proxy shells are not separate", decaderr.ErrUnsupported)
}

// placeProxy places the proxy for copy i. An exact copy's true solid is its
// exact rotation of the source's, so the proxy pays the gap between that
// rotation and the float move instead of the float matrix's stretch.
func placeProxy(ctx context.Context, set *FacetedCopySet, budget *proofbound.WorkBudget, proxy *topo.Mesh,
	i int, record *exactrotation.Rotation) (facetproof.PlacedMesh, error) {
	if record != set.Exact[i] {
		return facetproof.PlacedMesh{}, fmt.Errorf("%w: copied proxy tool %d changed its exact rotation",
			decaderr.ErrUnsupported, i+1)
	}
	if record != nil {
		linearGap, translationGap, ok := record.Gap(set.Moves[i])
		if !ok {
			return facetproof.PlacedMesh{}, fmt.Errorf("%w: copied proxy tool %d has no finite rotation gap",
				decaderr.ErrUnsupported, i+1)
		}
		return facetproof.PlaceMeshDenoting(ctx, budget, proxy.Vertices, proxy.Triangles,
			proxy.VertexBound, proxy.VolSymDiff, proxy.AreaSlack, set.Moves[i],
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
	return facetproof.PlaceMesh(ctx, budget, proxy.Vertices, proxy.Triangles,
		proxy.VertexBound, proxy.VolSymDiff, proxy.AreaSlack,
		charge.Stretch, charge.Volume, set.Moves[i])
}
