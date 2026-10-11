package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stationbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
)

// bodyPatchCircularPrismSource admits a complete single-circle surface
// extrusion. Its one source face supplies both rim station grids, and each
// selected rim is exactly one of the source body's own full-circle edges.
func bodyPatchCircularPrismSource(pp bodyPatchPayload) (prismPayload, bool) {
	if len(pp.faces) != 1 || len(pp.chains) == 0 || len(pp.chains) > 2 || pp.faces[0].body == nil {
		return prismPayload{}, false
	}
	prism, ok := pp.faces[0].body.payload.(prismPayload)
	if !ok || !prism.surfaceResult || prism.sectionDelta != 0 || len(prism.profile.Holes) != 0 ||
		len(prism.profile.Outer.Segments) != 1 {
		return prismPayload{}, false
	}
	if _, ok := pp.faces[0].surface.(Cylinder); !ok {
		return prismPayload{}, false
	}
	for _, chain := range pp.chains {
		if len(chain.edges) != 1 {
			return prismPayload{}, false
		}
		if _, ok := chain.edges[0].curve.(Circle3); !ok {
			return prismPayload{}, false
		}
		if len(chain.edges[0].faces) != 1 || chain.edges[0].faces[0] != pp.faces[0] {
			return prismPayload{}, false
		}
	}
	return prism, true
}

// tessellateBodyPatchCircularPrism reuses the wall mesh's own ring vertices
// when it fills either end. The source mesh has already certified the wall;
// the new caps' exact area readings and the shared-grid topology complete the
// two-sided sheet proof without making an occupied-volume claim.
func tessellateBodyPatchCircularPrism(ctx context.Context, b *Body, pp bodyPatchPayload,
	prism prismPayload, chord float64, verify Verification) (*Mesh, error) {
	source, err := tessellatePrism(ctx, pp.faces[0].body, prism, prismWallRole, chord, VerifyBoundary)
	if err != nil {
		return nil, err
	}
	faces := b.Faces()
	if len(faces) != 1+len(pp.chains) || len(source.source) == 0 {
		return nil, fmt.Errorf(`%w: a circular Body.Patch source has an incomplete face map`, ErrUnsupported)
	}
	work := freeform.NewFreeformWork()
	walks := prism.walks
	if walks.Reusable(prism.profile) {
		if err := walks.Charge(work); err != nil {
			return nil, err
		}
	} else {
		walks = nil
	}
	cl, err := tessellation.ChordLoop(ctx, prism.profile.Outer, chord, prism.z1-prism.z0,
		work, walks, 0, func(_ survey2d.SideWalk) (*Face, error) { return pp.faces[0], nil },
		stationbound.ChordStationBound)
	if err != nil {
		return nil, err
	}
	n := len(cl.Samples)
	if n < 3 || len(source.vertices) != 2*n {
		return nil, fmt.Errorf(`%w: a circular Body.Patch source has no matching wall stations`, ErrUnsupported)
	}
	capSag := proofbound.ProductUpper(cl.MaxSag, prismLiftFactor(prism))
	if proofbound.IsNonFinite(capSag) {
		return nil, fmt.Errorf(`%w: a circular Body.Patch source has no finite rim chording bound`, ErrUnsupported)
	}
	for j, p := range cl.Samples {
		if source.vertices[2*j] != prism.point(p.U, p.V, prism.z0) ||
			source.vertices[2*j+1] != prism.point(p.U, p.V, prism.z1) {
			return nil, fmt.Errorf(`%w: a circular Body.Patch source changed its wall stations`, ErrUnsupported)
		}
	}
	loop := make([]int, n)
	for j := range loop {
		loop[j] = j
	}
	capTris, err := triangulation.Triangulate(ctx, cl.Samples, [][]int{loop})
	if err != nil {
		return nil, err
	}
	mesh := &Mesh{
		vertices:  append([]r3.Vec(nil), source.vertices...),
		triangles: append([][3]int(nil), source.triangles...),
		source:    make([]*Face, len(source.source)),
	}
	for i := range mesh.source {
		if source.source[i] != pp.faces[0] {
			return nil, fmt.Errorf(`%w: a circular Body.Patch source has multiple wall faces`, ErrUnsupported)
		}
		mesh.source[i] = faces[0]
	}
	// A Body.Patch placement is applied once to the already placed source.
	// Charge its exact linear stretch and its held point-rounding separately.
	placementRound := 0.0
	stretch := 0.0
	if pp.xform != r3.Identity() {
		rotation, err := massmoment.PlacementRotation(pp.xform)
		if err != nil {
			return nil, err
		}
		charge, err := massmoment.MapChargeOf(rotation)
		if err != nil {
			return nil, err
		}
		stretch = charge.Stretch
		for i, v := range mesh.vertices {
			placementRound = math.Max(placementRound,
				proofbound.RigidRoundAllow(proofbound.VecMaxAbs(v),
					proofbound.VecMaxAbs(pp.xform.Translation())))
			mesh.vertices[i] = pp.xform.Apply(v)
		}
		if pp.xform.IsReflection() {
			for i := range mesh.triangles {
				mesh.triangles[i][1], mesh.triangles[i][2] = mesh.triangles[i][2], mesh.triangles[i][1]
			}
		}
	}
	lengthScale := proofbound.AbsSumUpper(1, stretch)
	mesh.coordBound = proofbound.AbsSumUpper(proofbound.ProductUpper(source.coordBound, lengthScale), placementRound)
	wallBound, ok := source.sourceBound(pp.faces[0])
	if !ok {
		return nil, fmt.Errorf(`%w: a circular Body.Patch source states no wall bound`, ErrUnsupported)
	}
	mesh.setFaceBound(faces[0], proofbound.AbsSumUpper(
		proofbound.ProductUpper(wallBound, lengthScale), placementRound))
	byFace := map[*Face][]int{faces[0]: {}}
	for i := range source.triangles {
		byFace[faces[0]] = append(byFace[faces[0]], i)
	}
	used := [2]bool{}
	for i, chain := range pp.chains {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		at := chain.edges[0].start.position
		ring := -1
		for k := range 2 {
			if at == source.vertices[k] {
				ring = k
			}
		}
		if ring < 0 || used[ring] {
			return nil, fmt.Errorf(`%w: a circular Body.Patch rim has no unique wall station ring`, ErrUnsupported)
		}
		used[ring] = true
		face := faces[i+1]
		first := len(mesh.triangles)
		for _, tri := range capTris {
			mesh.triangles = append(mesh.triangles, [3]int{2*tri[0] + ring, 2*tri[1] + ring, 2*tri[2] + ring})
			mesh.source = append(mesh.source, face)
			byFace[face] = append(byFace[face], len(mesh.triangles)-1)
		}
		// The circle has one B-rep edge but many mesh chords. Use the
		// first shared chord to match the source wall's winding.
		a, z := ring, 2+ring
		fillSense := bodyPatchMeshEdgeSense(mesh, byFace[face], a, z)
		wallSense := bodyPatchMeshEdgeSense(mesh, byFace[faces[0]], a, z)
		if fillSense == 0 || wallSense == 0 {
			return nil, fmt.Errorf(`%w: a circular Body.Patch rim has no shared facet chord`, ErrUnsupported)
		}
		if fillSense == wallSense {
			for ti := first; ti < len(mesh.triangles); ti++ {
				mesh.triangles[ti][1], mesh.triangles[ti][2] = mesh.triangles[ti][2], mesh.triangles[ti][1]
			}
		}
		mesh.setFaceBound(face, proofbound.AbsSumUpper(
			proofbound.ProductUpper(proofbound.AbsSumUpper(capSag, source.coordBound,
				prism.axialDelta()), lengthScale), placementRound, face.axialDelta))
	}
	if err := requireCapBlendFacetAreas(mesh, "Body.Patch circular sheet"); err != nil {
		return nil, err
	}
	if err := requireMeshAudit(ctx, true, b, mesh); err != nil {
		return nil, err
	}
	if verify >= VerifyBoundary {
		if err := loftmesh.LoftCrossingAudit(proofbound.NewWorkBudget(ctx), mesh.vertices, mesh.triangles); err != nil {
			if ctxErr := ctx.Err(); ctxErr != nil {
				return nil, ctxErr
			}
			return nil, fmt.Errorf(`%w: a Body.Patch circular sheet's facets are not proven embedded: %s`, ErrUnsupported, err)
		}
	}
	if verify < VerifyAll {
		return mesh, nil
	}
	for _, face := range faces {
		indices := byFace[face]
		tris := make([][3]int, len(indices))
		for i, ti := range indices {
			tris[i] = mesh.triangles[ti]
		}
		mesh.areaSlack = proofbound.AbsSumUpper(mesh.areaSlack,
			tessellation.FilletMeshAreaDeficit(mesh.vertices, tris, face.area, face.areaBound))
	}
	if proofbound.IsNonFinite(mesh.areaSlack) {
		return nil, fmt.Errorf(`%w: a Body.Patch circular sheet states no finite area bound`, ErrUnsupported)
	}
	return mesh, nil
}
