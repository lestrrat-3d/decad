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
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
)

// tessellateBodyPatchCircularPrism reuses the wall mesh's own ring vertices
// when it fills either end. The source mesh has already certified the wall;
// the new caps' exact area readings and the shared-grid topology complete the
// two-sided sheet proof without making an occupied-volume claim.
func tessellateBodyPatchCircularPrism(ctx context.Context, b *Body, pp bodyPatchPayload,
	prism prismPayload, chord float64, verify Verification) (*Mesh, error) {
	source, err := tessellatePrism(ctx, bodyHandle(pp.Faces[0].Body), prism, prismWallRole, chord, VerifyBoundary)
	if err != nil {
		return nil, err
	}
	faces := b.Faces()
	if len(faces) != 1+len(pp.Chains) || len(source.n.Source) == 0 {
		return nil, fmt.Errorf(`%w: a circular PatchEdges source has an incomplete face map`, ErrUnsupported)
	}
	work := freeform.NewFreeformWork()
	walks := prism.Walks
	if walks.Reusable(prism.Profile) {
		if err := walks.Charge(work); err != nil {
			return nil, err
		}
	} else {
		walks = nil
	}
	cl, err := tessellation.ChordLoop(ctx, prism.Profile.Outer, chord, prism.Z1-prism.Z0,
		work, walks, 0, func(_ survey2d.SideWalk) (*Face, error) { return faceHandle(pp.Faces[0]), nil },
		stationbound.ChordStationBound)
	if err != nil {
		return nil, err
	}
	n := len(cl.Samples)
	if n < 3 || len(source.n.Vertices) != 2*n {
		return nil, fmt.Errorf(`%w: a circular PatchEdges source has no matching wall stations`, ErrUnsupported)
	}
	capSag := proofbound.ProductUpper(cl.MaxSag, prism.LiftFactor())
	if proofbound.IsNonFinite(capSag) {
		return nil, fmt.Errorf(`%w: a circular PatchEdges source has no finite rim chording bound`, ErrUnsupported)
	}
	for j, p := range cl.Samples {
		if source.n.Vertices[2*j] != prism.Point(p.U, p.V, prism.Z0) ||
			source.n.Vertices[2*j+1] != prism.Point(p.U, p.V, prism.Z1) {
			return nil, fmt.Errorf(`%w: a circular PatchEdges source changed its wall stations`, ErrUnsupported)
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
	mesh := bornMesh(&Mesh{
		n: topo.Mesh{
			Vertices:  append([]r3.Vec(nil), source.n.Vertices...),
			Triangles: append([][3]int(nil), source.n.Triangles...),
			Source:    make([]*topo.Face, len(source.n.Source)),
		},
	})
	for i := range mesh.n.Source {
		if source.n.Source[i] != pp.Faces[0] {
			return nil, fmt.Errorf(`%w: a circular PatchEdges source has multiple wall faces`, ErrUnsupported)
		}
		mesh.n.Source[i] = faceNode(faces[0])
	}
	// A PatchEdges placement is applied once to the already placed source.
	// Charge its exact linear stretch and its held point-rounding separately.
	placementRound := 0.0
	stretch := 0.0
	if pp.Xform != r3.Identity() {
		rotation, err := massmoment.PlacementRotation(pp.Xform)
		if err != nil {
			return nil, err
		}
		charge, err := massmoment.MapChargeOf(rotation)
		if err != nil {
			return nil, err
		}
		stretch = charge.Stretch
		for i, v := range mesh.n.Vertices {
			placementRound = math.Max(placementRound,
				proofbound.RigidRoundAllow(proofbound.VecMaxAbs(v),
					proofbound.VecMaxAbs(pp.Xform.Translation())))
			mesh.n.Vertices[i] = pp.Xform.Apply(v)
		}
		if pp.Xform.IsReflection() {
			for i := range mesh.n.Triangles {
				mesh.n.Triangles[i][1], mesh.n.Triangles[i][2] = mesh.n.Triangles[i][2], mesh.n.Triangles[i][1]
			}
		}
	}
	lengthScale := proofbound.AbsSumUpper(1, stretch)
	mesh.n.CoordBound = proofbound.AbsSumUpper(proofbound.ProductUpper(source.n.CoordBound, lengthScale), placementRound)
	wallBound, ok := source.sourceBound(pp.Faces[0])
	if !ok {
		return nil, fmt.Errorf(`%w: a circular PatchEdges source states no wall bound`, ErrUnsupported)
	}
	topobuild.SetFaceBound(&mesh.n, faceNode(faces[0]), proofbound.AbsSumUpper(
		proofbound.ProductUpper(wallBound, lengthScale), placementRound))
	byFace := map[*Face][]int{faces[0]: {}}
	for i := range source.n.Triangles {
		byFace[faces[0]] = append(byFace[faces[0]], i)
	}
	used := [2]bool{}
	for i, chain := range pp.Chains {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		at := chain.Edges[0].Start.Position
		ring := -1
		for k := range 2 {
			if at == source.n.Vertices[k] {
				ring = k
			}
		}
		if ring < 0 || used[ring] {
			return nil, fmt.Errorf(`%w: a circular PatchEdges rim has no unique wall station ring`, ErrUnsupported)
		}
		used[ring] = true
		face := faces[i+1]
		first := len(mesh.n.Triangles)
		for _, tri := range capTris {
			mesh.n.Triangles = append(mesh.n.Triangles, [3]int{2*tri[0] + ring, 2*tri[1] + ring, 2*tri[2] + ring})
			mesh.n.Source = append(mesh.n.Source, faceNode(face))
			byFace[face] = append(byFace[face], len(mesh.n.Triangles)-1)
		}
		// The circle has one B-rep edge but many mesh chords. Use the
		// first shared chord to match the source wall's winding.
		a, z := ring, 2+ring
		fillSense := tessellation.PatchEdgeSense(mesh.n.Triangles, byFace[face], a, z)
		wallSense := tessellation.PatchEdgeSense(mesh.n.Triangles, byFace[faces[0]], a, z)
		if fillSense == 0 || wallSense == 0 {
			return nil, fmt.Errorf(`%w: a circular PatchEdges rim has no shared facet chord`, ErrUnsupported)
		}
		if fillSense == wallSense {
			for ti := first; ti < len(mesh.n.Triangles); ti++ {
				mesh.n.Triangles[ti][1], mesh.n.Triangles[ti][2] = mesh.n.Triangles[ti][2], mesh.n.Triangles[ti][1]
			}
		}
		topobuild.SetFaceBound(&mesh.n, faceNode(face), proofbound.AbsSumUpper(
			proofbound.ProductUpper(proofbound.AbsSumUpper(capSag, source.n.CoordBound,
				prism.AxialDelta()), lengthScale), placementRound, face.n.AxialDelta))
	}
	if err := tessellation.RequireCapBlendFacetAreas(mesh.n.Vertices, mesh.n.Triangles, "PatchEdges circular sheet"); err != nil {
		return nil, err
	}
	if err := requireMeshAudit(ctx, true, b, mesh); err != nil {
		return nil, err
	}
	if verify >= VerifyBoundary {
		if err := loftmesh.LoftCrossingAudit(proofbound.NewWorkBudget(ctx), mesh.n.Vertices, mesh.n.Triangles); err != nil {
			if ctxErr := ctx.Err(); ctxErr != nil {
				return nil, ctxErr
			}
			return nil, fmt.Errorf(`%w: a PatchEdges circular sheet's facets are not proven embedded: %s`, ErrUnsupported, err)
		}
	}
	if verify < VerifyAll {
		return mesh, nil
	}
	for _, face := range faces {
		indices := byFace[face]
		tris := make([][3]int, len(indices))
		for i, ti := range indices {
			tris[i] = mesh.n.Triangles[ti]
		}
		mesh.n.AreaSlack = proofbound.AbsSumUpper(mesh.n.AreaSlack,
			tessellation.FilletMeshAreaDeficit(mesh.n.Vertices, tris, face.n.Area, face.n.AreaBound))
	}
	if proofbound.IsNonFinite(mesh.n.AreaSlack) {
		return nil, fmt.Errorf(`%w: a PatchEdges circular sheet states no finite area bound`, ErrUnsupported)
	}
	return mesh, nil
}
