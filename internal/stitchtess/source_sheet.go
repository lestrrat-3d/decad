package stitchtess

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/stitchrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// SourceSheet copies a complete sheet's mesh when Stitch made no weld.
func SourceSheet(ctx context.Context, body *topo.Body, sp stitchrecord.Payload, chord float64, verify, areaProof bool,
	meshSource func(context.Context, *topo.Body, float64) (*topo.Mesh, error),
) (*topo.Mesh, bool, error) {
	if meshSource == nil || len(sp.Faces) == 0 || len(sp.Faces) != len(sp.LiveFaces) ||
		sp.Plan == nil || sp.Plan.Table == nil || sp.Plan.Groups != 0 || sp.Plan.MaxMerged() != 0 || len(sp.Plan.Table.Classes) != len(sp.Plan.Table.Vertices) || sp.Faces[0] == nil {
		return nil, false, nil
	}
	source := sp.Faces[0].Body
	if source == nil || source == body || source.Kind != topo.BodySheet {
		return nil, false, nil
	}
	paired := make(map[*topo.Face]*topo.Face, len(sp.Faces))
	liveSeen := make(map[*topo.Face]bool, len(sp.Faces))
	for i, face := range sp.Faces {
		if face == nil || face.Body != source || sp.LiveFaces[i] == nil || paired[face] != nil || liveSeen[sp.LiveFaces[i]] {
			return nil, false, nil
		}
		paired[face] = sp.LiveFaces[i]
		liveSeen[sp.LiveFaces[i]] = true
	}
	if len(topobuild.Faces(source)) != len(paired) {
		return nil, false, nil
	}
	for _, face := range topobuild.Faces(source) {
		if paired[face] == nil {
			return nil, false, nil
		}
	}
	if len(topobuild.Faces(body)) != len(liveSeen) {
		return nil, false, nil
	}
	for _, face := range topobuild.Faces(body) {
		if !liveSeen[face] {
			return nil, false, nil
		}
	}
	sourceChord := chord
	if sp.Xform != r3.Identity() {
		sourceChord /= 2
	}
	original, err := meshSource(ctx, source, sourceChord)
	if err != nil {
		return nil, true, err
	}
	if len(original.Triangles) != len(original.Source) {
		return nil, false, nil
	}
	mesh := &topo.Mesh{
		Vertices: make([]r3.Vec, len(original.Vertices)), Triangles: append([][3]int(nil), original.Triangles...),
		Source: make([]*topo.Face, len(original.Source)),
	}
	linear, err := massmoment.PlacementRotation(sp.Xform)
	if err != nil {
		return nil, true, err
	}
	charge, err := massmoment.MapChargeOf(linear)
	if err != nil {
		return nil, true, err
	}
	maxRound := 0.0
	work := proofbound.NewWorkBudget(ctx)
	for i, point := range original.Vertices {
		if err := work.Step(); err != nil {
			return nil, true, err
		}
		placed := sp.Xform.Apply(point)
		gap := revolvemesh.ExactRigidPointRound(sp.Xform, point, placed)
		if proofbound.IsNonFinite(gap) {
			return nil, false, nil
		}
		mesh.Vertices[i] = placed
		maxRound = math.Max(maxRound, gap)
	}
	mesh.CoordBound = proofbound.AbsSumUpper(massmoment.PlacedBound(original.CoordBound, charge), maxRound)
	for i, old := range original.Source {
		live := paired[old]
		if live == nil {
			return nil, false, nil
		}
		mesh.Source[i] = live
		if (old.Reversed != live.Reversed) != sp.Xform.IsReflection() {
			mesh.Triangles[i][1], mesh.Triangles[i][2] = mesh.Triangles[i][2], mesh.Triangles[i][1]
		}
	}
	budget := proofbound.NewReplayTrialBudget()
	for _, old := range topobuild.Faces(source) {
		bound, ok, err := original.ReplayFaceBound(ctx, old, budget)
		if err != nil {
			return nil, true, err
		}
		if sp.Xform != r3.Identity() {
			bound = proofbound.AbsSumUpper(massmoment.PlacedBound(bound, charge), maxRound)
		}
		if !ok || proofbound.IsNonFinite(bound) || (sp.Xform != r3.Identity() && bound > chord) {
			return nil, false, nil
		}
		topobuild.SetFaceBound(mesh, paired[old], bound)
		mesh.CoordBound = math.Max(mesh.CoordBound, bound)
	}
	if sp.Xform != r3.Identity() {
		audit, err := revolvemesh.RequireRevolveFacetAreas(work, mesh.Vertices, mesh.Triangles, maxRound)
		if err != nil {
			return nil, true, topobuild.LiftAuditError(err)
		}
		if verify {
			if err := revolvemesh.RevolveContactAuditBounded(work, audit, mesh.Triangles, maxRound); err != nil {
				return nil, true, topobuild.LiftAuditError(err)
			}
		}
	}
	if areaProof {
		area := proofbound.RatInterval{Lo: new(big.Rat), Hi: new(big.Rat)}
		for _, tri := range mesh.Triangles {
			if err := work.Step(); err != nil {
				return nil, true, err
			}
			lo, hi, ok := tessellation.ChainSweepTriangleArea(mesh.Vertices, tri)
			if !ok {
				return nil, false, nil
			}
			area.Lo.Add(area.Lo, proofarith.FloatRat(lo))
			area.Hi.Add(area.Hi, proofarith.FloatRat(hi))
		}
		mesh.AreaSlack, err = tessellation.ChainSweepFaceAreaSlack(body.Area.Value.Base(), body.Area.Bound.Base(), area)
		if err != nil {
			return nil, true, err
		}
	}
	if body.Kind == topo.BodySheet {
		if err := topobuild.RequireSheetMesh(ctx, body, mesh); err != nil {
			return nil, true, err
		}
		return mesh, true, nil
	}
	if err := tessellation.RequireClosedMesh(mesh.Triangles); err != nil {
		return nil, true, err
	}
	if err := tessellation.RequireVertexLinks(ctx, len(mesh.Vertices), mesh.Triangles); err != nil {
		return nil, true, err
	}
	if len(mesh.Vertices) == 0 || tessellation.OrientationSign(mesh.Vertices, mesh.Triangles, mesh.Vertices[0]) <= 0 {
		return nil, false, nil
	}
	return mesh, true, nil
}
