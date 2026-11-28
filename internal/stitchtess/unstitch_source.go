package stitchtess

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
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

// UnstitchSource extracts a face's attributed facets from its original body.
func UnstitchSource(ctx context.Context, body *topo.Body, up stitchrecord.UnstitchPayload, chord float64,
	verify, areaProof bool, meshSource func(context.Context, *topo.Body, float64) (*topo.Mesh, error),
) (*topo.Mesh, error) {
	bad := func() (*topo.Mesh, error) {
		return nil, fmt.Errorf("%w: an unstitched face has no reusable source mesh", decaderr.ErrUnsupported)
	}
	faces := topobuild.Faces(body)
	if body.Kind != topo.BodySheet || len(faces) != 1 || up.Face == nil || up.Face.Body == nil || up.Face.Body == body {
		return bad()
	}
	sourceChord := chord
	if up.Xform != r3.Identity() {
		sourceChord /= 2
	}
	source, err := meshSource(ctx, up.Face.Body, sourceChord)
	if err != nil {
		return nil, err
	}
	if len(source.Source) != len(source.Triangles) {
		return bad()
	}
	sourceBound, ok, err := source.ReplayFaceBound(ctx, up.Face, proofbound.NewReplayTrialBudget())
	if err != nil {
		return nil, err
	}
	if !ok || proofbound.IsNonFinite(sourceBound) {
		return bad()
	}
	linear, err := massmoment.PlacementRotation(up.Xform)
	if err != nil {
		return nil, err
	}
	charge, err := massmoment.MapChargeOf(linear)
	if err != nil {
		return nil, err
	}
	mesh := &topo.Mesh{}
	indices := map[int]int{}
	maxRound := 0.0
	budget := proofbound.NewWorkBudget(ctx)
	for i, tri := range source.Triangles {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		if source.Source[i] != up.Face {
			continue
		}
		var out [3]int
		for k, old := range tri {
			index, exists := indices[old]
			if !exists {
				point := source.Vertices[old]
				placed := up.Xform.Apply(point)
				gap := revolvemesh.ExactRigidPointRound(up.Xform, point, placed)
				if proofbound.IsNonFinite(gap) {
					return bad()
				}
				maxRound = math.Max(maxRound, gap)
				index = len(mesh.Vertices)
				indices[old] = index
				mesh.Vertices = append(mesh.Vertices, placed)
			}
			out[k] = index
		}
		if up.Xform.IsReflection() {
			out[1], out[2] = out[2], out[1]
		}
		topobuild.AddTriangle(mesh, out, faces[0])
	}
	if len(mesh.Triangles) == 0 {
		return bad()
	}
	bound := sourceBound
	if up.Xform != r3.Identity() {
		bound = proofbound.AbsSumUpper(massmoment.PlacedBound(sourceBound, charge), maxRound)
	}
	// Some source meshers publish their construction allowance only in the
	// face bound. The complete face bound is a conservative coordinate bound.
	mesh.CoordBound = proofbound.AbsSumUpper(massmoment.PlacedBound(math.Max(source.CoordBound, sourceBound), charge), maxRound)
	// Only chording must fit the requested tolerance. The inherited source
	// and placement displacement remain published even when they exceed it.
	if proofbound.IsNonFinite(bound) || (up.Xform != r3.Identity() && massmoment.PlacedBound(sourceChord, charge) > chord) {
		return bad()
	}
	topobuild.SetFaceBound(mesh, faces[0], bound)
	if err := topobuild.RequireSheetMesh(ctx, body, mesh); err != nil {
		return nil, err
	}
	audit, err := revolvemesh.RequireRevolveFacetAreas(budget, mesh.Vertices, mesh.Triangles, maxRound)
	if err != nil {
		return nil, topobuild.LiftAuditError(err)
	}
	if verify {
		if err := revolvemesh.RevolveContactAuditBounded(budget, audit, mesh.Triangles, maxRound); err != nil {
			return nil, topobuild.LiftAuditError(err)
		}
	}
	if areaProof {
		area := proofbound.RatInterval{Lo: new(big.Rat), Hi: new(big.Rat)}
		for _, tri := range mesh.Triangles {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			lo, hi, ok := tessellation.ChainSweepTriangleArea(mesh.Vertices, tri)
			if !ok {
				return bad()
			}
			area.Lo.Add(area.Lo, proofarith.FloatRat(lo))
			area.Hi.Add(area.Hi, proofarith.FloatRat(hi))
		}
		mesh.AreaSlack, err = tessellation.ChainSweepFaceAreaSlack(body.Area.Value.Base(), body.Area.Bound.Base(), area)
		if err != nil {
			return nil, err
		}
	}
	return mesh, nil
}
