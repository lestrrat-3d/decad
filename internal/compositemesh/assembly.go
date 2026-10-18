package compositemesh

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// Assembly holds only per-call station and proof state. The root caller keeps
// ownership of the live mesh handle, each span build, and the body audit.
type Assembly struct {
	mesh            *topo.Mesh
	byRole          map[string]*topo.Face
	join            [][]int
	joinBound       [][]float64
	triangleSpan    []int
	spanCount       int
	profileSamples  int
	closed          bool
	surface         bool
	freeformTarget  float64
	budget          *proofbound.WorkBudget
	resolve         ResolveRevolve
	verifyBoundary  bool
	verifyAll       bool
	coordinateBound float64
}

// NewAssembly prepares the existing station table after the root mesh and
// operation budget have been created.
func NewAssembly(mesh *topo.Mesh, byRole map[string]*topo.Face, spanCount, profileSamples int,
	closed, surface bool, freeformTarget float64, budget *proofbound.WorkBudget,
	resolve ResolveRevolve, verifyBoundary, verifyAll bool) *Assembly {
	stations := spanCount + 1
	if closed {
		stations = spanCount
	}
	a := &Assembly{
		mesh: mesh, byRole: byRole, spanCount: spanCount, profileSamples: profileSamples,
		closed: closed, surface: surface, freeformTarget: freeformTarget,
		budget: budget, resolve: resolve, verifyBoundary: verifyBoundary, verifyAll: verifyAll,
		join: make([][]int, stations), joinBound: make([][]float64, stations),
	}
	for i := range a.join {
		a.join[i] = make([]int, profileSamples)
		a.joinBound[i] = make([]float64, profileSamples)
		for j := range a.join[i] {
			a.join[i][j] = -1
		}
	}
	return a
}

// Append joins one root-built span node to the destination mesh in span order.
func (a *Assembly) Append(ctx context.Context, i int, span sweeprecord.SpanPayload,
	piece *topo.Mesh, vertexBounds []float64) error {
	start, end, spanCoordinateBound, err := SpanRings(ctx, span, piece,
		a.profileSamples, a.freeformTarget, a.resolve)
	if err != nil {
		return fmt.Errorf(`sweep path span %d: %w`, i, err)
	}
	local := make([]int, len(piece.Vertices))
	for j := range local {
		local[j] = -1
	}
	maxMove := 0.0
	endSection := i + 1
	if a.closed && endSection == a.spanCount {
		endSection = 0
	}
	for station := range a.profileSamples {
		if err := a.budget.Step(); err != nil {
			return err
		}
		for _, boundary := range []struct {
			section int
			index   int
		}{{i, start[station]}, {endSection, end[station]}} {
			shared := a.join[boundary.section][station]
			if shared < 0 {
				shared = len(a.mesh.Vertices)
				a.mesh.Vertices = append(a.mesh.Vertices, piece.Vertices[boundary.index])
				a.join[boundary.section][station] = shared
				a.joinBound[boundary.section][station] = vertexBounds[boundary.index]
			} else if a.mesh.Vertices[shared] != piece.Vertices[boundary.index] {
				delta2, ok := proofarith.DySquaredDistance3(
					a.mesh.Vertices[shared].X, a.mesh.Vertices[shared].Y, a.mesh.Vertices[shared].Z,
					piece.Vertices[boundary.index].X, piece.Vertices[boundary.index].Y, piece.Vertices[boundary.index].Z)
				if !ok {
					return fmt.Errorf(`%w: a shared sweep station has no finite displacement`, decaderr.ErrUnsupported)
				}
				move := proofarith.DySqrtUp(delta2)
				if proofbound.IsNonFinite(move) ||
					move > proofbound.AbsSumUpper(a.joinBound[boundary.section][station], vertexBounds[boundary.index]) {
					return fmt.Errorf(`%w: adjacent sweep spans disagree on profile station %d at section %d`,
						decaderr.ErrUnsupported, station, boundary.section)
				}
				maxMove = max(maxMove, move)
			}
			local[boundary.index] = shared
		}
	}
	for j, v := range piece.Vertices {
		if err := a.budget.Step(); err != nil {
			return err
		}
		if local[j] < 0 {
			local[j] = len(a.mesh.Vertices)
			a.mesh.Vertices = append(a.mesh.Vertices, v)
		}
	}
	a.coordinateBound = max(a.coordinateBound, proofbound.AbsSumUpper(spanCoordinateBound, maxMove))
	for t, triangle := range piece.Triangles {
		if err := a.budget.Step(); err != nil {
			return err
		}
		role, err := SpanFaceRole(piece.Source[t], i, span.ReverseArcCaps)
		if err != nil {
			return err
		}
		if a.closed && (role == provenance.RoleCapStart || role == provenance.RoleCapEnd) ||
			role == provenance.RoleCapStart && (a.surface || i != 0) ||
			role == provenance.RoleCapEnd && (a.surface || i != a.spanCount-1) {
			continue
		}
		face, ok := a.byRole[role]
		if !ok {
			return fmt.Errorf(`%w: the composite sweep carries no face for role %q`, decaderr.ErrDegenerate, role)
		}
		topobuild.AddTriangle(a.mesh, [3]int{local[triangle[0]], local[triangle[1]], local[triangle[2]]}, face)
		if len(a.mesh.Triangles) > MaxFacets {
			return fmt.Errorf(`%w: the composite sweep mesh exceeds the fixed facet ceiling`, decaderr.ErrUnsupported)
		}
		a.triangleSpan = append(a.triangleSpan, i)
		bound, ok := piece.FaceBound[piece.Source[t]]
		if !ok {
			return fmt.Errorf(`%w: sweep path span %d has no source bound for role %q`, decaderr.ErrUnsupported, i, role)
		}
		topobuild.SetFaceBound(a.mesh, face, proofbound.AbsSumUpper(bound, maxMove))
	}
	if a.verifyAll {
		if !piece.SymDiffOK {
			return fmt.Errorf(`%w: sweep path span %d has no occupied-volume proof`, decaderr.ErrUnsupported, i)
		}
		areaMove := revolvemesh.CoordinateAreaAllow(piece.Vertices, piece.Triangles, maxMove)
		volumeMove := proofbound.SweptVolumeAllow(maxMove,
			proofbound.PerturbedAreaUpper(piece.Vertices, piece.Triangles, maxMove))
		a.mesh.AreaSlack = proofbound.AbsSumUpper(a.mesh.AreaSlack, piece.AreaSlack, areaMove)
		a.mesh.VolSymDiff = proofbound.AbsSumUpper(a.mesh.VolSymDiff, piece.VolSymDiff, volumeMove)
	}
	return nil
}

// BeforeBodyAudit publishes the coordinate bound only after all four proof
// values are finite. Root then performs its body-kind audit.
func (a *Assembly) BeforeBodyAudit() error {
	if proofbound.IsNonFinite(a.mesh.Bound) || proofbound.IsNonFinite(a.coordinateBound) ||
		proofbound.IsNonFinite(a.mesh.AreaSlack) || proofbound.IsNonFinite(a.mesh.VolSymDiff) {
		return fmt.Errorf(`%w: the composite sweep mesh has no finite proof bound`, decaderr.ErrUnsupported)
	}
	a.mesh.CoordBound = a.coordinateBound
	return nil
}

// AfterBodyAudit preserves the separate final audit budget and publishes the
// occupied-volume flag only after all required audits pass.
func (a *Assembly) AfterBodyAudit(ctx context.Context) error {
	if !a.surface {
		if err := topobuild.LiftAuditError(tessellation.RequireVertexLinks(ctx,
			len(a.mesh.Vertices), a.mesh.Triangles)); err != nil {
			return err
		}
	}
	auditBudget := proofbound.NewWorkBudget(ctx)
	auditTris, err := revolvemesh.RequireRevolveFacetAreas(auditBudget,
		a.mesh.Vertices, a.mesh.Triangles, a.coordinateBound)
	if err != nil {
		return err
	}
	if a.verifyBoundary {
		if err := compositesweep.CrossSpanContactAudit(auditBudget, auditTris,
			a.mesh.Triangles, a.triangleSpan, a.coordinateBound); err != nil {
			return err
		}
	}
	if !a.surface && tessellation.OrientationSign(a.mesh.Vertices, a.mesh.Triangles, r3.Vec{}) <= 0 {
		return fmt.Errorf(`%w: the composite sweep mesh does not enclose a positive volume`, decaderr.ErrUnsupported)
	}
	a.mesh.SymDiffOK = a.verifyAll && !a.surface
	return nil
}
