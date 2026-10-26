package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/meshrepair"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// MeshRepairOption configures [RepairMesh].
type MeshRepairOption interface {
	option.Interface
	meshRepairOption()
}

type meshRepairOption struct{ option.Interface }

func (meshRepairOption) meshRepairOption() {}

type identMeshWeld struct{}

type identMeshOrient struct{}

type identMeshFillHoles struct{}

// WithMeshWeld merges vertices that lie within distance of one another into the
// earliest of them. distance is a non-negative length; zero merges only vertices
// at identical coordinates. A near-equal pair that straddles a seam is joined,
// so the seam closes, and a pair of distinct features closer than distance is
// joined too: pick a distance well below the mesh's finest detail.
func WithMeshWeld(distance units.Value) MeshRepairOption {
	return meshRepairOption{option.New(identMeshWeld{}, distance)}
}

// WithMeshOrient winds every shell consistently and turns a closed shell that
// is inside out right way out. Shells are the groups of triangles that meet
// along edges which exactly two triangles use; an edge that three or more
// triangles use joins nothing.
func WithMeshOrient() MeshRepairOption {
	return meshRepairOption{option.New(identMeshOrient{}, true)}
}

// WithMeshFillHoles closes each simple boundary loop, one whose every vertex
// has one way onward along the boundary. A loop of three is closed with one
// triangle and a longer one with a fan from a new vertex at the loop's centroid.
// The caps are flat only when the loop is, so a bent loop closes to a surface
// that need not match the part it came from. A boundary that branches is left
// open and counted in [MeshRepairReport].SkippedLoops.
func WithMeshFillHoles() MeshRepairOption {
	return meshRepairOption{option.New(identMeshFillHoles{}, true)}
}

// MeshRepairReport counts what [RepairMesh] changed.
type MeshRepairReport struct {
	// WeldedVertices counts the vertices merged into an earlier one.
	WeldedVertices int
	// DroppedTriangles counts the triangles removed: those that name a vertex
	// twice, have zero area, or repeat an earlier triangle's vertices.
	DroppedTriangles int
	// FlippedTriangles counts the triangles whose winding was reversed.
	FlippedTriangles int
	// FilledLoops counts the boundary loops closed, and SkippedLoops the
	// connected runs of boundary left open because they branch.
	FilledLoops, SkippedLoops int
	// AddedTriangles counts the triangles the fills added.
	AddedTriangles int
}

// RepairMesh mends m and returns the result as a new [MeshBody] with a report of
// what changed (agent-docs/mesh-repair-design.md). m is not changed.
//
// It always drops triangles that carry no surface: those that name a vertex
// twice, have zero area, or repeat an earlier triangle's vertices. Then, in this
// order, [WithMeshWeld] merges close vertices, [WithMeshOrient] winds shells
// consistently and [WithMeshFillHoles] closes simple holes. Nothing else is
// repaired: self-intersections, edges three triangles use, and sliver triangles
// stay as they are. The result reports its own state through
// [MeshBody.Report], which is what says whether the repair closed the mesh.
//
// ctx and m MUST NOT be nil, each otherwise ErrDegenerate. A weld distance that
// is negative is ErrNegativeMagnitude, one that is not a length is ErrUnitKind,
// and an option given twice or a nil option is ErrDegenerate. A mesh left with
// no triangle is ErrDegenerate. A canceled context returns its error.
func RepairMesh(ctx context.Context, m *MeshBody, opts ...MeshRepairOption) (*MeshBody, MeshRepairReport, error) {
	if ctx == nil {
		return nil, MeshRepairReport{}, fmt.Errorf(`%w: a nil context cannot control a mesh repair`, ErrDegenerate)
	}
	if m == nil {
		return nil, MeshRepairReport{}, fmt.Errorf(`%w: a mesh repair needs a mesh body`, ErrDegenerate)
	}
	var o meshrepair.Options
	for _, opt := range opts {
		if opt == nil {
			return nil, MeshRepairReport{}, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
		switch opt.Ident().(type) {
		case identMeshWeld:
			distance, _ := option.Get[units.Value](opt)
			if o.Weld.Enabled {
				return nil, MeshRepairReport{}, fmt.Errorf(`%w: WithMeshWeld was passed more than once`, ErrDegenerate)
			}
			mm, err := sectionrecord.MagnitudeIn(distance, units.Length, units.Millimeter, "the mesh weld distance")
			if err != nil {
				return nil, MeshRepairReport{}, err
			}
			o.Weld.Enabled, o.Weld.Distance = true, mm
		case identMeshOrient:
			if o.Orient {
				return nil, MeshRepairReport{}, fmt.Errorf(`%w: WithMeshOrient was passed more than once`, ErrDegenerate)
			}
			o.Orient = true
		case identMeshFillHoles:
			if o.FillHoles {
				return nil, MeshRepairReport{}, fmt.Errorf(`%w: WithMeshFillHoles was passed more than once`, ErrDegenerate)
			}
			o.FillHoles = true
		}
	}
	got, err := meshrepair.Repair(ctx, m.vertices, m.triangles, o)
	if err != nil {
		return nil, MeshRepairReport{}, err
	}
	out, err := NewMeshBody(ctx, got.Vertices, got.Triangles)
	if err != nil {
		return nil, MeshRepairReport{}, err
	}
	return out, MeshRepairReport(got.Stats), nil
}
