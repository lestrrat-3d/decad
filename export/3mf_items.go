package export

import (
	"context"
	"fmt"
	"io"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// ThreeMFItem is one object of a 3MF package: a body placed by a rigid motion.
type ThreeMFItem struct {
	Name      string
	Body      *decad.Body
	Placement r3.Transform
}

// ThreeMFItems writes several bodies as one deterministic 3MF Core package: one
// named object and one build item each, vertices moved by the item's
// Placement. A body named by several items is tessellated once. A Placement
// that reflects, or is not a valid transform, is decad.ErrDegenerate, because a
// reflection would turn the mesh inside out. An empty item list, or a nil body,
// is decad.ErrDegenerate. The writer's other rules are ThreeMF's. ctx and w
// must not be nil.
func ThreeMFItems(ctx context.Context, w io.Writer, items []ThreeMFItem, tol units.Value,
	opts ...decad.TessellateOption) error {
	if ctx == nil {
		return fmt.Errorf("export: 3MF: %w: nil context", decad.ErrDegenerate)
	}
	if w == nil {
		return fmt.Errorf("export: 3MF: %w: nil writer", decad.ErrDegenerate)
	}
	if len(items) == 0 {
		return fmt.Errorf("export: 3MF: %w: no items", decad.ErrDegenerate)
	}
	for i, it := range items {
		if it.Body == nil {
			return fmt.Errorf("export: 3MF: %w: item %d has no body", decad.ErrDegenerate, i)
		}
		if !it.Placement.IsValid() || it.Placement.IsReflection() {
			return fmt.Errorf("export: 3MF: %w: item %d has no proper rigid placement", decad.ErrDegenerate, i)
		}
	}
	meshes := map[*decad.Body]*decad.Mesh{}
	objects := make([]threeMFObject, 0, len(items))
	for _, it := range items {
		mesh, ok := meshes[it.Body]
		if !ok {
			var err error
			if mesh, err = tessellateForExport(ctx, it.Body, tol, opts); err != nil {
				return err
			}
			meshes[it.Body] = mesh
		}
		src := mesh.Vertices()
		moved := make([]r3.Vec, len(src))
		for i, v := range src {
			moved[i] = it.Placement.Apply(v)
		}
		objects = append(objects, threeMFObject{name: it.Name, kind: it.Body.Kind(), vertices: moved,
			triangles: mesh.Triangles()})
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	return writeThreeMFPackage(ctx, w, objects)
}
