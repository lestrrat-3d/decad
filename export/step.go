// Package export writes decad bodies as STL, OBJ, and faceted AP214 STEP files.
package export

import (
	"context"
	"fmt"
	"io"
	"time"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/step"
	"github.com/lestrrat-3d/step/ap214"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// STEPMetadata holds the four caller-supplied fields for a faceted AP214 file.
// Name, Author, and Organization must be nonempty; Timestamp must be nonzero.
type STEPMetadata struct {
	Name         string
	Timestamp    time.Time
	Author       string
	Organization string
}

// STEPOption configures the STEP writer.
type STEPOption interface {
	option.Interface
	stepOption()
}

type stepOption struct{ option.Interface }

func (stepOption) stepOption() {}

type identSTEPHeader struct{}

// WithSTEPHeader replaces all STEPMetadata fields and defaults with header.
// It allows callers to set the complete Part 21 header. If supplied more
// than once, the last header wins.
func WithSTEPHeader(header step.Header) STEPOption {
	return stepOption{option.New(identSTEPHeader{}, header)}
}

// NewSTEPFile builds an AP214 file from one solid's boundary-verified mesh.
// tol is Body.Tessellate's chord tolerance. The caller supplies the mandatory
// header fields, including a timestamp; AP214's schema replaces Header.Schemas.
// The result contains one planar ADVANCED_FACE per mesh triangle. It is a
// faceted approximation, and cannot preserve analytic cylinders or splines.
// Sheets and bodies with multiple shells are refused. ctx and body must not be nil.
func NewSTEPFile(ctx context.Context, body *decad.Body, tol units.Value, header step.Header) (step.File, error) {
	if ctx == nil {
		return step.File{}, fmt.Errorf("export: STEP: %w: nil context", decad.ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return step.File{}, err
	}
	if body == nil {
		return step.File{}, fmt.Errorf("export: STEP: %w: nil body", decad.ErrDegenerate)
	}
	if body.Kind() != decad.BodySolid || !body.IsSolid() {
		return step.File{}, fmt.Errorf("export: STEP: %w: body does not enclose a valid region", decad.ErrNotSolid)
	}
	shells := body.Shells()
	if len(shells) != 1 || shells[0].IsVoid() {
		return step.File{}, fmt.Errorf("export: STEP: %w: only a single non-void shell is supported", decad.ErrUnsupported)
	}

	mesh, err := body.Tessellate(ctx, tol, decad.WithVerification(decad.VerifyBoundary))
	if err != nil {
		return step.File{}, err
	}
	if !mesh.BoundaryVerified() {
		return step.File{}, fmt.Errorf("export: STEP: %w: mesh boundary is not verified", decad.ErrUnsupported)
	}
	vertices, triangles := mesh.Vertices(), mesh.Triangles()
	if err := checkMesh(ctx, vertices, triangles); err != nil {
		return step.File{}, err
	}

	b := &fileBuilder{}
	millimetre := b.add(ap214.Millimetre(0))
	radian := b.add(ap214.Radian(0))
	steradian := b.add(ap214.Steradian(0))
	representationContext := b.add(ap214.GeometricRepresentationContext(0, "3D", "model", millimetre, radian, steradian))
	app := b.add(ap214.ApplicationContext(0, "automotive design"))
	b.add(ap214.ApplicationProtocolDefinition(0, "international standard", 2003, app))
	productContext := b.add(ap214.ProductContext(0, "mechanical parts", app, "mechanical"))
	product := b.add(ap214.Product(0, "1", header.Name, "faceted decad solid", productContext))
	b.add(ap214.ProductRelatedProductCategory(0, "part", product))
	formation := b.add(ap214.ProductDefinitionFormation(0, "1", "", product))
	definitionContext := b.add(ap214.ProductDefinitionContext(0, "part definition", app, "design"))
	definition := b.add(ap214.ProductDefinition(0, "design", "", formation, definitionContext))
	shape := b.add(ap214.ProductDefinitionShape(0, "shape", "", definition))

	pointRefs := make([]step.Reference, len(vertices))
	vertexRefs := make([]step.Reference, len(vertices))
	for i, p := range vertices {
		if err := ctx.Err(); err != nil {
			return step.File{}, err
		}
		pointRefs[i] = b.add(ap214.CartesianPoint(0, "", real3(p)))
		vertexRefs[i] = b.add(ap214.VertexPoint(0, "", pointRefs[i]))
	}

	edges := make(map[edgeKey]step.Reference, len(triangles)*3/2)
	faces := make([]step.Reference, 0, len(triangles))
	for i, tri := range triangles {
		if err := ctx.Err(); err != nil {
			return step.File{}, err
		}
		face, err := b.addTriangle(tri, vertices, pointRefs, vertexRefs, edges)
		if err != nil {
			return step.File{}, fmt.Errorf("export: STEP triangle %d: %w", i, err)
		}
		faces = append(faces, face)
	}
	shell := b.add(ap214.ClosedShell(0, "", faces...))
	brep := b.add(ap214.ManifoldSolidBrep(0, header.Name, shell))
	representation := b.add(ap214.AdvancedBrepShapeRepresentation(0, header.Name, representationContext, brep))
	b.add(ap214.ShapeDefinitionRepresentation(0, shape, representation))
	if err := ctx.Err(); err != nil {
		return step.File{}, err
	}
	return ap214.NewFile(header, b.entities...), nil
}

// STEP writes one faceted AP214 file. Metadata supplies the required Part 21
// fields; WithSTEPHeader can replace it with a complete step.Header. Geometry
// and references are built before the writer is touched. Invalid header data
// also leaves the writer untouched. An I/O error may leave a partial file.
// ctx, w, and body must not be nil.
func STEP(ctx context.Context, w io.Writer, body *decad.Body, tol units.Value, metadata STEPMetadata, opts ...STEPOption) error {
	if w == nil {
		return fmt.Errorf("export: STEP: %w: nil writer", decad.ErrDegenerate)
	}
	var header step.Header
	var overridden bool
	for _, opt := range opts {
		if opt == nil {
			return fmt.Errorf("export: STEP: %w: nil option", decad.ErrDegenerate)
		}
		if _, ok := opt.Ident().(identSTEPHeader); !ok {
			continue
		}
		value, ok := option.Get[step.Header](opt)
		if !ok {
			return fmt.Errorf("export: STEP: %w: invalid header option", decad.ErrDegenerate)
		}
		header = value
		overridden = true
	}
	if !overridden {
		if metadata.Name == "" || metadata.Author == "" || metadata.Organization == "" || metadata.Timestamp.IsZero() {
			return fmt.Errorf("export: STEP: %w: name, timestamp, author, and organization are required", decad.ErrDegenerate)
		}
		header = step.Header{
			Description:         []string{"faceted decad solid"},
			Name:                metadata.Name,
			Timestamp:           metadata.Timestamp,
			Authors:             []string{metadata.Author},
			Organizations:       []string{metadata.Organization},
			PreprocessorVersion: "decad export",
			OriginatingSystem:   "decad",
		}
	}
	file, err := NewSTEPFile(ctx, body, tol, header)
	if err != nil {
		return err
	}
	return file.Write(w)
}

type fileBuilder struct {
	entities []step.Entity
}

func (b *fileBuilder) add(entity step.Entity) step.Reference {
	entity.ID = uint64(len(b.entities) + 1)
	b.entities = append(b.entities, entity)
	return step.Reference(entity.ID)
}

type edgeKey struct{ a, b int }

func orderedEdge(a, b int) edgeKey {
	if a < b {
		return edgeKey{a, b}
	}
	return edgeKey{b, a}
}

func real3(v r3.Vec) [3]step.Real {
	return [3]step.Real{step.Real(v.X), step.Real(v.Y), step.Real(v.Z)}
}

func (b *fileBuilder) addTriangle(tri [3]int, vertices []r3.Vec, points, topologicalVertices []step.Reference, edges map[edgeKey]step.Reference) (step.Reference, error) {
	a, c, d := vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]
	x, ok := c.Sub(a).Normalize()
	if !ok {
		return 0, fmt.Errorf("%w: triangle edge has no direction", decad.ErrDegenerate)
	}
	z, ok := c.Sub(a).Cross(d.Sub(a)).Normalize()
	if !ok {
		return 0, fmt.Errorf("%w: triangle has no plane", decad.ErrDegenerate)
	}
	axis := b.add(ap214.Direction(0, "", real3(z)))
	refDirection := b.add(ap214.Direction(0, "", real3(x)))
	placement := b.add(ap214.Axis2Placement3D(0, "", points[tri[0]], axis, refDirection))
	plane := b.add(ap214.Plane(0, "", placement))

	oriented := make([]step.Reference, 0, 3)
	for j := range 3 {
		u, v := tri[j], tri[(j+1)%3]
		key := orderedEdge(u, v)
		edge, exists := edges[key]
		if !exists {
			vector, ok := vertices[key.b].Sub(vertices[key.a]).Normalize()
			if !ok {
				return 0, fmt.Errorf("%w: mesh edge has no direction", decad.ErrDegenerate)
			}
			direction := b.add(ap214.Direction(0, "", real3(vector)))
			unitVector := b.add(ap214.Vector(0, "", direction, 1))
			line := b.add(ap214.Line(0, "", points[key.a], unitVector))
			edge = b.add(ap214.EdgeCurve(0, "", topologicalVertices[key.a], topologicalVertices[key.b], line, true))
			edges[key] = edge
		}
		oriented = append(oriented, b.add(ap214.OrientedEdge(0, "", edge, u == key.a)))
	}
	loop := b.add(ap214.EdgeLoop(0, "", oriented...))
	bound := b.add(ap214.FaceOuterBound(0, "", loop, true))
	return b.add(ap214.AdvancedFace(0, "", plane, true, bound)), nil
}

type edgeUse struct {
	face    int
	forward bool
	count   int
}

// checkMesh is a reject-only guard for the STEP shell's narrower topology:
// exactly two opposite uses of each edge and one connected facet component.
func checkMesh(ctx context.Context, vertices []r3.Vec, triangles [][3]int) error {
	if len(triangles) == 0 {
		return fmt.Errorf("export: STEP: %w: mesh has no facets", decad.ErrUnsupported)
	}
	parents := make([]int, len(triangles))
	for i := range parents {
		parents[i] = i
	}
	uses := make(map[edgeKey]edgeUse, len(triangles)*3/2)
	for i, tri := range triangles {
		if err := ctx.Err(); err != nil {
			return err
		}
		for _, index := range tri {
			if index < 0 || index >= len(vertices) {
				return fmt.Errorf("export: STEP: %w: facet %d has an invalid vertex index", decad.ErrUnsupported, i)
			}
		}
		for j := range 3 {
			u, v := tri[j], tri[(j+1)%3]
			if u == v {
				return fmt.Errorf("export: STEP: %w: facet %d has a repeated vertex", decad.ErrUnsupported, i)
			}
			key := orderedEdge(u, v)
			use, exists := uses[key]
			if !exists {
				uses[key] = edgeUse{face: i, forward: u == key.a, count: 1}
				continue
			}
			if use.count != 1 || use.forward == (u == key.a) {
				return fmt.Errorf("export: STEP: %w: inconsistent mesh edge", decad.ErrUnsupported)
			}
			parents[meshRoot(parents, i)] = meshRoot(parents, use.face)
			use.count = 2
			uses[key] = use
		}
	}
	for _, use := range uses {
		if use.count != 2 {
			return fmt.Errorf("export: STEP: %w: open mesh edge", decad.ErrUnsupported)
		}
	}
	for i := 1; i < len(triangles); i++ {
		if meshRoot(parents, i) != meshRoot(parents, 0) {
			return fmt.Errorf("export: STEP: %w: disconnected mesh facets", decad.ErrUnsupported)
		}
	}
	return nil
}

func meshRoot(parents []int, i int) int {
	for parents[i] != i {
		i = parents[i]
	}
	return i
}
