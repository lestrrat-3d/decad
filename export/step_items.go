package export

import (
	"context"
	"errors"
	"fmt"
	"image/color"
	"io"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/step"
	"github.com/lestrrat-3d/step/ap214"
	"github.com/lestrrat-3d/units"
)

// STEPItem is one occurrence of a body in a STEP assembly: a body placed by a rigid motion.
type STEPItem struct {
	// Name names the occurrence and, for the first item naming Body, the part's PRODUCT and solid.
	// It must be non-empty and unique among the items.
	Name string
	// Body is a solid that NewSTEPFile accepts. Several items may name one body; it is written once and
	// placed once per item.
	Body *decad.Body
	// Placement moves Body from its own coordinates into the assembly. r3.Identity() keeps it where it is,
	// which is the assembled pose of a parts.Part body. A reflection or an invalid transform is refused.
	Placement r3.Transform
	// Tolerance is the chord tolerance for this body. The zero Value uses the call's tolerance and lets a
	// held faceted mesh that refuses it be written at its own floor; a non-zero Tolerance is used as given
	// and its refusal is returned.
	Tolerance units.Value
	// Color, when not nil, colours every solid of the body. Alpha is ignored.
	Color *color.RGBA
}

// STEPPart reports how one distinct body was written.
type STEPPart struct {
	Name      string      // product name: the first item naming the body
	Analytic  bool        // analytic faces, else one planar face per mesh triangle
	Tolerance units.Value // chord tolerance the body was tessellated at
	HeldFloor bool        // Tolerance was raised to the body's held mesh floor (decad.MeshBoundError)
}

// STEPItemsFile is an AP214 assembly file and how each part was written.
type STEPItemsFile struct {
	File  step.File
	Parts []STEPPart // one per distinct body, in first-item order
}

// NewSTEPItemsFile builds one AP214 assembly file from items: one product per distinct body, each written as
// NewSTEPFile writes it, and one assembly product holding one placed occurrence per item
// (agent-docs/step-export-design.md §10). A body whose held faceted mesh refuses the call's tolerance is
// written at that mesh's floor, which writes the same facets; STEPPart reports it. A non-zero
// STEPItem.Tolerance is used as given. header.Name names the assembly. An empty item list, a nil body, an
// empty or repeated name, an invalid or reflecting placement, or two items of one body that disagree on
// Tolerance or Color is decad.ErrDegenerate. A part the single-body writer refuses fails the call with an
// error naming the item and wrapping the cause. ctx must not be nil.
func NewSTEPItemsFile(ctx context.Context, items []STEPItem, tol units.Value, header step.Header) (STEPItemsFile, error) {
	if ctx == nil {
		return STEPItemsFile{}, fmt.Errorf("export: STEP: %w: nil context", decad.ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return STEPItemsFile{}, err
	}
	bodies, err := checkSTEPItems(items, tol)
	if err != nil {
		return STEPItemsFile{}, err
	}

	b := &fileBuilder{}
	sc := b.addSTEPContext()
	definitionContext := b.add(ap214.ProductDefinitionContext(0, "part definition", sc.app, "design"))

	parts := make([]STEPPart, len(bodies))
	definitions := make([]step.Reference, len(bodies))
	absrs := make([]step.Reference, len(bodies))
	origins := make([]step.Reference, len(bodies))
	solids := make([][]step.Reference, len(bodies))
	for k, first := range bodies {
		it := items[first]
		plan, used, held, err := planSTEPItem(ctx, it, tol)
		if err != nil {
			return STEPItemsFile{}, fmt.Errorf("export: STEP item %q: %w", it.Name, err)
		}
		parts[k] = STEPPart{Name: it.Name, Analytic: plan.analytic, Tolerance: used, HeldFloor: held}
		description := "faceted decad solid"
		if plan.analytic {
			description = "analytic decad solid"
		}
		definitions[k] = b.addSTEPProduct(sc, definitionContext, it.Name, description)
		pds := b.add(ap214.ProductDefinitionShape(0, "shape", "", definitions[k]))
		if solids[k], err = b.addSTEPSolids(ctx, it.Body, it.Name, plan); err != nil {
			return STEPItemsFile{}, fmt.Errorf("export: STEP item %q: %w", it.Name, err)
		}
		origins[k] = b.addOrigin()
		members := append(append([]step.Reference(nil), solids[k]...), origins[k])
		absrs[k] = b.add(ap214.AdvancedBrepShapeRepresentation(0, it.Name, sc.representation, members...))
		b.add(ap214.ShapeDefinitionRepresentation(0, pds, absrs[k]))
	}

	// The assembly: its product, then the axis of each occurrence, which its shape representation lists.
	assembly := b.addSTEPProduct(sc, definitionContext, header.Name, "decad assembly")
	assemblyShape := b.add(ap214.ProductDefinitionShape(0, "shape", "", assembly))
	assemblyOrigin := b.addOrigin()
	axes := make([]step.Reference, len(items))
	for i, it := range items {
		basis := it.Placement.Basis()
		location := b.add(ap214.CartesianPoint(0, "", real3(it.Placement.Translation())))
		axis := b.add(ap214.Direction(0, "", real3(basis.EZ)))
		ref := b.add(ap214.Direction(0, "", real3(basis.EX)))
		axes[i] = b.add(ap214.Axis2Placement3D(0, "", location, axis, ref))
	}
	assemblyRep := b.add(ap214.ShapeRepresentation(0, header.Name, sc.representation,
		append([]step.Reference{assemblyOrigin}, axes...)...))
	b.add(ap214.ShapeDefinitionRepresentation(0, assemblyShape, assemblyRep))

	bodyIndex := make(map[*decad.Body]int, len(bodies))
	for k, first := range bodies {
		bodyIndex[items[first].Body] = k
	}
	for i, it := range items {
		if err := ctx.Err(); err != nil {
			return STEPItemsFile{}, err
		}
		k := bodyIndex[it.Body]
		nauo := b.add(ap214.NextAssemblyUsageOccurrence(0, fmt.Sprintf("NAUO%d", i+1), it.Name, "", assembly, definitions[k]))
		usageShape := b.add(ap214.ProductDefinitionShape(0, "placement", "", nauo))
		transform := b.add(ap214.ItemDefinedTransformation(0, "", "", origins[k], axes[i]))
		relationship := b.add(ap214.ShapeRepresentationRelationshipWithTransformation(0, "", "", absrs[k], assemblyRep, transform))
		b.add(ap214.ContextDependentShapeRepresentation(0, relationship, usageShape))
	}
	b.addSTEPColours(sc, items, bodies, solids)

	if err := ctx.Err(); err != nil {
		return STEPItemsFile{}, err
	}
	return STEPItemsFile{File: ap214.NewFile(header, b.entities...), Parts: parts}, nil
}

// STEPItems writes one AP214 assembly file of items, as NewSTEPItemsFile builds it. It takes STEP's header
// options; the default description is "decad assembly". The file is built before w is touched. An I/O error
// may leave a partial file. ctx and w must not be nil.
func STEPItems(ctx context.Context, w io.Writer, items []STEPItem, tol units.Value, opts ...STEPOption) error {
	if ctx == nil {
		return fmt.Errorf("export: STEP: %w: nil context", decad.ErrDegenerate)
	}
	if w == nil {
		return fmt.Errorf("export: STEP: %w: nil writer", decad.ErrDegenerate)
	}
	header, err := stepHeader(opts, "decad assembly")
	if err != nil {
		return err
	}
	file, err := NewSTEPItemsFile(ctx, items, tol, header)
	if err != nil {
		return err
	}
	return file.File.Write(w)
}

// checkSTEPItems validates items without doing any geometry work and returns the index of the first item of
// each distinct body, in first-item order.
func checkSTEPItems(items []STEPItem, tol units.Value) ([]int, error) {
	if len(items) == 0 {
		return nil, fmt.Errorf("export: STEP: %w: no items", decad.ErrDegenerate)
	}
	names := make(map[string]struct{}, len(items))
	first := make(map[*decad.Body]int, len(items))
	var bodies []int
	needTol := false
	for i, it := range items {
		if it.Body == nil {
			return nil, fmt.Errorf("export: STEP: %w: item %d has no body", decad.ErrDegenerate, i)
		}
		if it.Name == "" {
			return nil, fmt.Errorf("export: STEP: %w: item %d has no name", decad.ErrDegenerate, i)
		}
		if _, dup := names[it.Name]; dup {
			return nil, fmt.Errorf("export: STEP: %w: item name %q is repeated", decad.ErrDegenerate, it.Name)
		}
		names[it.Name] = struct{}{}
		if !it.Placement.IsValid() || it.Placement.IsReflection() {
			return nil, fmt.Errorf("export: STEP: %w: item %q has no proper rigid placement", decad.ErrDegenerate, it.Name)
		}
		if it.Tolerance != (units.Value{}) {
			if err := checkSTEPTolerance(it.Tolerance); err != nil {
				return nil, fmt.Errorf("export: STEP item %q: %w", it.Name, err)
			}
		} else {
			needTol = true
		}
		k, seen := first[it.Body]
		if !seen {
			first[it.Body] = i
			bodies = append(bodies, i)
			continue
		}
		prev := items[k]
		if prev.Tolerance != it.Tolerance || !sameSTEPColor(prev.Color, it.Color) {
			return nil, fmt.Errorf("export: STEP: %w: items %q and %q share a body but differ in Tolerance or Color",
				decad.ErrDegenerate, prev.Name, it.Name)
		}
	}
	if needTol {
		if err := checkSTEPTolerance(tol); err != nil {
			return nil, fmt.Errorf("export: STEP: call tolerance: %w", err)
		}
	}
	return bodies, nil
}

// checkSTEPTolerance requires a finite, positive length.
func checkSTEPTolerance(tol units.Value) error {
	switch {
	case tol.Kind() != units.Length:
		return fmt.Errorf("%w: a chord tolerance must be a length", decad.ErrUnitKind)
	case math.IsNaN(tol.Mag()) || math.IsInf(tol.Mag(), 0):
		return fmt.Errorf("%w: a chord tolerance must be finite", decad.ErrNotFinite)
	case tol.Mag() < 0:
		return fmt.Errorf("%w: a chord tolerance must not be negative", decad.ErrNegativeMagnitude)
	case tol.Mag() == 0:
		return fmt.Errorf("%w: a zero chord tolerance admits no chord", decad.ErrDegenerate)
	}
	return nil
}

func sameSTEPColor(a, b *color.RGBA) bool {
	if a == nil || b == nil {
		return a == b
	}
	return a.R == b.R && a.G == b.G && a.B == b.B
}

// planSTEPItem plans item's body. An item without its own tolerance uses tol and, when a held faceted mesh
// refuses it, retries once at the mesh's floor, which restates the same held facets. held reports the retry.
func planSTEPItem(ctx context.Context, it STEPItem, tol units.Value) (plan stepPlan, used units.Value, held bool, err error) {
	explicit := it.Tolerance != (units.Value{})
	used = tol
	if explicit {
		used = it.Tolerance
	}
	plan, err = planSTEPBody(ctx, it.Body, used)
	var bound *decad.MeshBoundError
	if err != nil && !explicit && errors.As(err, &bound) {
		used, held = bound.Floor(), true
		plan, err = planSTEPBody(ctx, it.Body, used)
	}
	return plan, used, held, err
}

// addSTEPProduct writes a product, its category, formation and definition, and returns the definition. The product's id and name are name.
func (b *fileBuilder) addSTEPProduct(sc stepContext, definitionContext step.Reference, name, description string,
) step.Reference {
	product := b.add(ap214.Product(0, name, name, description, sc.productContext))
	b.add(ap214.ProductRelatedProductCategory(0, "part", product))
	formation := b.add(ap214.ProductDefinitionFormation(0, "1", "", product))
	return b.add(ap214.ProductDefinition(0, "design", "", formation, definitionContext))
}

// addOrigin writes the axis placement at the origin with Z as its axis and X as its reference direction.
func (b *fileBuilder) addOrigin() step.Reference {
	location := b.add(ap214.CartesianPoint(0, "", real3(r3.Vec{})))
	axis := b.add(ap214.Direction(0, "", real3(r3.Vec{Z: 1})))
	ref := b.add(ap214.Direction(0, "", real3(r3.Vec{X: 1})))
	return b.add(ap214.Axis2Placement3D(0, "", location, axis, ref))
}

// addSTEPColours writes one colour style per distinct RGB, in first-use order, a STYLED_ITEM for each solid
// of each coloured body, and the presentation representation that holds them. It writes nothing when no item
// has a colour.
func (b *fileBuilder) addSTEPColours(sc stepContext, items []STEPItem, bodies []int, solids [][]step.Reference) {
	type rgb struct{ r, g, b uint8 }
	styles := map[rgb]step.Reference{}
	var styled []step.Reference
	for k, first := range bodies {
		c := items[first].Color
		if c == nil {
			continue
		}
		key := rgb{c.R, c.G, c.B}
		assignment, ok := styles[key]
		if !ok {
			colour := b.add(ap214.ColourRGB(0, "", step.Real(float64(c.R)/255), step.Real(float64(c.G)/255), step.Real(float64(c.B)/255)))
			fill := b.add(ap214.FillAreaStyleColour(0, "", colour))
			area := b.add(ap214.FillAreaStyle(0, "", fill))
			surface := b.add(ap214.SurfaceStyleFillArea(0, area))
			side := b.add(ap214.SurfaceSideStyle(0, "", surface))
			usage := b.add(ap214.SurfaceStyleUsage(0, "BOTH", side))
			assignment = b.add(ap214.PresentationStyleAssignment(0, usage))
			styles[key] = assignment
		}
		for _, solid := range solids[k] {
			styled = append(styled, b.add(ap214.StyledItem(0, "color", []step.Reference{assignment}, solid)))
		}
	}
	if len(styled) > 0 {
		b.add(ap214.MechanicalDesignGeometricPresentationRepresentation(0, "", sc.representation, styled...))
	}
}
