package export_test

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/hex"
	"image/color"
	"math"
	"slices"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/step"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

const (
	itemBoxA    = "box-a"
	itemPlate   = "plate"
	planeRecord = "PLANE"
)

var (
	itemRed  = &color.RGBA{R: 255, G: 0, B: 0, A: 255}
	itemBlue = &color.RGBA{R: 0, G: 51, B: 255, A: 255}
)

// stepNames lists the record names of an entity: its own, or those of a complex instance.
func stepNames(e step.Entity) []string {
	if e.Name != "" {
		return []string{e.Name}
	}
	names := make([]string, len(e.Components))
	for i, c := range e.Components {
		names[i] = c.Name
	}
	return names
}

// stepByName returns the entities with a record of the given name, in file order.
func stepByName(f step.File, name string) []step.Entity {
	var out []step.Entity
	for _, e := range f.Entities {
		if slices.Contains(stepNames(e), name) {
			out = append(out, e)
		}
	}
	return out
}

func stepParams(e step.Entity, record string) []step.Value {
	if e.Name == record {
		return e.Parameters
	}
	for _, c := range e.Components {
		if c.Name == record {
			return c.Parameters
		}
	}
	return nil
}

func stepRefs(v step.Value, into *[]step.Reference) {
	switch v := v.(type) {
	case step.Reference:
		*into = append(*into, v)
	case step.List:
		for _, x := range v {
			stepRefs(x, into)
		}
	case step.Typed:
		stepRefs(v.Value, into)
	}
}

func stepList(v step.Value) []step.Reference {
	var out []step.Reference
	stepRefs(v, &out)
	return out
}

func itemsIndex(f step.File) map[step.Reference]step.Entity {
	m := make(map[step.Reference]step.Entity, len(f.Entities))
	for _, e := range f.Entities {
		m[step.Reference(e.ID)] = e
	}
	return m
}

func stepVec(t *testing.T, idx map[step.Reference]step.Entity, ref step.Reference) r3.Vec {
	t.Helper()
	e, ok := idx[ref]
	require.True(t, ok)
	var vals []step.Value
	for _, v := range e.Parameters {
		if l, isList := v.(step.List); isList {
			vals = l
		}
	}
	require.Len(t, vals, 3)
	return r3.Vec{X: float64(vals[0].(step.Real)), Y: float64(vals[1].(step.Real)), Z: float64(vals[2].(step.Real))}
}

func rotatedPlacement(t *testing.T) r3.Transform {
	t.Helper()
	turn, err := r3.Rotation(r3.Vec{Z: 1}, units.Degrees(90))
	require.NoError(t, err)
	placed, err := turn.Then(translation(t, r3.Vec{X: 10, Y: 20, Z: 5}))
	require.NoError(t, err)
	return placed
}

// goldenItems is a small assembly: an analytic box twice, and a faceted plate. The box's two occurrences
// share one product.
func goldenItems(t *testing.T) []export.STEPItem {
	t.Helper()
	b := box(t, false)
	return []export.STEPItem{
		{Name: itemBoxA, Body: b, Placement: r3.Identity(), Color: itemRed},
		{Name: itemPlate, Body: facetedCutPlate(t), Placement: rotatedPlacement(t), Color: itemBlue},
		{Name: "box-b", Body: b, Placement: translation(t, r3.Vec{X: 50}), Color: itemRed},
	}
}

func itemsFile(t *testing.T, items []export.STEPItem, tol units.Value) export.STEPItemsFile {
	t.Helper()
	f, err := export.NewSTEPItemsFile(t.Context(), items, tol, header())
	require.NoError(t, err)
	return f
}

func hashWithoutTimestamp(t *testing.T, f step.File) string {
	t.Helper()
	data, err := f.Marshal()
	require.NoError(t, err)
	var kept []string
	for _, line := range strings.SplitAfter(string(data), "\n") {
		if !strings.HasPrefix(line, "FILE_NAME(") {
			kept = append(kept, line)
		}
	}
	sum := sha256.Sum256([]byte(strings.Join(kept, "")))
	return hex.EncodeToString(sum[:])
}

func TestSTEPItemsGoldenBytesArePinned(t *testing.T) {
	t.Parallel()
	f := itemsFile(t, goldenItems(t), units.Millimeters(0.1))
	require.Equal(t, "e1c7051bf4d558ba596b2c3d27fa9efb15055612b76a84706400f79625129086", hashWithoutTimestamp(t, f.File))
}

func TestSTEPItemsStructure(t *testing.T) {
	t.Parallel()
	items := goldenItems(t)
	f := itemsFile(t, items, units.Millimeters(0.1))

	counts := map[string]int{
		"PRODUCT": 3, "NEXT_ASSEMBLY_USAGE_OCCURRENCE": 3, "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION": 3,
		"ITEM_DEFINED_TRANSFORMATION": 3, "ADVANCED_BREP_SHAPE_REPRESENTATION": 2, "SHAPE_REPRESENTATION": 1,
		"SHAPE_DEFINITION_REPRESENTATION": 3, "GEOMETRIC_REPRESENTATION_CONTEXT": 1, "PRODUCT_DEFINITION_CONTEXT": 1,
		"SHAPE_REPRESENTATION_RELATIONSHIP": 3, "COLOUR_RGB": 2, "STYLED_ITEM": 2,
		"MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION": 1,
	}
	for name, want := range counts {
		require.Len(t, stepByName(f.File, name), want, name)
	}

	require.Equal(t, []export.STEPPart{
		{Name: itemBoxA, Analytic: true, Tolerance: units.Millimeters(0.1)},
		{Name: itemPlate, Analytic: false, Tolerance: units.Millimeters(0.1)},
	}, f.Parts)

	var productNames []string
	for _, p := range stepByName(f.File, "PRODUCT") {
		productNames = append(productNames, string(p.Parameters[1].(step.String)))
	}
	require.Equal(t, []string{itemBoxA, itemPlate, "box.step"}, productNames)

	// Each usage occurrence relates the assembly's definition to its body's definition and carries the item's name.
	pdOf := func(product string) step.Reference {
		for _, p := range stepByName(f.File, "PRODUCT") {
			if string(p.Parameters[1].(step.String)) != product {
				continue
			}
			for _, pdf := range stepByName(f.File, "PRODUCT_DEFINITION_FORMATION") {
				if pdf.Parameters[2] != step.Reference(p.ID) {
					continue
				}
				for _, pd := range stepByName(f.File, "PRODUCT_DEFINITION") {
					if pd.Parameters[2] == step.Reference(pdf.ID) {
						return step.Reference(pd.ID)
					}
				}
			}
		}
		t.Fatalf("no definition for product %q", product)
		return 0
	}
	wantRelated := []string{itemBoxA, itemPlate, itemBoxA}
	for i, n := range stepByName(f.File, "NEXT_ASSEMBLY_USAGE_OCCURRENCE") {
		require.Equal(t, step.String(items[i].Name), n.Parameters[1])
		require.Equal(t, pdOf("box.step"), n.Parameters[3])
		require.Equal(t, pdOf(wantRelated[i]), n.Parameters[4])
	}
}

func TestSTEPItemsPlacement(t *testing.T) {
	t.Parallel()
	items := goldenItems(t)
	f := itemsFile(t, items, units.Millimeters(0.1))
	idx := itemsIndex(f.File)

	assemblyRep := stepByName(f.File, "SHAPE_REPRESENTATION")[0]
	assemblyItems := stepList(assemblyRep.Parameters[1])
	absrItems := map[step.Reference][]step.Reference{}
	for _, a := range stepByName(f.File, "ADVANCED_BREP_SHAPE_REPRESENTATION") {
		absrItems[step.Reference(a.ID)] = stepList(a.Parameters[1])
	}

	idts := stepByName(f.File, "ITEM_DEFINED_TRANSFORMATION")
	cdsrs := stepByName(f.File, "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION")
	for i, it := range items {
		cdsr := cdsrs[i]
		rr := idx[cdsr.Parameters[0].(step.Reference)]
		pair := stepParams(rr, "REPRESENTATION_RELATIONSHIP")
		child, parent := pair[2].(step.Reference), pair[3].(step.Reference)
		require.Equal(t, step.Reference(assemblyRep.ID), parent)
		require.Contains(t, absrItems, child)
		idt := idx[stepParams(rr, "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION")[0].(step.Reference)]
		require.Equal(t, step.Reference(idts[i].ID), step.Reference(idt.ID))
		item1, item2 := idt.Parameters[2].(step.Reference), idt.Parameters[3].(step.Reference)
		require.True(t, slices.Contains(absrItems[child], item1), "item_1 is in the child representation")
		require.True(t, slices.Contains(assemblyItems, item2), "item_2 is in the assembly representation")

		axis := idx[item2]
		location := stepVec(t, idx, axis.Parameters[1].(step.Reference))
		basis := it.Placement.Basis()
		require.Equal(t, it.Placement.Translation(), location)
		require.Equal(t, basis.EZ, stepVec(t, idx, axis.Parameters[2].(step.Reference)))
		require.Equal(t, basis.EX, stepVec(t, idx, axis.Parameters[3].(step.Reference)))

		// The child axis is the origin, so the placement alone moves the part.
		origin := idx[item1]
		require.Equal(t, r3.Vec{}, stepVec(t, idx, origin.Parameters[1].(step.Reference)))
		require.Equal(t, r3.Vec{Z: 1}, stepVec(t, idx, origin.Parameters[2].(step.Reference)))
		require.Equal(t, r3.Vec{X: 1}, stepVec(t, idx, origin.Parameters[3].(step.Reference)))
	}
}

func TestSTEPItemsSolidsMatchSingleFile(t *testing.T) {
	t.Parallel()
	cases := map[string]*decad.Body{"analytic": box(t, false), "faceted": facetedCutPlate(t), "cavity": cavityCube(t)}
	solidRecords := []string{
		"MANIFOLD_SOLID_BREP", "BREP_WITH_VOIDS", "ORIENTED_CLOSED_SHELL", "CLOSED_SHELL", "ADVANCED_FACE", "FACE_OUTER_BOUND",
		"EDGE_LOOP", "ORIENTED_EDGE", "EDGE_CURVE", "VERTEX_POINT", planeRecord, "CYLINDRICAL_SURFACE", "LINE", "CIRCLE",
	}
	for name, body := range cases {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			tol := units.Millimeters(0.1)
			single, err := export.NewSTEPFile(t.Context(), body, tol, header())
			require.NoError(t, err)
			assembly := itemsFile(t, []export.STEPItem{{Name: "p", Body: body, Placement: r3.Identity()}}, tol)
			for _, r := range solidRecords {
				require.Len(t, stepByName(assembly.File, r), len(stepByName(single, r)), r)
			}
		})
	}
}

// TestSTEPItemsReaderRefusesAssembly pins that decad.ReadSTEP does not read placements, so it refuses an
// assembly file rather than return parts at their own origins.
func TestSTEPItemsReaderRefusesAssembly(t *testing.T) {
	t.Parallel()
	var out bytes.Buffer
	err := export.STEPItems(t.Context(), &out, []export.STEPItem{
		{Name: "a", Body: box(t, false), Placement: r3.Identity()},
		{Name: "b", Body: cavityCube(t), Placement: r3.Identity()},
	}, units.Millimeters(0.1), export.WithSTEPHeader(header()))
	require.NoError(t, err)
	_, err = decad.ReadSTEP(t.Context(), bytes.NewReader(out.Bytes()))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

func TestSTEPItemsEntityIntegrity(t *testing.T) {
	t.Parallel()
	f := itemsFile(t, goldenItems(t), units.Millimeters(0.1)).File
	seen := map[uint64]bool{}
	for i, e := range f.Entities {
		require.EqualValues(t, i+1, e.ID, "IDs run 1..N in order")
		require.False(t, seen[e.ID])
		seen[e.ID] = true
	}
	for _, e := range f.Entities {
		var refs []step.Reference
		for _, v := range e.Parameters {
			stepRefs(v, &refs)
		}
		for _, c := range e.Components {
			for _, v := range c.Parameters {
				stepRefs(v, &refs)
			}
		}
		for _, r := range refs {
			require.True(t, seen[uint64(r)], "#%d refers to missing #%d", e.ID, r)
		}
	}
	var out bytes.Buffer
	require.NoError(t, f.Write(&out))
}

func TestSTEPItemsDeterministic(t *testing.T) {
	t.Parallel()
	a := hashWithoutTimestamp(t, itemsFile(t, goldenItems(t), units.Millimeters(0.1)).File)
	b := hashWithoutTimestamp(t, itemsFile(t, goldenItems(t), units.Millimeters(0.1)).File)
	require.Equal(t, a, b)

	var one, two bytes.Buffer
	items := goldenItems(t)
	require.NoError(t, export.STEPItems(t.Context(), &one, items, units.Millimeters(0.1), export.WithSTEPHeader(header())))
	require.NoError(t, export.STEPItems(t.Context(), &two, items, units.Millimeters(0.1), export.WithSTEPHeader(header())))
	require.Equal(t, one.Bytes(), two.Bytes())
}

func TestSTEPItemsColour(t *testing.T) {
	t.Parallel()
	b := box(t, false)
	c := cavityCube(t)
	plain := facetedCutPlate(t)
	f := itemsFile(t, []export.STEPItem{
		{Name: "one", Body: b, Placement: r3.Identity(), Color: itemRed},
		{Name: "two", Body: c, Placement: translation(t, r3.Vec{X: 40}), Color: &color.RGBA{R: 255, A: 7}},
		{Name: "three", Body: plain, Placement: translation(t, r3.Vec{X: 80})},
	}, units.Millimeters(0.1))
	require.Len(t, stepByName(f.File, "COLOUR_RGB"), 1, "equal RGB share one colour chain")
	require.Equal(t, []step.Value{step.String(""), step.Real(1), step.Real(0), step.Real(0)},
		stepByName(f.File, "COLOUR_RGB")[0].Parameters)
	require.Len(t, stepByName(f.File, "PRESENTATION_STYLE_ASSIGNMENT"), 1)
	require.Len(t, stepByName(f.File, "STYLED_ITEM"), 2, "box and cavity cube solids; the uncoloured plate has none")
	mdgpr := stepByName(f.File, "MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION")
	require.Len(t, mdgpr, 1)
	require.Equal(t, step.Reference(stepByName(f.File, "GEOMETRIC_REPRESENTATION_CONTEXT")[0].ID), mdgpr[0].Parameters[2])
	require.Equal(t, uint64(len(f.File.Entities)), mdgpr[0].ID, "the presentation representation is last")

	none := itemsFile(t, []export.STEPItem{{Name: "one", Body: b, Placement: r3.Identity()}}, units.Millimeters(0.1))
	require.Empty(t, stepByName(none.File, "STYLED_ITEM"))
	require.Empty(t, stepByName(none.File, "MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION"))
}

func TestSTEPItemsHeldFloor(t *testing.T) {
	t.Parallel()
	plate := facetedCutPlate(t)
	held, err := plate.Tessellate(t.Context(), units.Millimeters(1))
	require.NoError(t, err)
	floor := held.Bound()
	require.Positive(t, floor.Mag())
	below := units.Millimeters(math.Nextafter(floor.Mag(), 0))
	cube := box(t, false)

	t.Run("AutomaticRetry", func(t *testing.T) {
		t.Parallel()
		f := itemsFile(t, []export.STEPItem{
			{Name: itemPlate, Body: plate, Placement: r3.Identity()},
			{Name: "cube", Body: cube, Placement: translation(t, r3.Vec{X: 40})},
		}, below)
		require.Equal(t, []export.STEPPart{
			{Name: itemPlate, Analytic: false, Tolerance: floor, HeldFloor: true},
			{Name: "cube", Analytic: true, Tolerance: below},
		}, f.Parts)
		single, err := export.NewSTEPFile(t.Context(), plate, floor, header())
		require.NoError(t, err)
		require.Len(t, stepByName(f.File, "ADVANCED_FACE"), len(stepByName(single, "ADVANCED_FACE"))+6)
	})
	t.Run("ExplicitIsNotRetried", func(t *testing.T) {
		t.Parallel()
		_, err := export.NewSTEPItemsFile(t.Context(), []export.STEPItem{
			{Name: itemPlate, Body: plate, Placement: r3.Identity(), Tolerance: below},
		}, units.Millimeters(1), header())
		var bound *decad.MeshBoundError
		require.ErrorAs(t, err, &bound)
		require.Equal(t, floor, bound.Floor())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, `"plate"`)
	})
	t.Run("ExplicitAtFloorIsUsedAsGiven", func(t *testing.T) {
		t.Parallel()
		f := itemsFile(t, []export.STEPItem{{Name: itemPlate, Body: plate, Placement: r3.Identity(), Tolerance: floor}},
			units.Millimeters(1))
		require.Equal(t, export.STEPPart{Name: itemPlate, Tolerance: floor}, f.Parts[0])
	})
}

func TestSTEPItemsRefusals(t *testing.T) {
	t.Parallel()
	body := box(t, false)
	sheet := box(t, true)
	plane, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	reflect, err := r3.Reflection(plane)
	require.NoError(t, err)
	tol := units.Millimeters(0.1)
	good := export.STEPItem{Name: "a", Body: body, Placement: r3.Identity()}
	with := func(f func(*export.STEPItem)) []export.STEPItem {
		it := good
		f(&it)
		return []export.STEPItem{it}
	}
	cases := []struct {
		name  string
		items []export.STEPItem
		tol   units.Value
		is    error
	}{
		{"NoItems", nil, tol, decad.ErrDegenerate},
		{"NilBody", with(func(i *export.STEPItem) { i.Body = nil }), tol, decad.ErrDegenerate},
		{"EmptyName", with(func(i *export.STEPItem) { i.Name = "" }), tol, decad.ErrDegenerate},
		{"DuplicateName", []export.STEPItem{good, good}, tol, decad.ErrDegenerate},
		{"InvalidPlacement", with(func(i *export.STEPItem) { i.Placement = r3.Transform{} }), tol, decad.ErrDegenerate},
		{"Reflection", with(func(i *export.STEPItem) { i.Placement = reflect }), tol, decad.ErrDegenerate},
		{"NegativeTolerance", with(func(i *export.STEPItem) { i.Tolerance = units.Millimeters(-1) }), tol, decad.ErrNegativeMagnitude},
		{"NonLengthTolerance", with(func(i *export.STEPItem) { i.Tolerance = units.Degrees(1) }), tol, decad.ErrUnitKind},
		{"ZeroCallTolerance", []export.STEPItem{good}, units.Millimeters(0), decad.ErrDegenerate},
		{"SharedBodyTolerance", []export.STEPItem{good,
			{Name: "b", Body: body, Placement: r3.Identity(), Tolerance: units.Millimeters(1)}}, tol, decad.ErrDegenerate},
		{"SharedBodyColor", []export.STEPItem{good,
			{Name: "b", Body: body, Placement: r3.Identity(), Color: itemRed}}, tol, decad.ErrDegenerate},
		{"Sheet", []export.STEPItem{{Name: "s", Body: sheet, Placement: r3.Identity()}}, tol, decad.ErrNotSolid},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			var out bytes.Buffer
			err := export.STEPItems(t.Context(), &out, c.items, c.tol, export.WithSTEPHeader(header()))
			require.ErrorIs(t, err, c.is)
			require.Zero(t, out.Len(), "the writer is untouched")
		})
	}

	t.Run("NilContext", func(t *testing.T) {
		t.Parallel()
		//nolint:staticcheck // a nil context is the case under test
		_, err := export.NewSTEPItemsFile(nil, []export.STEPItem{good}, tol, header())
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("NilWriter", func(t *testing.T) {
		t.Parallel()
		err := export.STEPItems(t.Context(), nil, []export.STEPItem{good}, tol)
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("Cancelled", func(t *testing.T) {
		t.Parallel()
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		var out bytes.Buffer
		err := export.STEPItems(ctx, &out, []export.STEPItem{good}, tol, export.WithSTEPHeader(header()))
		require.ErrorIs(t, err, context.Canceled)
		require.Zero(t, out.Len())
	})
	t.Run("PartErrorNamesItem", func(t *testing.T) {
		t.Parallel()
		_, err := export.NewSTEPItemsFile(t.Context(), []export.STEPItem{good,
			{Name: "the-sheet", Body: sheet, Placement: translation(t, r3.Vec{X: 9})}}, tol, header())
		require.ErrorIs(t, err, decad.ErrNotSolid)
		require.ErrorContains(t, err, `"the-sheet"`)
	})
}

func TestSTEPItemsDefaultDescription(t *testing.T) {
	t.Parallel()
	var out bytes.Buffer
	require.NoError(t, export.STEPItems(t.Context(), &out, []export.STEPItem{{Name: "a", Body: box(t, false), Placement: r3.Identity()}},
		units.Millimeters(0.1), export.WithSTEPName("asm.step"), export.WithSTEPAuthor("a"), export.WithSTEPOrganization("o")))
	require.Contains(t, out.String(), "'decad assembly'")
}
